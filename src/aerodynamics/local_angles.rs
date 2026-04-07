//! Step 0: per-zone effective angle of attack and sideslip.

/// Compute the local effective angle of attack and sideslip for a zone.
///
/// Body angular rates create local velocity increments at each zone that shift
/// its effective angle of attack and sideslip beyond the whole-aircraft values.
/// Three additive correction layers are applied on top of the zone-frame
/// geometric alpha/beta (which already captures dihedral, sweep, etc. via
/// velocity projection into the zone's local coordinate frame):
///
/// **Layer 1. Roll-rate delta-alpha (asymmetric stall, snap rolls, spins)**
///
/// A zone at spanwise station `y` (metres, positive starboard) sees a
/// body-Z velocity increment `Δw = p · y` from roll rate `p` (rad/s).
/// **Local angle-of-attack change from roll = roll rate (rad/s) × spanwise
/// distance from roll axis (m) ÷ airspeed (m/s).**
///
/// ```text
/// delta_alpha_roll = p * y / V
/// ```
///
/// **Layer 2. Pitch-rate delta-alpha (pitch damping, tail effectiveness)**
///
/// **Layer 2. Pitch-rate Δα (pitch damping, tail effectiveness)**
///
/// A zone at longitudinal station `x` (metres, positive forward from CG) sees
/// a body-Z velocity increment from pitch rate `q` (rad/s). The cross product
/// `ω × r` gives z-component `p·y − q·x`, so the pitch contribution is
/// `−q·x`. For the aft tail (x < 0), this is positive when q > 0 - the tail
/// rotates INTO the airstream during a pull, increasing its AoA and generating
/// more restoring lift. This is the physical mechanism behind pitch damping
/// (Cm_q). For a forward canard (x > 0) the effect is reversed.
///
/// The formula below follows directly from body-frame kinematics (ω × r):
///
/// ```text
/// delta_alpha_pitch = -q * x / V
/// ```
///
/// **Layer 3. Yaw-rate delta-beta (yaw damping, Dutch roll)**
///
/// A zone at longitudinal station `x` sees a body-Y velocity increment from
/// yaw rate `r` (rad/s). The cross product `ω × r` gives y-component `r·x`
/// (for a zone on the aircraft centerline, z ≈ 0). For the vertical tail
/// (x < 0, aft), a right yaw (r > 0) produces a leftward lateral velocity at
/// the tail, reducing β_local. The resulting side force at the aft position
/// opposes the yaw, providing yaw damping.
/// Dutch roll is a coupled yaw-roll oscillation. Yaw rate shifts the sideslip
/// seen by aft surfaces, providing yaw damping:
///
/// ```text
/// delta_beta_yaw = r * x / V
/// ```
///
/// Note: spanwise wing zones (y ≠ 0, x ≈ 0) see near-zero sideslip change
/// from yaw rate because the yaw-induced lateral velocity is proportional to
/// the longitudinal station x, not the spanwise station y.
///
/// # Arguments
///
/// - `alpha`: zone-frame angle of attack (rad), from projecting body velocity
///   into the zone's local coordinate frame. For zones with identity rotation
///   this equals the whole-aircraft alpha.
/// - `beta`: zone-frame sideslip (rad), same projection.
/// - `p`, `q`, `r`: body-axis angular rates (rad/s).
/// - `x`: zone body-frame longitudinal station (m) from CG; positive forward.
/// - `y`: zone body-frame spanwise station (m) from CG; positive starboard.
/// - `v`: true airspeed (m/s); must be > 0.
///
/// # Returns
///
/// `(alpha_local, beta_local)`, both in radians.
#[allow(clippy::too_many_arguments)]
pub fn zone_local_angles(
    alpha: f64,
    beta: f64,
    p: f64,
    q: f64,
    r: f64,
    x: f64,
    y: f64,
    v: f64,
) -> (f64, f64) {
    let alpha_local = alpha + (p * y - q * x) / v;
    let beta_local = beta + (r * x) / v;
    (alpha_local, beta_local)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::coefficients::evaluate_zone_coefficients;
    use crate::aerodynamics::world_forces::zone_force_world;
    use crate::components::{AeroZone, ControlInputs};
    use crate::components::aero_coeff::AeroCoeff;
    use bevy_math::{DQuat, DVec3};

    fn neutral_controls() -> ControlInputs {
        ControlInputs { elevator: 0.0, aileron: 0.0, rudder: 0.0, throttle: 0.0 }
    }

    /// Layer 1: positive roll rate + positive spanwise station → increased local α.
    #[test]
    fn roll_rate_increases_alpha_at_positive_y() {
        let (alpha_l, beta_l) = zone_local_angles(0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0, 50.0);
        let expected = 0.1 + 4.0 / 50.0;
        assert!((alpha_l - expected).abs() < 1e-12, "Δα_roll should be p·y/V, got {alpha_l}");
        assert!((beta_l - 0.0).abs() < 1e-12, "roll rate should not affect β");
    }

    /// Layer 1: port wing (y < 0) gets reduced α under positive roll rate.
    #[test]
    fn roll_rate_decreases_alpha_at_negative_y() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 1.0, 0.0, 0.0, 0.0, -4.0, 50.0);
        let expected = 0.1 - 4.0 / 50.0;
        assert!((alpha_l - expected).abs() < 1e-12, "port tip should see reduced α under positive roll");
    }

    /// Layer 2: tail (x < 0) sees increased α during pull (q > 0) — pitch damping.
    #[test]
    fn pitch_rate_increases_alpha_at_tail() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 0.0, 1.0, 0.0, -4.0, 0.0, 50.0);
        let expected = 0.1 + 4.0 / 50.0;
        assert!((alpha_l - expected).abs() < 1e-12, "tail should see increased α during pull, got {alpha_l}");
    }

    /// Layer 2: nose (x > 0) sees reduced α during pull — canard effect.
    #[test]
    fn pitch_rate_decreases_alpha_at_nose() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 0.0, 1.0, 0.0, 4.0, 0.0, 50.0);
        let expected = 0.1 - 4.0 / 50.0;
        assert!((alpha_l - expected).abs() < 1e-12, "nose should see reduced α during pull, got {alpha_l}");
    }

    /// Layer 3: aft zone (x < 0) sees reduced sideslip during right yaw (r > 0).
    #[test]
    fn yaw_rate_shifts_beta_at_longitudinal_station() {
        let (alpha_l, beta_l) = zone_local_angles(0.0, 0.05, 0.0, 0.0, 1.0, -3.0, 0.0, 50.0);
        let expected_beta = 0.05 + (-3.0) / 50.0;
        assert!((beta_l - expected_beta).abs() < 1e-12, "Δβ_yaw should be r·x/V, got {beta_l}");
        assert!((alpha_l - 0.0).abs() < 1e-12, "yaw rate should not affect α");
    }

    /// Layer 3: purely spanwise zone (x = 0) sees no β change from yaw rate.
    #[test]
    fn yaw_rate_does_not_shift_beta_at_spanwise_zone() {
        let (_, beta_l) = zone_local_angles(0.0, 0.05, 0.0, 0.0, 1.0, 0.0, 3.0, 50.0);
        assert!((beta_l - 0.05).abs() < 1e-12, "purely spanwise zone should see no β change from yaw rate, got {beta_l}");
    }

    /// Zero body rates: local angles equal global angles.
    #[test]
    fn zero_rates_leave_angles_unchanged() {
        let (a, b) = zone_local_angles(0.2, 0.1, 0.0, 0.0, 0.0, -2.0, 3.0, 30.0);
        assert!((a - 0.2).abs() < 1e-12, "zero rates should not alter α");
        assert!((b - 0.1).abs() < 1e-12, "zero rates should not alter β");
    }

    /// All three rate layers combine additively.
    #[test]
    fn all_layers_combine_additively() {
        // p=1, q=1, r=1; x=2, y=3; V=10
        // α_local = 0.1 + (1·3 − 1·2)/10 = 0.2
        // β_local = 0.05 + 1·2/10 = 0.25
        let (a, b) = zone_local_angles(0.1, 0.05, 1.0, 1.0, 1.0, 2.0, 3.0, 10.0);
        assert!((a - 0.2).abs() < 1e-12, "combined α layers, got {a}");
        assert!((b - 0.25).abs() < 1e-12, "combined β layers, got {b}");
    }

    /// Dihedral effect via zone rotation: projecting body velocity into a zone
    /// frame that is rotated by the dihedral angle naturally gives more alpha
    /// on the starboard wing and less on the port wing at positive sideslip.
    ///
    /// This test verifies the geometric projection that replaces the old
    /// explicit dihedral_rad correction layer.
    #[test]
    fn zone_rotation_dihedral_gives_correct_alpha_asymmetry() {
        use bevy_math::{DQuat, DVec3};

        let gamma = 0.0698_f64; // 4 degrees, J3 Cub dihedral
        let alpha_body = 0.05_f64;
        let beta = 0.2_f64; // wind from the right

        // Body velocity unit vector.
        let vel_body = DVec3::new(
            alpha_body.cos() * beta.cos(),
            beta.sin(),
            alpha_body.sin() * beta.cos(),
        );

        // Starboard wing: tip UP means from_rotation_x(-gamma).
        let zone_q_stbd = DQuat::from_rotation_x(-gamma);
        let vel_stbd = zone_q_stbd.inverse() * vel_body;
        let alpha_stbd = f64::atan2(vel_stbd.z, vel_stbd.x);

        // Port wing: tip UP means from_rotation_x(+gamma).
        let zone_q_port = DQuat::from_rotation_x(gamma);
        let vel_port = zone_q_port.inverse() * vel_body;
        let alpha_port = f64::atan2(vel_port.z, vel_port.x);

        assert!(
            alpha_stbd > alpha_body,
            "starboard dihedral zone sees MORE alpha at positive beta: {alpha_stbd:.5} vs body {alpha_body:.5}"
        );
        assert!(
            alpha_port < alpha_body,
            "port dihedral zone sees LESS alpha at positive beta: {alpha_port:.5} vs body {alpha_body:.5}"
        );
        // The asymmetry should be close to the small-angle approximation Γ·β
        // (not exact because alpha is non-zero and the projection is 3D).
        let expected_delta = gamma * beta.sin();
        assert!(
            (alpha_stbd - alpha_body - expected_delta).abs() < 5e-4,
            "asymmetry should be near Γ·sin(β) ≈ {expected_delta:.5}, got {:.5}",
            alpha_stbd - alpha_body
        );
    }

    /// Emergent roll damping: symmetric zones at ±y produce differential lift
    /// under roll rate that opposes the roll — no explicit derivative needed.
    #[test]
    fn emergent_roll_damping_from_symmetric_zones() {
        let zone = AeroZone {
            cl: AeroCoeff::Table1D {
                breakpoints: vec![-0.5, 0.5],
                values: vec![-2.5, 2.5],
            },
            cd: AeroCoeff::Scalar(0.0),
            ..Default::default()
        };
        let ctrl = neutral_controls();
        let alpha = 0.1_f64;
        let v = 50.0_f64;
        let p = 1.0_f64;
        let re = 1e6;
        let qbar = 1500.0;

        let (al_r, bl_r) = zone_local_angles(alpha, 0.0, p, 0.0, 0.0, 0.0, 4.0, v);
        let cr = evaluate_zone_coefficients(&zone, &ctrl, al_r, bl_r, re, qbar, 1.0);

        let (al_l, bl_l) = zone_local_angles(alpha, 0.0, p, 0.0, 0.0, 0.0, -4.0, v);
        let cl_coeffs = evaluate_zone_coefficients(&zone, &ctrl, al_l, bl_l, re, qbar, 1.0);

        assert!(cr.cl > cl_coeffs.cl, "starboard tip should have higher CL: {:.3} vs {:.3}", cr.cl, cl_coeffs.cl);

        let b2w = DQuat::IDENTITY;
        let vel_r = DVec3::new(al_r.cos() * bl_r.cos(), bl_r.sin(), al_r.sin() * bl_r.cos());
        let vel_l = DVec3::new(al_l.cos() * bl_l.cos(), bl_l.sin(), al_l.sin() * bl_l.cos());
        let wf_r = zone_force_world(&cr, qbar, 16.0, 10.0, 1.6, al_r, vel_r, b2w);
        let wf_l = zone_force_world(&cl_coeffs, qbar, 16.0, 10.0, 1.6, al_l, vel_l, b2w);

        let arm_r = DVec3::new(0.0, 4.0, 0.0);
        let arm_l = DVec3::new(0.0, -4.0, 0.0);
        let net_roll = arm_r.cross(wf_r.force).x + arm_l.cross(wf_l.force).x;

        assert!(net_roll < 0.0, "emergent roll damping should oppose p>0, net = {net_roll:.2} N·m");
    }
}
