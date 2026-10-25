//! Step 0: the local angle of attack and sideslip angle of each zone.

/// Calculates the local angle of attack and sideslip angle of a zone.
///
/// The angular rates of the body add a local velocity at each zone. This
/// velocity changes the angle of attack and the sideslip angle of the zone.
/// The local angles are then different from the values of the full aircraft.
///
/// The inputs `alpha` and `beta` are the geometric angles in the zone frame.
/// They already include dihedral and sweep, because the caller projects the
/// velocity into the local frame of the zone. The function adds three
/// correction layers to these angles:
///
/// **Layer 1. Change of alpha from the roll rate (asymmetric stall, snap rolls, spins)**
///
/// A zone at the spanwise station `y` (meters, positive to starboard) gets a
/// body-Z velocity increment `delta_w = p * y` from the roll rate `p` (rad/s).
/// **Local change of the angle of attack from roll = roll rate (rad/s) ×
/// spanwise distance from the roll axis (m) ÷ airspeed (m/s):**
///
/// ```text
/// delta_alpha_roll = p * y / V
/// ```
///
/// **Layer 2. Change of alpha from the pitch rate (pitch damping, tail effectiveness)**
///
/// A zone at the longitudinal station `x` (meters, positive forward of the
/// CG) gets a body-Z velocity increment from the pitch rate `q` (rad/s). The
/// cross product `omega × r` has the z-component `p*y − q*x`. Thus, the pitch
/// contribution is `−q*x`. For the aft tail (x < 0), this value is positive
/// when q > 0. During a pull-up, the tail turns INTO the airstream. Its AoA
/// increases, and it makes more restoring lift. This is the physical cause of
/// pitch damping (Cm_q). For a canard at the front (x > 0), the effect is the
/// opposite.
///
/// The formula comes directly from the body-frame kinematics (omega × r):
///
/// ```text
/// delta_alpha_pitch = -q * x / V
/// ```
///
/// **Layer 3. Change of beta from the yaw rate (yaw damping, Dutch roll)**
///
/// A zone at the longitudinal station `x` gets a body-Y velocity increment
/// from the yaw rate `r` (rad/s). The cross product `omega × r` has the
/// y-component `r*x` (for a zone on the aircraft centerline, z ≈ 0). At the
/// vertical tail (x < 0, aft), a right yaw (r > 0) gives a lateral velocity
/// to the left. This decreases beta_local. The side force at the aft
/// position then opposes the yaw. This is yaw damping. The Dutch roll is a
/// coupled oscillation in yaw and roll. The yaw damping from the aft surfaces
/// is one part of it:
///
/// ```text
/// delta_beta_yaw = r * x / V
/// ```
///
/// Note: for spanwise wing zones (y ≠ 0, x ≈ 0), the yaw rate changes the
/// sideslip angle almost zero. The lateral velocity from the yaw is
/// proportional to the longitudinal station x, not to the spanwise station y.
///
/// # Arguments
///
/// - `alpha`: angle of attack in the zone frame (rad). It comes from the
///   projection of the body velocity into the local frame of the zone. For a
///   zone with no rotation, it is equal to the alpha of the full aircraft.
/// - `beta`: sideslip angle in the zone frame (rad), from the same projection.
/// - `p`, `q`, `r`: angular rates about the body axes (rad/s).
/// - `x`: longitudinal station of the zone from the CG, in the body frame (m). Positive forward.
/// - `y`: spanwise station of the zone from the CG, in the body frame (m). Positive to starboard.
/// - `v`: true airspeed (m/s). It must be > 0.
///
/// # Returns
///
/// `(alpha_local, beta_local)`, both in radians.
#[allow(clippy::too_many_arguments)]
pub fn zone_local_angles(
    alpha: f32,
    beta: f32,
    p: f32,
    q: f32,
    r: f32,
    x: f32,
    y: f32,
    v: f32,
) -> (f32, f32) {
    let alpha_local = alpha + (p * y - q * x) / v;
    let beta_local = beta + (r * x) / v;
    (alpha_local, beta_local)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::coefficients::evaluate_zone_coefficients;
    use crate::aerodynamics::world_forces::zone_force_world;
    use crate::components::aero_coeff::AeroCoeff;
    use crate::components::{AeroZone, ControlInputs};
    use bevy_math::{Quat, Vec3};

    fn neutral_controls() -> ControlInputs {
        ControlInputs {
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
            throttle: 0.0,
        }
    }

    const EPS: f32 = 1e-5;

    /// Layer 1: positive roll rate + positive spanwise station → increased local α.
    #[test]
    fn roll_rate_increases_alpha_at_positive_y() {
        let (alpha_l, beta_l) = zone_local_angles(0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0, 50.0);
        let expected = 0.1 + 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < EPS,
            "Δα_roll should be p·y/V, got {alpha_l}"
        );
        assert!((beta_l - 0.0).abs() < EPS, "roll rate should not affect β");
    }

    /// Layer 1: port wing (y < 0) gets reduced α under positive roll rate.
    #[test]
    fn roll_rate_decreases_alpha_at_negative_y() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 1.0, 0.0, 0.0, 0.0, -4.0, 50.0);
        let expected = 0.1 - 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < EPS,
            "port tip should see reduced α under positive roll"
        );
    }

    /// Layer 2: tail (x < 0) sees increased α during pull (q > 0) — pitch damping.
    #[test]
    fn pitch_rate_increases_alpha_at_tail() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 0.0, 1.0, 0.0, -4.0, 0.0, 50.0);
        let expected = 0.1 + 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < EPS,
            "tail should see increased α during pull, got {alpha_l}"
        );
    }

    /// Layer 2: nose (x > 0) sees reduced α during pull — canard effect.
    #[test]
    fn pitch_rate_decreases_alpha_at_nose() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 0.0, 1.0, 0.0, 4.0, 0.0, 50.0);
        let expected = 0.1 - 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < EPS,
            "nose should see reduced α during pull, got {alpha_l}"
        );
    }

    /// Layer 3: aft zone (x < 0) sees reduced sideslip during right yaw (r > 0).
    #[test]
    fn yaw_rate_shifts_beta_at_longitudinal_station() {
        let (alpha_l, beta_l) = zone_local_angles(0.0, 0.05, 0.0, 0.0, 1.0, -3.0, 0.0, 50.0);
        let expected_beta = 0.05 + (-3.0) / 50.0;
        assert!(
            (beta_l - expected_beta).abs() < EPS,
            "Δβ_yaw should be r·x/V, got {beta_l}"
        );
        assert!((alpha_l - 0.0).abs() < EPS, "yaw rate should not affect α");
    }

    /// Layer 3: purely spanwise zone (x = 0) sees no β change from yaw rate.
    #[test]
    fn yaw_rate_does_not_shift_beta_at_spanwise_zone() {
        let (_, beta_l) = zone_local_angles(0.0, 0.05, 0.0, 0.0, 1.0, 0.0, 3.0, 50.0);
        assert!(
            (beta_l - 0.05).abs() < EPS,
            "purely spanwise zone should see no β change from yaw rate, got {beta_l}"
        );
    }

    /// Zero body rates: local angles equal global angles.
    #[test]
    fn zero_rates_leave_angles_unchanged() {
        let (a, b) = zone_local_angles(0.2, 0.1, 0.0, 0.0, 0.0, -2.0, 3.0, 30.0);
        assert!((a - 0.2).abs() < EPS, "zero rates should not alter α");
        assert!((b - 0.1).abs() < EPS, "zero rates should not alter β");
    }

    /// All three rate layers combine additively.
    #[test]
    fn all_layers_combine_additively() {
        // p = 1, q = 1, r = 1; x = 2, y = 3; V = 10
        // alpha_local = 0.1 + (1*3 - 1*2)/10 = 0.2
        // beta_local  = 0.05 + 1*2/10 = 0.25
        let (a, b) = zone_local_angles(0.1, 0.05, 1.0, 1.0, 1.0, 2.0, 3.0, 10.0);
        assert!((a - 0.2).abs() < EPS, "combined α layers, got {a}");
        assert!((b - 0.25).abs() < EPS, "combined β layers, got {b}");
    }

    /// Dihedral effect via zone rotation: projecting body velocity into a zone
    /// frame that is rotated by the dihedral angle naturally gives more alpha
    /// on the starboard wing and less on the port wing at positive sideslip.
    ///
    /// This test verifies the geometric projection that replaces the old
    /// explicit dihedral_rad correction layer.
    #[test]
    fn zone_rotation_dihedral_gives_correct_alpha_asymmetry() {
        let gamma: f32 = 0.0698; // 4 degrees, J3 Cub dihedral
        let alpha_body: f32 = 0.05;
        let beta: f32 = 0.2; // wind from the right

        // Unit velocity vector in the body frame.
        let vel_body = Vec3::new(
            alpha_body.cos() * beta.cos(),
            beta.sin(),
            alpha_body.sin() * beta.cos(),
        );

        // Starboard wing: a tip-up rotation is from_rotation_x(-gamma).
        let zone_q_stbd = Quat::from_rotation_x(-gamma);
        let vel_stbd = zone_q_stbd.inverse() * vel_body;
        let alpha_stbd = vel_stbd.z.atan2(vel_stbd.x);

        // Port wing: a tip-up rotation is from_rotation_x(+gamma).
        let zone_q_port = Quat::from_rotation_x(gamma);
        let vel_port = zone_q_port.inverse() * vel_body;
        let alpha_port = vel_port.z.atan2(vel_port.x);

        assert!(
            alpha_stbd > alpha_body,
            "starboard dihedral zone sees MORE alpha at positive beta: {alpha_stbd:.5} vs body {alpha_body:.5}"
        );
        assert!(
            alpha_port < alpha_body,
            "port dihedral zone sees LESS alpha at positive beta: {alpha_port:.5} vs body {alpha_body:.5}"
        );
        // The asymmetry must be near the small-angle approximation, gamma * beta.
        // It is not exact, because alpha is not zero and the projection is in 3-D.
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
        let alpha: f32 = 0.1;
        let v: f32 = 50.0;
        let p: f32 = 1.0;
        let re = 1e6;
        let qbar = 1500.0;

        let (al_r, bl_r) = zone_local_angles(alpha, 0.0, p, 0.0, 0.0, 0.0, 4.0, v);
        let cr = evaluate_zone_coefficients(&zone, &ctrl, al_r, bl_r, re, qbar, 1.0);

        let (al_l, bl_l) = zone_local_angles(alpha, 0.0, p, 0.0, 0.0, 0.0, -4.0, v);
        let cl_coeffs = evaluate_zone_coefficients(&zone, &ctrl, al_l, bl_l, re, qbar, 1.0);

        assert!(
            cr.cl > cl_coeffs.cl,
            "starboard tip should have higher CL: {:.3} vs {:.3}",
            cr.cl,
            cl_coeffs.cl
        );

        let b2w = Quat::IDENTITY;
        let vel_r = Vec3::new(al_r.cos() * bl_r.cos(), bl_r.sin(), al_r.sin() * bl_r.cos());
        let vel_l = Vec3::new(al_l.cos() * bl_l.cos(), bl_l.sin(), al_l.sin() * bl_l.cos());
        let wf_r = zone_force_world(&cr, qbar, 16.0, 10.0, 1.6, al_r, vel_r, b2w);
        let wf_l = zone_force_world(&cl_coeffs, qbar, 16.0, 10.0, 1.6, al_l, vel_l, b2w);

        let arm_r = Vec3::new(0.0, 4.0, 0.0);
        let arm_l = Vec3::new(0.0, -4.0, 0.0);
        let net_roll = arm_r.cross(wf_r.force).x + arm_l.cross(wf_l.force).x;

        assert!(
            net_roll < 0.0,
            "emergent roll damping should oppose p>0, net = {net_roll:.2} N·m"
        );
    }
}
