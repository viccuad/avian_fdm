//! Step 2: stability-frame forces and pure torques to world coordinates.

use bevy_math::{DQuat, DVec3};

use super::coefficients::ZoneCoefficients;

/// World-space force and torque produced by a single zone.
pub(crate) struct ZoneWorldForce {
    /// Aerodynamic force in world coordinates (N).
    pub force: DVec3,
    /// Pure aerodynamic torque in world coordinates (N·m).
    ///
    /// This is the couple that exists independently of the zone's position
    /// (e.g. an airfoil's pitching moment about its own aerodynamic centre).
    /// It is *not* the moment-arm torque, that is computed separately by the
    /// caller using `(zone_position − CG) × force`.
    pub torque: DVec3,
}

/// Convert non-dimensional coefficients into a world-space force and torque.
///
/// # Coordinate frame journey
///
/// Aerodynamic coefficients are defined in the **stability frame**, which is
/// the body frame rotated by −α about body-Y so that its X axis aligns with
/// the velocity vector.  Forces are constructed there.
/// **Each force component = aerodynamic coefficient × dynamic pressure × wing
/// area. Drag opposes motion (−X), side force is rightward (+Y), lift is
/// perpendicular to velocity upward (−Z in stability frame):**
///
/// ```text
/// F_stability = ( −CD · q̄ · S,     drag opposes motion (−X_s)
///                  CY · q̄ · S,     side force (Y_s = body Y, starboard)
///                 −CL · q̄ · S )    lift perpendicular to velocity (−Z_s)
/// ```
///
/// **Undo the angle-of-attack rotation (stability to body), then apply the
/// aircraft's orientation quaternion (body to world):**
///
/// ```text
/// F_body  = R_y(−α) · F_stability
/// F_world = q_root  · F_body
/// ```
///
/// # Pure aerodynamic torques
///
/// Moment coefficients (CM, Croll, Cn) represent pure couples expressed in
/// body frame.  **Roll and yaw use wingspan `b` as the reference length;
/// pitch uses chord `c̄`:**
///
/// ```text
/// τ_body = ( Croll · q̄ · S · b,
///            CM    · q̄ · S · c̄,
///            Cn    · q̄ · S · b )
/// ```
pub(crate) fn zone_force_world(
    coeffs: &ZoneCoefficients,
    qbar: f64,
    s: f64,
    b: f64,
    c: f64,
    stab_to_body: DQuat,
    body_to_world: DQuat,
) -> ZoneWorldForce {
    let force_stab = DVec3::new(
        -coeffs.cd * qbar * s,
         coeffs.cy * qbar * s,
        -coeffs.cl * qbar * s,
    );
    let force = body_to_world * (stab_to_body * force_stab);

    let torque_body = DVec3::new(
        coeffs.croll * qbar * s * b,
        coeffs.cm    * qbar * s * c,
        coeffs.cn    * qbar * s * b,
    );
    let torque = body_to_world * torque_body;

    ZoneWorldForce { force, torque }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::coefficients::ZoneCoefficients;

    fn unit_coeffs(cl: f64, cd: f64, cy: f64, cm: f64) -> ZoneCoefficients {
        ZoneCoefficients { cl, cd, cy, cm, croll: 0.0, cn: 0.0 }
    }

    /// Level flight at α=0: lift (−Z_stab) rotates to +Y world.
    #[test]
    fn lift_opposes_gravity_at_zero_alpha() {
        let coeffs = unit_coeffs(1.0, 0.0, 0.0, 0.0);
        let wf = zone_force_world(
            &coeffs, 1000.0, 16.0, 10.0, 1.6,
            DQuat::IDENTITY,
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        assert!(wf.force.y > 0.0, "lift should point up (+Y world), got {}", wf.force.y);
    }

    /// Drag (−X_stab) should oppose forward motion (−X world).
    #[test]
    fn drag_opposes_forward_motion() {
        let coeffs = unit_coeffs(0.0, 1.0, 0.0, 0.0);
        let wf = zone_force_world(
            &coeffs, 1000.0, 16.0, 10.0, 1.6,
            DQuat::IDENTITY,
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        assert!(wf.force.x < 0.0, "drag should oppose forward motion, got {}", wf.force.x);
    }

    /// Force magnitude scales linearly with dynamic pressure.
    #[test]
    fn force_scales_with_dynamic_pressure() {
        let coeffs = unit_coeffs(0.5, 0.0, 0.0, 0.0);
        let wf1 = zone_force_world(&coeffs, 500.0,  16.0, 10.0, 1.6, DQuat::IDENTITY, DQuat::IDENTITY);
        let wf2 = zone_force_world(&coeffs, 2000.0, 16.0, 10.0, 1.6, DQuat::IDENTITY, DQuat::IDENTITY);
        let ratio = wf2.force.length() / wf1.force.length();
        assert!((ratio - 4.0).abs() < 1e-10, "force should scale 4:1 with q̄, ratio = {ratio}");
    }

    /// Pitching moment scales with chord, not span.
    #[test]
    fn pitching_moment_uses_chord_not_span() {
        let coeffs = unit_coeffs(0.0, 0.0, 0.0, 1.0);
        let wf_a = zone_force_world(&coeffs, 1000.0, 1.0, 10.0, 2.0, DQuat::IDENTITY, DQuat::IDENTITY);
        let wf_b = zone_force_world(&coeffs, 1000.0, 1.0, 10.0, 4.0, DQuat::IDENTITY, DQuat::IDENTITY);
        let ratio = wf_b.torque.y / wf_a.torque.y;
        assert!((ratio - 2.0).abs() < 1e-10, "CM should scale with chord, ratio = {ratio}");
    }
}
