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
/// Drag, lift, and side force live in different frames and are handled
/// separately to keep each physically correct.
///
/// **Drag** always opposes the full 3D velocity vector. Passing the unit
/// velocity vector in zone frame and negating gives the drag direction
/// regardless of sideslip. After rotating by `zone_to_world`, drag correctly
/// opposes the velocity in world space regardless of dihedral or sweep:
///
/// ```text
/// F_drag_zone = −CD · q̄ · S · vel_zone_unit
/// ```
///
/// **Lift and side force** use the stability frame: the zone frame rotated
/// by −α_zone about zone-Y. CY is the zone-lateral force (stability +Y equals
/// zone +Y), so using the stability frame keeps CY rotating with the aircraft
/// and zone. Using the wind frame (which also rotates by beta) would lock CY
/// to a fixed world direction as the aircraft yaws.
///
/// ```text
/// F_lift_side_stab = ( 0,  CY · q̄ · S,  −CL · q̄ · S )
/// F_lift_side_zone  = R_y(−α_zone) · F_lift_side_stab
/// ```
///
/// Both are then rotated to world space using `zone_to_world`, which is the
/// composition of the aircraft body rotation and the zone's own rotation:
///
/// ```text
/// zone_to_world = body_to_world · zone_q
/// F_world = zone_to_world · (F_drag_zone + F_lift_side_zone)
/// ```
///
/// For zones with identity rotation (zone_q = identity), this reduces exactly
/// to the previous body-frame formulation.
///
/// # Pure aerodynamic torques
///
/// Moment coefficients (CM, Croll, Cn) represent pure couples expressed in
/// zone frame. **Roll and yaw use wingspan `b` as the reference length;
/// pitch uses chord `c̄`:**
///
/// ```text
/// τ_zone = ( Croll · q̄ · S · b,
///            CM    · q̄ · S · c̄,
///            Cn    · q̄ · S · b )
/// ```
pub(crate) fn zone_force_world(
    coeffs: &ZoneCoefficients,
    qbar: f64,
    s: f64,
    b: f64,
    c: f64,
    alpha: f64,
    vel_zone_unit: DVec3,
    zone_to_world: DQuat,
) -> ZoneWorldForce {
    // Drag: opposes the full 3D velocity vector in zone frame.
    let drag_zone = vel_zone_unit * (-coeffs.cd * qbar * s);

    // Lift and side force: stability frame (alpha rotation about zone-Y only).
    // CY = stability +Y = zone +Y, so this keeps side force rotating with the
    // aircraft and zone instead of being locked to a world direction.
    let stab_to_zone = DQuat::from_rotation_y(-alpha);
    let lift_side_stab = DVec3::new(0.0, coeffs.cy * qbar * s, -coeffs.cl * qbar * s);
    let lift_side_zone = stab_to_zone * lift_side_stab;

    let force = zone_to_world * (drag_zone + lift_side_zone);

    let torque_zone = DVec3::new(
        coeffs.croll * qbar * s * b,
        coeffs.cm    * qbar * s * c,
        coeffs.cn    * qbar * s * b,
    );
    let torque = zone_to_world * torque_zone;

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
            0.0, DVec3::X,
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        assert!(wf.force.y > 0.0, "lift should point up (+Y world), got {}", wf.force.y);
    }

    /// Drag (−velocity direction) should oppose forward motion (−X world).
    #[test]
    fn drag_opposes_forward_motion() {
        let coeffs = unit_coeffs(0.0, 1.0, 0.0, 0.0);
        // vel_body_unit = +X (flying forward at α=0, β=0)
        let wf = zone_force_world(
            &coeffs, 1000.0, 16.0, 10.0, 1.6,
            0.0, DVec3::X,
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        assert!(wf.force.x < 0.0, "drag should oppose forward motion, got {}", wf.force.x);
    }

    /// At 45 degrees of yaw, drag should still oppose the original velocity
    /// direction (world +X), not the aircraft nose direction.
    #[test]
    fn drag_opposes_velocity_not_nose_at_sideslip() {
        let coeffs = unit_coeffs(0.0, 1.0, 0.0, 0.0);
        // Aircraft yawed 45 degrees right. Velocity is still world +X.
        // In body frame, the velocity has a sideways (beta) component.
        let beta = -std::f64::consts::FRAC_PI_4; // 45 degrees of sideslip
        let vel_body_unit = DVec3::new(beta.cos(), beta.sin(), 0.0);
        // body_to_world: initial spawn rotation + 45 deg right yaw
        let body_to_world = DQuat::from_rotation_y(-std::f64::consts::FRAC_PI_4)
            * DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let wf = zone_force_world(&coeffs, 1000.0, 16.0, 10.0, 1.6, 0.0, vel_body_unit, body_to_world);
        // Drag must oppose velocity (world +X), so x-component is negative.
        assert!(wf.force.x < 0.0, "drag x should be negative, got {}", wf.force.x);
        // Drag must not have a significant world-Z component (not perpendicular to velocity).
        assert!(wf.force.z.abs() < wf.force.x.abs() * 0.01, "drag should not push sideways, z={}", wf.force.z);
    }

    /// After yaw, CY (rudder side force) must rotate with the aircraft,
    /// not stay fixed in the initial world direction.
    #[test]
    fn side_force_rotates_with_aircraft() {
        let cy = 1.0;
        let coeffs = unit_coeffs(0.0, 0.0, cy, 0.0);

        // Level aircraft (no yaw).
        let body_to_world_level = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let wf_level = zone_force_world(
            &coeffs, 1000.0, 16.0, 10.0, 1.6,
            0.0, DVec3::X,
            body_to_world_level,
        );

        // Aircraft yawed 90 degrees right.
        let body_to_world_yawed = DQuat::from_rotation_y(-std::f64::consts::FRAC_PI_2)
            * DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        // At 90 deg yaw, velocity (world +X) is now body -Y, so beta = -90 deg.
        let beta_90 = -std::f64::consts::FRAC_PI_2;
        let vel_body_unit_yawed = DVec3::new(beta_90.cos(), beta_90.sin(), 0.0);
        let wf_yawed = zone_force_world(
            &coeffs, 1000.0, 16.0, 10.0, 1.6,
            0.0, vel_body_unit_yawed,
            body_to_world_yawed,
        );

        // The force direction must differ between level and yawed (rotates with aircraft).
        let dir_level = wf_level.force.normalize();
        let dir_yawed = wf_yawed.force.normalize();
        let dot = dir_level.dot(dir_yawed);
        assert!(dot.abs() < 0.1, "CY direction should differ by ~90 deg after 90 deg yaw, dot={dot}");
    }

    /// Force magnitude scales linearly with dynamic pressure.
    #[test]
    fn force_scales_with_dynamic_pressure() {
        let coeffs = unit_coeffs(0.5, 0.0, 0.0, 0.0);
        let wf1 = zone_force_world(&coeffs, 500.0,  16.0, 10.0, 1.6, 0.0, DVec3::X, DQuat::IDENTITY);
        let wf2 = zone_force_world(&coeffs, 2000.0, 16.0, 10.0, 1.6, 0.0, DVec3::X, DQuat::IDENTITY);
        let ratio = wf2.force.length() / wf1.force.length();
        assert!((ratio - 4.0).abs() < 1e-10, "force should scale 4:1 with q̄, ratio = {ratio}");
    }

    /// Pitching moment scales with chord, not span.
    #[test]
    fn pitching_moment_uses_chord_not_span() {
        let coeffs = unit_coeffs(0.0, 0.0, 0.0, 1.0);
        let wf_a = zone_force_world(&coeffs, 1000.0, 1.0, 10.0, 2.0, 0.0, DVec3::X, DQuat::IDENTITY);
        let wf_b = zone_force_world(&coeffs, 1000.0, 1.0, 10.0, 4.0, 0.0, DVec3::X, DQuat::IDENTITY);
        let ratio = wf_b.torque.y / wf_a.torque.y;
        assert!((ratio - 2.0).abs() < 1e-10, "CM should scale with chord, ratio = {ratio}");
    }
}
