//! Step 2: turns the stability-frame forces and the pure torques into world coordinates.

use avian3d::math::{Quaternion, Scalar, Vector};

use super::coefficients::ZoneCoefficients;

/// Force and torque of one zone, in world space.
pub(crate) struct ZoneWorldForce {
    /// Aerodynamic force in world coordinates (N).
    pub force: Vector,
    /// Pure aerodynamic torque in world coordinates (N·m).
    ///
    /// This is the couple that does not depend on the position of the zone
    /// (for example, the pitching moment of an airfoil about its own
    /// aerodynamic center). It is *not* the torque from the moment arm. The
    /// caller calculates that torque separately, with
    /// `(zone_position − CG) × force`.
    pub torque: Vector,
}

/// Changes non-dimensional coefficients into a force and a torque in world
/// space.
///
/// # Path through the coordinate frames
///
/// Drag, lift, and side force use different frames. The function calculates
/// them separately, so that each one is physically correct.
///
/// **Drag** always opposes the full 3-D velocity vector. The function takes
/// the unit velocity vector in the zone frame and inverts it. The result is
/// the drag direction, for all sideslip angles. After the rotation by
/// `zone_to_world`, the drag opposes the velocity in world space, for all
/// dihedral and sweep angles:
///
/// ```text
/// F_drag_zone = −CD * qbar * S * vel_zone_unit
/// ```
///
/// **Lift and side force** use the stability frame. The stability frame is
/// the zone frame turned by −alpha_zone about zone Y. CY is the lateral force
/// of the zone (stability +Y is the same as zone +Y). Thus, in the stability
/// frame, CY turns with the aircraft and the zone. The wind frame also turns
/// by beta. In the wind frame, CY stays in one world direction when the
/// aircraft yaws, and this is not correct.
///
/// ```text
/// F_lift_side_stab = ( 0,  CY * qbar * S,  −CL * qbar * S )
/// F_lift_side_zone = R_y(−alpha_zone) * F_lift_side_stab
/// ```
///
/// Then the function turns both forces into world space with
/// `zone_to_world`. This rotation is the aircraft body rotation, followed by
/// the rotation of the zone:
///
/// ```text
/// zone_to_world = body_to_world * zone_q
/// F_world       = zone_to_world * (F_drag_zone + F_lift_side_zone)
/// ```
///
/// For a zone with no rotation (zone_q = identity), the zone frame and the
/// body frame are the same.
///
/// # Pure aerodynamic torques
///
/// The moment coefficients (CM, Croll, Cn) are pure couples in the zone frame.
/// **Roll and yaw use the aircraft wing span `b` as the reference length
/// (always the full span, not the span of the zone). Pitch uses the chord of
/// the zone, `c_bar_zone`:**
///
/// ```text
/// tau_zone = ( Croll * qbar * S_zone * b,
///              CM    * qbar * S_zone * c_bar_zone,
///              Cn    * qbar * S_zone * b )
/// ```
#[allow(clippy::too_many_arguments)]
pub(crate) fn zone_force_world(
    coeffs: &ZoneCoefficients,
    qbar: Scalar,
    zone_area: Scalar,
    span_ref: Scalar,
    zone_chord: Scalar,
    alpha: Scalar,
    vel_zone_unit: Vector,
    zone_to_world: Quaternion,
) -> ZoneWorldForce {
    // Drag: opposes the full 3D velocity vector in zone frame.
    let drag_zone = vel_zone_unit * (-coeffs.cd * qbar * zone_area);

    // Lift and side force: stability frame (alpha rotation about zone-Y only).
    // CY = stability +Y = zone +Y, so this keeps side force rotating with the
    // aircraft and zone instead of being locked to a world direction.
    let stab_to_zone = Quaternion::from_rotation_y(-alpha);
    let lift_side_stab = Vector::new(
        0.0,
        coeffs.cy * qbar * zone_area,
        -coeffs.cl * qbar * zone_area,
    );
    let lift_side_zone = stab_to_zone * lift_side_stab;

    let force = zone_to_world * (drag_zone + lift_side_zone);

    let torque_zone = Vector::new(
        coeffs.croll * qbar * zone_area * span_ref,
        coeffs.cm * qbar * zone_area * zone_chord,
        coeffs.cn * qbar * zone_area * span_ref,
    );
    let torque = zone_to_world * torque_zone;

    ZoneWorldForce { force, torque }
}

#[cfg(test)]
mod tests {
    use super::super::coefficients::ZoneCoefficients;
    use super::*;

    fn unit_coeffs(cl: Scalar, cd: Scalar, cy: Scalar, cm: Scalar) -> ZoneCoefficients {
        ZoneCoefficients {
            cl,
            cd,
            cy,
            cm,
            croll: 0.0,
            cn: 0.0,
        }
    }

    /// Level flight at alpha=0: lift (−Z_stab) rotates to +Y world.
    #[test]
    fn lift_opposes_gravity_at_zero_alpha() {
        let coeffs = unit_coeffs(1.0, 0.0, 0.0, 0.0);
        let wf = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            0.0,
            Vector::X,
            Quaternion::from_rotation_x(avian3d::math::FRAC_PI_2),
        );
        assert!(
            wf.force.y > 0.0,
            "lift should point up (+Y world), got {}",
            wf.force.y
        );
    }

    /// Drag (−velocity direction) should oppose forward motion (−X world).
    #[test]
    fn drag_opposes_forward_motion() {
        let coeffs = unit_coeffs(0.0, 1.0, 0.0, 0.0);
        // vel_body_unit = +X (flying forward at alpha=0, beta=0)
        let wf = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            0.0,
            Vector::X,
            Quaternion::from_rotation_x(avian3d::math::FRAC_PI_2),
        );
        assert!(
            wf.force.x < 0.0,
            "drag should oppose forward motion, got {}",
            wf.force.x
        );
    }

    /// At 45 degrees of yaw, drag should still oppose the original velocity
    /// direction (world +X), not the aircraft nose direction.
    #[test]
    fn drag_opposes_velocity_not_nose_at_sideslip() {
        let coeffs = unit_coeffs(0.0, 1.0, 0.0, 0.0);
        // Aircraft yawed 45 degrees right. Velocity is still world +X.
        // In body frame, the velocity has a sideways (beta) component.
        let frac_pi_4 = avian3d::math::FRAC_PI_2 / 2.0;
        let beta: Scalar = -frac_pi_4;
        let vel_body_unit = Vector::new(beta.cos(), beta.sin(), 0.0);
        // body_to_world: initial spawn rotation + 45 deg right yaw
        let body_to_world = Quaternion::from_rotation_y(-frac_pi_4)
            * Quaternion::from_rotation_x(avian3d::math::FRAC_PI_2);
        let wf = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            0.0,
            vel_body_unit,
            body_to_world,
        );
        // Drag must oppose velocity (world +X), so x-component is negative.
        assert!(
            wf.force.x < 0.0,
            "drag x should be negative, got {}",
            wf.force.x
        );
        // Drag must not have a significant world-Z component (not perpendicular to velocity).
        assert!(
            wf.force.z.abs() < wf.force.x.abs() * 0.01,
            "drag should not push sideways, z={}",
            wf.force.z
        );
    }

    /// After yaw, CY (rudder side force) must rotate with the aircraft,
    /// not stay fixed in the initial world direction.
    #[test]
    fn side_force_rotates_with_aircraft() {
        let cy = 1.0;
        let coeffs = unit_coeffs(0.0, 0.0, cy, 0.0);
        let frac_pi_2 = avian3d::math::FRAC_PI_2;

        // Level aircraft (no yaw).
        let body_to_world_level = Quaternion::from_rotation_x(frac_pi_2);
        let wf_level = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            0.0,
            Vector::X,
            body_to_world_level,
        );

        // Aircraft yawed 90 degrees right.
        let body_to_world_yawed =
            Quaternion::from_rotation_y(-frac_pi_2) * Quaternion::from_rotation_x(frac_pi_2);
        // At 90 deg yaw, velocity (world +X) is now body -Y, so beta = -90 deg.
        let beta_90: Scalar = -frac_pi_2;
        let vel_body_unit_yawed = Vector::new(beta_90.cos(), beta_90.sin(), 0.0);
        let wf_yawed = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            0.0,
            vel_body_unit_yawed,
            body_to_world_yawed,
        );

        // The force direction must differ between level and yawed (rotates with aircraft).
        let dir_level = wf_level.force.normalize();
        let dir_yawed = wf_yawed.force.normalize();
        let dot = dir_level.dot(dir_yawed);
        assert!(
            dot.abs() < 0.1,
            "CY direction should differ by ~90 deg after 90 deg yaw, dot={dot}"
        );
    }

    /// Force magnitude scales linearly with dynamic pressure.
    #[test]
    fn force_scales_with_dynamic_pressure() {
        let coeffs = unit_coeffs(0.5, 0.0, 0.0, 0.0);
        let wf1 = zone_force_world(
            &coeffs,
            500.0,
            16.0,
            10.0,
            1.6,
            0.0,
            Vector::X,
            Quaternion::IDENTITY,
        );
        let wf2 = zone_force_world(
            &coeffs,
            2000.0,
            16.0,
            10.0,
            1.6,
            0.0,
            Vector::X,
            Quaternion::IDENTITY,
        );
        let ratio = wf2.force.length() / wf1.force.length();
        assert!(
            (ratio - 4.0).abs() < 1e-4,
            "force should scale 4:1 with q̄, ratio = {ratio}"
        );
    }

    /// Pitching moment scales with chord, not span.
    #[test]
    fn pitching_moment_uses_chord_not_span() {
        let coeffs = unit_coeffs(0.0, 0.0, 0.0, 1.0);
        let wf_a = zone_force_world(
            &coeffs,
            1000.0,
            1.0,
            10.0,
            2.0,
            0.0,
            Vector::X,
            Quaternion::IDENTITY,
        );
        let wf_b = zone_force_world(
            &coeffs,
            1000.0,
            1.0,
            10.0,
            4.0,
            0.0,
            Vector::X,
            Quaternion::IDENTITY,
        );
        let ratio = wf_b.torque.y / wf_a.torque.y;
        assert!(
            (ratio - 2.0).abs() < 1e-4,
            "CM should scale with chord, ratio = {ratio}"
        );
    }
}
