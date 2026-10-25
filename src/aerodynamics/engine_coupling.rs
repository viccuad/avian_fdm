//! Adds the engine-zone forces to the totals of the aircraft root entity.
//!
//! In each physics step, `compute_engine_zone_forces` (in `propulsion/`)
//! calculates the engine thrust. It writes the thrust into a
//! [`crate::components::ZoneForce`] on each engine child entity. This module
//! reads these values. It adds them, and the torque from their moment arm, to
//! the force and torque totals of the root entity.

use bevy_math::Vec3;

use crate::components::ZoneForce;

/// Adds the thrust of one engine zone to the root force and torque.
///
/// `arm_world` goes from the CG of the aircraft to the engine, in world
/// orientation. If an engine is not on the centerline and the thrust is
/// asymmetric, the engine makes a yawing moment.
pub(super) fn accumulate_engine_force(
    zf: &ZoneForce,
    arm_world: Vec3,
    total_force: &mut Vec3,
    total_torque: &mut Vec3,
) {
    if zf.force != Vec3::ZERO {
        *total_force += zf.force;
        *total_torque += arm_world.cross(zf.force);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn thrust(force: Vec3) -> ZoneForce {
        ZoneForce {
            force,
            world_point: Vec3::ZERO,
        }
    }

    /// On-centre engine produces pure thrust, no torque.
    #[test]
    fn engine_at_cg_no_moment() {
        let zf = thrust(Vec3::new(500.0, 0.0, 0.0));
        let (mut f, mut t) = (Vec3::ZERO, Vec3::ZERO);
        accumulate_engine_force(&zf, Vec3::ZERO, &mut f, &mut t);
        assert!((f - Vec3::new(500.0, 0.0, 0.0)).length() < 1e-5);
        assert!(t.length() < 1e-5, "on-axis engine must not produce torque");
    }

    /// Starboard off-centre engine produces nose-left yaw torque.
    #[test]
    fn engine_offset_right_produces_yaw_torque() {
        let zf = thrust(Vec3::new(500.0, 0.0, 0.0));
        let (mut f, mut t) = (Vec3::ZERO, Vec3::ZERO);
        accumulate_engine_force(&zf, Vec3::new(0.0, 2.0, 0.0), &mut f, &mut t);
        // arm = (0, 2, 0), thrust = (500, 0, 0), torque = (0, 0, -1000)
        assert!(
            (t.z - (-1000.0)).abs() < 1e-4,
            "starboard engine → nose-left yaw, got z={}",
            t.z
        );
    }

    /// Zero-force engine short-circuits: totals unchanged.
    #[test]
    fn engine_zero_force_no_accumulation() {
        let zf = thrust(Vec3::ZERO);
        let (mut f, mut t) = (Vec3::new(100.0, 0.0, 0.0), Vec3::new(0.0, 50.0, 0.0));
        accumulate_engine_force(&zf, Vec3::new(0.0, 5.0, 0.0), &mut f, &mut t);
        assert!((f - Vec3::new(100.0, 0.0, 0.0)).length() < 1e-5);
        assert!((t - Vec3::new(0.0, 50.0, 0.0)).length() < 1e-5);
    }
}
