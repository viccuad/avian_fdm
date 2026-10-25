//! Adds the engine-zone forces to the totals of the aircraft root entity.
//!
//! In each physics step, `compute_engine_zone_forces` (in `propulsion/`)
//! calculates the engine thrust. It writes the thrust into a
//! [`crate::components::ZoneForce`] on each engine child entity. This module
//! reads these values. It adds them, and the torque from their moment arm, to
//! the force and torque totals of the root entity.

use avian3d::math::Vector;
use bevy_math::Vec3;

use crate::components::ZoneForce;
use crate::math::vec3_to_vector;

/// Adds the thrust of one engine zone to the root force and torque.
///
/// The moment arm goes from the CG of the aircraft to the world position of
/// the engine. If an engine is not on the centerline and the thrust is
/// asymmetric, the engine makes a yawing moment.
pub(super) fn accumulate_engine_force(
    zf: &ZoneForce,
    com_world: Vector,
    total_force: &mut Vector,
    total_torque: &mut Vector,
) {
    if zf.force != Vec3::ZERO {
        let force = vec3_to_vector(zf.force);
        *total_force += force;
        // ZoneForce.world_point is Vec3 (always f32, for visualization).
        *total_torque += (vec3_to_vector(zf.world_point) - com_world).cross(force);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// On-centre engine produces pure thrust, no torque.
    #[test]
    fn engine_at_cg_no_moment() {
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::ZERO,
        };
        let (mut f, mut t) = (Vector::ZERO, Vector::ZERO);
        accumulate_engine_force(&zf, Vector::ZERO, &mut f, &mut t);
        assert!((f - Vector::new(500.0, 0.0, 0.0)).length() < 1e-5);
        assert!(t.length() < 1e-5, "on-axis engine must not produce torque");
    }

    /// Starboard off-centre engine produces nose-left yaw torque.
    #[test]
    fn engine_offset_right_produces_yaw_torque() {
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::new(0.0, 2.0, 0.0),
        };
        let (mut f, mut t) = (Vector::ZERO, Vector::ZERO);
        accumulate_engine_force(&zf, Vector::ZERO, &mut f, &mut t);
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
        let zf = ZoneForce {
            force: Vec3::ZERO,
            world_point: Vec3::new(0.0, 5.0, 0.0),
        };
        let (mut f, mut t) = (Vector::new(100.0, 0.0, 0.0), Vector::new(0.0, 50.0, 0.0));
        accumulate_engine_force(&zf, Vector::ZERO, &mut f, &mut t);
        assert!((f - Vector::new(100.0, 0.0, 0.0)).length() < 1e-5);
        assert!((t - Vector::new(0.0, 50.0, 0.0)).length() < 1e-5);
    }
}
