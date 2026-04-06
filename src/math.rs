//! Shared math utilities: body-frame ↔ world-frame rotations, and other
//! helpers used across subsystems.
//!
//! All functions operate in `f64` (`DVec3`, `DQuat`). The `f64`-to-`f32`
//! conversion to Avian components is performed in [`crate::systems`].

use bevy_math::{DQuat, DVec3, Vec3};

/// Convert any avian3d `Vector` (f32 `Vec3` or f64 `DVec3`) to `DVec3` for
/// internal FDM arithmetic. Avian chooses `Vector = Vec3` with the `f32`
/// feature and `Vector = DVec3` with the `f64` feature. avian_fdm always
/// does its physics math in f64 regardless of the backend.
pub(crate) trait VecToF64 {
    fn vec_to_f64(self) -> DVec3;
}

impl VecToF64 for Vec3 {
    #[inline]
    fn vec_to_f64(self) -> DVec3 {
        DVec3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

impl VecToF64 for DVec3 {
    #[inline]
    fn vec_to_f64(self) -> DVec3 { self }
}

/// Convert a Bevy `Vec3` (f32) to a `DVec3` (f64) for FDM calculations.
///
/// Used throughout the codebase to bridge Avian's f32 physics transforms
/// with the FDM's f64 arithmetic.
#[inline]
pub fn to_dvec3(v: Vec3) -> DVec3 {
    DVec3::new(v.x as f64, v.y as f64, v.z as f64)
}

/// Rotate a vector from the aircraft body frame into the Bevy world frame.
///
/// `rotation` is the aircraft's world-space orientation quaternion, converted
/// from the `f32` [`bevy::transform::components::Transform::rotation`] to
/// `f64` before calling this function.
///
/// Rotate a vector from the aircraft body frame into the Bevy world frame.
///
/// # Body frame convention
/// X = forward (nose), Y = right wing, Z = down (belly).
/// At identity rotation, body X maps to world −Z.
#[cfg(test)]
pub(crate) fn body_to_world(rotation: DQuat, v_body: DVec3) -> DVec3 {
    rotation * v_body
}

/// Rotate a vector from the Bevy world frame into the aircraft body frame.
#[inline]
pub(crate) fn world_to_body(rotation: DQuat, v_world: DVec3) -> DVec3 {
    rotation.inverse() * v_world
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-10;

    fn approx_eq(a: DVec3, b: DVec3) -> bool {
        (a - b).length() < EPS
    }

    /// `body_to_world` with identity rotation is a no-op, it does not apply
    /// any implicit basis change. The "body X to world −Z at identity rotation"
    /// convention is enforced by *how the aircraft mesh is authored* (facing
    /// world −Z), not by this function. This test confirms the function is
    /// a pure quaternion rotation with no hidden transform.
    #[test]
    fn identity_rotation_is_passthrough() {
        assert!(approx_eq(body_to_world(DQuat::IDENTITY, DVec3::X), DVec3::X));
        assert!(approx_eq(body_to_world(DQuat::IDENTITY, DVec3::Y), DVec3::Y));
        assert!(approx_eq(body_to_world(DQuat::IDENTITY, DVec3::Z), DVec3::Z));
    }

    /// `world_to_body` is the exact inverse of `body_to_world`.
    #[test]
    fn round_trip() {
        let rot = DQuat::from_rotation_y(0.3) * DQuat::from_rotation_x(0.1);
        let v = DVec3::new(1.0, 2.0, 3.0);
        let round = world_to_body(rot, body_to_world(rot, v));
        assert!(approx_eq(round, v), "round-trip failed: {round:?} != {v:?}");
    }

    /// Rotating 90° about world +Y takes body +X to world −Z (right-hand rule).
    #[test]
    fn rotation_y_90_x_to_neg_z() {
        let rot = DQuat::from_rotation_y(FRAC_PI_2);
        let result = body_to_world(rot, DVec3::X);
        assert!(approx_eq(result, -DVec3::Z), "expected (0,0,-1), got {result:?}");
    }

    /// Rotating 90° about world +X takes body +Y to world +Z (right-hand rule).
    #[test]
    fn rotation_x_90_y_to_z() {
        let rot = DQuat::from_rotation_x(FRAC_PI_2);
        let result = body_to_world(rot, DVec3::Y);
        assert!(approx_eq(result, DVec3::Z), "expected (0,0,1), got {result:?}");
    }

    /// `world_to_body` correctly inverts a known rotation.
    #[test]
    fn world_to_body_inverts_rotation_y() {
        let rot = DQuat::from_rotation_y(FRAC_PI_2);
        // After 90° +Y rotation, world −Z is body +X.
        let result = world_to_body(rot, -DVec3::Z);
        assert!(approx_eq(result, DVec3::X), "expected (1,0,0), got {result:?}");
    }
}
