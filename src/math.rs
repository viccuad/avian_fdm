//! Shared math utilities: body-frame ↔ world-frame rotations, and other
//! helpers used across subsystems.
//!
//! All functions operate in the active avian3d precision (`Scalar` / `Vector`
//! / `Quaternion`), which is `f32`/`Vec3`/`Quat` when compiled with the
//! `f32` feature and `f64`/`DVec3`/`DQuat` with `f64`.

use avian3d::math::{Quaternion, Scalar, Vector};
use bevy_math::{Quat, Vec3};

/// Convert a Bevy `Vec3` (always f32) to the active-precision `Vector`.
#[inline]
#[allow(clippy::unnecessary_cast)]
pub(crate) fn vec3_to_vector(v: Vec3) -> Vector {
    Vector::new(v.x as Scalar, v.y as Scalar, v.z as Scalar)
}

/// Convert a Bevy `Quat` (always f32) to the active-precision `Quaternion`.
#[inline]
#[allow(clippy::unnecessary_cast)]
pub(crate) fn quat_to_quaternion(q: Quat) -> Quaternion {
    Quaternion::from_array(q.to_array().map(|x| x as Scalar))
}

/// Convert the active-precision `Vector` back to a Bevy `Vec3` (always f32).
#[inline]
#[allow(clippy::unnecessary_cast)]
pub(crate) fn vector_to_vec3(v: Vector) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Rotate a vector from the aircraft body frame into the Bevy world frame.
///
/// # Body frame convention
/// X = forward (nose), Y = right wing, Z = down (belly).
/// At identity rotation, body X maps to world −Z.
#[cfg(test)]
pub(crate) fn body_to_world(rotation: Quaternion, v_body: Vector) -> Vector {
    rotation * v_body
}

/// Rotate a vector from the Bevy world frame into the aircraft body frame.
#[inline]
pub(crate) fn world_to_body(rotation: Quaternion, v_world: Vector) -> Vector {
    rotation.inverse() * v_world
}

#[cfg(test)]
mod tests {
    use super::*;
    use avian3d::math::FRAC_PI_2;

    #[cfg(feature = "f32")]
    const EPS: Scalar = 1e-5;
    #[cfg(not(feature = "f32"))]
    const EPS: Scalar = 1e-10;

    fn approx_eq(a: Vector, b: Vector) -> bool {
        (a - b).length() < EPS
    }

    /// `body_to_world` with identity rotation is a no-op, it does not apply
    /// any implicit basis change. The "body X to world −Z at identity rotation"
    /// convention is enforced by *how the aircraft mesh is authored* (facing
    /// world −Z), not by this function. This test confirms the function is
    /// a pure quaternion rotation with no hidden transform.
    #[test]
    fn identity_rotation_is_passthrough() {
        assert!(approx_eq(
            body_to_world(Quaternion::IDENTITY, Vector::X),
            Vector::X
        ));
        assert!(approx_eq(
            body_to_world(Quaternion::IDENTITY, Vector::Y),
            Vector::Y
        ));
        assert!(approx_eq(
            body_to_world(Quaternion::IDENTITY, Vector::Z),
            Vector::Z
        ));
    }

    /// `world_to_body` is the exact inverse of `body_to_world`.
    #[test]
    fn round_trip() {
        let rot = Quaternion::from_rotation_y(0.3) * Quaternion::from_rotation_x(0.1);
        let v = Vector::new(1.0, 2.0, 3.0);
        let round = world_to_body(rot, body_to_world(rot, v));
        assert!(approx_eq(round, v), "round-trip failed: {round:?} != {v:?}");
    }

    /// Rotating 90° about world +Y takes body +X to world −Z (right-hand rule).
    #[test]
    fn rotation_y_90_x_to_neg_z() {
        let rot = Quaternion::from_rotation_y(FRAC_PI_2);
        let result = body_to_world(rot, Vector::X);
        assert!(
            approx_eq(result, -Vector::Z),
            "expected (0,0,-1), got {result:?}"
        );
    }

    /// Rotating 90° about world +X takes body +Y to world +Z (right-hand rule).
    #[test]
    fn rotation_x_90_y_to_z() {
        let rot = Quaternion::from_rotation_x(FRAC_PI_2);
        let result = body_to_world(rot, Vector::Y);
        assert!(
            approx_eq(result, Vector::Z),
            "expected (0,0,1), got {result:?}"
        );
    }

    /// `world_to_body` correctly inverts a known rotation.
    #[test]
    fn world_to_body_inverts_rotation_y() {
        let rot = Quaternion::from_rotation_y(FRAC_PI_2);
        // After 90° +Y rotation, world −Z is body +X.
        let result = world_to_body(rot, -Vector::Z);
        assert!(
            approx_eq(result, Vector::X),
            "expected (1,0,0), got {result:?}"
        );
    }
}
