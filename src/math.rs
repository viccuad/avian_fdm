//! Shared math utilities: rotations between the body frame and the world
//! frame, and other helper functions for the subsystems.
//!
//! All FDM math uses `f32`, `Vec3`, and `Quat`. Avian components are also
//! `f32`, except `Position`. `Position` uses the active avian3d precision
//! (`Real`, `RVector`): `f32` and `Vec3` by default, and `f64` and `DVec3`
//! with the `f64` feature. The helpers `*_to_*` convert at this boundary.

use avian3d::math::{RVector, Real};
use bevy_math::{Quat, Vec3};

/// Linear interpolation in a 1-D breakpoint table.
///
/// The caller clamps `x` to `[bp[0], bp[last]]`. This function assumes that
/// `bp` and `vals` have the same length, and that `bp` is in ascending order.
/// It returns `0.0` for an empty table, and `vals[0]` for a table with one
/// entry.
#[inline]
pub(crate) fn lerp_1d(x: f32, bp: &[f32], vals: &[f32]) -> f32 {
    debug_assert_eq!(bp.len(), vals.len());
    if bp.is_empty() {
        return 0.0;
    }
    if bp.len() == 1 {
        return vals[0];
    }
    let idx = bp.partition_point(|&b| b <= x).saturating_sub(1);
    let idx = idx.min(bp.len() - 2);
    let t = (x - bp[idx]) / (bp[idx + 1] - bp[idx]);
    vals[idx] + t * (vals[idx + 1] - vals[idx])
}

/// Changes a Bevy `Vec3` (always f32) into an `RVector` of the active precision.
#[inline]
#[allow(clippy::unnecessary_cast)]
pub(crate) fn vec3_to_vector(v: Vec3) -> RVector {
    RVector::new(v.x as Real, v.y as Real, v.z as Real)
}

/// Changes an `RVector` of the active precision back into a Bevy `Vec3` (always f32).
#[inline]
#[allow(clippy::unnecessary_cast)]
pub(crate) fn vector_to_vec3(v: RVector) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Changes a `Real` of the active precision into an `f32`.
#[inline]
#[allow(clippy::unnecessary_cast)]
pub(crate) fn scalar_to_f32(x: Real) -> f32 {
    x as f32
}

/// Turns a vector from the aircraft body frame into the Bevy world frame.
///
/// # Body frame convention
/// X = forward (nose), Y = right wing, Z = down (belly).
/// This function applies only `rotation`. With the identity rotation, the
/// body axes and the world axes are the same.
#[cfg(test)]
pub(crate) fn body_to_world(rotation: Quat, v_body: Vec3) -> Vec3 {
    rotation * v_body
}

/// Turns a vector from the Bevy world frame into the aircraft body frame.
#[inline]
pub(crate) fn world_to_body(rotation: Quat, v_world: Vec3) -> Vec3 {
    rotation.inverse() * v_world
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    const EPS: f32 = 1e-5;

    fn approx_eq(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < EPS
    }

    /// `body_to_world` with identity rotation is a no-op, it does not apply
    /// any implicit basis change. The "body X to world −Z at identity rotation"
    /// convention is enforced by *how the aircraft mesh is authored* (facing
    /// world −Z), not by this function. This test confirms the function is
    /// a pure quaternion rotation with no hidden transform.
    #[test]
    fn identity_rotation_is_passthrough() {
        assert!(approx_eq(body_to_world(Quat::IDENTITY, Vec3::X), Vec3::X));
        assert!(approx_eq(body_to_world(Quat::IDENTITY, Vec3::Y), Vec3::Y));
        assert!(approx_eq(body_to_world(Quat::IDENTITY, Vec3::Z), Vec3::Z));
    }

    /// `world_to_body` is the exact inverse of `body_to_world`.
    #[test]
    fn round_trip() {
        let rot = Quat::from_rotation_y(0.3) * Quat::from_rotation_x(0.1);
        let v = Vec3::new(1.0, 2.0, 3.0);
        let round = world_to_body(rot, body_to_world(rot, v));
        assert!(approx_eq(round, v), "round-trip failed: {round:?} != {v:?}");
    }

    /// Rotating 90° about world +Y takes body +X to world −Z (right-hand rule).
    #[test]
    fn rotation_y_90_x_to_neg_z() {
        let rot = Quat::from_rotation_y(FRAC_PI_2);
        let result = body_to_world(rot, Vec3::X);
        assert!(
            approx_eq(result, -Vec3::Z),
            "expected (0,0,-1), got {result:?}"
        );
    }

    /// Rotating 90° about world +X takes body +Y to world +Z (right-hand rule).
    #[test]
    fn rotation_x_90_y_to_z() {
        let rot = Quat::from_rotation_x(FRAC_PI_2);
        let result = body_to_world(rot, Vec3::Y);
        assert!(
            approx_eq(result, Vec3::Z),
            "expected (0,0,1), got {result:?}"
        );
    }

    /// `world_to_body` correctly inverts a known rotation.
    #[test]
    fn world_to_body_inverts_rotation_y() {
        let rot = Quat::from_rotation_y(FRAC_PI_2);
        // After a 90-deg rotation about +Y, world -Z is body +X.
        let result = world_to_body(rot, -Vec3::Z);
        assert!(
            approx_eq(result, Vec3::X),
            "expected (1,0,0), got {result:?}"
        );
    }
}
