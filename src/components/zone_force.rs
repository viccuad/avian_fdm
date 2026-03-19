//! [`ZoneForce`] — per-zone force output written by the FDM compute systems
//! and read by the accumulation system. Internal component; not part of the
//! public API.

use bevy::prelude::*;

/// World-space force and application point computed for one zone each frame.
///
/// Written by `compute_zone_forces` / `compute_engine_zone_forces`.
/// Read by `accumulate_zone_forces`, which sums contributions into the root
/// entity's [`avian3d::prelude::ConstantForce`] and
/// [`avian3d::prelude::ConstantTorque`].
///
/// Zero-initialised at spawn. Set to `default()` (zeroed) when the zone is
/// destroyed (`Damageable.health == 0.0`) or otherwise inactive.
///
/// **Do not read or write this component from game code.**
#[derive(Component, Default, Clone, Copy, Debug)]
pub struct ZoneForce {
    /// World-space force contribution (N). f32 matches Avian's `Vector` type.
    pub force: Vec3,
    /// World-space point at which the force acts (for moment arm calculation).
    pub world_point: Vec3,
}
