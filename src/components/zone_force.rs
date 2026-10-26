//! `ZoneForce`: the force output of one zone. The FDM system
//! `compute_aero_forces` writes it for aero zones, and
//! `compute_engine_zone_forces` writes it for engine zones. The debug
//! visualization reads it to draw a force arrow for each zone.

use crate::_bevy::*;

/// Force of one zone in world space, and the point where it acts.
///
/// `compute_aero_forces` and `compute_engine_zone_forces` write it.
/// `compute_aero_forces` reads `force` for engine zones, when it adds up the
/// forces. The debug visualization reads both fields.
///
/// Its value is zero at spawn. It is set to `default()` (zero) when the zone
/// has `Failure.remaining == 0.0` or is not active for another reason.
///
/// **Do not read or write this component from game code.**
#[derive(Component, Reflect, Default, Clone, Copy, Debug)]
#[reflect(Component)]
pub(crate) struct ZoneForce {
    /// Force contribution in world space (N), from the CL, CD, and CY coefficients.
    pub force: Vec3,
    /// Point in world space where the force acts. Only the debug visualization
    /// uses it. The FDM calculates moment arms in body frame, because a
    /// difference of world positions loses precision far from the origin.
    pub world_point: Vec3,
}
