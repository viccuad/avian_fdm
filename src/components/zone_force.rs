//! [`ZoneForce`], per-zone force and torque output written by the FDM
//! `compute_aero_forces` system (and `compute_engine_zone_forces` for thrust).
//! Also read by the debug visualisation for per-zone force arrows.

use bevy::prelude::*;

/// World-space force, pure torque, and application point for one zone.
///
/// Written by `compute_aero_forces` / `compute_engine_zone_forces`.
/// Read by `compute_aero_forces` (for engine zones during accumulation)
/// and by the debug visualisation.
///
/// Zero-initialised at spawn. Set to `default()` (zeroed) when the zone is
/// fully failed (`Failure.remaining == 0.0`) or otherwise inactive.
///
/// **Do not read or write this component from game code.**
#[derive(Component, Reflect, Default, Clone, Copy, Debug)]
#[reflect(Component)]
pub struct ZoneForce {
    /// World-space force contribution (N), from CL, CD, CY coefficients.
    pub force: Vec3,
    /// World-space pure aerodynamic torque (N·m), from CM, Croll, Cn
    /// coefficients. This is a couple (moment without net force), distinct
    /// from the moment-arm torque computed as (zone_pos − CG) × force.
    pub torque: Vec3,
    /// World-space point at which the force acts (for moment arm calculation).
    pub world_point: Vec3,
}
