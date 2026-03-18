//! Derived flight state and atmosphere state components, plus the optional
//! [`WindResource`].

use bevy::prelude::*;
use bevy::math::DVec3;
use serde::{Deserialize, Serialize};

/// Derived aerodynamic state quantities. Written each frame by
/// `update_flight_state`. Read-only for consumers (HUD, autopilot, debug).
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct FlightState {
    /// Angle of attack α (rad). Positive = nose above relative wind.
    pub alpha_rad: f64,
    /// Sideslip angle β (rad). Positive = nose left of relative wind.
    pub beta_rad: f64,
    /// True airspeed V (m/s).
    pub airspeed_ms: f64,
    /// Mach number M = V / a (dimensionless).
    pub mach: f64,
    /// Dynamic pressure q̄ = ½ρV² (Pa).
    pub dynamic_pressure_pa: f64,
    /// Reynolds number Re = ρVc̄/μ (dimensionless).
    pub reynolds_number: f64,
    /// Geometric altitude above sea level (m).
    pub altitude_m: f64,
}

/// ISA atmosphere conditions at this entity's altitude.
/// Written each frame by `update_atmosphere`.
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AtmosphereState {
    /// Air density ρ (kg/m³).
    pub density_kgm3: f64,
    /// Static pressure p (Pa).
    pub pressure_pa: f64,
    /// Temperature T (K).
    pub temperature_k: f64,
    /// Speed of sound a (m/s).
    pub speed_of_sound_ms: f64,
}

/// Optional uniform ambient wind resource. Insert into the Bevy [`World`] to
/// add a global wind to all aircraft.
///
/// If absent, relative wind = aircraft velocity only. Per-entity or
/// altitude-varying wind is a post-v1 feature (see Group D roadmap).
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::WindResource;
/// # use bevy::prelude::*;
/// # use bevy::math::DVec3;
/// // app.insert_resource(WindResource { velocity_world_ms: DVec3::new(-5.0, 0.0, 0.0) });
/// ```
#[derive(Resource, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Resource, Serialize, Deserialize)]
pub struct WindResource {
    /// Ambient wind velocity in world frame (m/s).
    /// Positive X = wind blowing in world +X direction.
    pub velocity_world_ms: DVec3,
}
