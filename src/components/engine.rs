//! Engine configuration and propwash state components.
//!
//! Only compiled with `features = ["propulsion"]`.

use bevy::prelude::*;
use bevy::math::DVec3;
use serde::{Deserialize, Serialize};

/// Simple fixed-pitch piston engine configuration.
///
/// Lives on the **aircraft root entity**.
///
/// Damage to the engine zone scales [`EngineConfig::max_thrust_n`]
/// proportionally through the zone-aggregation system — no separate field needed.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct EngineConfig {
    /// Sea-level static maximum thrust (N).
    pub max_thrust_n: f64,
    /// Throttle → thrust-fraction lookup table. Each entry is `[throttle, fraction]`.
    /// `throttle` range: 0–1. `fraction` range: 0–1. Must be strictly increasing in throttle.
    pub throttle_curve: Vec<[f64; 2]>,
    /// Propeller diameter (m). Used to compute induced velocity: V_ind = √(T / 2ρA).
    pub prop_diameter_m: f64,
    /// Thrust direction in body frame (unit vector). Usually `DVec3::X` (forward).
    pub thrust_axis_body: DVec3,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            max_thrust_n: 0.0,
            throttle_curve: vec![[0.0, 0.0], [1.0, 1.0]],
            prop_diameter_m: 1.0,
            thrust_axis_body: DVec3::X,
        }
    }
}

/// Propwash state — propeller-induced velocity over the wing root.
///
/// Written each frame by `compute_propulsion`. Read by the aerodynamics
/// system to apply a propwash lift increment.
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct PropwashState {
    /// Axial induced velocity from actuator disk theory (m/s).
    /// V_ind = √(T / (2 · ρ · A)), where A = π(d/2)².
    pub induced_velocity_ms: f64,
    /// Propwash direction in body frame (unit vector, usually = thrust_axis_body).
    pub direction_body: DVec3,
}

/// Bundle for propulsion components. Add alongside [`crate::components::AircraftCoreBundle`]
/// when `features = ["propulsion"]`.
#[derive(Bundle, Default)]
pub struct AircraftPropulsionBundle {
    /// Engine configuration.
    pub engine: EngineConfig,
    /// Propwash state (zeroed; filled by `compute_propulsion`).
    pub propwash: PropwashState,
}
