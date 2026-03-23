//! Engine zone component and propwash state.
//!
//! Only compiled with `features = ["propulsion"]`.

use bevy::prelude::*;
use bevy::math::DVec3;
use serde::{Deserialize, Serialize};

/// Piston engine configuration. Attach to the engine child entity alongside
/// an Avian [`avian3d::prelude::Collider`] and
/// [`avian3d::prelude::ColliderDensity`].
///
/// Thrust is applied via `apply_force_at_point` at the engine's world
/// position — Avian computes the torque contribution automatically.
///
/// Failure state is read from [`super::Failure`] if present on the same entity.
/// When absent the engine is treated as fully intact.
///
/// Lives on the **engine child entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct EngineZone {
    /// Sea-level static maximum thrust (N).
    pub max_thrust_n: f64,
    /// Throttle → thrust-fraction lookup. Each entry is `[throttle, fraction]`.
    /// `throttle` ∈ [0, 1]. Must be strictly increasing in throttle.
    pub throttle_curve: Vec<[f64; 2]>,
    /// Propeller diameter (m). Used to compute induced velocity:
    /// V_ind = √(T / (2ρA)).
    pub prop_diameter_m: f64,
    /// Thrust direction in body frame (unit vector). Usually `DVec3::X` (forward).
    pub thrust_axis_body: DVec3,

    /// Airspeed at which the propeller produces zero net thrust (m/s).
    ///
    /// Models the speed dependence of a fixed-pitch propeller. The thrust
    /// multiplier is `max(0, 1 − (V / V_zero)²)`:
    ///
    /// - At V = 0: factor = 1.0 (full static thrust)
    /// - At V = V_zero: factor = 0.0 (propeller windmilling)
    ///
    /// `None` disables speed dependence (constant-thrust model).
    /// Typical values for light GA: 70–90 m/s (J3 Cub ≈ 80 m/s).
    pub zero_thrust_speed_ms: Option<f64>,
}

impl Default for EngineZone {
    fn default() -> Self {
        Self {
            max_thrust_n: 0.0,
            throttle_curve: vec![[0.0, 0.0], [1.0, 1.0]],
            prop_diameter_m: 1.0,
            thrust_axis_body: DVec3::X,
            zero_thrust_speed_ms: None,
        }
    }
}

/// Propwash state — propeller-induced velocity over the wing root.
///
/// Written each frame by `compute_propulsion`. Read by `compute_aerodynamics`
/// to apply a propwash lift increment on nearby `AeroZone` children.
///
/// Lives on the **aircraft root entity** (nearest `RigidBody` ancestor).
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct PropwashState {
    /// Axial induced velocity from actuator disk theory (m/s).
    /// V_ind = √(T / (2 · ρ · A)), where A = π(d/2)².
    pub induced_velocity_ms: f64,
    /// Propwash direction in body frame (unit vector; usually = thrust_axis_body).
    pub direction_body: DVec3,
}
