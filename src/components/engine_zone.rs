//! Engine zone component.

use crate::_bevy::*;
use avian3d::math::{Scalar, Vector};
use serde::{Deserialize, Serialize};

/// Piston engine configuration. Attach to the engine child entity alongside
/// an Avian [`avian3d::prelude::Collider`] and
/// [`avian3d::prelude::ColliderDensity`].
///
/// Thrust is applied via `apply_force_at_point` at the engine's world
/// position. Avian computes the torque contribution automatically.
///
/// Failure state is read from [`super::Failure`] if present on the same entity.
/// When absent the engine is treated as fully intact.
///
/// Lives on the **engine child entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct EngineZone {
    /// Sea-level static maximum thrust (N).
    pub max_thrust_n: Scalar,
    /// Throttle-to-thrust-fraction lookup table. Each entry is `[throttle, fraction]`.
    /// `throttle` in [0, 1]. Must be strictly increasing in throttle.
    pub throttle_curve: Vec<[Scalar; 2]>,
    /// Thrust direction in body frame (unit vector). Usually `Vector::X` (forward).
    pub thrust_axis_body: Vector,

    /// Airspeed at which the propeller produces zero net thrust (m/s).
    ///
    /// Models the speed dependence of a fixed-pitch propeller. The thrust
    /// multiplier is `max(0, 1 − (V / V_zero)²)`:
    ///
    /// - At V = 0: factor = 1.0 (full static thrust)
    /// - At V = V_zero: factor = 0.0 (propeller windmilling)
    ///
    /// `None` disables speed dependence (constant-thrust model).
    /// Typical values for light GA: 70–90 m/s (J3 Cub ~ 80 m/s).
    pub zero_thrust_speed_ms: Option<Scalar>,
}

impl Default for EngineZone {
    fn default() -> Self {
        Self {
            max_thrust_n: 0.0,
            throttle_curve: vec![[0.0, 0.0], [1.0, 1.0]],
            thrust_axis_body: Vector::X,
            zero_thrust_speed_ms: None,
        }
    }
}
