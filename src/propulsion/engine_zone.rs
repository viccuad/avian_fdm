//! Engine zone component.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

/// Configuration of a piston engine. Attach it to the engine child entity,
/// together with an Avian [`avian3d::prelude::Collider`] and
/// [`avian3d::prelude::ColliderDensity`].
///
/// In each physics step, `compute_engine_zone_forces` writes the thrust and
/// the world position of the engine into the `ZoneForce` of this entity.
/// Then `compute_aero_forces` adds the thrust to the root entity. It also
/// adds the torque from the moment arm between the CG and the engine.
///
/// If the same entity has a [`crate::components::Failure`], the crate reads
/// the damage state from it. If it has no `Failure`, the engine is fully
/// intact.
///
/// It is on the **engine child entity**.
///
/// ## Required components
///
/// - ZoneForce: the FDM writes it in each physics step. Do not change it.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
#[require(crate::components::zone_force::ZoneForce)]
pub struct EngineZone {
    /// Maximum static thrust at sea level (N).
    pub max_thrust_n: f32,
    /// Table from the throttle position to the thrust fraction. Each entry is
    /// `[throttle, fraction]`. `throttle` is in [0, 1]. The throttle values
    /// must be strictly increasing.
    pub throttle_curve: Vec<[f32; 2]>,
    /// Thrust direction in the body frame (unit vector). Usually `Vec3::X` (forward).
    pub thrust_axis_body: Vec3,

    /// Airspeed at which the propeller makes zero net thrust (m/s).
    ///
    /// It models how the thrust of a fixed-pitch propeller changes with
    /// speed. The thrust multiplier is `max(0, 1 − (V / V_zero)²)`:
    ///
    /// - At V = 0: factor = 1.0 (full static thrust)
    /// - At V = V_zero: factor = 0.0 (the propeller windmills)
    ///
    /// `None` disables the speed dependence (constant-thrust model).
    /// Typical values for light general-aviation aircraft: 70–90 m/s
    /// (J-3 Cub: approximately 80 m/s).
    pub zero_thrust_speed_ms: Option<f32>,
}

impl Default for EngineZone {
    fn default() -> Self {
        Self {
            max_thrust_n: 0.0,
            throttle_curve: vec![[0.0, 0.0], [1.0, 1.0]],
            thrust_axis_body: Vec3::X,
            zero_thrust_speed_ms: None,
        }
    }
}
