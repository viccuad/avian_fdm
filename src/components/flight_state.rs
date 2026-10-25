//! Components for the flight state and the atmosphere state.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

/// Aerodynamic state values that the crate calculates from the physics state.
/// `update_flight_state` writes them in each physics step. Other systems (HUD,
/// autopilot, debug) must only read them.
///
/// It is on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct FlightState {
    /// Angle of attack, alpha (rad). Positive when the nose is above the relative wind.
    pub alpha_rad: f32,
    /// Sideslip angle, beta (rad). Positive when the nose is left of the relative wind.
    pub beta_rad: f32,
    /// True airspeed V (m/s).
    pub airspeed_ms: f32,
    /// Mach number = airspeed ÷ speed of sound (dimensionless).
    pub mach: f32,
    /// Dynamic pressure, qbar = 0.5 * density * airspeed^2 (Pa).
    pub dynamic_pressure_pa: f32,
    /// Geometric altitude above sea level (m).
    pub altitude_m: f32,
    /// Roll rate p in the body frame (rad/s).
    pub p_rads: f32,
    /// Pitch rate q in the body frame (rad/s).
    pub q_rads: f32,
    /// Yaw rate r in the body frame (rad/s).
    pub r_rads: f32,
}

/// ISA atmosphere conditions at the altitude of this entity.
/// `update_atmosphere` writes them in each physics step.
///
/// It is on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AtmosphereState {
    /// Air density ρ (kg/m³).
    pub density_kgm3: f32,
    /// Static pressure p (Pa).
    pub pressure_pa: f32,
    /// Temperature T (K).
    pub temperature_k: f32,
    /// Speed of sound a (m/s).
    pub speed_of_sound_ms: f32,
}
