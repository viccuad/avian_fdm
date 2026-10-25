//! Components for the flight state and the atmosphere state.

use crate::_bevy::*;
use avian3d::math::Scalar;
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
    pub alpha_rad: Scalar,
    /// Sideslip angle, beta (rad). Positive when the nose is left of the relative wind.
    pub beta_rad: Scalar,
    /// True airspeed V (m/s).
    pub airspeed_ms: Scalar,
    /// Mach number = airspeed ÷ speed of sound (dimensionless).
    pub mach: Scalar,
    /// Dynamic pressure, qbar = 0.5 * density * airspeed^2 (Pa).
    pub dynamic_pressure_pa: Scalar,
    /// Geometric altitude above sea level (m).
    pub altitude_m: Scalar,
    /// Roll rate p in the body frame (rad/s).
    pub p_rads: Scalar,
    /// Pitch rate q in the body frame (rad/s).
    pub q_rads: Scalar,
    /// Yaw rate r in the body frame (rad/s).
    pub r_rads: Scalar,
}

/// ISA atmosphere conditions at the altitude of this entity.
/// `update_atmosphere` writes them in each physics step.
///
/// It is on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AtmosphereState {
    /// Air density ρ (kg/m³).
    pub density_kgm3: Scalar,
    /// Static pressure p (Pa).
    pub pressure_pa: Scalar,
    /// Temperature T (K).
    pub temperature_k: Scalar,
    /// Speed of sound a (m/s).
    pub speed_of_sound_ms: Scalar,
}
