//! Derived flight state and atmosphere state components.

use crate::_bevy::*;
use avian3d::math::Scalar;
use serde::{Deserialize, Serialize};

/// Derived aerodynamic state quantities. Written each frame by
/// `update_flight_state`. Read-only for consumers (HUD, autopilot, debug).
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct FlightState {
    /// Angle of attack, alpha (rad). Positive = nose above relative wind.
    pub alpha_rad: Scalar,
    /// Sideslip angle, beta (rad). Positive = nose left of relative wind.
    pub beta_rad: Scalar,
    /// True airspeed V (m/s).
    pub airspeed_ms: Scalar,
    /// Mach number = airspeed / speed of sound (dimensionless).
    pub mach: Scalar,
    /// Dynamic pressure, q-bar = half * density * airspeed^2 (Pa).
    pub dynamic_pressure_pa: Scalar,
    /// Geometric altitude above sea level (m).
    pub altitude_m: Scalar,
    /// Roll rate p in body frame (rad/s). Written by `update_flight_state`.
    pub p_rads: Scalar,
    /// Pitch rate q in body frame (rad/s). Written by `update_flight_state`.
    pub q_rads: Scalar,
    /// Yaw rate r in body frame (rad/s). Written by `update_flight_state`.
    pub r_rads: Scalar,
}

/// ISA atmosphere conditions at this entity's altitude.
/// Written each frame by `update_atmosphere`.
///
/// Lives on the **aircraft root entity**.
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
