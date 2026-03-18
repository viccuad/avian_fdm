//! Control input component. Written each frame by the game's input system.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// Pilot or autopilot control surface positions.
///
/// Write these each frame from your input system (keyboard, gamepad, autopilot,
/// replay playback, or network-received command). The FDM reads them and never
/// writes to this component.
///
/// All values are normalised: 0–1 for throttle, −1 to +1 for surfaces.
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct ControlInputs {
    /// Engine throttle: 0.0 = idle, 1.0 = full power.
    pub throttle: f64,
    /// Elevator deflection: +1.0 = full nose-up, −1.0 = full nose-down.
    pub elevator: f64,
    /// Aileron deflection: +1.0 = right wing down (roll right), −1.0 = roll left.
    pub aileron: f64,
    /// Rudder deflection: +1.0 = nose right (yaw right), −1.0 = nose left.
    pub rudder: f64,
}
