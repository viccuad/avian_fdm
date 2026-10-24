//! Control input component. The input system of the game writes it in each frame.

use crate::_bevy::*;
use avian3d::math::Scalar;
use serde::{Deserialize, Serialize};

/// Control positions from the pilot or the autopilot.
///
/// Write these values in each frame from your input system. The source can be
/// a keyboard, a gamepad, an autopilot, a replay, or a command from the
/// network. The FDM reads this component. It never writes to it.
///
/// All values are normalized: 0 to 1 for the throttle, −1 to +1 for the
/// control surfaces.
///
/// It is on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct ControlInputs {
    /// Engine throttle: 0.0 = idle, 1.0 = full power.
    pub throttle: Scalar,
    /// Elevator deflection: +1.0 = full nose up, −1.0 = full nose down.
    pub elevator: Scalar,
    /// Aileron deflection: +1.0 = right wing down (roll right), −1.0 = roll left.
    pub aileron: Scalar,
    /// Rudder deflection: +1.0 = nose right (yaw right), −1.0 = nose left.
    pub rudder: Scalar,
}
