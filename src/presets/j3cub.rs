//! Piper J-3 Cub reference preset.
//!
//! Coefficients transcribed from the JSBSim J3Cub model (`J3Cub.xml`),
//! USA-35B airfoil data. Re breakpoints: 1 668 183 and 3 707 224.
//!
//! Use this preset to:
//! - Validate your FDM setup against the ±1% JSBSim fixture
//! - As a reference when decomposing your own aircraft into zones
//!
//! # Zone decomposition
//!
//! | Zone            | Role                              |
//! |-----------------|-----------------------------------|
//! | Left wing root  | Lift + inboard roll contribution  |
//! | Left wing mid   | Lift                              |
//! | Left wing tip   | Lift + outboard roll contribution |
//! | Right wing root | Lift + inboard roll contribution  |
//! | Right wing mid  | Lift                              |
//! | Right wing tip  | Lift + outboard roll contribution |
//! | Left aileron    | `AileronLeft` control surface     |
//! | Right aileron   | `AileronRight` control surface    |
//! | Fuselage        | Drag + yaw stability              |
//! | Horizontal tail | Pitch stability                   |
//! | Elevator        | `Elevator` control surface        |
//! | Vertical tail   | Yaw stability                     |
//! | Rudder          | `Rudder` control surface          |
//! | Engine zone     | Mass + propwash source            |

// TODO(j3cub-preset): transcribe J3Cub.xml tables and zone decomposition
