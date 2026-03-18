//! Reference aircraft implementations for consumers and validation.
//!
//! Only compiled with `features = ["presets"]`.
//!
//! Each preset provides a complete [`crate::components::AircraftCoreBundle`]
//! and child [`crate::components::AeroZoneBundle`] list that can be spawned
//! directly, or inspected as a reference when authoring a new aircraft.

#[cfg(feature = "presets")]
pub mod j3cub;
