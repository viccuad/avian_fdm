//! All ECS components, bundles, and shared data types.
//!
//! Every public type derives `Component` (or `Resource`), `Reflect`,
//! `Serialize`, `Deserialize`, and `Clone` so they are first-class in Bevy
//! scenes, RON files, and Bevy's inspector tools.

pub mod aero_coeff;
pub mod aircraft;
pub mod aero_zone;
pub mod controls;
pub mod engine;
pub mod flight_state;

pub use aero_coeff::AeroCoeff;
pub use aircraft::{AircraftGeometry, AircraftCoreBundle};
pub use controls::ControlInputs;
pub use flight_state::{FlightState, AtmosphereState, WindResource};

#[cfg(feature = "damage")]
pub use aircraft::{AircraftMass, AircraftAggregate, AircraftDamageBundle};

#[cfg(feature = "damage")]
pub use aero_zone::{AeroZone, AeroZoneBundle, AeroZoneHealth, ControlEffectiveness,
                    ControlSurfaceRole, ZoneMass, materials};

#[cfg(feature = "propulsion")]
pub use engine::{EngineConfig, PropwashState, AircraftPropulsionBundle};
