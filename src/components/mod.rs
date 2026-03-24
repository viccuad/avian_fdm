//! All ECS components, bundles, and shared data types.

pub mod aero_coeff;
pub mod aero_zone;
pub mod aircraft;
pub mod controls;
pub mod engine_zone;
pub mod failure;
pub mod flight_state;
pub mod zone_force;

pub mod gizmo_shape;

pub use aero_coeff::AeroCoeff;
pub use aero_zone::{AeroZone, AeroZoneBundle, ControlSurfaceRole};
pub use aircraft::{AircraftCoreBundle, AircraftGeometry, InducedDrag, LodDamping};
pub use controls::ControlInputs;
pub use failure::{get_remaining, Failure};
pub use flight_state::{AtmosphereState, FlightState, WindResource};
pub use gizmo_shape::{GizmoContours, GizmoShape};
pub use zone_force::ZoneForce;

#[cfg(feature = "propulsion")]
pub use engine_zone::{EngineZone, PropwashState};
