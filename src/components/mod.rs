//! All ECS components, bundles, and shared data types.

pub mod aero_coeff;
pub mod aero_zone;
pub mod aircraft;
pub mod controls;
pub mod failure;
pub mod engine_zone;
pub mod flight_state;
pub mod zone_force;

pub mod gizmo_shape;

pub use aero_coeff::AeroCoeff;
pub use aero_zone::{AeroZone, AeroZoneBundle, ControlSurfaceRole, materials};
pub use aircraft::{AircraftGeometry, AircraftCoreBundle, LodDamping, InducedDrag};
pub use controls::ControlInputs;
pub use failure::Failure;
pub use flight_state::{FlightState, AtmosphereState, WindResource};
pub use gizmo_shape::{GizmoShape, GizmoContours};
pub use zone_force::ZoneForce;

#[cfg(feature = "propulsion")]
pub use engine_zone::{EngineZone, PropwashState};
