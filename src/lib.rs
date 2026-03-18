//! # avian_fdm — 6-DoF Flight Dynamics Model for Bevy + Avian
//!
//! `avian_fdm` is a Bevy plugin that computes aerodynamic forces and moments
//! for rigid-body aircraft simulated with the [Avian](https://crates.io/crates/avian3d)
//! physics engine. Each physics frame the library evaluates lift, drag, side
//! force, and the three moment axes, then writes the results to Avian's
//! [`ExternalForce`] and [`ExternalTorque`] components — Avian's integrator
//! then propagates the rigid body. The library never moves an entity directly.
//!
//! ## What is a Flight Dynamics Model?
//!
//! A Flight Dynamics Model (FDM) is the mathematical description of all the
//! forces and moments acting on an aircraft. Newton's second law in 6 degrees
//! of freedom:
//!
//! ```text
//! F = m · a          (three translational axes)
//! M = I · α + ω × (I · ω)   (three rotational axes)
//! ```
//!
//! where **F** is the net external force vector, **m** is total mass, **a**
//! is linear acceleration, **M** is the net external moment, **I** is the
//! inertia tensor, **α** is angular acceleration, and **ω** is angular
//! velocity. Avian solves these equations every substep; the FDM's only job
//! is to compute **F** and **M** each frame.
//!
//! The dominant contributions to **F** and **M** are:
//! - **Aerodynamic forces** — lift, drag, side force, and the three moment
//!   axes, all proportional to dynamic pressure q̄ = ½ρV²
//! - **Propulsive thrust** — modelled as an actuator disk for piston engines
//! - **Gravity** — handled by Avian's gravity resource, not by this library
//!
//! ## Coordinate Frames
//!
//! ### Body frame (aircraft-fixed, SAE aerospace standard)
//!
//! ```text
//!         Z (down)
//!         │
//!         └──── Y (right wing)
//!        ╱
//!       X (forward, nose)
//! ```
//!
//! | Axis | Direction  | Positive rotation     |
//! |------|------------|-----------------------|
//! | X    | Nose       | Roll right            |
//! | Y    | Right wing | Pitch nose up         |
//! | Z    | Belly-down | Yaw nose right        |
//!
//! ### World frame (Bevy / Avian, Y-up right-handed)
//!
//! ```text
//!         Y (up)
//!         │
//!         └──── X (east, arbitrary)
//!        ╱
//!       Z (south, arbitrary)
//! ```
//!
//! At identity rotation (`Transform::default()`), body X maps to world −Z:
//! the aircraft faces into the screen in Bevy's default camera setup.
//!
//! All internal computation uses `f64` (`DVec3`, `DMat3`, `DQuat` from glam).
//! The only `f64 → f32` conversion occurs when writing to Avian's components.
//!
//! ### Unit conventions
//!
//! All quantities are **SI** throughout:
//!
//! | Quantity   | Unit          |
//! |------------|---------------|
//! | Distance   | metres (m)    |
//! | Mass       | kilograms (kg)|
//! | Force      | Newtons (N)   |
//! | Torque     | N·m           |
//! | Velocity   | m/s           |
//! | Angles     | radians (rad) |
//! | Pressure   | Pascals (Pa)  |
//! | Density    | kg/m³         |
//! | Temperature| Kelvin (K)    |
//!
//! ## Data Flow
//!
//! ```text
//! ┌─── PostStartup ────────────────────────────────────────────┐
//! │  init_zone_volumes   compute collider_volume_m3 + mass_kg  │
//! └────────────────────────────────────────────────────────────┘
//!
//! ┌─── PhysicsSet::Prepare (each physics frame) ───────────────────────┐
//! │  update_atmosphere   → AtmosphereState (ρ, p, T, a)               │
//! │  update_flight_state → FlightState (α, β, V, q̄, Re, Mach)         │
//! │  aggregate_zones     → AircraftAggregate (evaluated f64 totals)    │
//! │                         AircraftMass (total m, CG, inertia tensor) │
//! │  compute_propulsion  → ExternalForce (thrust) + PropwashState      │
//! │  compute_aerodynamics→ ExternalForce + ExternalTorque (aero)       │
//! └────────────────────────────────────────────────────────────────────┘
//!         │
//!         ▼ Avian substep integrator
//!    position, velocity, rotation updated
//! ```
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use avian_fdm::prelude::*;
//! use bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .add_plugins(AircraftFdmPlugin)
//!         .add_systems(Startup, spawn_aircraft)
//!         .run();
//! }
//!
//! fn spawn_aircraft(mut commands: Commands) {
//!     commands.spawn(AircraftCoreBundle {
//!         geometry: AircraftGeometry {
//!             wing_area_m2: 16.2,
//!             wing_span_m: 10.7,
//!             chord_m: 1.52,
//!         },
//!         ..default()
//!     });
//! }
//! ```
//!
//! ## Feature Flags
//!
//! | Feature      | Default | Enables                                    |
//! |--------------|---------|--------------------------------------------|
//! | `damage`     | on      | Zone health, mass aggregation, CG shifting |
//! | `propulsion` | on      | Piston engine + propwash model             |
//! | `debug-viz`  | off     | Bevy gizmo overlays + egui HUD             |
//! | `presets`    | off     | Reference aircraft (J3Cub)                 |

#![deny(missing_docs)]

use bevy::prelude::*;

pub mod components;
pub mod math;

#[cfg(feature = "damage")]
pub mod zone_aggregation;

pub mod atmosphere;
pub mod aerodynamics;

#[cfg(feature = "propulsion")]
pub mod propulsion;

pub mod systems;

#[cfg(feature = "debug-viz")]
pub mod debug;

#[cfg(feature = "presets")]
pub mod presets;

pub mod plugin;

/// Re-exports for convenient glob import: `use avian_fdm::prelude::*;`
pub mod prelude {
    pub use crate::components::{
        AeroZone, AeroZoneBundle,
        AircraftCoreBundle, AircraftGeometry,
        ControlInputs, FlightState, AtmosphereState,
        aero_coeff::AeroCoeff,
    };
    pub use crate::plugin::AircraftFdmPlugin;

    #[cfg(feature = "damage")]
    pub use crate::components::{
        AeroZoneHealth, AircraftAggregate, AircraftDamageBundle, AircraftMass,
        ZoneMass, materials,
    };

    #[cfg(feature = "propulsion")]
    pub use crate::components::{
        AircraftPropulsionBundle, EngineConfig, PropwashState,
    };
}
