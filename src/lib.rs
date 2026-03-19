//! # avian_fdm — 6-DoF Flight Dynamics Model for Bevy + Avian
//!
//! `avian_fdm` is a Bevy plugin that computes aerodynamic forces and moments
//! for rigid-body aircraft simulated with the [Avian](https://crates.io/crates/avian3d)
//! physics engine.
//!
//! Each physics frame the library iterates every [`components::AeroZone`]
//! child entity, evaluates lift/drag/moments, and calls Avian's
//! `apply_force_at_point` — Avian computes the moment arm automatically.
//! Mass, centre of gravity, and inertia are managed entirely by Avian via
//! [`avian3d::prelude::ColliderDensity`] on each child collider.
//!
//! ## What is a Flight Dynamics Model?
//!
//! A Flight Dynamics Model (FDM) is the mathematical description of all forces
//! and moments acting on an aircraft. Newton's second law in 6 degrees of
//! freedom:
//!
//! ```text
//! F = m · a
//! M = I · α + ω × (I · ω)
//! ```
//!
//! where **F** is net external force, **m** total mass, **a** linear
//! acceleration, **M** net external moment, **I** inertia tensor, **α**
//! angular acceleration, and **ω** angular velocity. Avian solves these
//! equations every substep; the FDM's only job is to supply **F** and **M**
//! at each zone's world position each frame.
//!
//! ## Coordinate Frames
//!
//! ### Body frame (aircraft-fixed, SAE aerospace)
//!
//! ```text
//!       X (forward/nose)
//!      ╱
//!     └──── Y (right wing)
//!     │
//!     Z (down)
//! ```
//!
//! | Axis | Direction  | Positive rotation |
//! |------|------------|-------------------|
//! | X    | Nose       | Roll right        |
//! | Y    | Right wing | Pitch nose up     |
//! | Z    | Belly-down | Yaw nose right    |
//!
//! ### World frame (Bevy / Avian, Y-up right-handed)
//!
//! At identity rotation, body X maps to world −Z (aircraft faces into screen).
//!
//! All internal computation uses `f64`. The only `f64 → f32` conversion is
//! when interfacing with Avian's `f32` APIs.
//!
//! ## Data Flow
//!
//! ```text
//! ┌─── PhysicsSet::Prepare (each physics frame) ─────────────────────────┐
//! │  update_atmosphere    → AtmosphereState (ρ, p, T, a)                 │
//! │  update_flight_state  → FlightState (α, β, V, q̄, Re, Mach)           │
//! │  compute_propulsion   → apply_force_at_point (thrust) + PropwashState │
//! │  compute_aerodynamics → apply_force_at_point per AeroZone child       │
//! └──────────────────────────────────────────────────────────────────────-┘
//!         │
//!         ▼  Avian substep integrator
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
//! | Feature      | Default | Enables                              |
//! |--------------|---------|--------------------------------------|
//! | `damage`     | on      | `Damageable` component + DetachPlugin |
//! | `propulsion` | on      | Piston engine + propwash model        |
//! | `debug-viz`  | off     | Bevy gizmo overlays + egui HUD        |
//! | `presets`    | off     | Reference aircraft (J3Cub)            |

#![deny(missing_docs)]

pub mod components;
pub mod math;
pub mod atmosphere;
pub mod aerodynamics;
pub mod systems;
pub mod plugin;

#[cfg(feature = "damage")]
pub mod detach;

#[cfg(feature = "propulsion")]
pub mod propulsion;

#[cfg(feature = "debug-viz")]
pub mod debug;

#[cfg(feature = "presets")]
pub mod presets;

/// Re-exports for convenient glob import: `use avian_fdm::prelude::*;`
pub mod prelude {
    pub use crate::components::{
        AeroZone, AeroZoneBundle, ControlSurfaceRole, materials,
        AircraftCoreBundle, AircraftGeometry,
        ControlInputs, FlightState, AtmosphereState,
        aero_coeff::AeroCoeff,
    };
    pub use crate::plugin::AircraftFdmPlugin;

    #[cfg(feature = "damage")]
    pub use crate::components::Damageable;

    #[cfg(feature = "propulsion")]
    pub use crate::components::{EngineZone, PropwashState};
}
