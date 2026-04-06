//! System wiring: registers all FDM systems into Avian's `PhysicsSchedule`
//! in the correct dependency order.
//!
//! ## Execution order within `PhysicsStepSystems::BroadPhase`
//!
//! ```text
//! Atmosphere -> FlightState -> Forces
//! ```
//!
//! The FDM chain runs in `BroadPhase` (not `First`) to avoid a Bevy static
//! ambiguity with Avian's `update_child_collider_position`, which writes
//! `Position`/`Rotation` for child colliders in `First`. Both systems operate
//! on disjoint entity sets at runtime, but the static checker cannot prove this.
//! `BroadPhase` runs after `First`, so the ordering is guaranteed.
//!
//! Avian's `ForceSystems::ApplyConstantForces` runs in `Solver` (after
//! `BroadPhase`), so forces are always written before they are read.
//!
//! ## Adding a custom system (e.g. autopilot)
//!
//! Use `AircraftFdmSystem` to schedule relative to named sets:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::*;
//! # use avian_fdm::systems::AircraftFdmSystem;
//! // app.add_systems(PhysicsSchedule,
//! //     my_autopilot.after(AircraftFdmSystem::FlightState)
//! //                 .before(AircraftFdmSystem::Forces));
//! ```

use crate::_bevy::*;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};

use crate::atmosphere::{update_atmosphere, update_flight_state};
use crate::aerodynamics::compute_aero_forces;

#[cfg(feature = "propulsion")]
use crate::propulsion::compute_engine_zone_forces;

/// Named system sets for the FDM pipeline. Use these to hook in custom systems.
///
/// Execution order: `Atmosphere` -> `FlightState` -> `Forces`
///
/// All sets run inside `PhysicsStepSystems::BroadPhase`.
///
/// Example: autopilot that reads `FlightState` and writes `ControlInputs`:
///
/// ```rust,no_run
/// # use bevy::prelude::*;
/// # use avian3d::prelude::*;
/// # use avian_fdm::systems::AircraftFdmSystem;
/// // app.add_systems(PhysicsSchedule,
/// //     my_autopilot.after(AircraftFdmSystem::FlightState)
/// //                 .before(AircraftFdmSystem::Forces));
/// ```
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub enum AircraftFdmSystem {
    /// Update ISA atmosphere density, temperature, and dynamic pressure.
    Atmosphere,
    /// Derive FlightState (alpha, beta, airspeed, altitude) from physics state.
    FlightState,
    /// Compute all zone forces (engine + aerodynamic) and accumulate onto the root body.
    Forces,
}

/// Registers all FDM frame systems in the correct order.
///
/// ## Why BroadPhase, not First?
///
/// Avian's `update_child_collider_position` (which writes `Position`/`Rotation`
/// for child colliders) also runs in `PhysicsStepSystems::First`. Bevy's static
/// ambiguity checker sees a conflict with `compute_aero_forces` reading
/// `Position`/`Rotation` on *root* entities, even though the entity sets are
/// disjoint at runtime. Placing our chain in `BroadPhase` (which runs after
/// `First`) eliminates the false ambiguity while keeping forces written well
/// before the `Solver` reads them via `ForceSystems::ApplyConstantForces`.
pub(crate) fn register_fdm_systems(app: &mut App) {
    app.configure_sets(
        PhysicsSchedule,
        (
            AircraftFdmSystem::Atmosphere,
            AircraftFdmSystem::FlightState,
            AircraftFdmSystem::Forces,
        )
            .chain()
            .in_set(PhysicsStepSystems::BroadPhase),
    );

    app.add_systems(
        PhysicsSchedule,
        update_atmosphere.in_set(AircraftFdmSystem::Atmosphere),
    );
    app.add_systems(
        PhysicsSchedule,
        update_flight_state.in_set(AircraftFdmSystem::FlightState),
    );

    #[cfg(not(feature = "propulsion"))]
    app.add_systems(
        PhysicsSchedule,
        compute_aero_forces.in_set(AircraftFdmSystem::Forces),
    );

    #[cfg(feature = "propulsion")]
    app.add_systems(
        PhysicsSchedule,
        (compute_engine_zone_forces, compute_aero_forces)
            .chain()
            .in_set(AircraftFdmSystem::Forces),
    );
}
