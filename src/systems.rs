//! System wiring, registers all FDM systems into Avian's `PhysicsSchedule`
//! in the correct dependency order.
//!
//! ## Execution order within `PhysicsStepSystems::BroadPhase`
//!
//! ```text
//! update_atmosphere
//!   then update_flight_state         (needs rho for Re; also writes p/q/r body rates)
//!   then compute_engine_zone_forces  (propulsion feature; writes ZoneForce + PropwashState)
//!   then compute_aero_forces         (per-zone eval + accumulation + damping)
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
//! Insert between `update_flight_state` and `compute_aero_forces` to read
//! `FlightState` and write `ControlInputs`:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::*;
//! // app.add_systems(PhysicsSchedule,
//! //     my_autopilot
//! //         .after(avian_fdm::atmosphere::update_flight_state)
//! //         .before(avian_fdm::aerodynamics::compute_aero_forces)
//! //         .in_set(PhysicsStepSystems::BroadPhase));
//! ```

use bevy::prelude::*;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};

use crate::atmosphere::{update_atmosphere, update_flight_state};
use crate::aerodynamics::compute_aero_forces;

#[cfg(feature = "propulsion")]
use crate::propulsion::compute_engine_zone_forces;

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
    // The engine force system is only compiled with the `propulsion` feature.
    // Both configurations chain in BroadPhase so forces are written before the
    // Avian Solver reads them via ForceSystems::ApplyConstantForces.
    #[cfg(not(feature = "propulsion"))]
    app.add_systems(
        PhysicsSchedule,
        (update_atmosphere, update_flight_state, compute_aero_forces)
            .chain()
            .in_set(PhysicsStepSystems::BroadPhase),
    );

    #[cfg(feature = "propulsion")]
    app.add_systems(
        PhysicsSchedule,
        (
            update_atmosphere,
            update_flight_state,
            compute_engine_zone_forces,
            compute_aero_forces,
        )
            .chain()
            .in_set(PhysicsStepSystems::BroadPhase),
    );
}
