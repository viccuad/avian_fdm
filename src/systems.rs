//! System wiring: registers all FDM systems into Avian's `PhysicsSchedule`
//! in the correct dependency order.
//!
//! ## Execution order within `PhysicsSet::Prepare`
//!
//! ```text
//! update_atmosphere
//!   → update_flight_state      (needs ρ for Re)
//!   → aggregate_zones          (needs α, Re to evaluate AeroCoeff tables)
//!   → compute_propulsion        (needs FlightState; writes ExternalForce + PropwashState)
//!   → compute_aerodynamics      (needs AircraftAggregate + FlightState + PropwashState)
//! ```
//!
//! All systems run before Avian's `SubstepSet` integrator.
//!
//! ## Adding a custom system (e.g. autopilot)
//!
//! Insert your system between `update_flight_state` and `aggregate_zones` to
//! read the latest `FlightState` and write `ControlInputs` before forces are
//! computed:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! // app.add_systems(PhysicsSchedule, my_autopilot.after(update_flight_state).before(aggregate_zones));
//! ```

use bevy::prelude::*;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};

use crate::atmosphere::{update_atmosphere, update_flight_state};
use crate::aerodynamics::compute_aerodynamics;
#[cfg(feature = "damage")]
use crate::zone_aggregation::aggregate_zones;
#[cfg(feature = "propulsion")]
use crate::propulsion::compute_propulsion;

/// Registers all FDM frame systems in the correct order within
/// `PhysicsSet::Prepare`.
pub(crate) fn register_fdm_systems(app: &mut App) {
    // All systems chain together so each step sees the previous step's output.
    #[cfg(all(feature = "damage", feature = "propulsion"))]
    app.add_systems(
        PhysicsSchedule,
        (
            update_atmosphere,
            update_flight_state,
            aggregate_zones,
            compute_propulsion,
            compute_aerodynamics,
        )
            .chain()
            .in_set(PhysicsStepSystems::First),
    );

    #[cfg(all(feature = "damage", not(feature = "propulsion")))]
    app.add_systems(
        PhysicsSchedule,
        (
            update_atmosphere,
            update_flight_state,
            aggregate_zones,
            compute_aerodynamics,
        )
            .chain()
            .in_set(PhysicsStepSystems::First),
    );

    #[cfg(all(not(feature = "damage"), feature = "propulsion"))]
    app.add_systems(
        PhysicsSchedule,
        (
            update_atmosphere,
            update_flight_state,
            compute_propulsion,
            compute_aerodynamics,
        )
            .chain()
            .in_set(PhysicsStepSystems::First),
    );

    #[cfg(all(not(feature = "damage"), not(feature = "propulsion")))]
    app.add_systems(
        PhysicsSchedule,
        (update_atmosphere, update_flight_state, compute_aerodynamics)
            .chain()
            .in_set(PhysicsStepSystems::First),
    );
}
