//! System wiring — registers all FDM systems into Avian's `PhysicsSchedule`
//! in the correct dependency order.
//!
//! ## Execution order within `PhysicsStepSystems::First`
//!
//! ```text
//! update_atmosphere
//!   → update_flight_state         (needs ρ for Re; also writes p/q/r body rates)
//!   → compute_engine_zone_forces  (propulsion feature; writes ZoneForce + PropwashState)
//!   → compute_zone_forces         (writes ZoneForce per AeroZone)
//!   → accumulate_zone_forces      (sums ZoneForce → ConstantForce + ConstantTorque)
//! ```
//!
//! Avian's `ForceSystems::ApplyConstantForces` (runs later in the same
//! `PhysicsSchedule`) picks up `ConstantForce`/`ConstantTorque` and writes
//! them to `VelocityIntegrationData`.
//!
//! ## Adding a custom system (e.g. autopilot)
//!
//! Insert between `update_flight_state` and `compute_zone_forces` to read
//! `FlightState` and write `ControlInputs`:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::*;
//! // app.add_systems(PhysicsSchedule,
//! //     my_autopilot
//! //         .after(avian_fdm::atmosphere::update_flight_state)
//! //         .before(avian_fdm::aerodynamics::compute_zone_forces)
//! //         .in_set(PhysicsStepSystems::First));
//! ```

use bevy::prelude::*;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};

use crate::atmosphere::{update_atmosphere, update_flight_state};
use crate::aerodynamics::{compute_zone_forces, accumulate_zone_forces};

#[cfg(feature = "propulsion")]
use crate::propulsion::compute_engine_zone_forces;

/// Registers all FDM frame systems in the correct order.
pub(crate) fn register_fdm_systems(app: &mut App) {
    #[cfg(feature = "propulsion")]
    app.add_systems(
        PhysicsSchedule,
        (
            update_atmosphere,
            update_flight_state,
            compute_engine_zone_forces,
            compute_zone_forces,
            accumulate_zone_forces,
        )
            .chain()
            .in_set(PhysicsStepSystems::First),
    );

    #[cfg(not(feature = "propulsion"))]
    app.add_systems(
        PhysicsSchedule,
        (
            update_atmosphere,
            update_flight_state,
            compute_zone_forces,
            accumulate_zone_forces,
        )
            .chain()
            .in_set(PhysicsStepSystems::First),
    );
}
