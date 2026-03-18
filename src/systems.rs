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

// TODO(systems): wire subsystem systems into PhysicsSchedule
