//! Aerodynamic force and moment pipeline.
//!
//! Reads [`crate::components::AircraftAggregate`] (pre-evaluated coefficient
//! totals), [`crate::components::FlightState`], and optionally
//! [`crate::components::PropwashState`], then writes to Avian's
//! [`avian3d::dynamics::solver::joints::ExternalForce`] and
//! `ExternalTorque` components.

// TODO(aerodynamics): implement compute_aerodynamics system
