//! ISA atmosphere model and wind resource.
//!
//! Computes [`crate::components::AtmosphereState`] each physics frame from
//! the aircraft's geometric altitude. Implements the International Standard
//! Atmosphere (ICAO Doc 7488) for the troposphere (0–11 km) and lower
//! stratosphere (11–20 km), plus Sutherland's law for dynamic viscosity.

// TODO(atmosphere): implement update_atmosphere system
