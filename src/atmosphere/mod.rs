//! ISA atmosphere model and wind resource.
//!
//! See [`model`] for the International Standard Atmosphere implementation and
//! Bevy systems. See [`resources`] for [`WindResource`].

pub mod model;
pub mod resources;

pub use model::{sutherland_viscosity, update_atmosphere, update_flight_state};
pub use resources::WindResource;
