//! ISA atmosphere model and wind resource.
//!
//! - [`isa`]: pure ISA and Sutherland functions and constants (no Bevy types).
//! - [`systems`]: Bevy systems that write the ISA values into ECS components.
//! - [`resources`]: [`WindResource`], the global wind vector and
//!   [`PlanetResource`], defining the planet center and radius.

pub(crate) mod isa;
pub(crate) mod resources;
pub(crate) mod systems;

pub(crate) use isa::sutherland_viscosity;
pub use resources::{PlanetResource, WindResource};
pub(crate) use systems::update_atmosphere;
