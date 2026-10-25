//! ISA atmosphere model and wind resource.
//!
//! - [`isa`]: pure ISA and Sutherland functions and constants (no Bevy types).
//! - [`systems`]: Bevy systems that write the ISA values into ECS components.
//! - [`resources`]: [`WindResource`], the global wind vector.

pub mod isa;
pub mod resources;
pub mod systems;

pub use isa::sutherland_viscosity;
pub use resources::WindResource;
pub use systems::update_atmosphere;
