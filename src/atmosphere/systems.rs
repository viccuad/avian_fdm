//! Bevy systems for the atmosphere subsystem.
//!
//! They connect the pure ISA functions in [`super::isa`] to the ECS data on
//! aircraft entities. At this time, there is one system,
//! [`update_atmosphere`]. It calculates [`AtmosphereState`] from the altitude
//! in world space.

use super::isa::atmosphere_at;
use crate::_bevy::*;
use crate::atmosphere::resources::{altitude_m, PlanetResource};
use crate::components::AtmosphereState;
use avian3d::prelude::Position;

/// Updates [`AtmosphereState`] on each aircraft from its altitude.
///
/// It calls [`altitude_m`] for the geometric altitude above sea level. If a
/// [`PlanetResource`] exists, the altitude is the distance from the planet
/// center minus `radius_m`. If not, the altitude is the `y` component of the
/// Avian [`Position`]. `Position` is the physics state of the current
/// step. `GlobalTransform` would be one step old during `BroadPhase`, and it
/// is not the physics position when the app uses a floating origin.
pub fn update_atmosphere(
    mut query: Query<(&Position, &mut AtmosphereState)>,
    planet_resource: Option<Res<PlanetResource>>,
) {
    for (pos, mut atm) in &mut query {
        let altitude_m = altitude_m(pos, planet_resource.as_deref());
        *atm = atmosphere_at(altitude_m);
    }
}
