//! Bevy systems for the atmosphere subsystem.
//!
//! They connect the pure ISA functions in [`super::isa`] to the ECS data on
//! aircraft entities. At this time, there is one system,
//! [`update_atmosphere`]. It calculates [`AtmosphereState`] from the altitude
//! in world space.

use super::isa::atmosphere_at;
use crate::_bevy::*;
use crate::components::AtmosphereState;
use crate::math::scalar_to_f32;
use avian3d::prelude::Position;

/// Updates [`AtmosphereState`] on each aircraft from its altitude in world space.
///
/// It uses the `y` component of the Avian [`Position`] as the geometric
/// altitude above sea level. `Position` is the physics state of the current
/// step. `GlobalTransform` would be one step old during `BroadPhase`, and it is not
/// the physics position when the app uses a floating origin.
pub fn update_atmosphere(mut query: Query<(&Position, &mut AtmosphereState)>) {
    for (pos, mut atm) in &mut query {
        *atm = atmosphere_at(scalar_to_f32(pos.0.y));
    }
}
