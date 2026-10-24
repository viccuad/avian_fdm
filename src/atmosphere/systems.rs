//! Bevy systems for the atmosphere subsystem.
//!
//! They connect the pure ISA functions in [`super::isa`] to the ECS data on
//! aircraft entities. At this time, there is one system,
//! [`update_atmosphere`]. It calculates [`AtmosphereState`] from the altitude
//! in world space.

use super::isa::atmosphere_at;
use crate::_bevy::*;
use crate::components::AtmosphereState;
use avian3d::math::Scalar;

/// Updates [`AtmosphereState`] on each aircraft from its altitude in world space.
///
/// It uses `GlobalTransform.translation().y` as the geometric altitude above
/// sea level.
#[allow(clippy::unnecessary_cast)]
pub fn update_atmosphere(mut query: Query<(&GlobalTransform, &mut AtmosphereState)>) {
    for (transform, mut atm) in &mut query {
        let altitude_m = transform.translation().y as Scalar;
        *atm = atmosphere_at(altitude_m);
    }
}
