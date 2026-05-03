//! Bevy systems for the atmosphere subsystem.
//!
//! Bridges the pure ISA functions in [`super::isa`] to ECS data on aircraft
//! entities. Currently exposes a single system, [`update_atmosphere`], which
//! samples [`AtmosphereState`] from the world-space altitude.

use crate::_bevy::*;
use crate::components::AtmosphereState;
use super::isa::atmosphere_at;
use avian3d::math::Scalar;

/// Updates [`AtmosphereState`] on each aircraft from its world-space altitude.
///
/// Reads `GlobalTransform.translation().y` as geometric altitude above sea level.
#[allow(clippy::unnecessary_cast)]
pub fn update_atmosphere(mut query: Query<(&GlobalTransform, &mut AtmosphereState)>) {
    for (transform, mut atm) in &mut query {
        let altitude_m = transform.translation().y as Scalar;
        *atm = atmosphere_at(altitude_m);
    }
}
