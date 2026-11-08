//! Bevy systems for the atmosphere subsystem.
//!
//! They connect the pure ISA functions in [`super::isa`] to the ECS data on
//! aircraft entities. At this time, there is one system,
//! [`update_atmosphere`]. It calculates [`AtmosphereState`] from the altitude
//! in world space.

use super::isa::atmosphere_at;
use crate::_bevy::*;
use crate::atmosphere::resources::PlanetResource;
use crate::components::AtmosphereState;
use avian3d::math::ToF32Precision;
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

/// Calculates the altitude from `PlanetResource`, or, if it's absent, the y-position in world space.
pub(crate) fn altitude_m(pos: &Position, planet: Option<&PlanetResource>) -> f32 {
    match planet {
        Some(p) => ((pos.0 - p.center).length() - p.radius_m).f32(),
        None => pos.0.y.f32(),
    }
}

#[cfg(test)]
mod tests {
    use crate::atmosphere::{systems::altitude_m, PlanetResource};
    use avian3d::{
        math::{RVector, Real},
        parry::glamx::approx::assert_abs_diff_eq,
        physics_transform::Position,
    };

    /// `altitude_m` returns `f32`, so the tolerance is `f32` in both builds.
    const EPS: f32 = if cfg!(feature = "f64") { 1e-3 } else { 0.01 };
    const EARTH_RADIUS_M: Real = PlanetResource::EARTH_RADIUS_M;

    #[test]
    fn altitude_flat_world() {
        assert_eq!(
            altitude_m(&Position::from_xyz(0.0, 1000.0, 0.0), None),
            1000.0
        )
    }

    #[test]
    fn altitude_round_y_axis() {
        let planet = PlanetResource::new(RVector::default(), EARTH_RADIUS_M);
        let pos = Position::from_xyz(0.0, EARTH_RADIUS_M + 1000.0, 0.0);

        assert_abs_diff_eq!(altitude_m(&pos, Some(&planet)), 1000.0, epsilon = EPS)
    }

    #[test]
    fn altitude_round_x_axis() {
        let planet = PlanetResource::new(RVector::default(), EARTH_RADIUS_M);
        let pos = Position::from_xyz(EARTH_RADIUS_M + 1000.0, 0.0, 0.0);

        assert_abs_diff_eq!(altitude_m(&pos, Some(&planet)), 1000.0, epsilon = EPS)
    }

    #[test]
    fn altitude_round_offset_center() {
        let center = RVector::new(0.0, -EARTH_RADIUS_M, 0.0);
        let planet = PlanetResource::new(center, EARTH_RADIUS_M);

        let pos = Position::from_xyz(0.0, 1000.0, 0.0);

        assert_abs_diff_eq!(altitude_m(&pos, Some(&planet)), 1000.0, epsilon = EPS)
    }

    #[test]
    fn altitude_round_below_sea_level() {
        let planet = PlanetResource::default();
        let pos = Position::from_xyz(0.0, -100.0, 0.0);

        assert_abs_diff_eq!(altitude_m(&pos, Some(&planet)), -100.0, epsilon = EPS)
    }

    /// The subtraction must happen in `Real`. A cast to `f32` before the
    /// subtraction gives an error of about 0.5 m at this radius.
    #[test]
    #[cfg(feature = "f64")]
    fn altitude_round_f64_precision() {
        let planet = PlanetResource::new(RVector::default(), EARTH_RADIUS_M);
        let distance_from_center = EARTH_RADIUS_M + 500.0;
        let angle: Real = 0.7;
        let pos = Position::from_xyz(
            distance_from_center * angle.cos(),
            distance_from_center * angle.sin(),
            0.0,
        );

        assert_abs_diff_eq!(altitude_m(&pos, Some(&planet)), 500.0, epsilon = 1e-3)
    }
}
