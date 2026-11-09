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
use bevy_math::DVec3;

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

/// Calculates the altitude from `PlanetResource`, or, if it's absent, the y-position in world
/// space.
///
/// With a planet, the distance to the center is about 6.36e6 m. At this size, `f32` has a
/// resolution of 0.5 m. The function calculates in `f64` in all builds, so the altitude keeps the
/// precision of the position. In the `f64` build, the conversions do nothing.
pub(crate) fn altitude_m(pos: &Position, planet: Option<&PlanetResource>) -> f32 {
    match planet {
        Some(p) => {
            #[allow(
                clippy::useless_conversion,
                reason = "The conversions to f64 are identities only in the f64 build"
            )]
            let (pos, center, radius) = (
                DVec3::from(pos.0),
                DVec3::from(p.center),
                f64::from(p.radius_m),
            );
            ((pos - center).length() - radius) as f32
        }
        None => pos.0.y.f32(),
    }
}

#[cfg(test)]
mod tests {
    use crate::atmosphere::{systems::altitude_m, PlanetResource};
    use avian3d::{
        math::{RVector, Real},
        physics_transform::Position,
    };

    /// `altitude_m` calculates in `f64` in both builds, and the test positions
    /// are exact in `f32`. Thus both builds use the same tolerance.
    const EPS: f32 = 1e-3;
    const EARTH_RADIUS_M: Real = PlanetResource::EARTH_RADIUS_M;

    fn assert_near(actual: f32, expected: f32, eps: f32) {
        assert!(
            (actual - expected).abs() <= eps,
            "actual {actual}, expected {expected}, tolerance {eps}"
        );
    }

    #[test]
    fn altitude_flat_world() {
        assert_eq!(
            altitude_m(&Position::from_xyz(123.0, 1000.0, -45.0), None),
            1000.0
        )
    }

    /// With the default planet, the aircraft is near the world origin, so its
    /// `f32` position is precise. The altitude must keep that precision, even
    /// though the distance to the planet center is about 6.36e6 m.
    #[test]
    fn altitude_round_default_planet_keeps_precision() {
        let planet = PlanetResource::default();
        let pos = Position::from_xyz(0.0, 1000.3, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.3, EPS)
    }

    #[test]
    fn altitude_round_y_axis() {
        let planet = PlanetResource {
            center: RVector::default(),
            radius_m: EARTH_RADIUS_M,
        };
        let pos = Position::from_xyz(0.0, EARTH_RADIUS_M + 1000.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.0, EPS)
    }

    #[test]
    fn altitude_round_x_axis() {
        let planet = PlanetResource {
            center: RVector::default(),
            radius_m: EARTH_RADIUS_M,
        };
        let pos = Position::from_xyz(EARTH_RADIUS_M + 1000.0, 0.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.0, EPS)
    }

    #[test]
    fn altitude_round_offset_center() {
        let center = RVector::new(0.0, -EARTH_RADIUS_M, 0.0);
        let planet = PlanetResource {
            center,
            radius_m: EARTH_RADIUS_M,
        };

        let pos = Position::from_xyz(0.0, 1000.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.0, EPS)
    }

    #[test]
    fn altitude_round_below_sea_level() {
        let planet = PlanetResource::default();
        let pos = Position::from_xyz(0.0, -100.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), -100.0, EPS)
    }

    /// The subtraction must happen in `Real`. A cast to `f32` before the
    /// subtraction gives an error of about 0.5 m at this radius.
    #[test]
    #[cfg(feature = "f64")]
    fn altitude_round_f64_precision() {
        let planet = PlanetResource {
            center: RVector::default(),
            radius_m: EARTH_RADIUS_M,
        };
        let distance_from_center = EARTH_RADIUS_M + 500.0;
        let angle: Real = 0.7;
        let pos = Position::from_xyz(
            distance_from_center * angle.cos(),
            distance_from_center * angle.sin(),
            0.0,
        );

        assert_near(altitude_m(&pos, Some(&planet)), 500.0, 1e-3)
    }
}
