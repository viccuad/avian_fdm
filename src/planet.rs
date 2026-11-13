//! Planet geometry: where sea level is, and the altitude above it.
//!
//! If no [`PlanetResource`] exists, the world is flat. Sea level is at `y = 0`,
//! and the altitude is the world Y position. If a [`PlanetResource`] exists,
//! the planet is a sphere, and the altitude is the distance from its center
//! minus its radius.
//!
//! This module is geometry only. It does not describe the air. The atmosphere
//! takes the geometric altitude from [`altitude_m`], and then applies the
//! standard atmosphere. The ISA converts the geometric altitude to geopotential
//! altitude with its own fixed radius, not with [`PlanetResource::radius_m`].
//! Refer to [`crate::atmosphere::isa`].
//!
//! [`altitude_m`], geometric altitude, is the only source of the altitude in
//! the crate.
//! [`crate::atmosphere::update_atmosphere`] and
//! [`crate::kinematics::update_flight_state`] both call it, so
//! [`AtmosphereState`](crate::components::AtmosphereState) and
//! [`FlightState::altitude_m`](crate::components::FlightState::altitude_m)
//! always agree.

use crate::_bevy::*;
use avian3d::math::{RVector, Real, ToF32Precision};
use avian3d::prelude::Position;
use bevy_math::DVec3;
use serde::{Deserialize, Serialize};

/// Optional resource for a spherical planet. Insert it into the Bevy
/// [`World`] to calculate the altitude of all aircraft above a sphere.
///
/// The altitude is the distance from `center` minus `radius_m`. This is
/// called geometric altitude, in contrast with the geopotential altitude
/// used in the atmospheric calculations.
/// [`AtmosphereState`](crate::components::AtmosphereState) and
/// [`FlightState::altitude_m`](crate::components::FlightState::altitude_m)
/// both use this altitude. If this resource is absent, the world is flat, sea
/// level is at `y = 0`, and the altitude is the world Y position.
///
/// # The planet is geometry only
///
/// This resource answers one question: how high is the aircraft above sea
/// level? The atmosphere model itself does not change with the planet. The
/// same altitude gives the same air, with or without this resource, for any
/// `radius_m`. In particular, the ISA converts the altitude to geopotential
/// altitude with the fixed ICAO radius (6,356,766 m), never with `radius_m`.
///
/// `radius_m` still matters: it decides where sea level is. If it does not
/// match the distance from `center` to the ground of your world, every
/// altitude is offset by that mismatch, and every aircraft flies in air of
/// the wrong density.
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::PlanetResource;
/// # use bevy::prelude::*;
/// # use avian3d::math::RVector;
/// # let mut app = App::new();
/// // Sea level at y = 0, with the planet center below the origin.
/// app.insert_resource(PlanetResource {
///     center: RVector::new(0.0, -PlanetResource::EARTH_RADIUS_M, 0.0),
///     radius_m: PlanetResource::EARTH_RADIUS_M,
/// });
/// ```
#[derive(Resource, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Resource, Serialize, Deserialize)]
pub struct PlanetResource {
    /// The center of the planet in world space (m).
    pub center: RVector,
    /// The distance from the center to sea level (m). It defines where the
    /// altitude is zero. The ISA does not use it (see "The planet is
    /// geometry only" above).
    pub radius_m: Real,
}

impl Default for PlanetResource {
    fn default() -> Self {
        Self {
            center: RVector::new(0.0, -Self::EARTH_RADIUS_M, 0.0),
            radius_m: Self::EARTH_RADIUS_M,
        }
    }
}

impl PlanetResource {
    /// Earth radius of the Bevy sky renderer (m). Use it as `radius_m` when you
    /// choose your own `center`. The default planet puts the center at
    /// `(0, -EARTH_RADIUS_M, 0)`, so that sea level is at `y = 0`.
    pub const EARTH_RADIUS_M: Real = sourced!(
        6_360_000.0,
        "Bevy 0.20 Atmosphere::earth EARTH_INNER_RADIUS"
    );
}

/// Calculates the geometric altitude above sea level (m). If `planet` is `None`, the altitude is
/// the world Y position.
///
/// With a planet, the function calculates the distance to the center in f64 in all builds. If
/// calculated in f32, the resolution at this distance would be only 0.5 m.
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
    use super::{altitude_m, PlanetResource};
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
