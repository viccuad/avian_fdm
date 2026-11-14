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
/// # let mut app = App::new();
/// // Sea level at y = 0, with the planet center below the origin.
/// app.insert_resource(PlanetResource::earth());
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
    /// The gravity at sea level (m/s^2). [`PlanetResource::gravity_at`] uses
    /// it. The ISA does not use it: the ISA has its own fixed gravity.
    pub surface_gravity_ms2: f32,
}

impl Default for PlanetResource {
    fn default() -> Self {
        Self::earth()
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

    /// Standard gravity of the Earth at sea level (m/s²).
    pub const EARTH_SURFACE_GRAVITY_MS2: f32 =
        sourced!(9.806_65, "ICAO Doc 7488 standard gravity g0");

    /// The Earth, with sea level at the world origin.
    ///
    /// - The center is at `(0, -EARTH_RADIUS_M, 0)`, so sea level is at
    ///   `y = 0` and the aircraft stays near the origin.
    /// - The radius is the radius of the Bevy sky renderer (6,360 km), not the
    ///   mean Earth radius (6,371 km). If the ground of your world is at a
    ///   different distance from the center, change `radius_m`. Otherwise,
    ///   every altitude is wrong by the difference.
    /// - The surface gravity is the standard gravity (9.806 65 m/s²).
    ///
    /// [`Default`] gives the same value.
    pub fn earth() -> Self {
        Self {
            center: RVector::new(0.0, -Self::EARTH_RADIUS_M, 0.0),
            radius_m: Self::EARTH_RADIUS_M,
            surface_gravity_ms2: Self::EARTH_SURFACE_GRAVITY_MS2,
        }
    }

    /// Gravity at the world position `pos` (m/s^2). It points to the planet
    /// center. Its size is `surface_gravity_ms2 * (radius_m / r)^2`, where `r`
    /// is the distance from the center (point-mass model). At the center, it
    /// is zero.
    pub fn gravity_at(&self, pos: RVector) -> Vec3 {
        // This calculation does not need f64, unlike `altitude_m`. The altitude
        // subtracts two large, nearly equal values (the distance to the center
        // minus the radius), so f32 loses the leading digits. Gravity only
        // divides: the direction `offset / |offset|` and the ratio `R / r` keep
        // the relative precision of f32 (about 1e-7). With the default planet in
        // an f32 build, the offset is about 6,361 km and rounds to 0.5 m. That
        // gives a direction error of about 8e-8 rad. Thus the function subtracts
        // in `Real` and converts once to `Vec3`, the type of Avian accelerations.
        let offset = (pos - self.center).f32();
        let Some(down) = (-offset).try_normalize() else {
            return Vec3::ZERO;
        };
        down * self.surface_gravity_ms2 * (self.radius_m.f32() / offset.length()).powi(2)
    }
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
        math::{RVector, Real, ToF32Precision},
        physics_transform::Position,
    };
    use bevy_math::Vec3;

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
        let planet = PlanetResource::earth();
        let pos = Position::from_xyz(0.0, 1000.3, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.3, EPS)
    }

    #[test]
    fn altitude_round_y_axis() {
        let planet = PlanetResource {
            center: RVector::ZERO,
            ..PlanetResource::earth()
        };
        let pos = Position::from_xyz(0.0, EARTH_RADIUS_M + 1000.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.0, EPS)
    }

    #[test]
    fn altitude_round_x_axis() {
        let planet = PlanetResource {
            center: RVector::ZERO,
            ..PlanetResource::earth()
        };
        let pos = Position::from_xyz(EARTH_RADIUS_M + 1000.0, 0.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.0, EPS)
    }

    #[test]
    fn altitude_round_offset_center() {
        let planet = PlanetResource::earth();
        let pos = Position::from_xyz(0.0, 1000.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), 1000.0, EPS)
    }

    #[test]
    fn altitude_round_below_sea_level() {
        let planet = PlanetResource::earth();
        let pos = Position::from_xyz(0.0, -100.0, 0.0);

        assert_near(altitude_m(&pos, Some(&planet)), -100.0, EPS)
    }

    /// The subtraction must happen in `Real`. A cast to `f32` before the
    /// subtraction gives an error of about 0.5 m at this radius.
    #[test]
    #[cfg(feature = "f64")]
    fn altitude_round_f64_precision() {
        let planet = PlanetResource {
            center: RVector::ZERO,
            ..PlanetResource::earth()
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

    const G0: f32 = PlanetResource::EARTH_SURFACE_GRAVITY_MS2;

    /// Point-mass gravity at `altitude_m` above sea level.
    fn expected_g(altitude_m: f32) -> f32 {
        let r = EARTH_RADIUS_M.f32();
        G0 * (r / (r + altitude_m)).powi(2)
    }

    fn assert_vec_near(actual: Vec3, expected: Vec3, eps: f32) {
        assert!(
            (actual - expected).length() <= eps,
            "actual {actual}, expected {expected}, tolerance {eps}"
        );
    }

    #[test]
    fn default_is_earth() {
        let default = PlanetResource::default();
        let earth = PlanetResource::earth();
        assert_eq!(default.center, earth.center);
        assert_eq!(default.radius_m, earth.radius_m);
        assert_eq!(default.surface_gravity_ms2, earth.surface_gravity_ms2);
    }

    #[test]
    fn gravity_points_to_center_y() {
        let planet = PlanetResource {
            center: RVector::ZERO,
            ..PlanetResource::earth()
        };
        let pos = RVector::new(0.0, EARTH_RADIUS_M + 1000.0, 0.0);

        let expected = Vec3::new(0.0, -expected_g(1000.0), 0.0);
        assert_vec_near(planet.gravity_at(pos), expected, 1e-5)
    }

    #[test]
    fn gravity_points_to_center_x() {
        let planet = PlanetResource {
            center: RVector::ZERO,
            ..PlanetResource::earth()
        };
        let pos = RVector::new(EARTH_RADIUS_M + 1000.0, 0.0, 0.0);

        let expected = Vec3::new(-expected_g(1000.0), 0.0, 0.0);
        assert_vec_near(planet.gravity_at(pos), expected, 1e-5)
    }

    #[test]
    fn gravity_at_surface() {
        let planet = PlanetResource::earth();

        let g = planet.gravity_at(RVector::ZERO);
        assert_vec_near(g, Vec3::new(0.0, -G0, 0.0), 1e-5)
    }

    #[test]
    fn gravity_at_center() {
        let planet = PlanetResource::earth();

        let g = planet.gravity_at(planet.center);
        assert_eq!(g, Vec3::ZERO);
    }

    /// Off the axis, gravity still points from the position to the center.
    #[test]
    fn gravity_direction_off_axis() {
        let planet = PlanetResource::earth();
        let pos = RVector::new(30_000.0, 1000.0, -20_000.0);

        let down = planet.gravity_at(pos).normalize();
        let expected = (planet.center - pos).normalize().f32();
        // Compare the unit vectors, not the angle. `Vec3::angle_between` uses
        // `acos` of the dot product. Near a dot product of 1, the smallest step of
        // f32 gives an angle of about 3.5e-4 rad. Thus `acos` returns either 0 or
        // a value far above a tolerance of 1e-6, and the test passes or fails by
        // rounding. For small angles, the length of the difference of two unit
        // vectors is equal to the angle, with no such step.
        assert!(
            (down - expected).length() < 1e-6,
            "down {down}, expected {expected}"
        );
    }
}
