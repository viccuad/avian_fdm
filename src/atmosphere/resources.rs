//! [`WindResource`]: optional global wind for all aircraft.
//! [`PlanetResource`]: optional global resource, enables round planet altitude for all aircraft.

use crate::_bevy::*;
use avian3d::math::{RVector, Real};
use serde::{Deserialize, Serialize};

/// Optional resource for a uniform ambient wind. Insert it into the Bevy
/// [`World`] to add a global wind to all aircraft.
///
/// If this resource is absent, the relative wind comes only from the aircraft
/// velocity. Wind for each entity, or wind that changes with altitude, is
/// planned for a version after v1 (Group D of the roadmap).
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::WindResource;
/// # use bevy::prelude::*;
/// # let mut app = App::new();
/// app.insert_resource(WindResource {
///     velocity_world_ms: Vec3::new(-5.0, 0.0, 0.0),
/// });
/// ```
#[derive(Resource, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Resource, Serialize, Deserialize)]
pub struct WindResource {
    /// Ambient wind velocity in the world frame (m/s).
    /// Positive X = the wind blows in the world +X direction.
    pub velocity_world_ms: Vec3,
}

/// Optional resource for a spherical planet. Insert it into the Bevy
/// [`World`] to calculate the altitude of all aircraft above a sphere.
///
/// The altitude is the distance from `center` minus `radius_m`.
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
