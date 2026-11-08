//! [`WindResource`]: optional global wind for all aircraft.

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
/// // app.insert_resource(WindResource { velocity_world_ms: Vec3::new(-5.0, 0.0, 0.0) });
/// ```
#[derive(Resource, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Resource, Serialize, Deserialize)]
pub struct WindResource {
    /// Ambient wind velocity in the world frame (m/s).
    /// Positive X = the wind blows in the world +X direction.
    pub velocity_world_ms: Vec3,
}

/// Optional resource for a round earth. Insert it into the Bevy
/// [`World`] make the atmosphere spherical.
///
/// If this resource is absent, the atmosphere will default to calculating flat-earth altitude.
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::PlanetResource;
/// # use bevy::prelude::*;
/// # use avian3d::math::RVector;
/// // app.insert_resource(PlanetResource { center: RVector::new(0.0, -6_360_000.0, 0.0), radius_m: 6_360_000.0 });
/// ```
#[derive(Resource, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Resource, Serialize, Deserialize)]
pub struct PlanetResource {
    /// The center of the earth in world space (m).
    pub center: RVector,
    /// The distance from the center to sea level (m).
    pub radius_m: Real,
}

impl Default for PlanetResource {
    fn default() -> Self {
        const EARTH_RADIUS_M: Real = sourced!(
            6_360_000.0,
            "Bevy 0.20 Atmosphere::earth EARTH_INNER_RADIUS"
        );
        Self {
            center: RVector::new(0.0, -EARTH_RADIUS_M, 0.0),
            radius_m: EARTH_RADIUS_M,
        }
    }
}
