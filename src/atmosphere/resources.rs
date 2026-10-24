//! [`WindResource`]: optional global wind for all aircraft.

use crate::_bevy::*;
use avian3d::math::Vector;
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
/// # use avian3d::math::Vector;
/// // app.insert_resource(WindResource { velocity_world_ms: Vector::new(-5.0, 0.0, 0.0) });
/// ```
#[derive(Resource, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Resource, Serialize, Deserialize)]
pub struct WindResource {
    /// Ambient wind velocity in the world frame (m/s).
    /// Positive X = the wind blows in the world +X direction.
    pub velocity_world_ms: Vector,
}
