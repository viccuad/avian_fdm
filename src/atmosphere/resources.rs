//! [`WindResource`]: optional global wind for all aircraft.

use crate::_bevy::*;
use avian3d::math::Vector;
use serde::{Deserialize, Serialize};

/// Optional uniform ambient wind resource. Insert into the Bevy [`World`] to
/// add a global wind to all aircraft.
///
/// If absent, relative wind = aircraft velocity only. Per-entity or
/// altitude-varying wind is a post-v1 feature (see Group D roadmap).
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
    /// Ambient wind velocity in world frame (m/s).
    /// Positive X = wind blowing in world +X direction.
    pub velocity_world_ms: Vector,
}
