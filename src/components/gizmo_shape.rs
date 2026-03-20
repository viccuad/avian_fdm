//! [`GizmoShape`] — optional visual representation for debug wireframe drawing.
//!
//! Attach to any zone entity alongside [`super::AeroZone`] or
//! [`super::EngineZone`] to control how it appears in the gizmo debug view.
//! Zones without a `GizmoShape` are drawn using their [`avian3d::prelude::Collider`]
//! extents as a fallback cuboid.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// Describes how a zone entity should be drawn in the gizmo debug view.
///
/// This is purely visual — it has no effect on physics or aerodynamics.
/// Multiple shapes can approximate curved surfaces (fuselage cross-sections,
/// rounded wingtips) that the physics collider doesn't need to model.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub enum GizmoShape {
    /// Axis-aligned box (full extents in metres).
    Box {
        /// Full extent along local X (forward).
        x: f32,
        /// Full extent along local Y (right).
        y: f32,
        /// Full extent along local Z (down).
        z: f32,
    },
    /// Cylinder aligned along local X (forward axis).
    Cylinder {
        /// Radius (metres).
        radius: f32,
        /// Length along local X (metres).
        length: f32,
    },
    /// Cone pointing along local +X (forward), base at −X.
    Cone {
        /// Base radius (metres).
        radius: f32,
        /// Length along local X (metres).
        length: f32,
    },
    /// Sphere.
    Sphere {
        /// Radius (metres).
        radius: f32,
    },
    /// Flat quadrilateral outline defined by four corners in local frame.
    /// Used for tapered fins, control surfaces, or any non-rectangular panel.
    Quad {
        /// Four corner points in local coordinates, drawn as a closed loop.
        corners: [Vec3; 4],
    },
    /// Thin strut drawn as a single line from `start` to `end` in local frame.
    Strut {
        /// Start point in local coordinates.
        start: Vec3,
        /// End point in local coordinates.
        end: Vec3,
    },
}
