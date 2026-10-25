//! [`GizmoShape`]: the visual shape of a zone in the debug wireframe view.
//!
//! # Module location
//!
//! This file is in `src/debug_render/`, but the crate root declares it (with
//! `#[path]`) **without** the `debug-plugin` feature gate. The core plugin,
//! [`crate::plugin::AircraftFdmPlugin`], always calls
//! `.register_type::<GizmoShape>()`. Thus, these components stay
//! scene-serializable in builds without `debug-plugin`. You can keep
//! `GizmoShape` values in scene files, and enable
//! [`crate::debug_render::AircraftFdmDebugPlugin`] only in development builds.
//!
//! ## Two wireframe views
//!
//! The debug plugin has two separate wireframe views:
//!
//! - **Zone wireframes** draw only the zones that have a `GizmoShape` or a
//!   [`GizmoContours`]. The color shows the zone type and the damage.
//! - **Collider wireframes** draw the raw [`avian3d::prelude::Collider`]
//!   shape of each zone, when [`crate::debug_render::ShowColliders`] is
//!   `true`. Use this view to make sure that the `GizmoShape` agrees with the
//!   physics collider.
//!
//! Select the `GizmoShape` variant that looks most like the part:
//!
//! | Use case              | Example                       | GizmoShape variant |
//! |-----------------------|-------------------------------|--------------------|
//! | Flat panel            | Wing, horizontal stabilizer   | `Box`              |
//! | Tapered surface       | Vertical fin                  | `Quad`             |
//! | Line-like structure   | Wing strut                    | `Strut`            |
//! | Round body            | Engine cowl                   | `Cylinder`         |
//! | Round instead of box  | Wheel                         | `Sphere`           |
//!
//! ## Contour detail
//!
//! [`GizmoContours`] adds free linestrips to a zone. They give more visual
//! detail than the shape: curved fuselage profiles, airfoil cross-sections,
//! and rounded wing tips. They are only decorative, but they belong to the
//! zone. They get the same damage color as the zone.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

fn default_cylinder_axis() -> Vec3 {
    Vec3::X
}

/// Sets how the gizmo debug view draws a zone entity.
///
/// It is only visual. It has no effect on the physics or the aerodynamics.
/// You can use it to show a shape that the physics collider does not need to
/// model.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub enum GizmoShape {
    /// Box aligned with the local axes (full extents in meters).
    Box {
        /// Full extent along local X (forward).
        x: f32,
        /// Full extent along local Y (right).
        y: f32,
        /// Full extent along local Z (down).
        z: f32,
    },
    /// Cylinder aligned along a local axis.
    Cylinder {
        /// Radius (meters).
        radius: f32,
        /// Length along the axis (meters).
        length: f32,
        /// Axis of the cylinder in the local frame. Default: `Vec3::X` (forward).
        #[serde(default = "default_cylinder_axis")]
        axis: Vec3,
    },
    /// Cone that points along local +X (forward), with its base at −X.
    Cone {
        /// Base radius (meters).
        radius: f32,
        /// Length along local X (meters).
        length: f32,
    },
    /// Sphere.
    Sphere {
        /// Radius (meters).
        radius: f32,
    },
    /// Flat quadrilateral outline with four corners in the local frame.
    /// Use it for tapered fins, control surfaces, or other panels that are not rectangular.
    Quad {
        /// Four corner points in local coordinates. The view draws them as a closed loop.
        corners: [Vec3; 4],
    },
    /// Thin strut. The view draws it as one line from `start` to `end` in the local frame.
    Strut {
        /// Start point in local coordinates.
        start: Vec3,
        /// End point in local coordinates.
        end: Vec3,
    },
}

/// More linestrip contours, for a detailed outline of the aircraft.
///
/// Each entry is a polyline (a sequence of points in the local coordinates of
/// the zone). The view draws it as a connected linestrip. Use it to add curved
/// profiles, cross-section rings, airfoil sections, or other details that the
/// collider shape does not show.
///
/// Contours get the damage color of the zone. As the health decreases, they
/// change toward gray. At `remaining = 0.0`, they are fully gray.
///
/// # Example
/// ```rust
/// # use avian_fdm::components::GizmoContours;
/// # use bevy::prelude::*;
/// // Elliptical fuselage cross-section at x = 0
/// let ring: Vec<Vec3> = (0..=12).map(|i| {
///     let angle = i as f32 * std::f32::consts::TAU / 12.0;
///     Vec3::new(0.0, 0.30 * angle.cos(), 0.35 * angle.sin())
/// }).collect();
/// let contours = GizmoContours { lines: vec![ring] };
/// ```
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct GizmoContours {
    /// Each entry is a polyline. The view draws it as a connected linestrip.
    pub lines: Vec<Vec<Vec3>>,
}
