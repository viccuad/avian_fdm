//! [`GizmoShape`], optional visual override for debug wireframe drawing.
//!
//! ## Hybrid visualisation approach
//!
//! Most zones are drawn directly from their [`avian3d::prelude::Collider`]
//! shape, no `GizmoShape` needed. This ensures the debug wireframe always
//! matches what the physics engine sees.
//!
//! Attach `GizmoShape` **only** when the collider shape doesn't represent the
//! desired visual:
//!
//! | Use case              | Example           | GizmoShape variant |
//! |-----------------------|-------------------|--------------------|
//! | Tapered surface       | Vertical fin      | `Quad`             |
//! | Line-like structure   | Wing strut        | `Strut`            |
//! | Different shape class | Engine cowl       | `Cylinder`         |
//! | Round instead of box  | Wheel             | `Sphere`           |
//!
//! Aerodynamic surfaces (wings, h-stab, ailerons, elevator) use thin colliders
//! (~ 2 cm) whose cuboid outline naturally looks like a flat panel, they need
//! no `GizmoShape` at all.
//!
//! ## Contour detail
//!
//! [`GizmoContours`] adds arbitrary linestrips to a zone for visual detail
//! beyond what the collider shape shows, curved fuselage profiles, airfoil
//! cross-sections, rounded wingtips. These are purely decorative but are
//! zone-aware: they inherit damage colouring and disappear when the zone is
//! destroyed.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

fn default_cylinder_axis() -> Vec3 {
    Vec3::X
}

/// Describes how a zone entity should be drawn in the gizmo debug view.
///
/// This is purely visual, it has no effect on physics or aerodynamics.
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
    /// Cylinder aligned along a local axis.
    Cylinder {
        /// Radius (metres).
        radius: f32,
        /// Length along the axis (metres).
        length: f32,
        /// Local-frame axis the cylinder is aligned along. Default: `Vec3::X` (forward).
        #[serde(default = "default_cylinder_axis")]
        axis: Vec3,
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

/// Additional linestrip contours for detailed aircraft outline rendering.
///
/// Each entry is a polyline (sequence of points in zone-local coordinates)
/// drawn as a connected linestrip. Use this to add curved profiles,
/// cross-section rings, airfoil sections, or any detail that the collider
/// shape doesn't capture.
///
/// Contours inherit the zone's damage colour, they fade toward red as
/// health decreases and vanish when the zone is destroyed.
///
/// # Example
/// ```rust
/// # use avian_fdm::components::gizmo_shape::GizmoContours;
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
    /// Each entry is a polyline drawn as a connected linestrip.
    pub lines: Vec<Vec<Vec3>>,
}
