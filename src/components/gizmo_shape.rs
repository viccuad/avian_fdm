//! [`GizmoShape`] — optional visual override for debug wireframe drawing.
//!
//! ## Hybrid visualisation approach
//!
//! Most zones are drawn directly from their [`avian3d::prelude::Collider`]
//! shape — no `GizmoShape` needed. This ensures the debug wireframe always
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
//! (≈ 2 cm) whose cuboid outline naturally looks like a flat panel — they need
//! no `GizmoShape` at all.

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
