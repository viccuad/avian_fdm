//! Debug gizmo systems for FDM overlays.
//!
//! Each `debug_render_*` function is a Bevy system added by [`AircraftFdmDebugPlugin`].
//! They run in `PostUpdate`, after transform propagation, and are gated by the
//! `FdmGizmos` config group being enabled.
//!
//! [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin

use avian3d::prelude::{ComputedCenterOfMass, Rotation};
use crate::_bevy::*;

use crate::components::{AeroZone, AircraftGeometry};
use super::configuration::FdmGizmos;

// ── Centre of gravity ─────────────────────────────────────────────────────────

/// Draw a sphere at the aircraft's centre of gravity (CG).
///
/// Uses Avian's [`ComputedCenterOfMass`] (local offset from entity origin) and
/// [`Rotation`] (physics rotation) to compute the world-space CG position.
pub(super) fn debug_render_cg(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(&GlobalTransform, &Rotation, &ComputedCenterOfMass), With<AircraftGeometry>>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.cg_color else { return };

    for (gt, rot, com) in &query {
        let cg = gt.translation() + rot.0 * com.0;
        gizmos.sphere(Isometry3d::from_translation(cg), config.marker_radius, color);
    }
}

// ── Aerodynamic centres ───────────────────────────────────────────────────────

/// Draw a sphere at each zone's aerodynamic centre (AC).
///
/// The AC is stored as [`AeroZone::ac_offset`] in the zone's local frame.
/// `GlobalTransform::transform_point` converts it to world space.
///
/// Rendered as a 3-axis cross (±X/Y/Z arms) so it remains clearly visible
/// even when the AC coincides with the zone entity origin, where a sphere
/// wireframe would blend into the zone outlines.
pub(super) fn debug_render_ac(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(&GlobalTransform, &AeroZone)>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.ac_color else { return };

    let arm = config.marker_radius;
    for (gt, zone) in &query {
        let ac = gt.transform_point(zone.ac_offset);
        gizmos.line(ac - Vec3::X * arm, ac + Vec3::X * arm, color);
        gizmos.line(ac - Vec3::Y * arm, ac + Vec3::Y * arm, color);
        gizmos.line(ac - Vec3::Z * arm, ac + Vec3::Z * arm, color);
    }
}

// ── Force arrows ──────────────────────────────────────────────────────────────

/// Draw per-zone lift, drag, and side-force arrows.
pub(super) fn debug_render_zone_forces(
    mut _gizmos: Gizmos<FdmGizmos>,
    _store: Res<GizmoConfigStore>,
) {
    // TODO(debug-plugin): query ZoneForce + GlobalTransform on AeroZone entities;
    // draw arrows scaled by store.config::<FdmGizmos>().1.force_scale.
}

/// Draw thrust arrows on engine zones.
pub(super) fn debug_render_thrust(
    mut _gizmos: Gizmos<FdmGizmos>,
    _store: Res<GizmoConfigStore>,
) {
    // TODO(debug-plugin): query ZoneForce on EngineZone entities (propulsion feature).
}

/// Draw the resultant aerodynamic + thrust force arrow on the aircraft root.
pub(super) fn debug_render_resultant(
    mut _gizmos: Gizmos<FdmGizmos>,
    _store: Res<GizmoConfigStore>,
) {
    // TODO(debug-plugin): query accumulated ExternalForce on aircraft root entities.
}

// ── Moment arcs ───────────────────────────────────────────────────────────────

/// Draw pitch / roll / yaw moment arcs centred on the CG.
pub(super) fn debug_render_moments(
    mut _gizmos: Gizmos<FdmGizmos>,
    _store: Res<GizmoConfigStore>,
) {
    // TODO(debug-plugin): query ExternalTorque on aircraft root; draw arcs around
    // each body axis proportional to moment magnitude.
}

// ── Zone health wireframes ────────────────────────────────────────────────────

/// Draw zone collider wireframes, tinted green to red by Failure::remaining.
pub(super) fn debug_render_zones(
    mut _gizmos: Gizmos<FdmGizmos>,
    _store: Res<GizmoConfigStore>,
) {
    // TODO(debug-plugin): query AeroZone + Collider + GlobalTransform + Option<Failure>
    // + Option<FdmDebugRender>; lerp zone_color from green (remaining=1) to red (0).
}

// ── Angle-of-attack / wind indicator ─────────────────────────────────────────

/// Draw the relative-wind arrow and angle-of-attack arc.
pub(super) fn debug_render_wind(
    mut _gizmos: Gizmos<FdmGizmos>,
    _store: Res<GizmoConfigStore>,
) {
    // TODO(debug-plugin): query FlightState on aircraft root; draw wind arrow from
    // CG in the freestream direction; draw AoA arc between body-X and wind vector.
}
