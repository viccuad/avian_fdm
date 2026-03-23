//! Debug gizmo systems for FDM overlays.
//!
//! Each `debug_render_*` function is a Bevy system added by [`AircraftFdmDebugPlugin`].
//! They run in `PostUpdate`, after transform propagation, and are gated by the
//! `FdmGizmos` config group being enabled.
//!
//! [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin

use bevy::prelude::*;

use super::configuration::FdmGizmos;

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

/// Draw zone collider wireframes, tinted green→red by Failure::remaining.
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
