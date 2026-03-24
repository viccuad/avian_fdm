//! Debug gizmo systems for FDM overlays.
//!
//! Each `debug_render_*` function is a Bevy system added by [`AircraftFdmDebugPlugin`].
//! They run in `PostUpdate`, after transform propagation, and are gated by the
//! `FdmGizmos` config group being enabled.
//!
//! [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin

use avian3d::prelude::{ComputedCenterOfMass, ComputedMass, ConstantForce, Rotation};
use crate::_bevy::*;

use crate::components::{AeroZone, AircraftGeometry, ZoneForce};
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

/// Draw a combined aero force arrow per zone, originating from the zone AC.
///
/// Uses [`AeroZone::ac_offset`] and the zone's [`GlobalTransform`] to find the
/// world-space AC. Running in `PostUpdate` after transform propagation means
/// there is no one-frame lag.
pub(super) fn debug_render_zone_forces(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(&ZoneForce, &GlobalTransform, &AeroZone)>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.lift_color else { return };

    for (zf, zone_gt, zone) in &query {
        if zf.force.length_squared() < 100.0 {
            continue;
        }
        let start = zone_gt.transform_point(zone.ac_offset);
        gizmos.arrow(start, start + zf.force * config.force_scale, color);
    }
}

/// Draw thrust arrows on engine zones.
pub(super) fn debug_render_thrust(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    #[cfg(feature = "propulsion")]
    query: Query<(&ZoneForce, &GlobalTransform), With<crate::components::EngineZone>>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.thrust_color else { return };

    #[cfg(feature = "propulsion")]
    for (zf, zone_gt) in &query {
        if zf.force.length_squared() < 1.0 {
            continue;
        }
        let start = zone_gt.translation();
        gizmos.arrow(start, start + zf.force * config.force_scale, color);
    }

    #[cfg(not(feature = "propulsion"))]
    let _ = (gizmos, color);
}

/// Draw the total aero+thrust force, weight, and net-force arrows from the CG.
pub(super) fn debug_render_resultant(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<
        (&Transform, &Rotation, &ConstantForce, &ComputedMass, &ComputedCenterOfMass),
        With<AircraftGeometry>,
    >,
) {
    let config = store.config::<FdmGizmos>().1;

    for (tf, rot, cf, mass, com) in &query {
        let cg = tf.translation + rot.0 * com.0;
        let scale = config.force_scale;

        if let Some(color) = config.total_force_color {
            gizmos.arrow(cg, cg + cf.0 * scale, color);
        }

        if let Some(color) = config.weight_color {
            let weight = Vec3::new(0.0, -mass.value() * 9.806_65, 0.0);
            gizmos.arrow(cg, cg + weight * scale, color);

            if let Some(net_color) = config.resultant_color {
                let net = cf.0 + weight;
                if net.length_squared() > 1.0 {
                    gizmos.arrow(cg, cg + net * scale, net_color);
                }
            }
        }
    }
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
