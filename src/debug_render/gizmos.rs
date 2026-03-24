//! Debug gizmo systems for FDM overlays.
//!
//! Each `debug_render_*` function is a Bevy system added by [`AircraftFdmDebugPlugin`].
//! They run in `PostUpdate`, after transform propagation, and are gated by the
//! `FdmGizmos` config group being enabled.
//!
//! [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin

use avian3d::prelude::{ComputedCenterOfMass, ComputedMass, ConstantForce, Rotation};
use crate::_bevy::*;

use crate::components::{AeroZone, AircraftGeometry, GizmoContours, GizmoShape, ZoneForce};
use super::configuration::{FdmDebugRender, FdmGizmos};

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

/// Draw zone outline wireframes using [`GizmoShape`] and [`GizmoContours`].
///
/// Color is chosen by zone type unless overridden by [`FdmDebugRender`]:
/// - Green: control surface (non-zero `control_role`)
/// - Cyan: lifting surface (non-zero CL at a sample alpha)
/// - Orange: non-aero zone (e.g. engine)
/// - Grey: structural / drag-only
///
/// The global [`FdmGizmos::zone_color`] acts as an on/off gate; set to `None`
/// to disable. Per-entity [`FdmDebugRender::zone_color`] overrides the color.
pub(super) fn debug_render_zones(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(
        &GlobalTransform,
        Option<&AeroZone>,
        Option<&GizmoContours>,
        Option<&GizmoShape>,
        Option<&FdmDebugRender>,
    ), Or<(With<GizmoShape>, With<GizmoContours>)>>,
) {
    let config = store.config::<FdmGizmos>().1;
    if config.zone_color.is_none() {
        return;
    }

    for (zone_gt, aero, contours, shape, dbg_render) in &query {
        let color = dbg_render
            .and_then(|d| d.zone_color)
            .unwrap_or_else(|| zone_type_color(aero));

        let wt = zone_gt.compute_transform();
        let zone_to_world = |local: Vec3| wt.translation + wt.rotation * local;
        let iso_at = |extra_rot: Quat| {
            Isometry3d::new(wt.translation, wt.rotation * extra_rot)
        };

        if let Some(contour_data) = contours {
            for line in &contour_data.lines {
                if line.len() < 2 {
                    continue;
                }
                let pts: Vec<Vec3> = line.iter().map(|p| zone_to_world(*p)).collect();
                gizmos.linestrip(pts, color);
            }
            if shape.is_none() {
                continue;
            }
        }

        if let Some(gs) = shape {
            draw_zone_shape(&mut gizmos, gs, &iso_at, &zone_to_world, color);
        }
    }
}

fn zone_type_color(aero: Option<&AeroZone>) -> Color {
    if aero.is_none() {
        Color::srgba(0.95, 0.65, 0.15, 0.9) // orange - engine / non-aero
    } else if aero.is_some_and(|a| a.control_role.is_some()) {
        Color::srgba(0.3, 1.0, 0.5, 0.7) // green - control surface
    } else if aero.is_some_and(|a| a.cl.evaluate(0.1, 2e6).abs() > 0.01) {
        Color::srgba(0.4, 0.85, 1.0, 0.7) // cyan - lifting surface
    } else {
        Color::srgba(0.6, 0.6, 0.6, 0.6) // grey - structural / drag-only
    }
}

fn draw_zone_shape(
    gizmos: &mut Gizmos<FdmGizmos>,
    shape: &GizmoShape,
    iso_at: &impl Fn(Quat) -> Isometry3d,
    zone_to_world: &impl Fn(Vec3) -> Vec3,
    color: Color,
) {
    match shape {
        GizmoShape::Box { x, y, z } => {
            gizmos.primitive_3d(&Cuboid::new(*x, *y, *z), iso_at(Quat::IDENTITY), color);
        }
        GizmoShape::Cylinder { radius, length, axis } => {
            gizmos
                .primitive_3d(
                    &Cylinder::new(*radius, *length),
                    iso_at(Quat::from_rotation_arc(Vec3::Y, *axis)),
                    color,
                )
                .resolution(32);
        }
        GizmoShape::Cone { radius, length } => {
            gizmos.primitive_3d(
                &Cone { radius: *radius, height: *length },
                iso_at(Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2)),
                color,
            );
        }
        GizmoShape::Sphere { radius } => {
            gizmos
                .primitive_3d(
                    &bevy_math::primitives::Sphere::new(*radius),
                    iso_at(Quat::IDENTITY),
                    color,
                )
                .resolution(32);
        }
        GizmoShape::Quad { corners } => {
            let pts: Vec<Vec3> = corners
                .iter()
                .chain(std::iter::once(&corners[0]))
                .map(|c| zone_to_world(*c))
                .collect();
            gizmos.linestrip(pts, color);
        }
        GizmoShape::Strut { start, end } => {
            gizmos.line(zone_to_world(*start), zone_to_world(*end), color);
        }
    }
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
