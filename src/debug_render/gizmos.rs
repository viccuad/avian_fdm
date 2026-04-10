//! Debug gizmo systems for FDM overlays.
//!
//! Each `debug_render_*` function is a Bevy system added by [`AircraftFdmDebugPlugin`].
//! They run in `PostUpdate`, after transform propagation, and are gated by the
//! `FdmGizmos` config group being enabled.
//!
//! [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin

use crate::_bevy::*;
use avian3d::prelude::{
    ComputedCenterOfMass, ComputedMass, ConstantForce, ConstantTorque, Rotation,
};

use super::configuration::{FdmDebugRender, FdmGizmos};
use crate::components::{
    AeroZone, AircraftGeometry, Failure, FlightState, GizmoContours, GizmoShape, ZoneForce,
};

//
// Centre of gravity
//

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
        gizmos.sphere(
            Isometry3d::from_translation(cg),
            config.marker_radius,
            color,
        );
    }
}

//
// Aerodynamic centres
//

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

//
// Force arrows
//

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
    let Some(color) = config.lift_color else {
        return;
    };

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
    query: Query<(&ZoneForce, &GlobalTransform), With<crate::components::EngineZone>>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.thrust_color else {
        return;
    };

    for (zf, zone_gt) in &query {
        if zf.force.length_squared() < 1.0 {
            continue;
        }
        let start = zone_gt.translation();
        gizmos.arrow(start, start + zf.force * config.force_scale, color);
    }
}

/// Draw the total aero+thrust force, weight, and net-force arrows from the CG.
pub(super) fn debug_render_resultant(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<
        (
            &Transform,
            &Rotation,
            &ConstantForce,
            &ComputedMass,
            &ComputedCenterOfMass,
        ),
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

        let weight = Vec3::new(0.0, -mass.value() * 9.806_65, 0.0);

        if let Some(color) = config.weight_color {
            gizmos.arrow(cg, cg + weight * scale, color);
        }

        if let Some(net_color) = config.resultant_color {
            let net = cf.0 + weight;
            if net.length_squared() > 1.0 {
                gizmos.arrow(cg, cg + net * scale, net_color);
            }
        }
    }
}

//
// Moment arcs
//

/// Draw pitch, roll, and yaw moment arrows centered on the CG.
///
/// Each arrow points along the corresponding body axis (X = roll, Y = pitch,
/// Z = yaw in Bevy convention) and is scaled by moment magnitude. Colors are
/// taken from [`FdmGizmos::roll_moment_color`], `pitch_moment_color`, and
/// `yaw_moment_color`.
pub(super) fn debug_render_moments(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<
        (
            &Transform,
            &Rotation,
            &ComputedCenterOfMass,
            &ConstantTorque,
        ),
        With<AircraftGeometry>,
    >,
) {
    let config = store.config::<FdmGizmos>().1;

    for (tf, rot, com, torque) in &query {
        let cg = tf.translation + rot.0 * com.0;
        let scale = config.force_scale;

        // Decompose torque into body-frame components and draw each axis.
        // Torque is already in world frame; project onto body axes for display.
        let t = torque.0;
        if t.length_squared() < 1.0 {
            continue;
        }

        // Body axes in world space.
        let body_x = rot.0 * Vec3::X; // roll axis
        let body_y = rot.0 * Vec3::Y; // pitch axis
        let body_z = rot.0 * Vec3::Z; // yaw axis

        let t_roll = t.dot(body_x);
        let t_pitch = t.dot(body_y);
        let t_yaw = t.dot(body_z);

        if let Some(color) = config.roll_moment_color {
            if t_roll.abs() > 0.1 {
                gizmos.arrow(cg, cg + body_x * t_roll * scale, color);
            }
        }
        if let Some(color) = config.pitch_moment_color {
            if t_pitch.abs() > 0.1 {
                gizmos.arrow(cg, cg + body_y * t_pitch * scale, color);
            }
        }
        if let Some(color) = config.yaw_moment_color {
            if t_yaw.abs() > 0.1 {
                gizmos.arrow(cg, cg + body_z * t_yaw * scale, color);
            }
        }
    }
}

//
// Zone health wireframes
//

/// Draw zone outline wireframes using [`GizmoShape`] and [`GizmoContours`].
///
/// Color is chosen by zone type unless overridden by [`FdmDebugRender`]:
/// - Green: control surface (non-zero `control_role`)
/// - Cyan: lifting surface (non-zero CL at a sample alpha)
/// - Orange: non-aero zone (e.g. engine)
/// - Grey: structural / drag-only
///
/// Zones with a [`Failure`] component are tinted toward grey as damage
/// accumulates. A fully failed zone (`remaining = 0.0`) is drawn grey
/// regardless of its type.
///
/// The global [`FdmGizmos::zone_color`] acts as an on/off gate; set to `None`
/// to disable. Per-entity [`FdmDebugRender::zone_color`] overrides the color
/// (damage tint still applies on top).
#[allow(clippy::unnecessary_cast)]
#[allow(clippy::type_complexity)]
pub(super) fn debug_render_zones(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<
        (
            &GlobalTransform,
            Option<&AeroZone>,
            Option<&GizmoContours>,
            Option<&GizmoShape>,
            Option<&FdmDebugRender>,
            Option<&Failure>,
        ),
        Or<(With<GizmoShape>, With<GizmoContours>)>,
    >,
) {
    let config = store.config::<FdmGizmos>().1;
    if config.zone_color.is_none() {
        return;
    }

    for (zone_gt, aero, contours, shape, dbg_render, failure) in &query {
        let base_color = dbg_render
            .and_then(|d| d.zone_color)
            .unwrap_or_else(|| zone_type_color(aero));
        let remaining = failure.map_or(1.0, |f| f.remaining as f32);
        let color = damage_tint(base_color, remaining);

        let wt = zone_gt.compute_transform();
        let zone_to_world = |local: Vec3| wt.translation + wt.rotation * local;
        let iso_at = |extra_rot: Quat| Isometry3d::new(wt.translation, wt.rotation * extra_rot);

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

/// Blend `color` toward grey proportional to damage (`remaining` 0 -> 1).
///
/// At `remaining = 1.0` the color is unchanged. At `remaining = 0.0` the
/// result is fully grey (`srgba(0.55, 0.55, 0.55, 0.6)`).
fn damage_tint(color: Color, remaining: f32) -> Color {
    let grey = Color::srgba(0.55, 0.55, 0.55, 0.6);
    color.mix(&grey, 1.0 - remaining)
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
        GizmoShape::Cylinder {
            radius,
            length,
            axis,
        } => {
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
                &Cone {
                    radius: *radius,
                    height: *length,
                },
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

//
// Angle-of-attack / wind indicator
//

/// Draw the relative-wind arrow and angle-of-attack / sideslip indicators.
///
/// Draws from the CG:
/// - A wind arrow in the freestream direction (opposite of velocity), length
///   proportional to airspeed, colored by [`FdmGizmos::wind_color`].
/// - A short body-axis forward arrow showing where the nose points.
/// - A label line showing the AoA angle between the two (no text; the angle
///   between nose-arrow and wind-arrow is the visual AoA).
#[allow(clippy::unnecessary_cast)]
pub(super) fn debug_render_wind(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<
        (&Transform, &Rotation, &ComputedCenterOfMass, &FlightState),
        With<AircraftGeometry>,
    >,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.wind_color else {
        return;
    };

    for (tf, rot, com, fs) in &query {
        if fs.airspeed_ms < 1.0 {
            continue;
        }

        let cg = tf.translation + rot.0 * com.0;

        // Body-frame forward (nose direction) in world space.
        let nose_dir = rot.0 * Vec3::X;

        // Freestream direction: the wind comes from opposite the velocity vector.
        // Reconstruct velocity direction from alpha and beta in body frame, then
        // rotate to world. Body-frame velocity components from AoA / sideslip:
        // u = cos(alpha)*cos(beta), v = sin(beta), w = sin(alpha)*cos(beta)
        let (sa, ca) = (fs.alpha_rad.sin() as f32, fs.alpha_rad.cos() as f32);
        let (sb, cb) = (fs.beta_rad.sin() as f32, fs.beta_rad.cos() as f32);
        let vel_body_dir = Vec3::new(ca * cb, sb, sa * cb); // unit vector
        let vel_world_dir = rot.0 * vel_body_dir;

        // Arrow scale: use a fixed 2 m length so it doesn't dwarf force arrows.
        let arm = 2.0_f32;

        // Wind arrow: from CG in the freestream direction (into-wind = -vel).
        gizmos.arrow(cg, cg - vel_world_dir * arm, color);

        // Nose arrow: shows aircraft heading for quick AoA reading.
        gizmos.arrow(cg, cg + nose_dir * arm, color.with_alpha(0.5));
    }
}
