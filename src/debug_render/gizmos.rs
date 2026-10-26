//! Debug gizmo systems for the FDM overlays.
//!
//! Each `debug_render_*` function is a Bevy system that [`AircraftFdmDebugPlugin`] adds.
//! They run in `PostUpdate`, after transform propagation. They run only when the
//! `FdmGizmos` configuration group is enabled.
//!
//! [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin

use crate::_bevy::*;
use avian3d::prelude::{ComputedCenterOfMass, ComputedMass, ConstantForce, ConstantTorque};
use bevy_shape::{Cone, Cuboid, Cylinder, Sphere};

use super::configuration::{FdmDebugRender, FdmGizmos};
use crate::components::{
    AeroZone, AircraftGeometry, Failure, FlightState, GizmoContours, GizmoShape, ZoneForce,
};

//
// Center of gravity
//

/// Draws a sphere at the center of gravity (CG) of the aircraft.
///
/// It calculates the CG position in world space from two values. The first
/// is the Avian [`ComputedCenterOfMass`] (a local offset from the entity
/// origin). The second is the [`GlobalTransform`] rotation of the entity. Bevy
/// copies the physics rotation into it before `PostUpdate`.
pub(super) fn debug_render_cg(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(&GlobalTransform, &ComputedCenterOfMass), With<AircraftGeometry>>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.cg_color else { return };

    for (gt, com) in &query {
        let cg = gt.translation() + gt.rotation() * com.0;
        gizmos.sphere(
            Isometry3d::from_translation(cg),
            config.marker_radius,
            color,
        );
    }
}

//
// Aerodynamic centers
//

/// Draws a marker at the aerodynamic center (AC) of each zone.
///
/// The AC is [`AeroZone::ac_offset`], in the local frame of the zone.
/// `GlobalTransform::transform_point` changes it into world space.
///
/// The marker is a 3-axis cross (±X, ±Y, ±Z arms). The cross stays clearly
/// visible when the AC is at the origin of the zone entity. At that position,
/// a sphere wireframe is difficult to see among the zone outlines.
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

/// Draws one combined aerodynamic force arrow for each zone, from the AC of
/// the zone. It uses [`FdmGizmos::lift_color`].
///
/// It uses [`AeroZone::ac_offset`] and the [`GlobalTransform`] of the zone to
/// find the AC in world space. The system runs in `PostUpdate`, after
/// transform propagation. Thus, there is no lag of one frame.
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

/// Draws the thrust arrows on the engine zones.
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

/// Draws three arrows from the CG: the total aerodynamic and thrust force, the weight, and the net force.
pub(super) fn debug_render_resultant(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<
        (
            &Transform,
            &ConstantForce,
            &ComputedMass,
            &ComputedCenterOfMass,
        ),
        With<AircraftGeometry>,
    >,
) {
    let config = store.config::<FdmGizmos>().1;

    for (tf, cf, mass, com) in &query {
        let cg = tf.translation + tf.rotation * com.0;
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
// Moment arrows
//

/// Draws the pitch, roll, and yaw moment arrows from the CG.
///
/// Each arrow points along its body axis (X = roll, Y = pitch, Z = yaw). Its
/// length is proportional to the moment. The colors come from
/// [`FdmGizmos::roll_moment_color`], `pitch_moment_color`, and
/// `yaw_moment_color`.
pub(super) fn debug_render_moments(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(&Transform, &ComputedCenterOfMass, &ConstantTorque), With<AircraftGeometry>>,
) {
    let config = store.config::<FdmGizmos>().1;

    for (tf, com, torque) in &query {
        let cg = tf.translation + tf.rotation * com.0;
        let scale = config.force_scale;

        // Splits the torque into body-frame components, and draws each axis.
        // The torque is already in world space. The function projects it onto
        // the body axes to draw it.
        let t = torque.0;
        if t.length_squared() < 1.0 {
            continue;
        }

        // Body axes in world space.
        let body_x = tf.rotation * Vec3::X; // roll axis
        let body_y = tf.rotation * Vec3::Y; // pitch axis
        let body_z = tf.rotation * Vec3::Z; // yaw axis

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

/// Draws the zone outline wireframes from [`GizmoShape`] and [`GizmoContours`].
///
/// It draws only the zones that have one of these components. The zone type
/// sets the color, unless [`FdmDebugRender`] gives a different color:
/// - Green: control surface (`control_role` is `Some`)
/// - Cyan: lifting surface (CL is not zero at a sample alpha)
/// - Orange: zone without aerodynamics (for example, an engine)
/// - Gray: structural or drag-only zone
///
/// For a zone with a [`Failure`] component, the color changes toward gray as
/// the damage increases. A fully failed zone (`remaining = 0.0`) is gray, for
/// all zone types.
///
/// The global [`FdmGizmos::zone_color`] is an on/off switch. To disable the
/// wireframes, set it to `None`. [`FdmDebugRender::zone_color`] on an entity
/// replaces the color of the zone type. The damage tint applies on top.
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
        let remaining = failure.map_or(1.0, |f| f.remaining);
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

/// Mixes `color` with gray, in proportion to the damage (`remaining` 0 -> 1).
///
/// At `remaining = 1.0`, the color does not change. At `remaining = 0.0`, the
/// result is fully gray (`srgba(0.55, 0.55, 0.55, 0.6)`).
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
                .primitive_3d(&Sphere::new(*radius), iso_at(Quat::IDENTITY), color)
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
// Indicator for the angle of attack and the wind
//

/// Draws the relative-wind arrow and the nose arrow, to show the angle of
/// attack and the sideslip angle.
///
/// It draws two arrows from the CG, each 2 m long:
/// - A wind arrow in the freestream direction (opposite to the velocity), in
///   [`FdmGizmos::wind_color`].
/// - A nose arrow along body X, in the same color with half alpha.
///
/// The wind arrow points almost opposite to the nose arrow. Its deviation
/// from the line of the nose arrow shows the angle of attack and the sideslip
/// angle. The view shows no text.
pub(super) fn debug_render_wind(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    query: Query<(&Transform, &ComputedCenterOfMass, &FlightState), With<AircraftGeometry>>,
) {
    let config = store.config::<FdmGizmos>().1;
    let Some(color) = config.wind_color else {
        return;
    };

    for (tf, com, fs) in &query {
        if fs.airspeed_ms < 1.0 {
            continue;
        }

        let cg = tf.translation + tf.rotation * com.0;

        // Forward direction of the body frame (the nose) in world space.
        let nose_dir = tf.rotation * Vec3::X;

        // Freestream direction: the wind comes from the direction opposite to
        // the velocity vector. The function rebuilds the velocity direction
        // from alpha and beta in the body frame, and then turns it into world
        // space. Body-frame velocity components from alpha and beta:
        // u = cos(alpha) * cos(beta), v = sin(beta), w = sin(alpha) * cos(beta)
        let (sa, ca) = (fs.alpha_rad.sin(), fs.alpha_rad.cos());
        let (sb, cb) = (fs.beta_rad.sin(), fs.beta_rad.cos());
        let vel_body_dir = Vec3::new(ca * cb, sb, sa * cb); // unit vector
        let vel_world_dir = tf.rotation * vel_body_dir;

        // Arrow length: a fixed 2 m, so that it does not dwarf the force arrows.
        let arm = 2.0_f32;

        // Wind arrow: from the CG, in the freestream direction (into the wind = -velocity).
        gizmos.arrow(cg, cg - vel_world_dir * arm, color);

        // Nose arrow: shows the heading of the aircraft, to read the angle of attack.
        gizmos.arrow(cg, cg + nose_dir * arm, color.with_alpha(0.5));
    }
}
