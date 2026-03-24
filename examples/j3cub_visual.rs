//! J-3 Cub simulation with real-time force visualisation.
//!
//! Opens a 3D window showing the aircraft outline and aerodynamic force arrows:
//!
//! | Arrow colour | Meaning                                                   |
//! |--------------|-----------------------------------------------------------|
//! | **Cyan**     | Per-zone aero + engine force (`ZoneForce`)                |
//! | **Yellow**   | Total accumulated force from CG (`ConstantForce`)         |
//! | **Red**      | Weight (m × g downward from CG)                           |
//! | **White**    | Net force = aero − weight (shows trim quality)            |
//!
//! **Run with:**
//! ```sh
//! cargo run --example j3cub_visual --features presets,debug-plugin --release
//! ```
//!
//! **Camera controls:**
//! - **Left-drag** — orbit (rotate around the aircraft)
//! - **Scroll wheel** — zoom in / out
//!
//! The simulation runs on its own. Press **Escape** to quit.

use avian3d::prelude::{
    ComputedCenterOfMass, ComputedMass, ConstantForce, LinearVelocity, PhysicsPlugins, Rotation,
};
use avian_fdm::prelude::*;
use avian_fdm::presets::j3cub;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::math::{Isometry3d, Quat};
use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "avian_fdm — J-3 Cub force visualisation".into(),
                resolution: (1280u32, 720u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin::default())
        .add_plugins(AircraftFdmDebugPlugin)
        // At cruise (~4300 N lift), the total-force arrow will be ~7 m long —
        // comparable to the aircraft fuselage, so it's easy to read.
        .insert_gizmo_config(
            FdmGizmos {
                force_scale: 1.0 / 600.0,
                ..FdmGizmos::default()
            },
            GizmoConfig::default(),
        )
        .init_resource::<OrbitCamera>()
        .add_systems(Startup, (spawn_aircraft, spawn_scene))
        .add_systems(
            Update,
            (
                ramp_throttle,
                orbit_camera,
                toggle_colliders,
                draw_aircraft_outline,
                draw_forces,
                update_hud,
            ),
        )
        .run();
}

// ── Marker components ─────────────────────────────────────────────────────────

#[derive(Component)]
struct TrackingCamera;

#[derive(Component)]
struct HudText;

// ── Orbit camera resource ─────────────────────────────────────────────────────

/// Spherical-coordinate state for the orbit camera.
///
/// The camera always looks at the aircraft's current world position.
/// Angles are in radians; yaw rotates around world Y, pitch tilts up/down.
#[derive(Resource)]
struct OrbitCamera {
    /// Horizontal angle (radians). 0 = aircraft's +X side, π = behind (−X).
    yaw: f32,
    /// Vertical angle (radians, clamped to avoid flipping).
    pitch: f32,
    /// Distance from the focus point (metres).
    radius: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            // Isometric view from port
            yaw: -std::f32::consts::PI / 3.0,
            pitch: 0.60,
            radius: 15.0,
        }
    }
}

// ── Startup ───────────────────────────────────────────────────────────────────

fn spawn_aircraft(mut commands: Commands) {
    let level = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let root = j3cub::spawn(
        &mut commands,
        Transform::from_xyz(0.0, 300.0, 0.0).with_rotation(level),
    );
    commands
        .entity(root)
        .insert(LinearVelocity(Vec3::new(27.0, 0.0, 0.0)));
}

fn spawn_scene(mut commands: Commands) {
    // ── Camera ────────────────────────────────────────────────────────────────
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 308.0, 0.0).looking_at(Vec3::new(0.0, 300.0, 0.0), Vec3::Y),
        TrackingCamera,
    ));

    // ── Directional light ─────────────────────────────────────────────────────
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, 0.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // ── HUD text (upper-left corner) ──────────────────────────────────────────
    commands.spawn((
        Text::new("Initialising…"),
        TextFont {
            font_size: 18.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        HudText,
    ));

    // ── Legend ────────────────────────────────────────────────────────────────
    commands.spawn((
        Text::new(
            "-> Cyan   per-zone aero forces\n\
             -> Green  thrust\n\
             -> Yellow total aero+thrust (from CG)\n\
             -> Red    weight\n\
             -> White  net force (~0 at trim)\n\
              o Grey   centre of gravity (CG)\n\
              + Cyan   aerodynamic centre (AC)\n\
             \n\
             LMB drag  orbit\n\
             Scroll    zoom",
        ),
        TextFont {
            font_size: 15.0,
            ..default()
        },
        TextColor(Color::srgba(0.9, 0.9, 0.9, 0.85)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

// ── Per-frame systems ─────────────────────────────────────────────────────────

/// Linearly ramp throttle from 50 % → 75 % over the first 12.5 seconds.
fn ramp_throttle(mut query: Query<&mut ControlInputs, With<AircraftGeometry>>, time: Res<Time>) {
    let t = time.elapsed_secs_f64();
    let throttle = (0.5 + t / 50.0).min(0.75);
    for mut ctrl in &mut query {
        ctrl.throttle = throttle;
    }
}

/// Orbit camera: left-drag to rotate, scroll to zoom, always looks at the aircraft.
fn orbit_camera(
    mut orbit: ResMut<OrbitCamera>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    aircraft: Query<&Transform, With<AircraftGeometry>>,
    mut camera: Query<&mut Transform, (With<TrackingCamera>, Without<AircraftGeometry>)>,
) {
    // Scroll to zoom.
    orbit.radius = (orbit.radius - scroll.delta.y * 2.0).clamp(5.0, 300.0);

    // Left-drag to orbit.
    if buttons.pressed(MouseButton::Left) {
        orbit.yaw -= motion.delta.x * 0.005;
        orbit.pitch = (orbit.pitch - motion.delta.y * 0.005).clamp(-1.48, 1.48);
        // stay just short of poles
    }

    let Ok(ac) = aircraft.single() else { return };
    let Ok(mut cam) = camera.single_mut() else {
        return;
    };

    // Spherical → Cartesian offset from the focus point.
    let (sy, cy) = orbit.yaw.sin_cos();
    let (sp, cp) = orbit.pitch.sin_cos();
    let offset = Vec3::new(cp * cy, sp, cp * sy) * orbit.radius;

    let focus = ac.translation;
    cam.translation = focus + offset;
    cam.look_at(focus, Vec3::Y);
}

/// Toggles the collider wireframe overlay on/off when **C** is pressed.
///
/// The [`ShowColliders`] resource is provided by [`AircraftFdmDebugPlugin`].
fn toggle_colliders(keys: Res<ButtonInput<KeyCode>>, mut show: ResMut<ShowColliders>) {
    if keys.just_pressed(KeyCode::KeyC) {
        show.0 = !show.0;
    }
}

/// Draws all zone entities using their [`GizmoShape`] and [`GizmoContours`] components.
fn draw_zone_shape(
    gizmos: &mut Gizmos<FdmGizmos>,
    shape: &GizmoShape,
    zone_tf: &Transform,
    iso_at: &impl Fn(&Transform, Quat) -> Isometry3d,
    zone_to_world: &impl Fn(&Transform, Vec3) -> Vec3,
    color: Color,
) {
    match shape {
        GizmoShape::Box { x, y, z } => {
            gizmos.primitive_3d(&Cuboid::new(*x, *y, *z), iso_at(zone_tf, Quat::IDENTITY), color);
        }
        GizmoShape::Cylinder { radius, length, axis } => {
            gizmos
                .primitive_3d(
                    &Cylinder::new(*radius, *length),
                    iso_at(zone_tf, Quat::from_rotation_arc(Vec3::Y, *axis)),
                    color,
                )
                .resolution(32);
        }
        GizmoShape::Cone { radius, length } => {
            gizmos.primitive_3d(
                &Cone { radius: *radius, height: *length },
                iso_at(zone_tf, Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2)),
                color,
            );
        }
        GizmoShape::Sphere { radius } => {
            gizmos
                .primitive_3d(
                    &bevy::math::primitives::Sphere::new(*radius),
                    iso_at(zone_tf, Quat::IDENTITY),
                    color,
                )
                .resolution(32);
        }
        GizmoShape::Quad { corners } => {
            let pts: Vec<Vec3> = corners
                .iter()
                .chain(std::iter::once(&corners[0]))
                .map(|c| zone_to_world(zone_tf, *c))
                .collect();
            gizmos.linestrip(pts, color);
        }
        GizmoShape::Strut { start, end } => {
            gizmos.line(zone_to_world(zone_tf, *start), zone_to_world(zone_tf, *end), color);
        }
    }
}

/// Draws all zone entities using their [`GizmoShape`] and [`GizmoContours`] components.
///
/// Press **C** to toggle the collider wireframe overlay provided by
/// [`AircraftFdmDebugPlugin`] (via [`ShowColliders`]).
///
///
/// | Colour      | Meaning                                      |
/// |-------------|----------------------------------------------|
/// | Cyan        | Lifting surface (wing, h-stab)               |
/// | Green       | Control surface (aileron/elevator/rudder)     |
/// | Orange      | Engine zone                                  |
/// | Grey        | Non-lifting / structural (fuselage, struts)  |
fn draw_aircraft_outline(
    mut gizmos: Gizmos<FdmGizmos>,
    root_query: Query<&Transform, With<AircraftGeometry>>,
    zone_query: Query<
        (
            &Transform,
            Option<&AeroZone>,
            Option<&GizmoContours>,
            Option<&GizmoShape>,
            Option<&EngineZone>,
        ),
        Or<(With<AeroZone>, With<EngineZone>)>,
    >,
) {
    let Ok(ac) = root_query.single() else { return };
    let t = ac.translation;
    let r = ac.rotation;

    let to_world = |body: Vec3| t + r * body;
    // Zone-local point to world, respecting zone rotation.
    let zone_to_world =
        |zone_tf: &Transform, local: Vec3| to_world(zone_tf.translation + zone_tf.rotation * local);
    let iso_at = |zone_tf: &Transform, extra_rot: Quat| {
        Isometry3d::new(
            t + r * zone_tf.translation,
            Quat::from_array(r.to_array())
                * Quat::from_array(zone_tf.rotation.to_array())
                * extra_rot,
        )
    };

    for (zone_tf, aero, contours, shape, engine) in &zone_query {
        // Pick colour by zone type.
        let color = if engine.is_some() {
            Color::srgba(0.95, 0.65, 0.15, 0.9) // orange
        } else if aero.is_some_and(|a| a.control_role.is_some()) {
            Color::srgba(0.3, 1.0, 0.5, 0.7) // green
        } else if aero.is_some_and(|a| a.cl.evaluate(0.1, 2e6).abs() > 0.01) {
            Color::srgba(0.4, 0.85, 1.0, 0.7) // cyan — lifting surface
        } else {
            Color::srgba(0.6, 0.6, 0.6, 0.6) // grey — structural / drag-only
        };

        // Draw contour linestrips when present.
        if let Some(contour_data) = contours {
            for line in &contour_data.lines {
                if line.len() < 2 {
                    continue;
                }
                let world_pts: Vec<Vec3> =
                    line.iter().map(|p| zone_to_world(zone_tf, *p)).collect();
                gizmos.linestrip(world_pts, color);
            }
            // If there's no GizmoShape, contours are the only visual.
            if shape.is_none() {
                continue;
            }
        }

        // Draw GizmoShape.
        if let Some(gs) = shape {
            draw_zone_shape(&mut gizmos, gs, zone_tf, &iso_at, &zone_to_world, color);
        }

        // (contour zones handled above with `continue`)
    }
}

/// Draws per-zone forces (cyan/green) plus total-force, weight and net-force arrows.
fn draw_forces(
    mut gizmos: Gizmos<FdmGizmos>,
    store: Res<GizmoConfigStore>,
    // No ColliderOf — Avian adds it lazily and it may be absent on the engine
    // zone, causing a silent query miss. With one aircraft, root_query.single()
    // gives us the root Transform for all world-position calculations.
    zone_query: Query<(&ZoneForce, &Transform, Has<EngineZone>)>,
    root_query: Query<
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
    let Ok((root_tf, rot, cf, mass, com)) = root_query.single() else {
        return;
    };
    let force_scale = store.config::<FdmGizmos>().1.force_scale;

    // ── Per-zone force arrows ─────────────────────────────────────────────────
    for (zf, zone_local_tf, is_engine) in &zone_query {
        if zf.force.length_squared() < 100.0 {
            continue; // skip tiny / inactive zones (< 10 N)
        }
        // Compute world position from root's current Transform (no GlobalTransform lag).
        let start = root_tf.transform_point(zone_local_tf.translation);
        if is_engine {
            gizmos.arrow(
                start,
                start + zf.force * force_scale,
                Color::srgb(0.1, 1.0, 0.1),
            );
        } else {
            gizmos.arrow(
                start,
                start + zf.force * force_scale,
                Color::srgb(0.0, 0.9, 0.9),
            );
        }
    }

    // ── Per-aircraft arrows from CG ───────────────────────────────────────────
    let cg = root_tf.translation + rot.0 * com.0;

    // Total aerodynamic + propulsive force (yellow).
    gizmos.arrow(cg, cg + cf.0 * force_scale, Color::srgb(1.0, 1.0, 0.0));

    // Weight (red, downward).
    let weight = Vec3::new(0.0, -mass.value() * 9.806_65, 0.0);
    gizmos.arrow(cg, cg + weight * force_scale, Color::srgb(1.0, 0.2, 0.2));

    // Net force = aero + weight (white) — near-zero at trim.
    let net = cf.0 + weight;
    if net.length_squared() > 1.0 {
        gizmos.arrow(cg, cg + net * force_scale, Color::WHITE);
    }
}

/// Updates the flight-state HUD in the upper-left corner.
fn update_hud(
    flight_query: Query<&FlightState, With<AircraftGeometry>>,
    time: Res<Time>,
    mut hud: Query<&mut Text, With<HudText>>,
) {
    let Ok(fs) = flight_query.single() else {
        return;
    };
    let Ok(mut text) = hud.single_mut() else {
        return;
    };
    let t = time.elapsed_secs();

    text.0 = format!(
        "t   = {t:.1} s\n\
         alt = {alt:.0} m\n\
         TAS = {tas:.1} m/s\n\
         AoA = {aoa:+.2}°\n\
         q̄   = {q:.0} Pa\n\
         Re  = {re:.2e}",
        alt = fs.altitude_m,
        tas = fs.airspeed_ms,
        aoa = fs.alpha_rad.to_degrees(),
        q = fs.dynamic_pressure_pa,
        re = fs.reynolds_number,
    );
}
