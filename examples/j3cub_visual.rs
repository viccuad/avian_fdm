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
//! cargo run --example j3cub_visual --features presets
//! ```
//!
//! **Camera controls:**
//! - **Left-drag** — orbit (rotate around the aircraft)
//! - **Scroll wheel** — zoom in / out
//!
//! The simulation runs on its own. Press **Escape** to quit.

use avian_fdm::prelude::*;
use avian_fdm::presets::j3cub;
use avian3d::prelude::{
    Collider, ComputedCenterOfMass, ComputedMass, ConstantForce, LinearVelocity, PhysicsPlugins,
    Rotation,
};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::math::{Isometry3d, Quat};
use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "avian_fdm — J-3 Cub force visualisation".into(),
                    resolution: (1280u32, 720u32).into(),
                    ..default()
                }),
                ..default()
            }),
        )
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin)
        .init_resource::<OrbitCamera>()
        .add_systems(Startup, (spawn_aircraft, spawn_scene))
        .add_systems(
            Update,
            (
                ramp_throttle,
                orbit_camera,
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
            // Start roughly behind the aircraft (flies in +X) and above.
            yaw: std::f32::consts::PI,
            pitch: 0.25,
            radius: 35.0,
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
        Transform::from_xyz(-30.0, 308.0, 0.0)
            .looking_at(Vec3::new(0.0, 300.0, 0.0), Vec3::Y),
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
            "■ Cyan   per-zone aero forces\n\
             ■ Green  thrust\n\
             ■ Yellow total aero+thrust (from CG)\n\
             ■ Red    weight\n\
             ■ White  net force (≈0 at trim)\n\
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
fn ramp_throttle(
    mut query: Query<&mut ControlInputs, With<AircraftGeometry>>,
    time: Res<Time>,
) {
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
        orbit.yaw   -= motion.delta.x * 0.005;
        orbit.pitch  = (orbit.pitch - motion.delta.y * 0.005)
            .clamp(-1.48, 1.48); // stay just short of poles
    }

    let Ok(ac)      = aircraft.single()   else { return };
    let Ok(mut cam) = camera.single_mut() else { return };

    // Spherical → Cartesian offset from the focus point.
    let (sy, cy) = orbit.yaw.sin_cos();
    let (sp, cp) = orbit.pitch.sin_cos();
    let offset = Vec3::new(cp * cy, sp, cp * sy) * orbit.radius;

    let focus = ac.translation;
    cam.translation = focus + offset;
    cam.look_at(focus, Vec3::Y);
}

// ── Force scale: 1 N → FORCE_SCALE metres of arrow ───────────────────────────
// At cruise (~4300 N lift), the yellow total-force arrow will be ~7 m long —
// comparable to the aircraft fuselage length, so it's easy to read.
const FORCE_SCALE: f32 = 1.0 / 600.0;

/// Draws all zone entities using their [`Collider`] shape directly, with
/// optional [`GizmoShape`] overrides for non-standard visuals.  Zones are
/// color-coded by type:
///
/// | Colour | Meaning |
/// |--------|---------|
/// | Cyan   | Lifting surface (wing, h-stab) |
/// | Green  | Control surface (aileron/elevator/rudder) |
/// | Grey   | Engine zone |
/// | Orange | Structural / drag-only (fuselage, struts) |
///
/// Damaged zones fade toward red; fully destroyed zones disappear.
fn draw_aircraft_outline(
    mut gizmos: Gizmos,
    root_query: Query<&Transform, With<AircraftGeometry>>,
    zone_query: Query<(
        &Transform,
        Option<&AeroZone>,
        Option<&Collider>,
        Option<&GizmoShape>,
        Option<&Damageable>,
        Option<&EngineZone>,
    ), Or<(With<AeroZone>, With<EngineZone>)>>,
) {
    use parry3d::shape::TypedShape;

    let Ok(ac) = root_query.single() else { return };
    let t = ac.translation;
    let r = ac.rotation;

    let to_world = |body: Vec3| t + r * body;
    let iso_at = |body_offset: Vec3, extra_rot: Quat| {
        Isometry3d::new(
            t + r * body_offset,
            Quat::from_array(r.to_array()) * extra_rot,
        )
    };

    for (zone_tf, aero, collider, shape, dmg, engine) in &zone_query {
        let health = dmg.map(|d| d.health as f32).unwrap_or(1.0);
        if health <= 0.0 { continue; }

        // Pick base colour by zone type.
        let base = if engine.is_some() {
            Color::srgba(0.6, 0.6, 0.6, 0.9)
        } else if aero.is_some_and(|a| a.control_role.is_some()) {
            Color::srgba(0.3, 1.0, 0.5, 0.7)
        } else if aero.is_some_and(|a| a.cl.evaluate(0.1, 2e6).abs() > 0.01) {
            Color::srgba(0.4, 0.85, 1.0, 0.7) // lifting surface
        } else {
            Color::srgba(0.9, 0.75, 0.2, 0.7) // drag-only structural
        };

        // Fade toward red with damage.
        let color = if health < 1.0 {
            let r_c = base.to_srgba().red * health + 1.0 * (1.0 - health);
            let g_c = base.to_srgba().green * health;
            let b_c = base.to_srgba().blue * health;
            let a_c = base.to_srgba().alpha;
            Color::srgba(r_c, g_c, b_c, a_c)
        } else {
            base
        };

        let pos = zone_tf.translation;

        // Priority: GizmoShape override > Collider shape > nothing.
        if let Some(gs) = shape {
            match gs {
                GizmoShape::Box { x, y, z } => {
                    gizmos.primitive_3d(
                        &Cuboid::new(*x, *y, *z),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
                GizmoShape::Cylinder { radius, length } => {
                    gizmos.primitive_3d(
                        &Cylinder::new(*radius, *length),
                        iso_at(pos, Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                        color,
                    );
                }
                GizmoShape::Cone { radius, length } => {
                    gizmos.primitive_3d(
                        &Cone { radius: *radius, height: *length },
                        iso_at(pos, Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2)),
                        color,
                    );
                }
                GizmoShape::Sphere { radius } => {
                    gizmos.primitive_3d(
                        &bevy::math::primitives::Sphere::new(*radius),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
                GizmoShape::Quad { corners } => {
                    let pts: Vec<Vec3> = corners.iter()
                        .chain(std::iter::once(&corners[0]))
                        .map(|c| to_world(pos + *c))
                        .collect();
                    gizmos.linestrip(pts, color);
                }
                GizmoShape::Strut { start, end } => {
                    gizmos.line(
                        to_world(pos + *start),
                        to_world(pos + *end),
                        color,
                    );
                }
            }
        } else if let Some(col) = collider {
            // Draw the collider shape directly — what you see IS what the
            // physics engine sees.
            match col.shape_scaled().as_typed_shape() {
                TypedShape::Cuboid(c) => {
                    let he = c.half_extents;
                    gizmos.primitive_3d(
                        &Cuboid::new(he.x as f32 * 2.0, he.y as f32 * 2.0, he.z as f32 * 2.0),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
                TypedShape::Ball(b) => {
                    gizmos.primitive_3d(
                        &bevy::math::primitives::Sphere::new(b.radius as f32),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
                TypedShape::Cylinder(c) => {
                    gizmos.primitive_3d(
                        &Cylinder::new(c.radius as f32, c.half_height as f32 * 2.0),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
                TypedShape::Capsule(c) => {
                    gizmos.primitive_3d(
                        &Capsule3d::new(c.radius as f32, c.half_height() as f32 * 2.0),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
                _ => {
                    // Unsupported collider shape — draw a small marker.
                    gizmos.primitive_3d(
                        &Cuboid::new(0.1, 0.1, 0.1),
                        iso_at(pos, Quat::IDENTITY),
                        color,
                    );
                }
            }
        }
    }
}

/// Draws per-zone forces (cyan/green) plus total-force, weight and net-force arrows.
fn draw_forces(
    mut gizmos: Gizmos,
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
    let Ok((root_tf, rot, cf, mass, com)) = root_query.single() else { return };

    // ── Per-zone force arrows ─────────────────────────────────────────────────
    for (zf, zone_local_tf, is_engine) in &zone_query {
        if zf.force.length_squared() < 100.0 {
            continue; // skip tiny / inactive zones (< 10 N)
        }
        // Compute world position from root's current Transform (no GlobalTransform lag).
        let start = root_tf.transform_point(zone_local_tf.translation);
        if is_engine {
            gizmos.arrow(start, start + zf.force * FORCE_SCALE, Color::srgb(0.1, 1.0, 0.1));
        } else {
            gizmos.arrow(start, start + zf.force * FORCE_SCALE, Color::srgb(0.0, 0.9, 0.9));
        }
    }

    // ── Per-aircraft arrows from CG ───────────────────────────────────────────
    let cg = root_tf.translation + rot.0 * com.0;

    // Total aerodynamic + propulsive force (yellow).
    gizmos.arrow(cg, cg + cf.0 * FORCE_SCALE, Color::srgb(1.0, 1.0, 0.0));

    // Weight (red, downward).
    let weight = Vec3::new(0.0, -mass.value() * 9.806_65, 0.0);
    gizmos.arrow(cg, cg + weight * FORCE_SCALE, Color::srgb(1.0, 0.2, 0.2));

    // Net force = aero + weight (white) — near-zero at trim.
    let net = cf.0 + weight;
    if net.length_squared() > 1.0 {
        gizmos.arrow(cg, cg + net * FORCE_SCALE, Color::WHITE);
    }

    // CG marker cross.
    let h = 0.5;
    gizmos.line(cg - Vec3::X * h, cg + Vec3::X * h, Color::WHITE);
    gizmos.line(cg - Vec3::Y * h, cg + Vec3::Y * h, Color::WHITE);
    gizmos.line(cg - Vec3::Z * h, cg + Vec3::Z * h, Color::WHITE);
}

/// Updates the flight-state HUD in the upper-left corner.
fn update_hud(
    flight_query: Query<&FlightState, With<AircraftGeometry>>,
    time: Res<Time>,
    mut hud: Query<&mut Text, With<HudText>>,
) {
    let Ok(fs) = flight_query.single() else { return };
    let Ok(mut text) = hud.single_mut() else { return };
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
        q   = fs.dynamic_pressure_pa,
        re  = fs.reynolds_number,
    );
}
