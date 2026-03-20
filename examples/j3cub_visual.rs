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
    ComputedCenterOfMass, ComputedMass, ConstantForce, LinearVelocity, PhysicsPlugins,
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

/// Draws a simplified structural outline of the aircraft using gizmo cuboids.
fn draw_aircraft_outline(
    mut gizmos: Gizmos,
    query: Query<&Transform, With<AircraftGeometry>>,
) {
    let Ok(ac) = query.single() else { return };
    let t = ac.translation;
    let r = ac.rotation;

    // Helper: world Isometry3d for a body-frame offset.
    let iso_at = |body_offset: Vec3| {
        Isometry3d::new(t + r * body_offset, Quat::from_array(r.to_array()))
    };

    // Engine / cowl block (Continental A-65: 0.5 × 0.4 × 0.4 m).
    // Front face at x = 1.90 m — sets the nose limit for the fuselage.
    gizmos.primitive_3d(
        &Cuboid::new(0.50, 0.40, 0.40),
        iso_at(Vec3::new(1.65, 0.0, 0.04)),
        Color::srgba(0.6, 0.6, 0.6, 0.9),
    );

    // Cabin section: firewall (x = 1.40) back to start of tail boom (x = −0.80).
    // Length = 2.20 m, centred at x = +0.30.  Full cabin width / height.
    gizmos.primitive_3d(
        &Cuboid::new(2.20, 0.68, 0.72),
        iso_at(Vec3::new(0.30, 0.0, 0.0)),
        Color::srgba(0.95, 0.85, 0.3, 0.75),
    );

    // Tail boom: x = −0.80 back to tail end (x = −3.70).
    // Length = 2.90 m, centred at x = −2.25.  Narrow cross-section.
    gizmos.primitive_3d(
        &Cuboid::new(2.90, 0.44, 0.38),
        iso_at(Vec3::new(-2.25, 0.0, 0.0)),
        Color::srgba(0.95, 0.85, 0.3, 0.65),
    );

    // Left wing (spans y = 0 → −5.37 m in body frame, centred at −2.685 m).
    gizmos.primitive_3d(
        &Cuboid::new(0.80, 5.37, 0.155),
        iso_at(Vec3::new(-0.10, -2.685, -0.58)),
        Color::srgba(0.4, 0.8, 1.0, 0.6),
    );
    // Right wing (mirror).
    gizmos.primitive_3d(
        &Cuboid::new(0.80, 5.37, 0.155),
        iso_at(Vec3::new(-0.10, 2.685, -0.58)),
        Color::srgba(0.4, 0.8, 1.0, 0.6),
    );

    // Horizontal stabiliser (tail arm ≈ 3.96 m aft).
    gizmos.primitive_3d(
        &Cuboid::new(0.60, 2.20, 0.08),
        iso_at(Vec3::new(-3.96, 0.0, -0.10)),
        Color::srgba(0.95, 0.85, 0.3, 0.6),
    );

    // Vertical fin.
    gizmos.primitive_3d(
        &Cuboid::new(0.50, 0.10, 0.60),
        iso_at(Vec3::new(-4.16, 0.0, -0.50)),
        Color::srgba(0.95, 0.85, 0.3, 0.6),
    );
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
