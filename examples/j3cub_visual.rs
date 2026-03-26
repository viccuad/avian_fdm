//! J-3 Cub simulation with real-time force visualisation.
//!
//! Opens a 3D window showing the aircraft outline and aerodynamic force arrows.
//! The in-app legend is generated at startup from the live [`FdmGizmos`] config,
//! so each label is rendered in its actual gizmo colour. Disabled channels (`None`)
//! are automatically omitted from the legend.
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

use avian3d::prelude::{LinearVelocity, PhysicsPlugins};
use avian_fdm::prelude::*;
use avian_fdm::presets::j3cub;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::math::Quat;
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
        .add_systems(Startup, (spawn_aircraft, spawn_scene, spawn_legend))
        .add_systems(
            Update,
            (
                ramp_throttle,
                orbit_camera,
                toggle_colliders,
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
}

/// Build the legend overlay from the live [`FdmGizmos`] config.
///
/// Each row is a [`TextSpan`] child rendered in the gizmo's actual colour —
/// no colour names are hardcoded. Rows whose channel is disabled (`None`) are
/// omitted automatically, so the legend always matches the active config.
fn spawn_legend(mut commands: Commands, store: Res<GizmoConfigStore>) {
    let config = store.config::<FdmGizmos>().1;
    let dim = Color::srgba(0.7, 0.7, 0.7, 0.85);

    // (prefix, description, channel colour)
    let rows: &[(&str, &str, Option<Color>)] = &[
        ("→ ", "per-zone aero force",  config.lift_color),
        ("→ ", "engine thrust",        config.thrust_color),
        ("→ ", "total aero+thrust",    config.total_force_color),
        ("→ ", "weight",               config.weight_color),
        ("→ ", "net force (~0 at trim)",config.resultant_color),
        ("~ ", "pitch moment",         config.pitch_moment_color),
        ("~ ", "roll moment",          config.roll_moment_color),
        ("~ ", "yaw moment",           config.yaw_moment_color),
        ("— ", "relative wind / AoA",  config.wind_color),
        ("○ ", "CG",                   config.cg_color),
        ("○ ", "zone AC",              config.ac_color),
    ];

    commands.spawn((
        Text::default(),
        TextFont { font_size: 15.0, ..default() },
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ))
    .with_children(|p| {
        for (prefix, label, color_opt) in rows {
            if let Some(color) = *color_opt {
                p.spawn((TextSpan::new(format!("{prefix}{label}\n")), TextColor(color)));
            }
        }
        p.spawn((
            TextSpan::new("\nLMB drag  orbit\nScroll    zoom"),
            TextColor(dim),
        ));
    });
}



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
