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
//! **Flight controls (keyboard):**
//! - **W / S** — elevator (pitch up / down)
//! - **A / D** — ailerons (roll left / right)
//! - **Q / E** — rudder (yaw left / right)
//! - **Left Shift / Left Ctrl** — throttle up / down
//!
//! **Flight controls (gamepad):**
//! - **Left stick** — elevator (Y) + ailerons (X)
//! - **Right stick X** — rudder
//! - **Right trigger** — throttle
//!
//! **Camera controls:**
//! - **Left-drag** — orbit (rotate around the aircraft)
//! - **Scroll wheel** — zoom in / out
//!
//! - **R** — restart (reset position, velocity, controls)
//!
//! Press **Escape** to quit.

use avian3d::prelude::{AngularVelocity, LinearVelocity, PhysicsPlugins, Rotation};
use avian_fdm::prelude::*;
use avian_fdm_j3cub_jsbsim::presets::j3cub;
use bevy::input::gamepad::{Gamepad, GamepadAxis};
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
        // The net force (white) shows deviations from trim equilibrium.
        // Scale chosen so a 500 N net force produces a 5 m arrow: easy to read.
        // The total aero and weight arrows are turned off so the net force
        // is not visually dwarfed by the ~4000 N lift/weight arrows.
        .insert_gizmo_config(
            FdmGizmos {
                force_scale: 1.0 / 600.0,
                total_force_color: None,
                weight_color: None,
                ..FdmGizmos::default()
            },
            GizmoConfig::default(),
        )
        .init_resource::<OrbitCamera>()
        .add_systems(Startup, (spawn_aircraft, spawn_scene, spawn_legend))
        .add_systems(
            Update,
            (
                handle_input,
                restart_aircraft,
                orbit_camera,
                toggle_colliders,
                update_hud,
                debug_print_rotation,
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
    commands.entity(root).insert((
        LinearVelocity(Vec3::new(27.0, 0.0, 0.0)),
        ControlInputs {
            throttle: 0.6,
            ..default()
        },
    ));
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
        ("→ ", "per-zone aero force", config.lift_color),
        ("→ ", "engine thrust", config.thrust_color),
        ("→ ", "total aero+thrust", config.total_force_color),
        ("→ ", "weight", config.weight_color),
        ("→ ", "net force (~0 at trim)", config.resultant_color),
        ("~ ", "pitch moment", config.pitch_moment_color),
        ("~ ", "roll moment", config.roll_moment_color),
        ("~ ", "yaw moment", config.yaw_moment_color),
        ("— ", "relative wind / AoA", config.wind_color),
        ("○ ", "CG", config.cg_color),
        ("○ ", "zone AC", config.ac_color),
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
            TextSpan::new("\nLMB drag  orbit\nScroll    zoom\nW/S  elevator  A/D  aileron\nQ/E  rudder   Shift/Ctrl  throttle\nR  restart"),
            TextColor(dim),
        ));
    });
}

/// Resets the aircraft to its initial state when **R** is pressed.
///
/// Restores position, orientation, velocity, and control inputs without
/// despawning — zone child entities and all other components are preserved.
fn restart_aircraft(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<
        (
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &mut ControlInputs,
        ),
        With<AircraftGeometry>,
    >,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    let level = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    for (mut transform, mut lin_vel, mut ang_vel, mut ctrl) in &mut query {
        *transform = Transform::from_xyz(0.0, 300.0, 0.0).with_rotation(level);
        *lin_vel = LinearVelocity(Vec3::new(27.0, 0.0, 0.0));
        *ang_vel = AngularVelocity::default();
        *ctrl = ControlInputs {
            throttle: 0.6,
            ..default()
        };
    }
}

///
/// Keyboard deflections are binary (full deflection while key held).
/// Gamepad axes map directly to the [-1, 1] range via the `Gamepad` component.
/// Throttle is rate-based: a held key changes it by 0.5 per second.
fn handle_input(
    mut query: Query<&mut ControlInputs, With<AircraftGeometry>>,
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    time: Res<Time>,
) {
    let dt = time.delta_secs_f64();

    // Keyboard: binary deflection while key is held.
    // W = stick forward = nose down (-1), S = pull back = nose up (+1).
    let kb_elevator = keys.pressed(KeyCode::KeyS) as i32 - keys.pressed(KeyCode::KeyW) as i32;
    let kb_aileron = keys.pressed(KeyCode::KeyD) as i32 - keys.pressed(KeyCode::KeyA) as i32;
    let kb_rudder = keys.pressed(KeyCode::KeyE) as i32 - keys.pressed(KeyCode::KeyQ) as i32;
    // Shift = throttle up, Ctrl = throttle down.
    let kb_throttle =
        keys.pressed(KeyCode::ShiftLeft) as i32 - keys.pressed(KeyCode::ControlLeft) as i32;

    // Gamepad: use first connected pad if any.
    let (gp_elevator, gp_aileron, gp_rudder, gp_throttle) = gamepads
        .iter()
        .next()
        .map(|pad| {
            // Left stick: aileron (X) + elevator (Y, push forward = nose up).
            let aileron = pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0) as f64;
            let elevator = -pad.get(GamepadAxis::LeftStickY).unwrap_or(0.0) as f64;
            let rudder = pad.get(GamepadAxis::RightStickX).unwrap_or(0.0) as f64;
            // Right trigger (RightZ): range [0, 1].
            let throttle = pad.get(GamepadAxis::RightZ).unwrap_or(-1.0) as f64;
            (elevator, aileron, rudder, throttle)
        })
        .unwrap_or((0.0, 0.0, 0.0, -1.0)); // -1 on throttle = no gamepad connected

    for mut ctrl in &mut query {
        // Deflection: keyboard overrides gamepad when both active.
        ctrl.elevator = if kb_elevator != 0 {
            kb_elevator as f64
        } else {
            gp_elevator
        }
        .clamp(-1.0, 1.0);

        ctrl.aileron = if kb_aileron != 0 {
            kb_aileron as f64
        } else {
            gp_aileron
        }
        .clamp(-1.0, 1.0);

        ctrl.rudder = if kb_rudder != 0 {
            kb_rudder as f64
        } else {
            gp_rudder
        }
        .clamp(-1.0, 1.0);

        // Throttle: rate-based from keyboard; direct from gamepad trigger.
        if kb_throttle != 0 {
            ctrl.throttle = (ctrl.throttle + kb_throttle as f64 * 0.5 * dt).clamp(0.0, 1.0);
        } else if gp_throttle >= 0.0 {
            ctrl.throttle = gp_throttle;
        }
        // If neither active, throttle holds its current value.
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
    flight_query: Query<
        (
            &FlightState,
            &ControlInputs,
            &Transform,
            &avian3d::prelude::ConstantForce,
            &avian3d::prelude::ComputedMass,
        ),
        With<AircraftGeometry>,
    >,
    time: Res<Time>,
    mut hud: Query<&mut Text, With<HudText>>,
) {
    let Ok((fs, ctrl, tf, cf, mass)) = flight_query.single() else {
        return;
    };
    let Ok(mut text) = hud.single_mut() else {
        return;
    };
    let t = time.elapsed_secs();

    // Euler yaw from the Bevy Transform rotation (y-axis rotation).
    let (yaw_sin, yaw_cos) = (tf.rotation.y, tf.rotation.w);
    let yaw_deg = 2.0 * f32::atan2(yaw_sin, yaw_cos).to_degrees();

    // Net force = total aero - weight.
    let weight_y = mass.value() * 9.806_65;
    let net = cf.0 - Vec3::new(0.0, weight_y, 0.0);

    text.0 = format!(
        "t   = {t:.1} s\n\
         alt = {alt:.0} m\n\
         TAS = {tas:.1} m/s\n\
         AoA = {aoa:+.2}  beta = {beta:+.2}°\n\
         yaw = {yaw:+.1}°\n\
         q̄   = {q:.0} Pa\n\
         Re  = {re:.2e}\n\
         thr = {thr:.0}%  elv = {elv:+.2}  ail = {ail:+.2}  rud = {rud:+.2}\n\
         net = ({nx:+.0}, {ny:+.0}, {nz:+.0}) N",
        alt = fs.altitude_m,
        tas = fs.airspeed_ms,
        aoa = fs.alpha_rad.to_degrees(),
        beta = fs.beta_rad.to_degrees(),
        yaw = yaw_deg,
        q = fs.dynamic_pressure_pa,
        re = fs.reynolds_number,
        thr = ctrl.throttle * 100.0,
        elv = ctrl.elevator,
        ail = ctrl.aileron,
        rud = ctrl.rudder,
        nx = net.x,
        ny = net.y,
        nz = net.z,
    );
}

/// Prints Avian Rotation vs Bevy Transform rotation every 0.25 seconds to
/// check whether the physics rotation tracks the visual rotation.
fn debug_print_rotation(
    query: Query<
        (
            &Rotation,
            &Transform,
            &FlightState,
            &avian3d::prelude::ConstantForce,
        ),
        With<AircraftGeometry>,
    >,
    time: Res<Time>,
    mut last: Local<f32>,
) {
    if time.elapsed_secs() - *last < 0.25 {
        return;
    }
    *last = time.elapsed_secs();
    for (avian_rot, tf, fs, cf) in &query {
        let avian_q = avian_rot.0;
        let bevy_q = tf.rotation;
        println!(
            "[debug] t={:.2}  avian_rot=({:.3},{:.3},{:.3},{:.3})  bevy_rot=({:.3},{:.3},{:.3},{:.3})  alpha={:.2}  beta={:.2}  cf0=({:.0},{:.0},{:.0})",
            time.elapsed_secs(),
            avian_q.x, avian_q.y, avian_q.z, avian_q.w,
            bevy_q.x,  bevy_q.y,  bevy_q.z,  bevy_q.w,
            fs.alpha_rad.to_degrees(),
            fs.beta_rad.to_degrees(),
            cf.0.x, cf.0.y, cf.0.z,
        );
    }
}
