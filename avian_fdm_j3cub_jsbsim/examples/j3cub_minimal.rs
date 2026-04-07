//! Minimal J-3 Cub simulation — headless, 60 seconds, prints flight state.
//!
//! Demonstrates the full FDM pipeline with a real aerodynamic model derived
//! from JSBSim J3Cub.xml data.
//!
//! **Run with:**
//! ```sh
//! cargo run --example j3cub_minimal --features presets
//! ```
//!
//! Expected output: the aircraft holds ~27 m/s cruise speed as throttle ramps
//! from 50% to 75%, enters a phugoid oscillation (the natural long-period pitch
//! mode), and gradually converges on trim. AoA remains bounded near −2°,
//! demonstrating pitch stability from the horizontal stabiliser.

use avian3d::math::{Scalar, Vector};
use avian3d::prelude::{LinearVelocity, PhysicsPlugins};
use avian_fdm::prelude::*;
use avian_fdm_j3cub_jsbsim::presets::j3cub;
use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

fn main() {
    App::new()
        // Headless: MinimalPlugins supplies the time, task-pool, and schedule loop.
        // TransformPlugin is needed for GlobalTransform propagation on zone children.
        // Hierarchy is built into bevy_ecs 0.18 — no separate HierarchyPlugin required.
        .add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin::default())
        .add_systems(Startup, spawn_aircraft)
        .add_systems(Update, (ramp_throttle, print_state, check_done))
        .run();
}

fn spawn_aircraft(mut commands: Commands) {
    // Spawn with Quat::from_rotation_x(FRAC_PI_2) so the FDM body frame aligns
    // correctly with Bevy's world frame:
    //   body X (forward) → world +X   (matches initial velocity)
    //   body Y (right)   → world +Z
    //   body Z (down)    → world −Y   (gravity direction)
    // Without this rotation, body Z = world +Z and lift forces are horizontal.
    let level = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let root = j3cub::spawn(
        &mut commands,
        Transform::from_xyz(0.0, 300.0, 0.0).with_rotation(level),
    );
    commands
        .entity(root)
        .insert(LinearVelocity(Vector::new(27.0, 0.0, 0.0)));
}

/// Linearly ramp throttle from 50% → 75% over the first 12.5 seconds.
/// Starting at 50% avoids the zoom-climb that occurs at zero throttle.
#[allow(clippy::unnecessary_cast)]
fn ramp_throttle(mut query: Query<&mut ControlInputs, With<AircraftGeometry>>, time: Res<Time>) {
    let t = time.elapsed_secs_f64();
    let throttle = (0.5 + t / 50.0).min(0.75) as Scalar;
    for mut ctrl in &mut query {
        ctrl.throttle = throttle;
    }
}

/// Print a one-line flight summary every 0.5 s of simulation time.
fn print_state(
    query: Query<&FlightState, With<AircraftGeometry>>,
    time: Res<Time>,
    mut last_print: Local<f64>,
) {
    let t = time.elapsed_secs_f64();
    if t - *last_print < 0.5 {
        return;
    }
    *last_print = t;

    for state in &query {
        println!(
            "t={t:5.1}s  alt={alt:6.1}m  TAS={tas:5.1}m/s  AoA={aoa:+5.2}deg  q={q:.0}Pa",
            alt = state.altitude_m,
            tas = state.airspeed_ms,
            aoa = state.alpha_rad.to_degrees(),
            q = state.dynamic_pressure_pa,
        );
    }
}

/// Exit cleanly after 60 seconds of simulation time.
fn check_done(time: Res<Time>, mut exit: MessageWriter<AppExit>) {
    if time.elapsed_secs_f64() >= 60.0 {
        println!("\nSimulation complete after 60 s.");
        exit.write(AppExit::Success);
    }
}
