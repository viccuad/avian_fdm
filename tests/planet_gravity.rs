//! Integration tests for the planet gravity.
//!
//! Verifies that `update_planet_gravity` writes the radial gravity of a
//! `PlanetResource` into the `ConstantLinearAcceleration` of each body with
//! `PlanetGravity`, and of no other body.

use avian3d::math::{RVector, Real, ToF32Precision};
use avian3d::prelude::*;
use avian_fdm::components::{AircraftCoreBundle, PlanetGravity, PlanetResource};
use avian_fdm::plugin::AircraftFdmPlugin;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

const R: Real = PlanetResource::EARTH_RADIUS_M;
const G0: f32 = PlanetResource::EARTH_SURFACE_GRAVITY_MS2;

/// The bodies fall for a few steps before the check, so the position changes
/// by millimeters. The gravity changes by much less than this tolerance.
const ACC_TOL: f32 = 1e-4;

fn app(planet: Option<PlanetResource>) -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin::default());
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / 60.0,
    )));
    app.insert_resource(Gravity::ZERO);
    if let Some(p) = planet {
        app.insert_resource(p);
    }
    app.finish();
    app
}

fn spawn_aircraft(app: &mut App, position: Vec3) -> Entity {
    app.world_mut()
        .spawn((
            AircraftCoreBundle {
                transform: Transform::from_translation(position),
                ..Default::default()
            },
            Collider::sphere(1.0),
        ))
        .id()
}

fn acceleration(app: &App, entity: Entity) -> Vec3 {
    app.world()
        .get::<ConstantLinearAcceleration>(entity)
        .expect("no ConstantLinearAcceleration")
        .0
}

fn assert_vec_near(actual: Vec3, expected: Vec3) {
    assert!(
        (actual - expected).length() <= ACC_TOL,
        "actual {actual}, expected {expected}"
    );
}

/// Point-mass gravity at `altitude_m` above sea level of the Earth planet.
fn expected_g(altitude_m: f32) -> f32 {
    let r = R.f32();
    G0 * (r / (r + altitude_m)).powi(2)
}

fn planet_at_origin() -> PlanetResource {
    PlanetResource {
        center: RVector::ZERO,
        ..PlanetResource::earth()
    }
}

/// Each aircraft gets the gravity at its own position. Two aircraft on
/// different axes get two different directions.
#[test]
fn planet_gravity_direction() {
    let mut app = app(Some(planet_at_origin()));
    let on_y = spawn_aircraft(&mut app, Vec3::new(0.0, (R + 1000.0).f32(), 0.0));
    let on_x = spawn_aircraft(&mut app, Vec3::new((R + 1000.0).f32(), 0.0, 0.0));

    app.update();
    app.update();

    let g = expected_g(1000.0);
    assert_vec_near(acceleration(&app, on_y), Vec3::new(0.0, -g, 0.0));
    assert_vec_near(acceleration(&app, on_x), Vec3::new(-g, 0.0, 0.0));
}

/// Without a planet, the crate does not write gravity. Avian `Gravity` applies.
#[test]
fn planet_gravity_off_without_planet() {
    let mut app = app(None);
    let aircraft = spawn_aircraft(&mut app, Vec3::new(0.0, 1000.0, 0.0));

    app.update();
    app.update();

    assert_eq!(acceleration(&app, aircraft), Vec3::ZERO);
}

/// If the app removes the planet, the last radial value does not stay.
#[test]
fn planet_gravity_cleared_on_removal() {
    let mut app = app(Some(PlanetResource::earth()));
    let aircraft = spawn_aircraft(&mut app, Vec3::new(0.0, 1000.0, 0.0));

    app.update();
    app.update();
    assert_ne!(acceleration(&app, aircraft), Vec3::ZERO);

    app.world_mut().remove_resource::<PlanetResource>();
    app.update();

    assert_eq!(acceleration(&app, aircraft), Vec3::ZERO);
}

/// Avian applies `GravityScale` only to the global `Gravity`. The crate applies
/// it to the planet gravity, so it has the same effect.
#[test]
fn planet_gravity_respects_gravity_scale() {
    let mut app = app(Some(PlanetResource::earth()));
    let aircraft = spawn_aircraft(&mut app, Vec3::new(0.0, 1000.0, 0.0));
    app.world_mut()
        .entity_mut(aircraft)
        .insert(GravityScale(0.5));

    app.update();
    app.update();

    assert_vec_near(
        acceleration(&app, aircraft),
        Vec3::new(0.0, -0.5 * expected_g(1000.0), 0.0),
    );
}

/// A body that is not an aircraft gets the planet gravity if it has the marker.
#[test]
fn planet_gravity_on_marked_body() {
    let mut app = app(Some(PlanetResource::earth()));
    let cargo = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::sphere(1.0),
            Transform::from_xyz(0.0, 1000.0, 0.0),
            PlanetGravity,
        ))
        .id();

    app.update();
    app.update();

    assert_vec_near(
        acceleration(&app, cargo),
        Vec3::new(0.0, -expected_g(1000.0), 0.0),
    );
}

/// Without the marker, the crate does not touch the component. The user owns it.
#[test]
fn planet_gravity_not_on_unmarked_body() {
    let mut app = app(Some(PlanetResource::earth()));
    let user_value = Vec3::new(1.0, 2.0, 3.0);
    let body = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::sphere(1.0),
            Transform::from_xyz(0.0, 1000.0, 0.0),
            ConstantLinearAcceleration(user_value),
        ))
        .id();

    app.update();
    app.update();

    assert_eq!(acceleration(&app, body), user_value);
}

/// An aircraft with no zones at circular-orbit speed stays at the same
/// distance from the center. This checks the sign and the 1/r^2 law end to
/// end, through Avian.
///
/// Only in the `f64` build: in 30 s, the aircraft flies 235 km from the
/// origin, beyond the limit of f32 positions.
#[test]
#[cfg(feature = "f64")]
fn orbit_free_fall() {
    const ALTITUDE_M: f64 = 100_000.0;
    const STEPS: usize = 1800;
    /// Measured: 0.28 m in 30 s, from the time steps. A constant gravity
    /// without the 1/r^2 law gives about 130 m, and no gravity gives about
    /// 4,300 m.
    const RADIUS_TOL_M: f64 = 2.0;

    let planet = PlanetResource::earth();
    let r0 = planet.radius_m + ALTITUDE_M;
    let g = f64::from(planet.surface_gravity_ms2) * (planet.radius_m / r0).powi(2);
    let speed = (g * r0).sqrt();

    let mut app = app(Some(planet.clone()));
    let aircraft = spawn_aircraft(&mut app, Vec3::new(0.0, ALTITUDE_M as f32, 0.0));
    app.world_mut()
        .entity_mut(aircraft)
        .insert(LinearVelocity(Vec3::new(speed as f32, 0.0, 0.0)));

    let mut max_error_m: f64 = 0.0;
    for _ in 0..STEPS {
        app.update();
        let pos = app.world().get::<Position>(aircraft).unwrap().0;
        max_error_m = max_error_m.max(((pos - planet.center).length() - r0).abs());
    }

    assert!(
        max_error_m < RADIUS_TOL_M,
        "the distance from the center changed by up to {max_error_m:.2} m"
    );
}
