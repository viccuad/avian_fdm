//! Integration tests for the altitude of `PlanetResource`.
//!
//! Verifies that `update_atmosphere` and `update_flight_state` use the same
//! altitude, with and without a planet.

use avian3d::math::{RVector, Real, ToF32Precision};
use avian3d::prelude::*;
use avian_fdm::components::{AircraftCoreBundle, AtmosphereState, FlightState, PlanetResource};
use avian_fdm::plugin::AircraftFdmPlugin;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

const R: Real = PlanetResource::EARTH_RADIUS_M;

/// The crate calculates the altitude in `f64` in both builds, and the test
/// positions are exact in `f32`. Thus both builds use the same tolerance.
const ALT_TOL_M: f32 = 1e-3;

/// The atmospheres come from altitudes that agree within `ALT_TOL_M`. This
/// tolerance only allows for the `f32` rounding in the ISA functions.
const DENSITY_REL_TOL: f32 = 1e-3;

/// Flies one aircraft for two frames and returns the values the crate wrote.
fn fly(planet: Option<PlanetResource>, position: Vec3) -> (AtmosphereState, FlightState) {
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

    app.world_mut().spawn((
        AircraftCoreBundle {
            transform: Transform::from_translation(position),
            ..Default::default()
        },
        Collider::sphere(1.0),
    ));

    app.update();
    app.update();

    let world = app.world_mut();
    let mut q = world.query::<(&AtmosphereState, &FlightState)>();
    let (atm, fs) = q.iter(world).next().expect("no aircraft");
    (atm.clone(), fs.clone())
}

fn assert_same_atmosphere(a: &AtmosphereState, b: &AtmosphereState) {
    let rel = |x: f32, y: f32| ((x - y) / y).abs();
    assert!(
        rel(a.density_kgm3, b.density_kgm3) < DENSITY_REL_TOL,
        "density {} against {}",
        a.density_kgm3,
        b.density_kgm3
    );
    assert!(
        rel(a.pressure_pa, b.pressure_pa) < DENSITY_REL_TOL,
        "pressure {} against {}",
        a.pressure_pa,
        b.pressure_pa
    );
    assert!(
        rel(a.temperature_k, b.temperature_k) < DENSITY_REL_TOL,
        "temperature {} against {}",
        a.temperature_k,
        b.temperature_k
    );
}

/// Without a `PlanetResource`, the altitude is the world Y position.
#[test]
fn flat_world_altitude_is_world_y() {
    let (_, fs) = fly(None, Vec3::new(500.0, 3000.0, -200.0));
    assert!(
        (fs.altitude_m - 3000.0).abs() < 1e-2,
        "altitude {} m",
        fs.altitude_m
    );
}

/// A planet with its center below the origin gives the same state as a flat
/// world, for an aircraft above the origin.
#[test]
fn round_planet_below_origin_matches_flat_world() {
    let planet = PlanetResource::default();
    let (atm_round, fs_round) = fly(Some(planet), Vec3::new(0.0, 3000.0, 0.0));
    let (atm_flat, fs_flat) = fly(None, Vec3::new(0.0, 3000.0, 0.0));

    assert!(
        (fs_round.altitude_m - fs_flat.altitude_m).abs() < ALT_TOL_M,
        "altitude {} m against {} m",
        fs_round.altitude_m,
        fs_flat.altitude_m
    );
    assert_same_atmosphere(&atm_round, &atm_flat);
}

/// "Up" is radial. An aircraft on the world X axis has the same altitude and
/// atmosphere as one on the Y axis.
#[test]
fn round_planet_up_is_radial() {
    let planet = PlanetResource {
        center: RVector::ZERO,
        radius_m: R,
    };
    let (atm_x, fs_x) = fly(
        Some(planet.clone()),
        Vec3::new((R + 3000.0).f32(), 0.0, 0.0),
    );
    let (atm_y, fs_y) = fly(Some(planet), Vec3::new(0.0, (R + 3000.0).f32(), 0.0));
    let (atm_flat, _) = fly(None, Vec3::new(0.0, 3000.0, 0.0));

    for fs in [&fs_x, &fs_y] {
        assert!(
            (fs.altitude_m - 3000.0).abs() < ALT_TOL_M,
            "altitude {} m",
            fs.altitude_m
        );
    }
    assert_same_atmosphere(&atm_x, &atm_flat);
    assert_same_atmosphere(&atm_y, &atm_flat);
}

/// `AtmosphereState` and `FlightState.altitude_m` come from the same altitude.
/// The reference atmosphere is the flat-world atmosphere at `fs.altitude_m`.
#[test]
fn atmosphere_matches_flight_state_altitude() {
    let planet = PlanetResource {
        center: RVector::new(100.0, 0.0, -50.0),
        radius_m: R,
    };
    // The aircraft is far from the Y axis. A system that reads the world Y
    // position gets about 20 m, and not about 4000 m.
    let (atm, fs) = fly(Some(planet), Vec3::new((R + 4000.0).f32(), 20.0, 0.0));
    let (atm_ref, _) = fly(None, Vec3::new(0.0, fs.altitude_m, 0.0));

    assert_same_atmosphere(&atm, &atm_ref);
}
