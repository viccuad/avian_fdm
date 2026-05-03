//! Integration tests for the airfoil name resolution system.
//!
//! Verifies that `resolve_airfoil_names` correctly populates AeroZone fields
//! from the AirfoilLibrary at spawn time.

#![cfg(feature = "f32")]

use avian3d::prelude::*;
use avian_fdm::airfoil::{AirfoilData, RegisterAirfoil};
use avian_fdm::components::aero_coeff::AeroCoeff;
use avian_fdm::components::{AeroZone, AeroZoneBundle};
use avian_fdm::plugin::AircraftFdmPlugin;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

fn make_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin::default());
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / 60.0,
    )));
    app.insert_resource(Gravity(Vec3::ZERO));
    app.finish();
    app
}

fn read_zone(app: &mut App) -> AeroZone {
    let world = app.world_mut();
    let mut q = world.query::<&AeroZone>();
    q.iter(world).next().expect("no AeroZone").clone()
}

/// Placeholder cl/cd are overwritten when airfoil_name matches a library entry.
#[test]
fn placeholder_fields_resolved_from_library() {
    let mut app = make_app();
    app.register_airfoil(
        "TestFoil",
        AirfoilData {
            cl: AeroCoeff::Scalar(3.19),
            cd: AeroCoeff::Scalar(0.07),
            cm: AeroCoeff::Absent,
        },
    );

    app.world_mut().spawn(AeroZoneBundle {
        zone: AeroZone {
            airfoil_name: "TestFoil".into(),
            // cl and cd stay Placeholder (default), resolution should fill them
            ..Default::default()
        },
        collider: Collider::cuboid(1.0, 0.1, 1.0),
        ..Default::default()
    });

    // Two updates: command flush + PreUpdate resolution
    app.update();
    app.update();

    let zone = read_zone(&mut app);
    assert_eq!(
        zone.cl,
        AeroCoeff::Scalar(3.19),
        "cl should be resolved from library"
    );
    assert_eq!(
        zone.cd,
        AeroCoeff::Scalar(0.07),
        "cd should be resolved from library"
    );
}

/// Explicit (non-Placeholder) cl is NOT overwritten by the resolution system.
#[test]
fn explicit_field_not_overwritten() {
    let mut app = make_app();
    app.register_airfoil(
        "TestFoil",
        AirfoilData {
            cl: AeroCoeff::Scalar(99.0),
            cd: AeroCoeff::Scalar(99.0),
            cm: AeroCoeff::Absent,
        },
    );

    let explicit_cl = AeroCoeff::Scalar(1.23);
    app.world_mut().spawn(AeroZoneBundle {
        zone: AeroZone {
            airfoil_name: "TestFoil".into(),
            cl: explicit_cl.clone(), // explicitly set, must not be overwritten
            ..Default::default()
        },
        collider: Collider::cuboid(1.0, 0.1, 1.0),
        ..Default::default()
    });

    app.update();
    app.update();

    let zone = read_zone(&mut app);
    assert_eq!(zone.cl, explicit_cl, "explicit cl must not be overwritten");
    assert_eq!(
        zone.cd,
        AeroCoeff::Scalar(99.0),
        "placeholder cd should still be resolved"
    );
}

/// Absent cl is NOT overwritten (Absent means intentionally no data).
#[test]
fn absent_field_not_overwritten() {
    let mut app = make_app();
    app.register_airfoil(
        "TestFoil",
        AirfoilData {
            cl: AeroCoeff::Scalar(5.0),
            cd: AeroCoeff::Scalar(0.05),
            cm: AeroCoeff::Scalar(-0.02),
        },
    );

    app.world_mut().spawn(AeroZoneBundle {
        zone: AeroZone {
            airfoil_name: "TestFoil".into(),
            cm: AeroCoeff::Absent, // intentionally absent
            ..Default::default()
        },
        collider: Collider::cuboid(1.0, 0.1, 1.0),
        ..Default::default()
    });

    app.update();
    app.update();

    let zone = read_zone(&mut app);
    assert_eq!(
        zone.cm,
        AeroCoeff::Absent,
        "Absent cm must not be overwritten"
    );
}

/// Empty airfoil_name: no resolution, no warning, fields stay as-is.
#[test]
fn empty_airfoil_name_skipped() {
    let mut app = make_app();

    app.world_mut().spawn(AeroZoneBundle {
        zone: AeroZone {
            airfoil_name: "".into(), // default: no named airfoil
            ..Default::default()
        },
        collider: Collider::cuboid(1.0, 0.1, 1.0),
        ..Default::default()
    });

    app.update();
    app.update();

    let zone = read_zone(&mut app);
    // Fields should stay at their defaults (Placeholder)
    assert_eq!(
        zone.cl,
        AeroCoeff::Placeholder,
        "cl should stay Placeholder when airfoil_name is empty"
    );
}

/// Unknown airfoil name: fields stay Placeholder (and a warn_once fires).
#[test]
fn unknown_airfoil_name_leaves_placeholder() {
    let mut app = make_app();

    app.world_mut().spawn(AeroZoneBundle {
        zone: AeroZone {
            airfoil_name: "ThisDoesNotExist".into(),
            ..Default::default()
        },
        collider: Collider::cuboid(1.0, 0.1, 1.0),
        ..Default::default()
    });

    app.update();
    app.update();

    let zone = read_zone(&mut app);
    assert_eq!(
        zone.cl,
        AeroCoeff::Placeholder,
        "cl should stay Placeholder for unknown airfoil"
    );
}

/// Explicitly registered airfoil resolves on zone spawn.
#[test]
fn registered_airfoil_resolves() {
    let mut app = make_app();

    // Register a hand-built airfoil (no builtins exist; callers always register).
    app.register_airfoil(
        "TestFoil",
        AirfoilData {
            cl: AeroCoeff::Scalar(5.0),
            cd: AeroCoeff::Scalar(0.02),
            cm: AeroCoeff::Absent,
        },
    );

    app.world_mut().spawn(AeroZoneBundle {
        zone: AeroZone {
            airfoil_name: "TestFoil".into(),
            ..Default::default()
        },
        collider: Collider::cuboid(1.0, 0.1, 1.0),
        ..Default::default()
    });

    app.update();
    app.update();

    let zone = read_zone(&mut app);
    assert!(
        !matches!(zone.cl, AeroCoeff::Placeholder),
        "TestFoil cl should be resolved from library"
    );
    assert!(
        !matches!(zone.cd, AeroCoeff::Placeholder),
        "TestFoil cd should be resolved from library"
    );
}
