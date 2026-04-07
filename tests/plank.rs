//! Plank validation aircraft.
//!
//! A synthetic aircraft with rectangular surfaces and symmetric thin-airfoil
//! coefficients. Every stability derivative has an exact closed-form answer,
//! so any discrepancy between the simulation and the analytical value is a
//! bug in avian_fdm, not a data problem.
//!
//! Development aided by LLM.
//!
//! Geometry (all rectangular, no sweep/taper/twist):
//!   Wing:   b=10m, c=1.5m, S=15m^2, AR=6.667
//!   H-stab: bt=3m, ct=1.0m, St=3m^2, ARt=3.0, arm lt=4.0m
//!   V-stab: bv=1.5m, cv=1.0m, Sv=1.5m^2, ARv=1.5, arm lv=4.0m
//!
//! All lift slopes use the Helmbold finite-wing correction:
//!   a = a0 * AR / (AR + 2), where a0 = 2*pi.

#![cfg(feature = "f32")]

use avian3d::math::Scalar;
use avian3d::prelude::*;
use avian_fdm::components::*;
use avian_fdm::plugin::AircraftFdmPlugin;
use bevy::prelude::*;

use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

// ---------------------------------------------------------------------------
// Geometry constants
// ---------------------------------------------------------------------------

#[allow(clippy::unnecessary_cast)]
const A0: Scalar = 2.0 * std::f64::consts::PI as Scalar; // 2D thin-airfoil lift slope

// Wing
#[allow(clippy::unnecessary_cast)]
const WING_SPAN: Scalar = 10.0; // m
#[allow(clippy::unnecessary_cast)]
const WING_CHORD: Scalar = 1.5; // m
#[allow(clippy::unnecessary_cast)]
const WING_AREA: Scalar = WING_SPAN * WING_CHORD; // 15.0 m^2
#[allow(clippy::unnecessary_cast)]
const WING_AR: Scalar = WING_SPAN / WING_CHORD; // 6.667
#[allow(clippy::unnecessary_cast)]
const WING_CL_ALPHA: Scalar = A0 * WING_AR / (WING_AR + 2.0); // 4.833 /rad
#[allow(clippy::unnecessary_cast)]
const WING_CD0: Scalar = 0.008;

// Horizontal stabilizer
#[allow(clippy::unnecessary_cast)]
const HSTAB_SPAN: Scalar = 3.0;
#[allow(clippy::unnecessary_cast)]
const HSTAB_CHORD: Scalar = 1.0;
#[allow(clippy::unnecessary_cast)]
const HSTAB_AREA: Scalar = HSTAB_SPAN * HSTAB_CHORD; // 3.0 m^2
#[allow(clippy::unnecessary_cast)]
const HSTAB_AR: Scalar = HSTAB_SPAN / HSTAB_CHORD; // 3.0
#[allow(clippy::unnecessary_cast)]
const HSTAB_CL_ALPHA: Scalar = A0 * HSTAB_AR / (HSTAB_AR + 2.0); // 3.770 /rad
#[allow(clippy::unnecessary_cast)]
const HSTAB_ARM: Scalar = 4.0; // m aft of wing AC

// Vertical stabilizer
#[allow(clippy::unnecessary_cast)]
const VSTAB_SPAN: Scalar = 1.5;
#[allow(clippy::unnecessary_cast)]
const VSTAB_CHORD: Scalar = 1.0;
#[allow(clippy::unnecessary_cast)]
const VSTAB_AREA: Scalar = VSTAB_SPAN * VSTAB_CHORD; // 1.5 m^2
#[allow(clippy::unnecessary_cast)]
const VSTAB_AR: Scalar = VSTAB_SPAN / VSTAB_CHORD; // 1.5
                                                   // Fin lift slope (magnitude). CY_beta on the aircraft is NEGATIVE because
                                                   // positive beta (wind from starboard) produces a port-ward side force.
#[allow(clippy::unnecessary_cast)]
const VSTAB_FIN_SLOPE: Scalar = A0 * VSTAB_AR / (VSTAB_AR + 2.0); // 2.693 /rad
#[allow(clippy::unnecessary_cast)]
const VSTAB_ARM: Scalar = 4.0;

// Derived non-dimensional parameters
//
// Tail volume ratio: VH = lt * St / (c * S)
#[allow(clippy::unnecessary_cast)]
const V_H: Scalar = HSTAB_ARM * HSTAB_AREA / (WING_CHORD * WING_AREA);
// Vertical tail volume: VV = lv * Sv / (b * S)
#[allow(clippy::unnecessary_cast)]
const V_V: Scalar = VSTAB_ARM * VSTAB_AREA / (WING_SPAN * WING_AREA);

// ---------------------------------------------------------------------------
// Analytical targets (Nelson 1998 formulas, no downwash)
// ---------------------------------------------------------------------------

// Total lift curve slope:
//   CL_alpha = aw + at * (St/S) = 4.833 + 3.770 * 0.2 = 5.587 /rad
#[allow(clippy::unnecessary_cast)]
const TARGET_CL_ALPHA: Scalar = WING_CL_ALPHA + HSTAB_CL_ALPHA * (HSTAB_AREA / WING_AREA);

// Pitch stiffness (CG at wing AC, no wing pitching moment contribution):
//   Cm_alpha = -at * VH = -3.770 * 0.533 = -2.010 /rad
#[allow(clippy::unnecessary_cast)]
const TARGET_CM_ALPHA: Scalar = -HSTAB_CL_ALPHA * V_H;

// Pitch damping:
//   Cm_q = -2 * at * VH * (lt/c) = -2 * 3.770 * 0.533 * 2.667 = -10.72 /rad
#[allow(clippy::unnecessary_cast)]
const TARGET_CM_Q: Scalar = -2.0 * HSTAB_CL_ALPHA * V_H * (HSTAB_ARM / WING_CHORD);

// Roll damping (discrete: 2 zones at y = +/-WING_SPAN/4):
//   For n discrete zones, Cl_p = -CL_alpha * 2 * sum(S_i * y_i^2) / (S * b^2).
//   With 2 zones at y = +/-2.5, each S/2:
//   Cl_p = -aw * 2 * (2 * (S/2) * (b/4)^2) / (S * b^2) = -aw * 2 * y^2 / b^2 = -aw/8
#[allow(clippy::unnecessary_cast)]
const WING_ZONE_Y: Scalar = WING_SPAN / 4.0; // 2.5 m, center of each half-wing
#[allow(clippy::unnecessary_cast)]
const TARGET_CL_P: Scalar =
    -WING_CL_ALPHA * 2.0 * WING_ZONE_Y * WING_ZONE_Y / (WING_SPAN * WING_SPAN);

// Weathercock stability:
//   The vtail has CY_beta = -a_v (negative: positive beta -> port force).
//   The cross product at x = -l_v gives positive (restoring) yaw torque.
//   Cn_beta = a_v * VV = 2.693 * 0.04 = 0.1077 /rad
#[allow(clippy::unnecessary_cast)]
const TARGET_CN_BETA: Scalar = VSTAB_FIN_SLOPE * V_V;

// Yaw damping:
//   Cn_r = -2 * a_v * VV * (lv/b) = -2 * 2.693 * 0.04 * 0.4 = -0.0862 /rad
#[allow(clippy::unnecessary_cast)]
const TARGET_CN_R: Scalar = -2.0 * VSTAB_FIN_SLOPE * V_V * (VSTAB_ARM / WING_SPAN);

// ---------------------------------------------------------------------------
// AeroCoeff helpers (linear Table1D from slope)
// ---------------------------------------------------------------------------

/// Build a symmetric linear Table1D from a slope over +/-20 degrees.
fn linear_cl(slope: Scalar) -> AeroCoeff {
    let alpha_max: Scalar = (20.0 as Scalar).to_radians();
    AeroCoeff::Table1D {
        breakpoints: vec![-alpha_max, 0.0, alpha_max],
        values: vec![-alpha_max * slope, 0.0, alpha_max * slope],
    }
}

// ---------------------------------------------------------------------------
// Mass layout
// ---------------------------------------------------------------------------

// Target: CG at x = 0.0 (wing AC).
// We use fuselage + engine point-mass to balance.
//
// Body frame: +X forward, +Y starboard, +Z down.
//
// Fuselage box: 5.0 m long, 0.5 m wide, 0.5 m tall, centered at x = -1.5 m.
//   Avian cuboid(hx, hy, hz) uses half-extents -> cuboid(2.5, 0.25, 0.25).
//   Volume for mass: hx * hy * hz = 2.5 * 0.25 * 0.25 = 0.15625 m^3.
//
// Engine mass block: small cube at x = +2.0 m.
//   cuboid(0.25, 0.25, 0.25), volume = 0.015625 m^3.
//
// Wing colliders: thin boxes, density chosen for realistic wing mass.
// Tail colliders: thin boxes, low density.
//
// We want:
//   sum(m_i * x_i) = 0  =>  CG_x = 0
//   sum(m_i) ~ 400 kg
//
// Wing: 2 panels at x=0, mass negligible for CG but contributes to inertia.
//   Each: cuboid(0.75, 2.5, 0.05), density such that wing total ~ 30 kg.
//   Volume per panel = 0.75 * 2.5 * 0.05 = 0.09375 m^3
//   density = 15 / 0.09375 = 160 kg/m^3 per panel -> 30 kg wing total.
//   At x = 0.0, contributes nothing to CG_x.
//
// H-stab: cuboid(0.5, 1.5, 0.03), at x = -4.0.
//   Volume = 0.5 * 1.5 * 0.03 = 0.0225, density = 10/0.0225 = 444 -> ~10 kg at x=-4.
//
// V-stab: cuboid(0.5, 0.03, 0.75), at x = -4.0.
//   Volume = 0.5 * 0.03 * 0.75 = 0.01125, density = 5/0.01125 = 444 -> ~5 kg at x=-4.
//
// Tail mass: 15 kg at x = -4.0, moment = -60.
// Need engine moment to cancel: engine at x = +2.0 needs mass = 30 kg.
//   Volume = 0.015625, density = 30/0.015625 = 1920.
//
// Fuselage carries the rest: 400 - 30 - 15 - 30 = 325 kg.
//   At x = -1.5, moment = -487.5.
//   Total non-fuse moment: wing 0 + tail (-60) + engine (+60) = 0.
//   Fuselage moment: -487.5. That breaks CG = 0.
//
// Let me recalculate. Move fuselage center to x = 0 to keep CG at 0.
//   Fuselage: 5.0 m long centered at x = 0 => from x = +2.5 to x = -2.5.
//   cuboid(2.5, 0.25, 0.25), at x = 0.0. Moment = 0.
//   Tail moment: -60. Engine at x = +2 needs mass = 30 kg. Moment = +60. Balanced.
//   Fuselage mass: 400 - 30 - 15 - 30 = 325 kg.
//   Density = 325 / 0.15625 = 2080.

#[allow(clippy::unnecessary_cast)]
const WING_PANEL_DENSITY: Scalar = 160.0;
#[allow(clippy::unnecessary_cast)]
const HSTAB_DENSITY: Scalar = 444.0;
#[allow(clippy::unnecessary_cast)]
const VSTAB_DENSITY: Scalar = 444.0;
#[allow(clippy::unnecessary_cast)]
const FUSELAGE_DENSITY: Scalar = 2080.0;
#[allow(clippy::unnecessary_cast)]
const ENGINE_DENSITY: Scalar = 1920.0;

// ---------------------------------------------------------------------------
// Spawn helper
// ---------------------------------------------------------------------------

/// Spawn the Plank validation aircraft, returning the root entity.
///
/// The aircraft is oriented in Avian convention: entity +X = forward,
/// but the world orientation must be set via `transform` (typically
/// `Quat::from_rotation_x(PI/2)` to fly in world +X with world +Y = up).
pub fn spawn_plank(commands: &mut Commands, transform: Transform) -> Entity {
    let root = commands
        .spawn((
            AircraftCoreBundle {
                geometry: AircraftGeometry {
                    wing_area_m2: WING_AREA,
                    wing_span_m: WING_SPAN,
                    chord_m: WING_CHORD,
                },
                transform,
                ..Default::default()
            },
            InducedDrag {
                oswald_factor: 0.85,
            },
        ))
        .id();

    // -- Left wing (y < 0 in body frame) --
    commands.entity(root).with_children(|parent| {
        parent.spawn((
            AeroZoneBundle {
                zone: AeroZone {
                    cl: linear_cl(WING_CL_ALPHA),
                    cd: AeroCoeff::Scalar(WING_CD0),
                    area_m2: WING_AREA / 2.0,
                    chord_m: WING_CHORD,
                    ..Default::default()
                },
                collider: Collider::cuboid(0.75, 2.5, 0.05),
                transform: Transform::from_xyz(0.0, -2.5, -0.3),
                ..Default::default()
            },
            ColliderDensity(WING_PANEL_DENSITY as f32),
        ));
    });

    // -- Right wing (y > 0) --
    commands.entity(root).with_children(|parent| {
        parent.spawn((
            AeroZoneBundle {
                zone: AeroZone {
                    cl: linear_cl(WING_CL_ALPHA),
                    cd: AeroCoeff::Scalar(WING_CD0),
                    area_m2: WING_AREA / 2.0,
                    chord_m: WING_CHORD,
                    ..Default::default()
                },
                collider: Collider::cuboid(0.75, 2.5, 0.05),
                transform: Transform::from_xyz(0.0, 2.5, -0.3),
                ..Default::default()
            },
            ColliderDensity(WING_PANEL_DENSITY as f32),
        ));
    });

    // -- Horizontal stabilizer --
    commands.entity(root).with_children(|parent| {
        parent.spawn((
            AeroZoneBundle {
                zone: AeroZone {
                    cl: linear_cl(HSTAB_CL_ALPHA),
                    cd: AeroCoeff::Scalar(0.006),
                    area_m2: HSTAB_AREA,
                    chord_m: HSTAB_CHORD,
                    ..Default::default()
                }
                .with_post_stall_extension(),
                collider: Collider::cuboid(0.5, 1.5, 0.03),
                transform: Transform::from_xyz(-HSTAB_ARM as f32, 0.0, 0.0),
                ..Default::default()
            },
            ColliderDensity(HSTAB_DENSITY as f32),
        ));
    });

    // -- Vertical stabilizer --
    // CY vs beta (sideforce from sideslip). Negative slope: positive beta
    // (wind from starboard) produces port-ward force (vane effect).
    commands.entity(root).with_children(|parent| {
        parent.spawn((
            AeroZoneBundle {
                zone: AeroZone {
                    cl: AeroCoeff::Absent,
                    cd: AeroCoeff::Scalar(0.006),
                    cy: linear_cl(-VSTAB_FIN_SLOPE),
                    area_m2: VSTAB_AREA,
                    chord_m: VSTAB_CHORD,
                    ..Default::default()
                }
                .with_post_stall_extension(),
                collider: Collider::cuboid(0.5, 0.03, 0.75),
                transform: Transform::from_xyz(-VSTAB_ARM as f32, 0.0, -0.75),
                ..Default::default()
            },
            ColliderDensity(VSTAB_DENSITY as f32),
        ));
    });

    // -- Fuselage (mass only) --
    commands.entity(root).with_children(|parent| {
        parent.spawn((
            AeroZoneBundle {
                zone: AeroZone {
                    cl: AeroCoeff::Absent,
                    cd: AeroCoeff::Absent,
                    area_m2: 0.0,
                    ..Default::default()
                },
                collider: Collider::cuboid(2.5, 0.25, 0.25),
                transform: Transform::from_xyz(0.0, 0.0, 0.0),
                ..Default::default()
            },
            ColliderDensity(FUSELAGE_DENSITY as f32),
        ));
    });

    // -- Engine mass block --
    commands.entity(root).with_children(|parent| {
        parent.spawn((
            AeroZoneBundle {
                zone: AeroZone {
                    cl: AeroCoeff::Absent,
                    cd: AeroCoeff::Absent,
                    area_m2: 0.0,
                    ..Default::default()
                },
                collider: Collider::cuboid(0.25, 0.25, 0.25),
                transform: Transform::from_xyz(2.0, 0.0, 0.0),
                ..Default::default()
            },
            ColliderDensity(ENGINE_DENSITY as f32),
        ));
    });

    root
}

// ---------------------------------------------------------------------------
// Test infrastructure
// ---------------------------------------------------------------------------

const PHYSICS_DT: f32 = 1.0 / 60.0;

/// Run a minimal Bevy app for `n_frames` update cycles.
/// Returns the app after stepping.
fn run_plank_app(n_frames: u32, spawn_fn: fn(Commands)) -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AircraftFdmPlugin::default());
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        PHYSICS_DT as f64,
    )));
    app.insert_resource(Gravity(Vec3::ZERO));
    app.add_systems(Startup, spawn_fn);
    app.finish();

    for _ in 0..n_frames {
        app.update();
    }
    app
}

/// Standard Plank orientation: flying in world +X, world +Y = up.
/// Body +X (fwd) -> world +X, body +Z (down) -> world -Y.
fn plank_transform(altitude: f32, speed: f32) -> Transform {
    let _ = speed; // speed is set via LinearVelocity, not transform
    Transform::from_xyz(0.0, altitude, 0.0)
        .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2))
}

// ---------------------------------------------------------------------------
// Query helpers
// ---------------------------------------------------------------------------

fn read_force(app: &mut App) -> Vec3 {
    let world = app.world_mut();
    let mut q = world.query::<&ConstantForce>();
    q.iter(world).next().expect("no ConstantForce").0
}

fn read_torque(app: &mut App) -> Vec3 {
    let world = app.world_mut();
    let mut q = world.query::<&ConstantTorque>();
    q.iter(world).next().expect("no ConstantTorque").0
}

/// ISA density at altitude (m).
fn isa_density(alt_m: Scalar) -> Scalar {
    1.225 * (1.0 - 0.0065 * alt_m / 288.15).powf(4.2561)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Verify mass, CG, and inertia tensor from collider geometry.
#[test]
fn plank_inertia() {
    fn spawn(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    let mut app = run_plank_app(2, spawn);

    let world = app.world_mut();
    let mut query = world.query::<(&ComputedMass, &ComputedCenterOfMass)>();
    let (mass, com) = query.iter(world).next().expect("no aircraft found");

    let total_mass = mass.value() as Scalar;
    let cg_x = com.x as Scalar;

    // Mass should be ~400 kg
    let mass_error = (total_mass - 400.0).abs() / 400.0;
    assert!(
        mass_error < 0.05,
        "mass {total_mass:.1} kg, expected ~400, error {:.1}%",
        mass_error * 100.0
    );

    // CG should be at x ~ 0.0 (wing AC)
    assert!(cg_x.abs() < 0.05, "CG_x = {cg_x:.4} m, expected ~0.0");

    eprintln!("Plank: mass = {total_mass:.1} kg, CG_x = {cg_x:.4} m");
}

/// Verify emergent CL_alpha matches the analytical value.
///
/// Method: spawn at two different alphas, measure the Z-force difference.
/// CL_alpha = delta_Fz / (q * S * delta_alpha).
#[test]
fn plank_cl_alpha() {
    // We test by reading the ConstantForce after one physics step at two
    // different angles of attack. At alpha=0 the symmetric airfoil produces
    // zero lift, at alpha=2deg it produces CL_alpha * alpha.
    let alpha_deg: Scalar = 2.0;
    let alpha_rad: Scalar = alpha_deg.to_radians();
    let v: Scalar = 50.0;

    fn spawn_alpha0(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    fn spawn_alpha2(mut commands: Commands) {
        let root = spawn_plank(
            &mut commands,
            Transform::from_xyz(0.0, 500.0, 0.0).with_rotation(
                // Pitch up by 2 deg in body frame.
                // Body pitch = rotation around world Z (since body Y-starboard
                // maps to world Z after the PI/2 rotation_x).
                Quat::from_rotation_z(2.0_f32.to_radians())
                    * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ),
        );
        // Velocity still along world +X (unchanged freestream).
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    let mut app0 = run_plank_app(2, spawn_alpha0);
    let mut app2 = run_plank_app(2, spawn_alpha2);

    let force0 = read_force(&mut app0);
    let force2 = read_force(&mut app2);

    // Lift is world +Y (upward). At alpha=0 symmetric airfoil gives ~0 lift.
    let lift0 = force0.y as Scalar;
    let lift2 = force2.y as Scalar;
    let delta_lift = lift2 - lift0;

    let rho = isa_density(500.0);
    let q = 0.5 * rho * v * v;

    // CL_alpha_emergent = delta_lift / (q * S * delta_alpha)
    let cl_alpha_emergent = delta_lift / (q * WING_AREA * alpha_rad);

    let error = (cl_alpha_emergent - TARGET_CL_ALPHA).abs() / TARGET_CL_ALPHA;
    eprintln!(
        "Plank CL_alpha: emergent = {cl_alpha_emergent:.3}, target = {TARGET_CL_ALPHA:.3}, error = {:.1}%",
        error * 100.0
    );
    eprintln!("  lift0 = {lift0:.1} N, lift2 = {lift2:.1} N, delta = {delta_lift:.1} N");
    eprintln!("  q = {q:.1} Pa, alpha = {alpha_deg} deg");

    assert!(
        error < 0.10,
        "CL_alpha error {:.1}% exceeds 10% tolerance",
        error * 100.0
    );
}

/// Verify emergent Cm_alpha matches the analytical value.
///
/// Method: spawn at alpha=0 and alpha=2deg, read pitch torque difference.
/// Cm_alpha = delta_M / (q * S * c * delta_alpha).
#[test]
fn plank_cm_alpha() {
    let alpha_deg: Scalar = 2.0;
    let alpha_rad: Scalar = alpha_deg.to_radians();
    let v: Scalar = 50.0;

    fn spawn_alpha0(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    fn spawn_alpha2(mut commands: Commands) {
        let root = spawn_plank(
            &mut commands,
            Transform::from_xyz(0.0, 500.0, 0.0).with_rotation(
                Quat::from_rotation_z(2.0_f32.to_radians())
                    * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ),
        );
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    let mut app0 = run_plank_app(2, spawn_alpha0);
    let mut app2 = run_plank_app(2, spawn_alpha2);

    // Pitch torque is around body Y (starboard) = world Z after rotation_x(PI/2).
    let torque0 = read_torque(&mut app0);
    let torque2 = read_torque(&mut app2);

    let pitch0 = torque0.z as Scalar;
    let pitch2 = torque2.z as Scalar;
    let delta_pitch = pitch2 - pitch0;

    let rho = isa_density(500.0);
    let q = 0.5 * rho * v * v;

    let cm_alpha_emergent = delta_pitch / (q * WING_AREA * WING_CHORD * alpha_rad);

    let error = (cm_alpha_emergent - TARGET_CM_ALPHA).abs() / TARGET_CM_ALPHA.abs();
    eprintln!(
        "Plank Cm_alpha: emergent = {cm_alpha_emergent:.3}, target = {TARGET_CM_ALPHA:.3}, error = {:.1}%",
        error * 100.0
    );
    eprintln!("  pitch0 = {pitch0:.1} Nm, pitch2 = {pitch2:.1} Nm, delta = {delta_pitch:.1} Nm");

    assert!(
        error < 0.10,
        "Cm_alpha error {:.1}% exceeds 10% tolerance",
        error * 100.0
    );
}

/// Verify emergent Cm_q (pitch damping) matches the analytical value.
///
/// Method: spawn with zero and nonzero pitch rate, measure delta pitch torque.
/// Cm_q = delta_M / (q_bar * S * c * (q_rate * c / (2V))).
#[test]
fn plank_cm_q() {
    let v: Scalar = 50.0;
    let q_rate: Scalar = (5.0 as Scalar).to_radians(); // 5 deg/s pitch rate

    fn spawn_q0(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    fn spawn_q5(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands.entity(root).insert((
            LinearVelocity(Vec3::new(50.0, 0.0, 0.0)),
            // Body pitch rate q -> world Z angular velocity
            AngularVelocity(Vec3::new(0.0, 0.0, 5.0_f32.to_radians())),
        ));
    }

    let mut app0 = run_plank_app(2, spawn_q0);
    let mut app5 = run_plank_app(2, spawn_q5);

    let torque0 = read_torque(&mut app0);
    let torque5 = read_torque(&mut app5);

    let delta_pitch = (torque5.z - torque0.z) as Scalar;

    let rho = isa_density(500.0);
    let q_bar = 0.5 * rho * v * v;
    let q_hat = q_rate * WING_CHORD / (2.0 * v); // non-dimensional pitch rate

    let cm_q_emergent = delta_pitch / (q_bar * WING_AREA * WING_CHORD * q_hat);

    let error = (cm_q_emergent - TARGET_CM_Q).abs() / TARGET_CM_Q.abs();
    eprintln!(
        "Plank Cm_q: emergent = {cm_q_emergent:.3}, target = {TARGET_CM_Q:.3}, error = {:.1}%",
        error * 100.0
    );

    assert!(
        error < 0.15,
        "Cm_q error {:.1}% exceeds 15% tolerance",
        error * 100.0
    );
}

/// Verify emergent Cl_p (roll damping) matches the analytical value.
///
/// Method: spawn with zero and nonzero roll rate, measure delta roll torque.
/// Cl_p = delta_L / (q_bar * S * b * (p * b / (2V))).
#[test]
fn plank_cl_p() {
    let v: Scalar = 50.0;
    let p_rate: Scalar = (10.0 as Scalar).to_radians(); // 10 deg/s roll rate

    fn spawn_p0(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    fn spawn_p10(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands.entity(root).insert((
            LinearVelocity(Vec3::new(50.0, 0.0, 0.0)),
            // Body roll rate p -> world X angular velocity
            AngularVelocity(Vec3::new(10.0_f32.to_radians(), 0.0, 0.0)),
        ));
    }

    let mut app0 = run_plank_app(2, spawn_p0);
    let mut app10 = run_plank_app(2, spawn_p10);

    let torque0 = read_torque(&mut app0);
    let torque10 = read_torque(&mut app10);

    // Roll torque is around body X (forward) = world X after rotation_x(PI/2).
    let delta_roll = (torque10.x - torque0.x) as Scalar;

    let rho = isa_density(500.0);
    let q_bar = 0.5 * rho * v * v;
    let p_hat = p_rate * WING_SPAN / (2.0 * v); // non-dimensional roll rate

    let cl_p_emergent = delta_roll / (q_bar * WING_AREA * WING_SPAN * p_hat);

    let error = (cl_p_emergent - TARGET_CL_P).abs() / TARGET_CL_P.abs();
    eprintln!(
        "Plank Cl_p: emergent = {cl_p_emergent:.3}, target = {TARGET_CL_P:.3}, error = {:.1}%",
        error * 100.0
    );

    assert!(
        error < 0.15,
        "Cl_p error {:.1}% exceeds 15% tolerance",
        error * 100.0
    );
}

/// Verify emergent Cn_beta (weathercock stability) matches the analytical value.
///
/// Method: spawn at zero and nonzero sideslip, read yaw torque difference.
/// Cn_beta = delta_N / (q_bar * S * b * delta_beta).
#[test]
fn plank_cn_beta() {
    let beta_deg: Scalar = 3.0;
    let beta_rad = beta_deg.to_radians();
    let v: Scalar = 50.0;

    fn spawn_beta0(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    fn spawn_beta3(mut commands: Commands) {
        let root = spawn_plank(
            &mut commands,
            Transform::from_xyz(0.0, 500.0, 0.0).with_rotation(
                // Yaw 3 deg: rotate around world Y (body Z-down maps to world -Y,
                // body yaw = rotation around body Z).
                // After rotation_x(PI/2): body Z -> world -Y.
                // A yaw angle means the nose points slightly off the velocity vector.
                // In world frame, rotate around Y axis.
                Quat::from_rotation_y(3.0_f32.to_radians())
                    * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ),
        );
        // Velocity still along world +X (unchanged freestream).
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    let mut app0 = run_plank_app(2, spawn_beta0);
    let mut app3 = run_plank_app(2, spawn_beta3);

    let torque0 = read_torque(&mut app0);
    let torque3 = read_torque(&mut app3);

    // Yaw torque is around body Z (down) = world -Y after rotation_x(PI/2).
    let yaw0 = -torque0.y as Scalar;
    let yaw3 = -torque3.y as Scalar;
    let delta_yaw = yaw3 - yaw0;

    let rho = isa_density(500.0);
    let q_bar = 0.5 * rho * v * v;

    let cn_beta_emergent = delta_yaw / (q_bar * WING_AREA * WING_SPAN * beta_rad);

    let error = (cn_beta_emergent - TARGET_CN_BETA).abs() / TARGET_CN_BETA.abs();
    eprintln!(
        "Plank Cn_beta: emergent = {cn_beta_emergent:.3}, target = {TARGET_CN_BETA:.3}, error = {:.1}%",
        error * 100.0
    );

    assert!(
        error < 0.15,
        "Cn_beta error {:.1}% exceeds 15% tolerance",
        error * 100.0
    );
}

/// Verify emergent Cn_r (yaw damping) matches the analytical value.
///
/// Method: spawn with zero and nonzero yaw rate, measure delta yaw torque.
/// Cn_r = delta_N / (q_bar * S * b * (r * b / (2V))).
#[test]
fn plank_cn_r() {
    let v: Scalar = 50.0;
    let r_rate: Scalar = (5.0 as Scalar).to_radians(); // 5 deg/s yaw rate

    fn spawn_r0(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands
            .entity(root)
            .insert(LinearVelocity(Vec3::new(50.0, 0.0, 0.0)));
    }

    fn spawn_r5(mut commands: Commands) {
        let root = spawn_plank(&mut commands, plank_transform(500.0, 50.0));
        commands.entity(root).insert((
            LinearVelocity(Vec3::new(50.0, 0.0, 0.0)),
            // Body yaw rate r (around body Z-down) -> world -Y angular velocity
            AngularVelocity(Vec3::new(0.0, -(5.0_f32.to_radians()), 0.0)),
        ));
    }

    let mut app0 = run_plank_app(2, spawn_r0);
    let mut app5 = run_plank_app(2, spawn_r5);

    let torque0 = read_torque(&mut app0);
    let torque5 = read_torque(&mut app5);

    // Yaw torque around body Z (down) = world -Y.
    let delta_yaw = (-(torque5.y - torque0.y)) as Scalar;

    let rho = isa_density(500.0);
    let q_bar = 0.5 * rho * v * v;
    let r_hat = r_rate * WING_SPAN / (2.0 * v); // non-dimensional yaw rate

    let cn_r_emergent = delta_yaw / (q_bar * WING_AREA * WING_SPAN * r_hat);

    let error = (cn_r_emergent - TARGET_CN_R).abs() / TARGET_CN_R.abs();
    eprintln!(
        "Plank Cn_r: emergent = {cn_r_emergent:.3}, target = {TARGET_CN_R:.3}, error = {:.1}%",
        error * 100.0
    );

    assert!(
        error < 0.20,
        "Cn_r error {:.1}% exceeds 20% tolerance",
        error * 100.0
    );
}
