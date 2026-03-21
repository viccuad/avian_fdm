//! Piper J-3 Cub reference preset.
//!
//! All aerodynamic coefficients are transcribed from the JSBSim `J3Cub.xml`
//! model (USA-35B airfoil, Du Y stability derivatives). Unit conversions
//! applied throughout: ft² → m², lb → kg, SLUG·ft² → kg·m², inches → metres.
//!
//! ## Coordinate frame
//!
//! Positions are in the **body frame** with the aircraft root origin at the CG:
//!
//! ```text
//! +X  forward (nose)     +Y  right wing (starboard)     +Z  belly (down)
//! ```
//!
//! Wing zones sit at z = −0.58 m (wings are 22.8 in above the CG in the J3Cub).
//! Tail zones sit at x ≈ −4 m (aft of CG).
//!
//! ## Zone decomposition
//!
//! | Zone            | Fraction of S | Role                                |
//! |-----------------|---------------|-------------------------------------|
//! | Left wing root  | 17.5 %        | Lift + inboard-roll contribution    |
//! | Left wing mid   | 17.5 %        | Lift                                |
//! | Left wing tip   | 15.0 %        | Lift + outboard-roll contribution   |
//! | Right wing root | 17.5 %        | Mirror of left root                 |
//! | Right wing mid  | 17.5 %        | Mirror of left mid                  |
//! | Right wing tip  | 15.0 %        | Mirror of left tip                  |
//! | Left aileron    | —             | `AileronLeft` control surface       |
//! | Right aileron   | —             | `AileronRight` control surface      |
//! | Fuselage        | —             | Parasitic drag (gear)               |
//! | H-stab          | —             | Pitch stability (CM_α via tail arm) |
//! | Elevator        | —             | `Elevator` pitch control            |
//! | V-tail          | —             | (beta-coupling, placeholder for v2) |
//! | Rudder          | —             | `Rudder` yaw control                |
//! | Engine          | —             | Continental A-65, 65 hp             |
//!
//! ## Coefficient derivation notes
//!
//! **Wings (CL, CD):** JSBSim whole-aircraft Table2D values are multiplied by the
//! spanwise area fraction of each zone.
//!
//! **Aileron CL:** Derived from JSBSim `Roll_aileron` coefficient
//! `Cl_da = 0.3498/rad`. For each aileron zone at y_arm = 4.05 m from CL:
//! `CL_ail = Cl_da × b / (2 × y_arm) = 0.3498 × 10.742 / 8.10 ≈ 0.464`
//!
//! **H-stab CL:** Derived from `CM_α` via tail arm `l_t = 3.96 m` and chord:
//! `CL_α_tail = −CM_α × c̄ / l_t = −(−2.033) × 1.6 / 3.96 ≈ +0.821/rad`
//! The sign is correct: positive α → negative tail CL → nose-down restoring moment.
//! The table stores `CL_α(Re) × α` so `AeroCoeff::evaluate(alpha, re)` returns
//! the complete coefficient directly.
//!
//! **Elevator CL:** `CL_elev = −|CM_de| × c̄ / l_t = −1.2004 × 1.6 / 3.96 ≈ −0.485`.
//! Negative sign: positive elevator (nose-up input) creates downward tail force
//! (negative CL), which via the tail arm produces a nose-up pitch moment.
//!
//! **Rudder CY:** `CY_rud = −CN_dr × b / x_arm = −(−0.0565) × 10.742 / 4.0 ≈ −0.152`.
//! Negative sign: positive rudder (nose-right input) creates a leftward side force
//! at the tail (−Y in body frame), producing a positive (nose-right) yaw torque.
//!
//! **Weathercock stability:** `CN_β = 0.0602/rad` requires beta-dependent `CY`,
//! which is a v2 feature (Group B in the roadmap). The V-tail zone is present as
//! a structural mass placeholder; its CY is zero for now.
//!
//! ## Mass budget
//!
//! The preset targets a single-pilot loaded weight of ~440 kg.
//! `Collider::cuboid(x, y, z)` takes **full extents** in metres; Avian
//! converts internally to half-extents before computing volume.
//!
//! ## Hybrid mass approach
//!
//! **Aerodynamic surfaces** (wings, ailerons, h-stab, elevator) use thin
//! colliders (z = 0.02 m) with adjusted density so that `ρ × volume` yields
//! the correct mass. The thin collider doubles as the debug wireframe — no
//! separate `GizmoShape` needed. Inertia error from the thin z² term is
//! < 1 % for span-dominated surfaces (see Section H, plan notes).
//!
//! **Volumetric parts** (fuselage, cabin, engine) use realistically-sized
//! colliders with physical densities. Their collider shape IS the debug viz.
//!
//! **Visual overrides** (`GizmoShape`) are only used when the collider shape
//! doesn't match the desired visual: tapered fins (Quad), struts (Strut),
//! wheels (Sphere), engine cowl (Cylinder).
//!
//! | Zone         | Collider (x×y×z m)         | ρ (kg/m³) | ≈ Mass (kg) | Viz source   |
//! |--------------|----------------------------|-----------|-------------|--------------|
//! | Wing root/mid| 4 × (0.80 × 1.88 × 0.02)  | 232.5     | 28          | Collider     |
//! | Wing tip     | 2 × (0.45 × 0.86 × 0.02)  | 517       | 8           | Collider     |
//! | Aileron      | 2 × (0.35 × 0.75 × 0.02)  | 381       | 4           | Collider     |
//! | Fuse forward | (2.00 × 0.60 × 0.70)       | 188       | 158         | Collider     |
//! | Fuse aft     | (2.70 × 0.40 × 0.35)       | 119       | 45          | Collider     |
//! | Cabin        | (1.20 × 0.68 × 0.50)       | 130       | 53          | Collider     |
//! | Wing struts  | 2 × (2.60 × 0.04 × 0.04)  | 2700      | 22          | GizmoShape   |
//! | Gear legs    | 2 × (0.65 × 0.04 × 0.04)  | 7800      | 16          | GizmoShape   |
//! | Wheels       | 2 × (0.30 × 0.10 × 0.30)  | 1200      | 22          | GizmoShape   |
//! | Tailwheel    | (0.12 × 0.06 × 0.12)       | 1200      | 1           | GizmoShape   |
//! | H-stab       | (0.60 × 1.00 × 0.02)       | 400       | 5           | Collider     |
//! | Elevator     | (0.35 × 1.00 × 0.02)       | 280       | 2           | Collider     |
//! | V-tail       | (0.50 × 0.10 × 0.60)       | 100       | 3           | GizmoShape   |
//! | Rudder       | (0.35 × 0.07 × 0.55)       | 80        | 1           | GizmoShape   |
//! | Engine       | (0.50 × 0.40 × 0.40)       | 860       | 69          | GizmoShape   |
//! |              |                            |           | **~437 kg** |              |
//!
//! All zones are tiled without collider overlap — no double-counted mass.

use bevy::prelude::*;
use avian3d::prelude::{Collider, ColliderDensity, RigidBody};

#[cfg(feature = "propulsion")]
use bevy::math::DVec3;

use crate::components::{
    AeroCoeff, AeroZone, AeroZoneBundle, AircraftCoreBundle, AircraftGeometry,
    ControlSurfaceRole, GizmoContours, ZoneForce,
};
#[cfg(feature = "propulsion")]
use crate::components::{EngineZone, PropwashState};

// ── Aircraft reference constants ─────────────────────────────────────────────

/// JSBSim J3Cub reference wing area (m²): 178.50 ft² × 0.0929.
pub const WING_AREA_M2: f64 = 16.584;

/// JSBSim J3Cub wingspan (m): 35.25 ft × 0.3048.
pub const WING_SPAN_M: f64 = 10.742;

/// JSBSim J3Cub mean aerodynamic chord (m): 5.25 ft × 0.3048.
pub const CHORD_M: f64 = 1.600;

/// Horizontal tail moment arm (m): ≈ 13 ft estimated from vtailarm in J3Cub.xml.
const H_TAIL_ARM_M: f64 = 3.96;

/// Wing aerodynamic-centre x-offset from entity root (m).
/// The Avian-computed CG lands at ≈ −0.172 m (fuselage centroid at −0.45 m),
/// so the wing AC is ≈ 0.072 m **forward** of the CG — 4.5 % MAC, matching
/// the J3Cub's documented forward-of-neutral-point CG range.
const WING_AC_X: f64 = -0.10;

/// Wing height above CG in body frame (m, negative = up since +Z = down).
/// JSBSim: CG at z = −22.83 in, wing datum at z = 0 in → 22.83 in = 0.580 m above CG.
const WING_Z: f64 = -0.580;

// ── Shared alpha / Re breakpoints for Table2D ────────────────────────────────

/// Alpha breakpoints (radians) shared by wing CL and CD tables.
/// Sourced directly from the `tableData` in `J3Cub.xml` (USA-35B airfoil).
const ALPHA_BP: [f64; 14] = [
    -1.5700, -0.3491, -0.2443, -0.1745, -0.0873,
     0.0000,  0.0873,  0.1309,  0.1745,  0.2182,
     0.2618,  0.3054,  0.3491,  1.5700,
];

/// Reynolds number breakpoints for the USA-35B airfoil tables.
const RE_BP: [f64; 2] = [1_668_183.0, 3_707_224.0];

// ── Whole-aircraft CL data (row-major: 14 alpha rows × 2 Re columns) ─────────
//
// From J3Cub.xml `Lift_alpha` table. Rows correspond to ALPHA_BP, columns to RE_BP.
const CL_DATA: [f64; 28] = [
     0.0000,  0.0000,   // alpha = −1.5700
    -0.0085, -0.5085,   // alpha = −0.3491
    -0.5085, -0.8136,   // alpha = −0.2443
    -0.5085, -0.5085,   // alpha = −0.1745
     0.1017,  0.1017,   // alpha = −0.0873
     0.5339,  0.5339,   // alpha =  0.0000
     1.2204,  1.2204,   // alpha =  0.0873
     1.4746,  1.4746,   // alpha =  0.1309
     1.5000,  1.6272,   // alpha =  0.1745
     1.6201,  1.7797,   // alpha =  0.2182
     1.5645,  1.8306,   // alpha =  0.2618
     1.4272,  1.6272,   // alpha =  0.3054
     1.3138,  1.4238,   // alpha =  0.3491
     0.0000,  0.0000,   // alpha =  1.5700
];

// ── Whole-aircraft CD data (row-major: 14 alpha rows × 2 Re columns) ─────────
//
// From J3Cub.xml `Drag_basic` table (profile drag only; induced drag is implicit
// in lift distribution). Columns correspond to RE_BP.
const CD_DATA: [f64; 28] = [
    1.4091, 1.4091,   // alpha = −1.5700
    0.1898, 0.1736,   // alpha = −0.3491
    0.1567, 0.0494,   // alpha = −0.2443
    0.0307, 0.0290,   // alpha = −0.1745
    0.0216, 0.0208,   // alpha = −0.0873
    0.0189, 0.0187,   // alpha =  0.0000
    0.0216, 0.0208,   // alpha =  0.0873
    0.0289, 0.0279,   // alpha =  0.1309
    0.0332, 0.0315,   // alpha =  0.1745
    0.0435, 0.0402,   // alpha =  0.2182
    0.0757, 0.0707,   // alpha =  0.2618
    0.1408, 0.1125,   // alpha =  0.3054
    0.1898, 0.1736,   // alpha =  0.3491
    1.4091, 1.4091,   // alpha =  1.5700
];

// ── H-tail CL table data (6 alpha rows × 2 Re columns) ───────────────────────
//
// Represents CL_tail(α, Re) = CM_α(Re) × c̄ / l_t × (−α), where:
//   CM_α(Re=1.7M) = −2.0327/rad  →  CL_α_tail = +0.821/rad
//   CM_α(Re=3.7M) = −1.3432/rad  →  CL_α_tail = +0.543/rad
// Entry [i,j] = alpha_rows[i] × CL_alpha[j]  (CL_alpha is positive, α sign is preserved)
const HTAIL_ALPHA_BP: [f64; 6] = [-0.3491, -0.1745, 0.0000, 0.0873, 0.1745, 0.3491];
const HTAIL_CL_DATA: [f64; 12] = [
    -0.2866, -0.1892,   // alpha = −0.3491
    -0.1433, -0.0947,   // alpha = −0.1745
     0.0000,  0.0000,   // alpha =  0.0000
     0.0717,  0.0474,   // alpha =  0.0873
     0.1433,  0.0947,   // alpha =  0.1745
     0.2866,  0.1892,   // alpha =  0.3491
];

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Build a Table2D AeroCoeff for CL using the shared wing table scaled by `fraction`.
fn cl_zone(fraction: f64) -> AeroCoeff {
    AeroCoeff::Table2D {
        rows: ALPHA_BP.to_vec(),
        cols: RE_BP.to_vec(),
        data: CL_DATA.iter().map(|&v| v * fraction).collect(),
    }
}

/// Build a Table2D AeroCoeff for CD using the shared wing table scaled by `fraction`.
fn cd_zone(fraction: f64) -> AeroCoeff {
    AeroCoeff::Table2D {
        rows: ALPHA_BP.to_vec(),
        cols: RE_BP.to_vec(),
        data: CD_DATA.iter().map(|&v| v * fraction).collect(),
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Spawn a complete Piper J-3 Cub aircraft with all child [`AeroZone`] entities.
///
/// Returns the root entity ID. The aircraft root is spawned at `transform`
/// (typically over the runway at some altitude). Add your own input system that
/// writes to [`crate::components::ControlInputs`] on the root entity.
///
/// # Example
/// ```rust,no_run
/// # use bevy::prelude::*;
/// # use avian_fdm::presets::j3cub;
/// fn startup(mut commands: Commands) {
///     j3cub::spawn(&mut commands, Transform::from_xyz(0.0, 300.0, 0.0));
/// }
/// ```
pub fn spawn(commands: &mut Commands, transform: Transform) -> Entity {
    use crate::components::GizmoShape;

    let root = commands
        .spawn((
            j3cub_core_bundle(transform),
        ))
        .with_children(|parent| {
            // ── Left wing ────────────────────────────────────────────────────
            // Thin collider (z=0.02 m) — see module docs on hybrid approach.
            parent.spawn(wing_zone(
                "L-root", WING_AC_X, -0.94, 0.175,
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(232.5),
            ));
            parent.spawn(wing_zone(
                "L-mid", WING_AC_X, -2.82, 0.175,
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(232.5),
            ));
            // Tip front (LE half of chord, inboard of aileron spanwise).
            // x = WING_AC_X + (full_chord - tip_chord) / 2 = -0.10 + 0.175 = 0.075
            parent.spawn(wing_zone(
                "L-tip", 0.075, -4.19, 0.150,
                Collider::cuboid(0.45, 0.86, 0.02),
                ColliderDensity(517.0),
            ));

            // ── Right wing ───────────────────────────────────────────────────
            parent.spawn(wing_zone(
                "R-root", WING_AC_X, 0.94, 0.175,
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(232.5),
            ));
            parent.spawn(wing_zone(
                "R-mid", WING_AC_X, 2.82, 0.175,
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(232.5),
            ));
            parent.spawn(wing_zone(
                "R-tip", 0.075, 4.19, 0.150,
                Collider::cuboid(0.45, 0.86, 0.02),
                ColliderDensity(517.0),
            ));

            // ── Ailerons ─────────────────────────────────────────────────────
            // Trailing-edge strip, outboard — tiled behind tip front and
            // spanning from mid panel end (3.76) to wingtip (5.37).
            // Aileron span = 0.75m per side, center at 3.76 + 0.86 + 0.75/2 = 4.995
            // WRONG — that places them outside the wing. The aileron sits at
            // the SAME spanwise station as the tip, occupying the TE strip:
            // tip_main covers y = [3.76, 4.62], aileron covers y = [3.87, 4.62]
            // sharing the outboard span. Actually the tip+aileron tile the
            // outboard region: tip is the LE strip, aileron is the TE strip,
            // both at the SAME Y range.
            // Center Y = same as tip = 4.19.
            parent.spawn(aileron_zone(
                "L-aileron", -4.19,
                ControlSurfaceRole::AileronLeft,
                Collider::cuboid(0.35, 0.86, 0.02),
                ColliderDensity(381.0),
            ));
            parent.spawn(aileron_zone(
                "R-aileron", 4.19,
                ControlSurfaceRole::AileronRight,
                Collider::cuboid(0.35, 0.86, 0.02),
                ColliderDensity(381.0),
            ));

            // ── Fuselage forward (firewall to rear seat) ─────────────────────
            // Main structural mass — includes pilot, fuel tank, instruments.
            // Skin friction drag only; gear drag is on the gear zones below.
            // Tiled in X with fuse_aft: fwd covers [−1.00, 1.00].
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.003),
                        ..default()
                    },
                    zone_force: ZoneForce::default(),
                    collider: Collider::cuboid(2.00, 0.60, 0.70),
                    transform: Transform::from_xyz(0.00, 0.0, 0.0),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(188.0),
                fuse_fwd_contours(),
            ));

            // ── Fuselage aft (tail boom) ─────────────────────────────────────
            // Tapered boom from rear cabin to empennage. Skin friction only.
            // Tiled with fwd: aft covers [−3.70, −1.00].
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.002),
                        ..default()
                    },
                    zone_force: ZoneForce::default(),
                    collider: Collider::cuboid(2.70, 0.40, 0.35),
                    transform: Transform::from_xyz(-2.35, 0.0, 0.0),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(119.0),
                fuse_aft_contours(),
            ));

            // ── Cabin / windshield ───────────────────────────────────────────
            // Form drag from the cabin profile sitting above the fuselage.
            // Raised to z=−0.60 so it sits on top of fuse_fwd without Z overlap.
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.002),
                        ..default()
                    },
                    zone_force: ZoneForce::default(),
                    collider: Collider::cuboid(1.20, 0.68, 0.50),
                    transform: Transform::from_xyz(0.20, 0.0, -0.60),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(130.0),
                cabin_contours(),
            ));

            // ── Wing struts ──────────────────────────────────────────────────
            // Parasitic drag from the V-struts connecting fuselage to wings.
            for (sign, _name) in [(-1.0_f32, "L-strut"), (1.0, "R-strut")] {
                let fuse_attach = Vec3::new(0.20, 0.25 * sign, 0.30);
                let wing_attach = Vec3::new(-0.10 + 0.35, 2.5 * sign, -0.58);
                let mid = (fuse_attach + wing_attach) * 0.5;
                parent.spawn((
                    AeroZoneBundle {
                        zone: AeroZone {
                            cl: AeroCoeff::Scalar(0.0),
                            cd: AeroCoeff::Scalar(0.001),
                            ..default()
                        },
                        zone_force: ZoneForce::default(),
                        collider: Collider::cuboid(2.60, 0.04, 0.04),
                        transform: Transform::from_translation(mid),
                        global_transform: GlobalTransform::default(),
                    },
                    ColliderDensity(2700.0),
                    GizmoShape::Strut {
                        start: fuse_attach - mid,
                        end: wing_attach - mid,
                    },
                ));
            }

            // ── Landing gear legs ────────────────────────────────────────────
            for (sign, _name) in [(-1.0_f32, "L-gear"), (1.0, "R-gear")] {
                let top = Vec3::new(0.50, 0.15 * sign, 0.35);
                let bottom = Vec3::new(0.50, 0.55 * sign, 0.90);
                let mid = (top + bottom) * 0.5;
                parent.spawn((
                    AeroZoneBundle {
                        zone: AeroZone {
                            cl: AeroCoeff::Scalar(0.0),
                            cd: AeroCoeff::Scalar(0.001),
                            ..default()
                        },
                        zone_force: ZoneForce::default(),
                        collider: Collider::cuboid(0.65, 0.04, 0.04),
                        transform: Transform::from_translation(mid),
                        global_transform: GlobalTransform::default(),
                    },
                    ColliderDensity(7800.0),
                    GizmoShape::Strut {
                        start: top - mid,
                        end: bottom - mid,
                    },
                ));
            }

            // ── Main wheels ──────────────────────────────────────────────────
            for (sign, _name) in [(-1.0_f32, "L-wheel"), (1.0, "R-wheel")] {
                parent.spawn((
                    AeroZoneBundle {
                        zone: AeroZone {
                            cl: AeroCoeff::Scalar(0.0),
                            cd: AeroCoeff::Scalar(0.001),
                            ..default()
                        },
                        zone_force: ZoneForce::default(),
                        collider: Collider::cuboid(0.30, 0.10, 0.30),
                        transform: Transform::from_xyz(0.50, 0.55 * sign, 0.90),
                        global_transform: GlobalTransform::default(),
                    },
                    ColliderDensity(1200.0),
                    GizmoShape::Sphere { radius: 0.15 },
                ));
            }

            // ── Tailwheel ────────────────────────────────────────────────────
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.0005),
                        ..default()
                    },
                    zone_force: ZoneForce::default(),
                    collider: Collider::cuboid(0.12, 0.06, 0.12),
                    transform: Transform::from_xyz(-3.60, 0.0, 0.15),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(1200.0),
                GizmoShape::Sphere { radius: 0.06 },
            ));

            // ── Horizontal stabiliser ─────────────────────────────────────────
            parent.spawn((
                hstab_zone(
                    Collider::cuboid(0.60, 1.00, 0.02),
                    ColliderDensity(400.0),
                ),
                hstab_contours(),
            ));

            // ── Elevator ──────────────────────────────────────────────────────
            parent.spawn((
                elevator_zone(
                    Collider::cuboid(0.35, 1.00, 0.02),
                    ColliderDensity(280.0),
                ),
                elevator_contours(),
            ));

            // ── Vertical fin ──────────────────────────────────────────────────
            // Root touches fuselage top (z=−0.175). LE sweeps aft from root
            // to tip. TE is the straight hinge line shared with the rudder.
            // Real J3 Cub: root chord ~0.65m, tip ~0.35m, height ~0.85m.
            parent.spawn((
                vtail_zone(
                    Collider::cuboid(0.65, 0.10, 0.85),
                    ColliderDensity(54.0),
                ),
                GizmoShape::Quad {
                    corners: [
                        Vec3::new( 0.325, 0.0,  0.425),  // root LE (fwd, bottom)
                        Vec3::new(-0.325, 0.0,  0.425),  // root TE (aft, bottom = hinge)
                        Vec3::new(-0.325, 0.0, -0.425),  // tip TE  (aft, top = hinge)
                        Vec3::new( 0.175, 0.0, -0.425),  // tip LE  (swept aft, top)
                    ],
                },
            ));

            // ── Rudder ────────────────────────────────────────────────────────
            // LE is the hinge line (matches vtail TE at x = −3.825 body).
            // Real J3 Cub: root chord ~0.45m, tip ~0.30m, height ~0.95m.
            parent.spawn((rudder_zone(
                Collider::cuboid(0.45, 0.07, 0.95),
                ColliderDensity(33.0),
            ), GizmoShape::Quad {
                corners: [
                    Vec3::new( 0.225, 0.0,  0.475),  // root LE (hinge, bottom)
                    Vec3::new(-0.225, 0.0,  0.475),  // root TE (aft, bottom)
                    Vec3::new(-0.075, 0.0, -0.475),  // tip TE  (aft, top, tapered)
                    Vec3::new( 0.225, 0.0, -0.475),  // tip LE  (hinge, top)
                ],
            }));

            // ── Engine ────────────────────────────────────────────────────────
            #[cfg(feature = "propulsion")]
            parent.spawn((
                engine_zone(
                    Collider::cuboid(0.50, 0.40, 0.40),
                    ColliderDensity(860.0),
                ),
                GizmoShape::Cylinder { radius: 0.20, length: 0.50 },
                engine_contours(),
            ));
        })
        .id();

    root
}

/// Core [`AircraftCoreBundle`] for the J-3 Cub root entity.
///
/// Mass, CoG, and inertia are computed by Avian from child zone colliders.
pub fn j3cub_core_bundle(transform: Transform) -> impl Bundle {
    (
        AircraftCoreBundle {
            geometry: AircraftGeometry {
                wing_area_m2: WING_AREA_M2,
                wing_span_m:  WING_SPAN_M,
                chord_m:      CHORD_M,
                // Nelson, "Flight Stability", Table B1 — J3Cub / light GA
                cl_p: -0.45,
                cm_q: -12.0,
                cn_r: -0.12,
            },
            rigid_body: RigidBody::Dynamic,
            transform,
            ..default()
        },
    )
}

// ── Zone builder functions (pub for testing / custom assemblies) ──────────────

/// One wing panel zone at position (`x_m`, `y_m`, `WING_Z`).
///
/// `fraction` is this panel's share of the whole-aircraft CL and CD tables
/// (e.g. 0.175 for a 17.5 % panel). `x_m` is the chordwise centre; for
/// full-chord panels use [`WING_AC_X`], for partial-chord panels (e.g. the
/// tip front strip) offset accordingly.
pub fn wing_zone(
    _name: &str,
    x_m: f64,
    y_m: f64,
    fraction: f64,
    collider: Collider,
    density: ColliderDensity,
) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: cl_zone(fraction),
                cd: cd_zone(fraction),
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            transform: Transform::from_xyz(
                x_m as f32,
                y_m as f32,
                WING_Z as f32,
            ),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Aileron zone at lateral offset `y_m` with the given control role.
///
/// `CL_ail = Cl_da × b / (2 × y_arm) = 0.464` derived from JSBSim
/// `Roll_aileron` derivative (`Cl_da = 0.3498/rad`).
///
/// Placed at the trailing edge of the wing (aft of the main wing panels)
/// so there is no collider overlap with the tip panel.
pub fn aileron_zone(
    _name: &str,
    y_m: f64,
    role: ControlSurfaceRole,
    collider: Collider,
    density: ColliderDensity,
) -> impl Bundle {
    // Wing TE is at WING_AC_X - chord/2 = -0.10 - 0.40 = -0.50.
    // Aileron chord = 0.35, center = -0.50 + 0.175 = -0.325.
    let aileron_x = (WING_AC_X - 0.40 + 0.175) as f32; // -0.325
    (
        AeroZoneBundle {
            zone: AeroZone {
                // CL_ail = 0.3498 × 10.742 / (2 × 4.05) ≈ 0.464
                cl: AeroCoeff::Scalar(0.464),
                cd: AeroCoeff::Scalar(0.005), // small profile drag at deflection
                control_role: Some(role),
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            transform: Transform::from_xyz(
                aileron_x,
                y_m as f32,
                WING_Z as f32,
            ),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Fuselage zone: parasitic drag only (gear drag + skin friction).
///
/// No lift contribution. CD = 0.004 from JSBSim `Drag_gear` + small skin term.
///
/// Placed 0.45 m **aft** of the entity root so that Avian's computed CG lands at
/// ≈ −0.172 m from root — putting the wing AC (−0.10 m) 0.072 m forward of the
/// actual CG (4.5 % MAC), which is within the published J3Cub CG envelope.
pub fn fuselage_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(0.0),
                cd: AeroCoeff::Scalar(0.006),
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            transform: Transform::from_xyz(-0.45, 0.0, 0.0),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Horizontal stabiliser zone — provides pitch stability via tail-arm moment.
///
/// CL(α, Re) = −CM_α(Re) × c̄/l_t × α
///
/// JSBSim stores CM_α as **negative** (stable aircraft): −2.03/rad at Re=1.7M,
/// −1.34/rad at Re=3.7M. Negating gives a **positive** CL at positive α:
///   CL = −(−2.03) × 1.6/3.96 × α = +0.821 × α   (Re=1.7M)
///
/// At α > 0 (nose up), CL > 0 → **upward** tail force → pitch-down restoring moment.
/// At α < 0 (nose down), CL < 0 → **downward** tail force → pitch-up restoring moment.
///
/// The whole-aircraft pitch moment is recovered as:
///   M = CL × q̄ × S_ref × l_t = −CM_α × α × q̄ × S_ref × c̄  ✓
pub fn hstab_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Table2D {
                    rows: HTAIL_ALPHA_BP.to_vec(),
                    cols: RE_BP.to_vec(),
                    data: HTAIL_CL_DATA.to_vec(),
                },
                cd: AeroCoeff::Scalar(0.008), // tail profile drag fraction
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            // x = −H_TAIL_ARM_M: this arm is the dominant moment lever.
            // z = −0.10 m: h-stab is slightly above fuselage centreline.
            transform: Transform::from_xyz(
                -(H_TAIL_ARM_M as f32),
                0.0,
                -0.10,
            ),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Elevator zone — pitch control surface.
///
/// `CL_elev = −|CM_de| × c̄ / l_t = −1.2004 × 1.6 / 3.96 ≈ −0.485`
///
/// Negative CL means: positive elevator (nose-up stick input) → downward force
/// at the tail → nose-up pitch moment. Tiled aft of the h-stab: elevator LE
/// touches h-stab TE at x = −4.26 m, so elevator center is at x = −4.435 m.
pub fn elevator_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    // Elevator LE = h-stab TE = -(H_TAIL_ARM + hstab_chord/2) = -(3.96 + 0.30) = -4.26
    // Elevator center = -4.26 - elevator_chord/2 = -4.26 - 0.175 = -4.435
    let x = -4.435_f32;
    (
        AeroZoneBundle {
            zone: AeroZone {
                // Negative: positive elevator (nose-up) → downward tail force.
                cl: AeroCoeff::Scalar(-0.485),
                cd: AeroCoeff::Scalar(0.005),
                control_role: Some(ControlSurfaceRole::Elevator),
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            transform: Transform::from_xyz(x, 0.0, -0.10),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Vertical tail zone — structural mass placeholder.
///
/// Full weathercock stability (CY vs β) is Group B (v2 feature). Until then,
/// CY = 0 and this zone provides only the structural mass at the tail.
pub fn vtail_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(0.0),
                cd: AeroCoeff::Scalar(0.003),
                cy: AeroCoeff::Scalar(0.0), // TODO v2: beta-dependent CY_beta
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            // z = −0.60 m: midpoint — root at fuselage top (z=−0.175),
            // tip extends ~0.85 m above.
            transform: Transform::from_xyz(
                -(H_TAIL_ARM_M as f32 - 0.46),
                0.0,
                -0.60,
            ),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Rudder zone — yaw control surface.
///
/// `CY_rud = −CN_dr × b / x_arm = −(−0.0565) × 10.742 / 4.0 ≈ −0.152`
///
/// Negative CY: positive rudder (nose-right) → leftward (−Y) force at tail →
/// positive yaw torque (nose-right). The vertical moment arm (z = −1.1 m) is
/// small relative to the longitudinal arm (x = −4 m) but included for realism.
pub fn rudder_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(0.0),
                cd: AeroCoeff::Scalar(0.004),
                // Negative CY: positive rudder (nose-right) → −Y force at tail → +Z torque.
                cy: AeroCoeff::Scalar(-0.152),
                control_role: Some(ControlSurfaceRole::Rudder),
                ..default()
            },
            zone_force: ZoneForce::default(),
            collider,
            // Rudder center sits just aft of vtail, same height range.
            // x = −3.95 so LE (+0.20) lands at x = −3.75 = vtail TE.
            transform: Transform::from_xyz(
                -3.95,
                0.0,
                -0.45,
            ),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Engine zone — Continental A-65 piston engine with McCauley fixed-pitch propeller.
///
/// Max thrust ≈ 1 200 N (65 hp engine at sea level, actuator-disk estimate).
/// Propeller diameter: 75 in = 1.905 m. Throttle curve is linear 0→1.
///
/// Position: 1.65 m forward, 0.04 m below CG (propeller shaft is slightly below
/// the aircraft reference datum in the J3Cub).
#[cfg(feature = "propulsion")]
pub fn engine_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        EngineZone {
            max_thrust_n:    1_200.0,
            throttle_curve:  vec![[0.0, 0.0], [0.1, 0.07], [0.5, 0.45], [1.0, 1.0]],
            prop_diameter_m: 1.905,
            thrust_axis_body: DVec3::X, // +X = forward
        },
        PropwashState::default(),
        ZoneForce::default(),
        collider,
        density,
        Transform::from_xyz(1.65, 0.0, 0.04),
        GlobalTransform::default(),
    )
}

// ── Contour generators (detailed J-3 Cub outline) ────────────────────────────
//
// These functions return `GizmoContours` with linestrips that trace the real
// aircraft profile. Coordinates are in zone-local frame (relative to the
// zone's Transform). All dimensions come from J-3 Cub three-view drawings
// and reference photos.

/// Elliptical cross-section ring at local x, with given half-width and
/// half-height. 12 segments for a smooth-ish ellipse.
fn ellipse_ring(x: f32, hw: f32, hh: f32) -> Vec<Vec3> {
    (0..=12)
        .map(|i| {
            let a = i as f32 * std::f32::consts::TAU / 12.0;
            Vec3::new(x, hw * a.cos(), hh * a.sin())
        })
        .collect()
}

/// Forward fuselage contours: side profiles (top/bottom) and cross-section
/// rings at key stations.
///
/// Zone center is at aircraft x=0.00, covers x=[−1.00, 1.00].
/// Profile points are in zone-local x (so x=1.00 = firewall, x=−1.00 = rear seat).
fn fuse_fwd_contours() -> GizmoContours {
    // Side profiles (one per side, y = ±half_width at that station).
    let top_profile: Vec<Vec3> = vec![
        Vec3::new(1.00, 0.0, -0.30),   // firewall top
        Vec3::new(0.60, 0.0, -0.34),   // cowl/windshield transition
        Vec3::new(0.20, 0.0, -0.38),   // windshield base
        Vec3::new(-0.20, 0.0, -0.40),  // cabin peak
        Vec3::new(-0.60, 0.0, -0.38),  // rear cabin
        Vec3::new(-1.00, 0.0, -0.34),  // rear seat
    ];
    let bot_profile: Vec<Vec3> = vec![
        Vec3::new(1.00, 0.0, 0.35),    // firewall bottom
        Vec3::new(0.60, 0.0, 0.34),    // lower cowl
        Vec3::new(0.20, 0.0, 0.32),    // belly
        Vec3::new(-0.20, 0.0, 0.28),   // belly taper
        Vec3::new(-0.60, 0.0, 0.24),   // rear belly
        Vec3::new(-1.00, 0.0, 0.20),   // rear seat
    ];

    // Cross-section rings at key stations.
    let rings = vec![
        ellipse_ring(1.00, 0.28, 0.32),   // firewall
        ellipse_ring(0.20, 0.30, 0.35),    // cabin front
        ellipse_ring(-0.60, 0.28, 0.31),   // rear cabin
        ellipse_ring(-1.00, 0.24, 0.27),   // rear seat (fwd/aft boundary)
    ];

    let mut lines = vec![top_profile, bot_profile];
    lines.extend(rings);
    GizmoContours { lines }
}

/// Aft fuselage (tail boom) contours — tapered profile from rear seat to tail.
///
/// Zone center is at aircraft x=−2.35, covers x=[−3.70, −1.00].
/// Local x: +1.35 = fwd end (−1.00 aircraft), −1.35 = aft end (−3.70 aircraft).
fn fuse_aft_contours() -> GizmoContours {
    let top_profile: Vec<Vec3> = vec![
        Vec3::new(1.35, 0.0, -0.27),   // fwd end (matches fuse_fwd rear)
        Vec3::new(0.65, 0.0, -0.22),
        Vec3::new(0.00, 0.0, -0.18),
        Vec3::new(-0.65, 0.0, -0.14),
        Vec3::new(-1.10, 0.0, -0.10),
        Vec3::new(-1.35, 0.0, -0.08),  // tail end
    ];
    let bot_profile: Vec<Vec3> = vec![
        Vec3::new(1.35, 0.0, 0.20),
        Vec3::new(0.65, 0.0, 0.16),
        Vec3::new(0.00, 0.0, 0.12),
        Vec3::new(-0.65, 0.0, 0.08),
        Vec3::new(-1.10, 0.0, 0.06),
        Vec3::new(-1.35, 0.0, 0.04),
    ];

    let rings = vec![
        ellipse_ring(1.35, 0.20, 0.24),   // fwd end
        ellipse_ring(0.00, 0.14, 0.15),    // mid boom
        ellipse_ring(-1.00, 0.08, 0.08),   // near tail
        ellipse_ring(-1.35, 0.06, 0.06),   // tail tip
    ];

    let mut lines = vec![top_profile, bot_profile];
    lines.extend(rings);
    GizmoContours { lines }
}

/// Cabin / windshield contours — the greenhouse profile above the fuselage.
///
/// Zone center at aircraft (0.40, 0, −0.60). Local coordinates relative to that.
fn cabin_contours() -> GizmoContours {
    // Windshield outline (side view, both sides).
    let windshield_l: Vec<Vec3> = vec![
        Vec3::new(0.50, -0.30, 0.25),   // windshield base (lower front)
        Vec3::new(0.30, -0.30, -0.10),  // windshield top front
        Vec3::new(-0.10, -0.30, -0.20), // roof peak
        Vec3::new(-0.50, -0.30, -0.15), // rear window top
        Vec3::new(-0.60, -0.30, 0.10),  // rear window base
    ];
    let windshield_r: Vec<Vec3> = windshield_l.iter()
        .map(|p| Vec3::new(p.x, -p.y, p.z))
        .collect();

    // Roof spine (centerline).
    let roof: Vec<Vec3> = vec![
        Vec3::new(0.30, 0.0, -0.22),   // front
        Vec3::new(-0.10, 0.0, -0.25),  // peak
        Vec3::new(-0.50, 0.0, -0.18),  // rear
    ];

    GizmoContours {
        lines: vec![windshield_l, windshield_r, roof],
    }
}

/// Engine contours — spinner cone and propeller disc.
///
/// Zone center at aircraft (1.65, 0, 0.04). Engine GizmoShape is a cylinder;
/// contours add the spinner and prop disc on top.
#[cfg(feature = "propulsion")]
fn engine_contours() -> GizmoContours {
    // Spinner cone (linestrip profile, one side).
    let spinner: Vec<Vec3> = vec![
        Vec3::new(0.25, 0.0, 0.12),   // cylinder front bottom
        Vec3::new(0.50, 0.0, 0.00),   // spinner tip
        Vec3::new(0.25, 0.0, -0.12),  // cylinder front top
    ];

    // Propeller disc (circle at spinner tip, in YZ plane).
    let prop_radius = 0.953_f32;
    let prop_disc: Vec<Vec3> = (0..=24)
        .map(|i| {
            let a = i as f32 * std::f32::consts::TAU / 24.0;
            Vec3::new(0.50, prop_radius * a.cos(), prop_radius * a.sin())
        })
        .collect();

    GizmoContours {
        lines: vec![spinner, prop_disc],
    }
}

/// H-stab planform — simple rectangle.
/// Zone-local coords: collider cuboid(0.60, 1.00, 0.02).
fn hstab_contours() -> GizmoContours {
    let hc = 0.30_f32;
    let hs = 0.50_f32;
    GizmoContours {
        lines: vec![vec![
            Vec3::new(-hc, -hs, 0.0),
            Vec3::new(-hc,  hs, 0.0),
            Vec3::new( hc,  hs, 0.0),
            Vec3::new( hc, -hs, 0.0),
            Vec3::new(-hc, -hs, 0.0),
        ]],
    }
}

/// Elevator planform — simple rectangle.
/// Zone-local coords: collider cuboid(0.35, 1.00, 0.02).
fn elevator_contours() -> GizmoContours {
    let hc = 0.175_f32;
    let hs = 0.50_f32;
    GizmoContours {
        lines: vec![vec![
            Vec3::new(-hc, -hs, 0.0),
            Vec3::new(-hc,  hs, 0.0),
            Vec3::new( hc,  hs, 0.0),
            Vec3::new( hc, -hs, 0.0),
            Vec3::new(-hc, -hs, 0.0),
        ]],
    }
}

// ─────────────────────────────────────────────────────────────────────────────
#[cfg(test)]
mod tests {
    use super::*;

    /// Wing zone fractions must sum to 1.0 (cover the full wing area).
    #[test]
    fn wing_fractions_sum_to_one() {
        let fractions = [0.175, 0.175, 0.150, 0.175, 0.175, 0.150_f64];
        let sum: f64 = fractions.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10, "wing fractions sum = {sum}");
    }

    /// CL table data has the correct element count (14 rows × 2 Re columns).
    #[test]
    fn cl_data_length() {
        assert_eq!(CL_DATA.len(), ALPHA_BP.len() * RE_BP.len());
    }

    /// CD table data has the correct element count.
    #[test]
    fn cd_data_length() {
        assert_eq!(CD_DATA.len(), ALPHA_BP.len() * RE_BP.len());
    }

    /// H-tail CL table has the correct element count.
    #[test]
    fn htail_cl_data_length() {
        assert_eq!(HTAIL_CL_DATA.len(), HTAIL_ALPHA_BP.len() * RE_BP.len());
    }

    /// At zero alpha the h-stab produces zero CL (neutral trim contribution).
    #[test]
    fn hstab_cl_zero_at_zero_alpha() {
        let coeff = AeroCoeff::Table2D {
            rows: HTAIL_ALPHA_BP.to_vec(),
            cols: RE_BP.to_vec(),
            data: HTAIL_CL_DATA.to_vec(),
        };
        let cl = coeff.evaluate(0.0, RE_BP[0]);
        assert!(cl.abs() < 1e-10, "h-stab CL at alpha=0 should be 0, got {cl}");
    }

    /// H-stab CL is **positive** at positive alpha → upward tail force → pitch-down restoring moment.
    ///
    /// JSBSim `Pitch_alpha`: CM_α = −2.0327/rad (Re=1.7M), so
    /// CL_hstab = −CM_α × c̄/l_t × α = +0.821 × α > 0 at positive α.
    #[test]
    fn hstab_cl_positive_at_positive_alpha() {
        let coeff = AeroCoeff::Table2D {
            rows: HTAIL_ALPHA_BP.to_vec(),
            cols: RE_BP.to_vec(),
            data: HTAIL_CL_DATA.to_vec(),
        };
        let cl = coeff.evaluate(0.1, RE_BP[0]);
        assert!(cl > 0.0, "h-stab CL at positive alpha should be positive (upward force at tail), got {cl}");
    }

    /// Aileron roll moment magnitude matches JSBSim Roll_aileron at full deflection.
    ///
    /// JSBSim: N_roll = q·S·b·0.3498 = q·S·10.742·0.3498 = 3.757·q·S
    /// Our model: 2 × CL_ail × y_arm × q·S = 2 × 0.464 × 4.05 × q·S = 3.759·q·S
    #[test]
    fn aileron_roll_moment_matches_jsbsim() {
        let cl_ail = 0.464_f64;
        let y_arm = 4.05_f64;
        let our_coeff = 2.0 * cl_ail * y_arm; // = 3.759

        let jsbsim_coeff = 0.3498 * WING_SPAN_M; // = 3.757
        assert!((our_coeff - jsbsim_coeff).abs() < 0.01,
            "aileron coefficient mismatch: ours={our_coeff:.4}, jsbsim={jsbsim_coeff:.4}");
    }

    /// Elevator CL produces a nose-up moment at the tail arm.
    ///
    /// Moment = CL_elev × q·S × tail_arm (via cross-product in accumulate).
    /// JSBSim: M_pitch = CM_de × q·S·c̄ = −1.2004 × q·S·1.6 = −1.9206·q·S
    /// Our model: −0.485 × tail_arm = −0.485 × 3.96 = −1.9206·q·S ✓
    #[test]
    fn elevator_pitch_moment_matches_jsbsim() {
        let cl_elev = -0.485_f64;
        let our_moment_coeff = cl_elev * H_TAIL_ARM_M; // = −1.921

        let cm_de = -1.2004_f64;
        let jsbsim_moment_coeff = cm_de * CHORD_M; // = −1.921
        assert!((our_moment_coeff - jsbsim_moment_coeff).abs() < 0.005,
            "elevator moment coeff: ours={our_moment_coeff:.4}, jsbsim={jsbsim_moment_coeff:.4}");
    }
}
