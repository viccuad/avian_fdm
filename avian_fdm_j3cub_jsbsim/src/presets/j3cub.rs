#![allow(clippy::unnecessary_cast)]
//! Piper J-3 Cub reference preset.
//!
//! All aerodynamic coefficients are transcribed from the JSBSim `J3Cub.xml`
//! model (USA-35B airfoil, Du Y stability derivatives). Unit conversions
//! applied throughout: ft² to m², lb to kg, SLUG·ft² to kg·m², inches to metres.
//!
//! Extraction of this preset aided by LLM.
//!
//! ## Coordinate frame
//!
//! Positions are in the **body frame** with the aircraft root origin at the CG:
//!
//! ```text
//! +X  forward (nose)     +Y  right wing (starboard)     +Z  belly (down)
//! ```
//!
//! Wing zones sit at z = −0.590 m (wings are 23.2 in above the CG in the J3Cub).
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
//! | Left aileron    | - | `AileronLeft` control surface       |
//! | Right aileron   | - | `AileronRight` control surface      |
//! | Fuselage        | - | Parasitic drag (gear)               |
//! | H-stab          | - | Pitch stability (CM_α via tail arm) |
//! | Elevator        | - | `Elevator` pitch control            |
//! | V-tail          | - | (beta-coupling, placeholder for v2) |
//! | Rudder          | - | `Rudder` yaw control                |
//! | Engine          | - | Continental A-65, 65 hp             |
//!
//! ## Coefficient derivation notes
//!
//! **Wings (CL, CD):** Each wing zone stores the unscaled USA-35B airfoil
//! CL/CD Table2D data. The zone's `area_m2` field holds the planform area
//! fraction (e.g. root panel = 17.5% of S_ref = 2.90 m2). Force is computed
//! as `CL * q_bar * zone.area_m2`, so the airfoil data is reusable across
//! any number of wing zones without re-scaling.
//!
//! **Aileron CL:** Derived from JSBSim `Roll_aileron` coefficient
//! `Cl_da = 0.3498/rad`. For each aileron zone at y_arm = 4.05 m from CL:
//! `CL_ail = Cl_da × b / (2 × y_arm) = 0.3498 × 10.742 / 8.10 ≈ 0.464`
//!
//! **H-stab CL:** Derived from `CM_α` via tail arm `l_t = 4.023 m` and chord:
//! `CL_α_tail = −CM_α × c̄ / l_t = −(−2.033) × 1.6 / 4.023 ≈ +0.808/rad`
//! The sign is correct: positive α means negative tail CL means nose-down restoring moment.
//! The table stores `CL_α(Re) × α` so `AeroCoeff::evaluate(alpha, re)` returns
//! the complete coefficient directly.
//!
//! **Elevator CL:** `CL_elev = −|CM_de| × c̄ / l_t = −1.2004 × 1.6 / 4.023 ≈ −0.477`.
//! Negative sign: positive elevator (nose-up input) creates downward tail force
//! (negative CL), which via the tail arm produces a nose-up pitch moment.
//!
//! **Rudder CY:** `CY_rud = −CN_dr × b / x_arm = −(−0.0565) × 10.742 / 4.0 ≈ −0.152`.
//! Negative sign: positive rudder (nose-right input) creates a leftward side force
//! at the tail (−Y in body frame), producing a positive (nose-right) yaw torque.
//!
//! **Weathercock stability:** `CY_β = −CN_β × b / x_arm = −0.0602 × 10.742 / 4.0 ≈ −0.162/rad`.
//! The vertical fin generates a side force proportional to sideslip that, at the aft
//! moment arm, produces a restoring yaw moment (positive CN at positive beta).
//! Implemented as a linear Table1D (CY vs beta) on the vtail zone.
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
//! the correct mass. The thin collider doubles as the debug wireframe. No
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
//! | Fuse forward | (2.00 × 0.60 × 0.70)       | 177       | 149         | Collider     |
//! | Fuse aft     | (2.70 × 0.40 × 0.35)       | 144       | 54          | Collider     |
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
//! All zones are tiled without collider overlap. No double-counted mass.

use avian3d::math::{Scalar, Vector};
use avian3d::prelude::{Collider, ColliderDensity, RigidBody};
use bevy_ecs::prelude::*;
use bevy_math::prelude::*;
use bevy_transform::prelude::*;

use avian_fdm::airfoil::AirfoilData;
use avian_fdm::components::{
    AeroCoeff, AeroZone, AeroZoneBundle, AircraftCoreBundle, AircraftGeometry, ControlSurfaceRole,
    EngineZone, GizmoContours, InducedDrag,
};
use avian_fdm::sourced;

// ── Aircraft reference constants ─────────────────────────────────────────────

/// JSBSim J3Cub reference wing area (m²): 178.50 ft² × 0.0929.
pub const WING_AREA_M2: Scalar = sourced!(
    16.584,
    "JSBSim:J3Cub.xml: wing_area 178.50 ft² × 0.0929 m²/ft²"
);

/// JSBSim J3Cub wingspan (m): 35.25 ft × 0.3048.
pub const WING_SPAN_M: Scalar =
    sourced!(10.742, "JSBSim:J3Cub.xml: wingspan 35.25 ft × 0.3048 m/ft");

/// JSBSim J3Cub mean aerodynamic chord (m): 5.25 ft × 0.3048.
pub const CHORD_M: Scalar = sourced!(1.600, "JSBSim:J3Cub.xml: chord 5.25 ft × 0.3048 m/ft");

/// Horizontal tail moment arm (m): 13.20 ft from J3Cub FlightGear repo (rev 1.26).
const H_TAIL_ARM_M: Scalar = sourced!(
    4.023,
    "JSBSim:J3Cub_FlightGear.xml: htailarm = 13.20 ft × 0.3048 m/ft"
);

// ── Horizontal tail geometry ─────────────────────────────────────────────────

/// H-stab span (m): ~10 ft measured from J3 Cub three-view drawings.
const HSTAB_SPAN_M: Scalar = sourced!(
    3.05,
    "Geometry: J3 Cub h-stab span, approx 10 ft from type certificate drawings"
);

/// H-stab chord (m): ~2 ft constant chord.
const HSTAB_CHORD_M: Scalar = sourced!(
    0.61,
    "Geometry: J3 Cub h-stab chord, approx 2 ft from type certificate drawings"
);

/// H-stab planform area (m2): span * chord.
const HSTAB_AREA_M2: Scalar = HSTAB_SPAN_M * HSTAB_CHORD_M; // 1.86 m2

/// H-stab lift curve slope (per radian).
///
/// Thin-airfoil theory corrected for finite span:
///   CL_alpha = 2*pi / (1 + 2/AR)
///
/// With AR = span^2 / area = 3.05^2 / 1.86 = 5.0:
///   CL_alpha = 6.283 / 1.4 = 4.49/rad
///
/// Multiplied by tail efficiency eta_t = 0.90 (dynamic pressure ratio at the
/// tail for a high-wing tractor configuration) and downwash factor
/// (1 - d_epsilon/d_alpha) which for a high-wing is roughly 0.60:
///   CL_alpha_eff = 4.49 * 0.90 * ... no, let's keep it simple.
///
/// The JSBSim CM_alpha = -2.03/rad at Re=1.7M implies an effective tail force
/// gradient of 13.4 N per (Pa * rad) when multiplied by S_ref. With the
/// physical tail area of 1.86 m2, the required CL_alpha_eff = 13.4/1.86 = 7.2/rad.
/// This exceeds the isolated tail value because JSBSim CM_alpha includes
/// downwash reduction, body pitching moment, and fuselage interference.
///
/// We use CL_alpha_eff = 7.2/rad to preserve the calibrated pitch dynamics.
const HSTAB_CL_ALPHA: Scalar = sourced!(
    7.2,
    "Calibration: CL_alpha_eff = CM_alpha_JSBSim × S_ref × c_ref / (S_tail × l_t) = 2.033 × 16.584 × 1.6 / (1.86 × 4.023); includes downwash and body effects"
);

/// Elevator chord (m): ~1.15 ft, trailing edge of h-stab.
const ELEVATOR_CHORD_M: Scalar = sourced!(
    0.35,
    "Geometry: J3 Cub elevator chord, approx 1.15 ft from type certificate drawings"
);

/// Elevator planform area (m2): same span as h-stab times elevator chord.
const ELEVATOR_AREA_M2: Scalar = HSTAB_SPAN_M * ELEVATOR_CHORD_M; // 1.07 m2

/// Elevator CL per radian of deflection.
///
/// From JSBSim CM_de = -1.2004/rad. The whole-aircraft pitch moment from
/// elevator is: M = CM_de * delta * qbar * S_ref * c_ref.
///
/// With physical area: M = CL_elev * delta * qbar * S_elev * l_t.
/// So: CL_elev = CM_de * S_ref * c_ref / (S_elev * l_t)
///             = 1.2004 * 16.584 * 1.6 / (1.07 * 4.023) = 7.40/rad.
///
/// Negative: positive elevator (nose-up stick) produces downward tail force.
const ELEVATOR_CL_DELTA: Scalar = sourced!(
    -7.40,
    "Calibration: CL_elev = |CM_de| × S_ref × c / (S_elev × l_t) = 1.2004 × 16.584 × 1.6 / (1.07 × 4.023); negative for nose-up convention"
);

// ── Vertical tail geometry ───────────────────────────────────────────────────

/// Vertical fin height (m): from three-view drawings, root to tip.
const VFIN_HEIGHT_M: Scalar = sourced!(
    0.85,
    "Geometry: J3 Cub vertical fin height from three-view drawings"
);

/// Vertical fin mean chord (m): average of root (~0.65m) and tip (~0.35m).
const VFIN_MEAN_CHORD_M: Scalar = sourced!(
    0.50,
    "Geometry: J3 Cub vertical fin mean chord, (root 0.65 + tip 0.35) / 2"
);

/// Vertical fin planform area (m2): height * mean chord.
const VFIN_AREA_M2: Scalar = VFIN_HEIGHT_M * VFIN_MEAN_CHORD_M; // 0.425 m2

/// Vertical fin moment arm from CG (m). The fin AC is roughly at 25% of the
/// mean chord, which places it at about x = -3.6 m in body frame.
const VFIN_ARM_M: Scalar = sourced!(
    3.6,
    "Geometry: J3 Cub vertical fin aerodynamic center, approx 25% mean chord aft of fin LE"
);

/// Vertical fin CY per radian of sideslip.
///
/// From JSBSim CN_beta = 0.0602/rad. The whole-aircraft yaw moment from
/// sideslip is: N = CN_beta * beta * qbar * S_ref * b.
///
/// With physical fin area: N = CY_fin * beta * qbar * S_fin * x_arm.
/// So: CY_fin = CN_beta * S_ref * b / (S_fin * x_arm)
///            = 0.0602 * 16.584 * 10.742 / (0.425 * 3.6) = 7.01/rad.
///
/// Negative: positive beta (wind from right) produces leftward force at the
/// tail, restoring the nose toward the wind (weathercock stability).
const VFIN_CY_BETA: Scalar = sourced!(
    -7.01,
    "Calibration: CY_fin = CN_beta × S_ref × b / (S_fin × x_arm) = 0.0602 × 16.584 × 10.742 / (0.425 × 3.6); negative for restoring (weathercock) convention"
);

/// Rudder height (m): extends slightly beyond the fin (horn balance).
const RUDDER_HEIGHT_M: Scalar = sourced!(
    0.95,
    "Geometry: J3 Cub rudder height from three-view drawings"
);

/// Rudder mean chord (m): average of root (~0.45m) and tip (~0.30m).
const RUDDER_MEAN_CHORD_M: Scalar = sourced!(
    0.375,
    "Geometry: J3 Cub rudder mean chord, (root 0.45 + tip 0.30) / 2"
);

/// Rudder planform area (m2): height * mean chord.
const RUDDER_AREA_M2: Scalar = RUDDER_HEIGHT_M * RUDDER_MEAN_CHORD_M; // 0.356 m2

/// Rudder CY per radian of deflection.
///
/// From JSBSim CN_dr = -0.0565/rad. The whole-aircraft yaw moment from
/// rudder is: N = CN_dr * delta_r * qbar * S_ref * b.
///
/// With physical area: N = CY_rud * delta_r * qbar * S_rud * x_arm.
/// So: CY_rud = CN_dr * S_ref * b / (S_rud * x_arm)
///            = 0.0565 * 16.584 * 10.742 / (0.356 * 3.6) = 7.86/rad.
///
/// Negative: positive rudder (nose-right) produces leftward force at tail.
const RUDDER_CY_DELTA: Scalar = sourced!(
    -7.86,
    "Calibration: CY_rud = |CN_dr| × S_ref × b / (S_rud × x_arm) = 0.0565 × 16.584 × 10.742 / (0.356 × 3.6); negative for −Y force convention"
);

// ── Aileron geometry ─────────────────────────────────────────────────────────

/// Aileron span per side (m): occupies the outboard wing tip region.
const AILERON_SPAN_M: Scalar = sourced!(
    0.86,
    "Geometry: J3 Cub aileron span per side, from wing zone layout"
);

/// Aileron effective area (m2): aileron_span * wing_chord.
///
/// The aileron changes the effective camber over the full wing chord, not just
/// the trailing-edge strip. The influenced wing panel area is the correct
/// reference for the lift increment.
const AILERON_AREA_M2: Scalar = AILERON_SPAN_M * CHORD_M; // 1.376 m2

/// Aileron CL per radian of deflection.
///
/// From JSBSim Cl_da = 0.3498/rad. The whole-aircraft roll moment from
/// one aileron: M_roll = CL_ail * delta * qbar * S_ail * y_arm.
/// Two ailerons (differential): M_total = 2 * CL_ail * qbar * S_ail * y_arm * delta.
/// JSBSim: M_total = Cl_da * qbar * S_ref * b * delta.
///
/// So: CL_ail = Cl_da * S_ref * b / (2 * S_ail * y_arm)
///            = 0.3498 * 16.584 * 10.742 / (2 * 1.376 * 4.05) = 5.59/rad.
const AILERON_CL_DELTA: Scalar = sourced!(
    5.59,
    "Calibration: CL_ail = Cl_da × S_ref × b / (2 × S_ail × y_arm) = 0.3498 × 16.584 × 10.742 / (2 × 1.376 × 4.05)"
);

// ── Landing gear geometry ────────────────────────────────────────────────────

/// Gear leg frontal area (m2): exposed axle/bungee strut, approx 0.6m long * 0.04m diameter.
const GEAR_LEG_AREA_M2: Scalar = sourced!(
    0.024,
    "Geometry: J3 Cub gear leg frontal area, 0.6 m × 0.04 m exposed axle + bungee"
);

/// Gear leg drag coefficient (based on frontal area).
///
/// From JSBSim Drag_gear: each leg contributes CD = 0.001 against S_ref.
/// Physical: CD_leg = 0.001 * S_ref / S_leg = 0.001 * 16.584 / 0.024 = 0.691.
/// Typical for a partially faired strut (bare cylinder ~ 1.0-1.2).
const GEAR_LEG_CD: Scalar = sourced!(
    0.691,
    "Calibration: CD_leg = 0.001 × S_ref / S_leg = 0.001 × 16.584 / 0.024; partially faired strut"
);

/// Wheel frontal area (m2): circle with radius 0.15m (8-inch tyre).
const WHEEL_AREA_M2: Scalar = sourced!(
    0.0707,
    "Geometry: J3 Cub main wheel frontal area, pi × 0.15^2"
);

/// Wheel drag coefficient (based on frontal area).
///
/// From JSBSim Drag_gear: each wheel contributes CD = 0.001 against S_ref.
/// Physical: CD_wheel = 0.001 * S_ref / S_wheel = 0.001 * 16.584 / 0.0707 = 0.235.
/// Lower than a bare disc (~0.4-0.6) because JSBSim models it as residual drag.
const WHEEL_CD: Scalar = sourced!(
    0.235,
    "Calibration: CD_wheel = 0.001 × S_ref / S_wheel = 0.001 × 16.584 / 0.0707; JSBSim residual"
);

/// Wing aerodynamic-centre x-offset from entity root (m).
/// The Avian-computed CG lands at ≈ −0.172 m (fuselage centroid at −0.45 m),
/// so the wing AC is ≈ 0.072 m **forward** of the CG. This is 4.5 % MAC, matching
/// the J3Cub's documented forward-of-neutral-point CG range.
const WING_AC_X: Scalar = sourced!(
    -0.10,
    "Geometry: AC at 25% MAC; tuned so Avian CG sits 4.5% MAC forward of AC"
);

/// Wing height above CG in body frame (m, negative = up since +Z = down).
/// JSBSim: CG at z = −23.23 in, wing datum at z = 0 in: 23.23 in = 0.590 m above CG.
const WING_Z: Scalar = sourced!(
    -0.590,
    "JSBSim:J3Cub_FlightGear.xml: CG z = −23.23 in; wing datum z = 0 -> 23.23 in = 0.590 m"
);

/// Geometric dihedral of each wing panel (radians).
/// The J3 Cub has approximately 4 degrees of dihedral. Each wing zone's
/// `Transform` is rotated by this angle about the body X axis so that velocity
/// projection into the zone's local frame naturally captures the dihedral
/// effect (more alpha on the upwind wing at sideslip, providing Cl_beta < 0).
const WING_DIHEDRAL_RAD: Scalar = sourced!(
    0.0698,
    "Geometry: J3 Cub wing dihedral approximately 4 deg; provides Cl_beta lateral stability"
);

// ── Shared alpha / Re breakpoints for Table2D ────────────────────────────────

/// Alpha breakpoints (radians) shared by wing CL and CD tables.
/// Sourced directly from the `tableData` in `J3Cub.xml` (USA-35B airfoil).
/// Kept for use in unit tests; not used in production code (wing zones now receive
/// airfoil data from the `avian_fdm` built-in library).
#[allow(dead_code)]
const ALPHA_BP: [Scalar; 14] = sourced!(
    [
        -1.5700, -0.3491, -0.2443, -0.1745, -0.0873, 0.0000, 0.0873, 0.1309, 0.1745, 0.2182,
        0.2618, 0.3054, 0.3491, 1.5700
    ],
    "JSBSim:J3Cub.xml: alpha breakpoints from Lift_alpha and Drag_basic tableData"
);

/// Reynolds number breakpoints for the USA-35B airfoil tables.
/// Kept for use in unit tests.
#[allow(dead_code)]
const RE_BP: [Scalar; 2] = sourced!(
    [1_668_183.0, 3_707_224.0],
    "JSBSim:J3Cub.xml: Re at cruise (V=27 m/s) and fast cruise (V=40 m/s), chord=1.6 m, ν=1.46e-5"
);

// ── Whole-aircraft CL data (row-major: 14 alpha rows × 2 Re columns) ─────────
//
// From J3Cub.xml `Lift_alpha` table. Rows correspond to ALPHA_BP, columns to RE_BP.
// Kept for use in unit tests.
#[allow(dead_code)]
const CL_DATA: [Scalar; 28] = sourced!(
    [
        0.0000, 0.0000, // alpha = −1.5700
        -0.0085, -0.5085, // alpha = −0.3491
        -0.5085, -0.8136, // alpha = −0.2443
        -0.5085, -0.5085, // alpha = −0.1745
        0.1017, 0.1017, // alpha = −0.0873
        0.5339, 0.5339, // alpha =  0.0000
        1.2204, 1.2204, // alpha =  0.0873
        1.4746, 1.4746, // alpha =  0.1309
        1.5000, 1.6272, // alpha =  0.1745
        1.6201, 1.7797, // alpha =  0.2182
        1.5645, 1.8306, // alpha =  0.2618
        1.4272, 1.6272, // alpha =  0.3054
        1.3138, 1.4238, // alpha =  0.3491
        0.0000, 0.0000, // alpha =  1.5700
    ],
    "JSBSim:J3Cub.xml: Lift_alpha table (USA-35B airfoil); whole-aircraft CL"
);

// ── Whole-aircraft CD data (row-major: 14 alpha rows × 2 Re columns) ─────────
//
// From J3Cub.xml `Drag_basic` table (profile drag only; induced drag is implicit
// in lift distribution). Columns correspond to RE_BP.
// Kept for use in unit tests.
#[allow(dead_code)]
const CD_DATA: [Scalar; 28] = sourced!(
    [
        1.4091, 1.4091, // alpha = −1.5700
        0.1898, 0.1736, // alpha = −0.3491
        0.1567, 0.0494, // alpha = −0.2443
        0.0307, 0.0290, // alpha = −0.1745
        0.0216, 0.0208, // alpha = −0.0873
        0.0189, 0.0187, // alpha =  0.0000
        0.0216, 0.0208, // alpha =  0.0873
        0.0289, 0.0279, // alpha =  0.1309
        0.0332, 0.0315, // alpha =  0.1745
        0.0435, 0.0402, // alpha =  0.2182
        0.0757, 0.0707, // alpha =  0.2618
        0.1408, 0.1125, // alpha =  0.3054
        0.1898, 0.1736, // alpha =  0.3491
        1.4091, 1.4091, // alpha =  1.5700
    ],
    "JSBSim:J3Cub.xml: Drag_basic table (profile drag only, parasite; no induced drag)"
);

// ── Public API ────────────────────────────────────────────────────────────────

/// Spawn a complete Piper J-3 Cub aircraft with all child [`AeroZone`] entities.
///
/// Returns the root entity ID. The aircraft root is spawned at `transform`
/// (typically over the runway at some altitude). Add your own input system that
/// writes to [`avian_fdm::components::ControlInputs`] on the root entity.
///
/// # Example
/// ```rust,no_run
/// # use bevy::prelude::*;
/// # use avian_fdm_j3cub_jsbsim::presets::j3cub;
/// fn startup(mut commands: Commands) {
///     j3cub::spawn(&mut commands, Transform::from_xyz(0.0, 300.0, 0.0));
/// }
/// ```
pub fn spawn(commands: &mut Commands, transform: Transform) -> Entity {
    use crate::airfoils::usa35b;
    use avian_fdm::components::GizmoShape;

    let root = commands
        .spawn((
            j3cub_core_bundle(transform),
            // Lift-induced drag: J3Cub has a high-wing strut-braced layout.
            // e = 0.94 from JSBSim: CD_i = CL² × 0.0485, so e = 1/(π × 0.0485 × AR=6.956)
            InducedDrag {
                oswald_factor: sourced!(
                    0.94,
                    "JSBSim:J3Cub.xml: CD_i = CL²×0.0485 -> e = 1/(π×0.0485×AR=6.956) ≈ 0.94"
                ),
            },
            // No LodDamping. Roll/pitch/yaw damping emerges from zone geometry.
        ))
        .with_children(|parent| {
            // ── Left wing ────────────────────────────────────────────────────
            // Thin collider (z=0.02 m). See module docs on hybrid approach.
            parent.spawn((wing_zone(
                "L-root", WING_AC_X, WING_AC_X, -0.94, 0.175,
                usa35b(),
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: uniform wing density; total wing mass ~80 kg for Ixx=729")),
            ), GizmoShape::Box { x: 0.80, y: 1.88, z: 0.02 }));
            parent.spawn((wing_zone(
                "L-mid", WING_AC_X, WING_AC_X, -2.82, 0.175,
                usa35b(),
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: uniform wing density; total wing mass ~80 kg for Ixx=729")),
            ), GizmoShape::Box { x: 0.80, y: 1.88, z: 0.02 }));
            // Tip strip (LE portion of chord, outboard alongside the aileron).
            // Entity at geometric center of the strip (0.075) for correct
            // collider position; ac_offset inside AeroZone shifts the force
            // application point to WING_AC_X (25% of the full wing chord).
            parent.spawn((wing_zone(
                "L-tip", 0.075, WING_AC_X, -4.19, 0.150,
                usa35b(),
                Collider::cuboid(0.45, 0.86, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: uniform wing density matches panel for physical consistency")),
            ), GizmoShape::Box { x: 0.45, y: 0.86, z: 0.02 }));

            // ── Right wing ───────────────────────────────────────────────────
            parent.spawn((wing_zone(
                "R-root", WING_AC_X, WING_AC_X, 0.94, 0.175,
                usa35b(),
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: uniform wing density; total wing mass ~80 kg for Ixx=729")),
            ), GizmoShape::Box { x: 0.80, y: 1.88, z: 0.02 }));
            parent.spawn((wing_zone(
                "R-mid", WING_AC_X, WING_AC_X, 2.82, 0.175,
                usa35b(),
                Collider::cuboid(0.80, 1.88, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: uniform wing density; total wing mass ~80 kg for Ixx=729")),
            ), GizmoShape::Box { x: 0.80, y: 1.88, z: 0.02 }));
            parent.spawn((wing_zone(
                "R-tip", 0.075, WING_AC_X, 4.19, 0.150,
                usa35b(),
                Collider::cuboid(0.45, 0.86, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: uniform wing density matches panel for physical consistency")),
            ), GizmoShape::Box { x: 0.45, y: 0.86, z: 0.02 }));

            // ── Ailerons ─────────────────────────────────────────────────────
            // Trailing-edge strip, outboard: tiled behind tip front and
            // spanning from mid panel end (3.76) to wingtip (5.37).
            // Aileron span = 0.75m per side, center at 3.76 + 0.86 + 0.75/2 = 4.995
            // WRONG. That places them outside the wing. The aileron sits at
            // the SAME spanwise station as the tip, occupying the TE strip:
            // tip_main covers y = [3.76, 4.62], aileron covers y = [3.87, 4.62]
            // sharing the outboard span. Actually the tip+aileron tile the
            // outboard region: tip is the LE strip, aileron is the TE strip,
            // both at the SAME Y range.
            // Center Y = same as tip = 4.19.
            parent.spawn((aileron_zone(
                "L-aileron", -4.19,
                ControlSurfaceRole::AileronLeft,
                Collider::cuboid(0.35, 0.86, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: same as wing panels (control surface + structure)")),
            ), GizmoShape::Box { x: 0.35, y: 0.86, z: 0.02 }));
            parent.spawn((aileron_zone(
                "R-aileron", 4.19,
                ControlSurfaceRole::AileronRight,
                Collider::cuboid(0.35, 0.86, 0.02),
                ColliderDensity(sourced!(585.0, "Inertia-calibrated: same as wing panels (control surface + structure)")),
            ), GizmoShape::Box { x: 0.35, y: 0.86, z: 0.02 }));

            // ── Fuselage forward (firewall to rear seat) ─────────────────
            // Main structural mass, includes pilot, fuel tank, instruments.
            // Profile drag is already in the wing CD_basic table (Drag_basic).
            // Compact collider centred near CG for low Iyy contribution.
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.0),
                        ..Default::default()
                    },
                    collider: Collider::cuboid(1.33, 0.60, 0.70),
                    transform: Transform::from_xyz(-0.47, 0.0, 0.0),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(sourced!(222.0, "Inertia-calibrated: fuselage + pilot + fuel + instruments; shorter collider concentrates mass near CG for correct Iyy")),
                fuse_fwd_contours(),
            ));

            // ── Fuselage aft (tail boom) ─────────────────────────────────────
            // Tapered boom from rear cabin to empennage.
            // Profile drag included in wing CD_basic table.
            // Shorter and closer to CG than full tailboom for correct Iyy.
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.0),
                        ..Default::default()
                    },
                    collider: Collider::cuboid(2.05, 0.40, 0.35),
                    transform: Transform::from_xyz(-1.82, 0.0, 0.0),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(sourced!(177.0, "Inertia-calibrated: tailboom + control runs; shorter collider reduces Iyy to match Datcom target")),
                fuse_aft_contours(),
            ));

            // ── Cabin / windshield ───────────────────────────────────────────
            // Mass contribution only; drag included in wing CD_basic table.
            // Raised to z=−0.60 so it sits on top of fuse_fwd without Z overlap.
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.0),
                        ..Default::default()
                    },
                    collider: Collider::cuboid(1.20, 0.68, 0.50),
                    transform: Transform::from_xyz(0.20, 0.0, -0.60),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(sourced!(50.0, "Inertia-calibrated: cabin/windshield structure; mass concentrated in fuse_fwd collider instead")),
                cabin_contours(),
            ));

            // ── Wing struts ──────────────────────────────────────────────────
            // J3 Cub has a single N-strut per side running from the lower cabin
            // longeron to the front spar at roughly 60% of the half-span.
            // Reference: J3 Cub three-view drawings, JSBSim FlightGear model.
            //
            // Fuselage end: at the cabin side (y = ±0.55m, near the outer face
            // of the fuse_fwd collider at ±0.60), at the lower cabin longeron
            // (z = +0.15m, just below the cabin floor at z = -0.10m).
            //
            // Wing end: at y = ±3.2m (60% of 5.37m half-span), on the dihedral
            // plane so the strut meets the actual wing surface:
            //   z_wing = WING_Z - |y| * sin(Γ)
            //
            // Mass/structural contribution; drag included in wing CD_basic.
            for (sign, _name) in [(-1.0_f32, "L-strut"), (1.0, "R-strut")] {
                let strut_y = 3.2_f32;
                let wing_z = (WING_Z - strut_y as Scalar * WING_DIHEDRAL_RAD.sin()) as f32;
                let fuse_attach = Vec3::new(WING_AC_X as f32, 0.30 * sign, 0.15);
                let wing_attach = Vec3::new(WING_AC_X as f32, strut_y * sign, wing_z);
                let mid = (fuse_attach + wing_attach) * 0.5;
                let dir = wing_attach - fuse_attach;
                let length = dir.length();
                let rot = Quat::from_rotation_arc(Vec3::X, dir.normalize());
                let half = length * 0.5;
                parent.spawn((
                    AeroZoneBundle {
                        zone: AeroZone {
                            cl: AeroCoeff::Scalar(0.0),
                            cd: AeroCoeff::Scalar(0.0),
                            ..Default::default()
                        },
                        collider: Collider::cuboid(length as Scalar, 0.04, 0.04),
                        transform: Transform::from_translation(mid).with_rotation(rot),
                        global_transform: GlobalTransform::default(),
                    },
                    ColliderDensity(sourced!(1720.0, "Inertia-calibrated: hollow 2024-T3 Al tube; solid density 2700 reduced for tube wall fraction")),
                    GizmoShape::Strut {
                        start: Vec3::new(-half, 0.0, 0.0),
                        end: Vec3::new(half, 0.0, 0.0),
                    },
                ));
            }

            // ── Landing gear legs ────────────────────────────────────────────
            for (sign, _name) in [(-1.0_f32, "L-gear"), (1.0, "R-gear")] {
                let top = Vec3::new(0.50, 0.15 * sign, 0.35);
                let bottom = Vec3::new(0.50, 0.55 * sign, 0.90);
                let mid = (top + bottom) * 0.5;
                let dir = bottom - top;
                let length = dir.length();
                let rot = Quat::from_rotation_arc(Vec3::X, dir.normalize());
                let half = length * 0.5;
                parent.spawn((
                    AeroZoneBundle {
                        zone: AeroZone {
                            cl: AeroCoeff::Scalar(0.0),
                            cd: AeroCoeff::Scalar(GEAR_LEG_CD),
                            area_m2: GEAR_LEG_AREA_M2,
                            ..Default::default()
                        },
                        collider: Collider::cuboid(length as Scalar, 0.04, 0.04),
                        transform: Transform::from_translation(mid).with_rotation(rot),
                        global_transform: GlobalTransform::default(),
                    },
                    ColliderDensity(sourced!(7800.0, "Literature: steel axle/bungee landing gear; standard mild steel density")),
                    GizmoShape::Strut {
                        start: Vec3::new(-half, 0.0, 0.0),
                        end: Vec3::new(half, 0.0, 0.0),
                    },
                ));
            }

            // ── Main wheels ──────────────────────────────────────────────────
            for (sign, _name) in [(-1.0_f32, "L-wheel"), (1.0, "R-wheel")] {
                parent.spawn((
                    AeroZoneBundle {
                        zone: AeroZone {
                            cl: AeroCoeff::Scalar(0.0),
                            cd: AeroCoeff::Scalar(WHEEL_CD),
                            area_m2: WHEEL_AREA_M2,
                            ..Default::default()
                        },
                        collider: Collider::cuboid(0.30, 0.10, 0.30),
                        transform: Transform::from_xyz(0.50, 0.55 * sign, 0.90),
                        global_transform: GlobalTransform::default(),
                    },
                    ColliderDensity(sourced!(1200.0, "Estimate: 8-ply tyre + aluminium rim; composite density ≈ rubber 1100 + Al 2700")),
                    // Wheels roll around Y (spanwise axis); radius 0.15 m, width 0.10 m.
                    GizmoShape::Cylinder { radius: 0.15, length: 0.10, axis: Vec3::Y },
                ));
            }

            // ── Tailwheel ────────────────────────────────────────────────────
            parent.spawn((
                AeroZoneBundle {
                    zone: AeroZone {
                        cl: AeroCoeff::Scalar(0.0),
                        cd: AeroCoeff::Scalar(0.0),
                        ..Default::default()
                    },
                    collider: Collider::cuboid(0.12, 0.06, 0.12),
                    transform: Transform::from_xyz(-3.60, 0.0, 0.15),
                    global_transform: GlobalTransform::default(),
                },
                ColliderDensity(sourced!(1200.0, "Estimate: tailwheel tyre + stub axle; same composite density as main wheels")),
                // Tailwheel rolls around Y; radius 0.06 m, width 0.06 m.
                GizmoShape::Cylinder { radius: 0.06, length: 0.06, axis: Vec3::Y },
            ));

            // ── Horizontal stabiliser ─────────────────────────────────────────
            parent.spawn((
                hstab_zone(
                    Collider::cuboid(0.60, 1.00, 0.02),
                    ColliderDensity(sourced!(150.0, "Inertia-calibrated: h-stab fabric/tube structure; ~1.8 kg total")),
                ),
                hstab_contours(),
            ));

            // ── Elevator ──────────────────────────────────────────────────────
            parent.spawn((
                elevator_zone(
                    Collider::cuboid(0.35, 1.00, 0.02),
                    ColliderDensity(sourced!(100.0, "Inertia-calibrated: elevator lighter than h-stab; ~0.7 kg total")),
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
                    ColliderDensity(sourced!(30.0, "Inertia-calibrated: vertical fin fabric/wood structure; ~1.7 kg")),
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
                ColliderDensity(sourced!(15.0, "Inertia-calibrated: rudder lighter than fin; ~0.4 kg")),
            ), GizmoShape::Quad {
                corners: [
                    Vec3::new( 0.225, 0.0,  0.475),  // root LE (hinge, bottom)
                    Vec3::new(-0.225, 0.0,  0.475),  // root TE (aft, bottom)
                    Vec3::new(-0.075, 0.0, -0.475),  // tip TE  (aft, top, tapered)
                    Vec3::new( 0.225, 0.0, -0.475),  // tip LE  (hinge, top)
                ],
            }));

            // ── Engine ────────────────────────────────────────────────────────
            parent.spawn((
                engine_zone(
                    Collider::cuboid(0.50, 0.40, 0.40),
                    ColliderDensity(sourced!(860.0, "JSBSim:J3Cub.xml: Continental A-65 dry mass ≈ 69 kg; 69/(0.50×0.40×0.40)≈862")),
                ),
                GizmoShape::Cylinder { radius: 0.20, length: 0.50, axis: Vec3::X },
                engine_contours(),
            ));
        })
        .id();

    root
}

/// Core [`AircraftCoreBundle`] for the J-3 Cub root entity.
///
/// Mass, CoG, and inertia are computed by Avian from child zone colliders.
///
/// Pair with [`InducedDrag`] (already included by [`spawn`]) for lift-induced
/// drag.  No [`LodDamping`](avian_fdm::components::LodDamping). Roll/pitch/yaw
/// damping emerges from per-zone local α/β physics.
pub fn j3cub_core_bundle(transform: Transform) -> impl Bundle {
    (AircraftCoreBundle {
        geometry: AircraftGeometry {
            wing_area_m2: WING_AREA_M2,
            wing_span_m: WING_SPAN_M,
            chord_m: CHORD_M,
        },
        rigid_body: RigidBody::Dynamic,
        transform,
        ..Default::default()
    },)
}

// ── Zone builder functions (pub for testing / custom assemblies) ──────────────

/// One wing panel zone at position (`x_m`, `y_m`, on the dihedral plane).
///
/// `x_m` is the entity and collider center along the chord axis (physical
/// position, used for mass distribution). `ac_x_m` is the aerodynamic center
/// where lift forces are applied; for all wing panels this should be
/// `WING_AC_X` regardless of how the chord is partitioned. When
/// `ac_x_m == x_m` the `ac_offset` inside [`AeroZone`] is zero.
///
/// `fraction` is the fraction of the total wing area this panel represents.
/// The panel's aerodynamic area is `fraction * WING_AREA_M2` and the CL/CD
/// tables are taken from `airfoil` (unscaled).
#[allow(clippy::too_many_arguments)]
pub fn wing_zone(
    _name: &str,
    x_m: Scalar,
    ac_x_m: Scalar,
    y_m: Scalar,
    fraction: Scalar,
    airfoil: AirfoilData,
    collider: Collider,
    density: ColliderDensity,
) -> impl Bundle {
    let ac_offset = Vec3::new((ac_x_m - x_m) as f32, 0.0, 0.0);
    let z_m = WING_Z - y_m.abs() * WING_DIHEDRAL_RAD.sin();
    let dihedral_rot = Quat::from_rotation_x(-(WING_DIHEDRAL_RAD * y_m.signum()) as f32);
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: airfoil.cl,
                cd: airfoil.cd,
                ac_offset,
                area_m2: fraction * WING_AREA_M2,
                chord_m: CHORD_M,
                ..Default::default()
            }
            .with_post_stall_extension(),
            collider,
            transform: Transform::from_xyz(x_m as f32, y_m as f32, z_m as f32)
                .with_rotation(dihedral_rot),
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
    y_m: Scalar,
    role: ControlSurfaceRole,
    collider: Collider,
    density: ColliderDensity,
) -> impl Bundle {
    // Wing TE is at WING_AC_X - chord/2 = -0.10 - 0.40 = -0.50.
    // Aileron chord = 0.35, center = -0.50 + 0.175 = -0.325.
    let aileron_x = (WING_AC_X - 0.40 + 0.175) as f32; // -0.325
    let z_m = WING_Z - y_m.abs() * WING_DIHEDRAL_RAD.sin();
    let dihedral_rot = Quat::from_rotation_x(-(WING_DIHEDRAL_RAD * y_m.signum()) as f32);
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(AILERON_CL_DELTA),
                cd: AeroCoeff::Scalar(0.0), // included in wing CD_basic
                control_role: Some(role),
                area_m2: AILERON_AREA_M2,
                chord_m: CHORD_M,
                ..Default::default()
            },
            collider,
            transform: Transform::from_xyz(aileron_x, y_m as f32, z_m as f32)
                .with_rotation(dihedral_rot),
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
/// ≈ −0.172 m from root, putting the wing AC (−0.10 m) 0.072 m forward of the
/// actual CG (4.5 % MAC), which is within the published J3Cub CG envelope.
pub fn fuselage_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(0.0),
                cd: AeroCoeff::Scalar(0.0),
                ..Default::default()
            },
            collider,
            transform: Transform::from_xyz(-0.45, 0.0, 0.0),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Horizontal stabiliser zone: provides pitch stability via tail-arm moment.
///
/// Uses the physical h-stab planform area (HSTAB_AREA_M2 = 1.86 m2) and an
/// effective lift curve slope (HSTAB_CL_ALPHA = 7.2/rad) calibrated to match
/// JSBSim CM_alpha. The CL vs alpha relationship is linear (symmetric airfoil):
///   CL = HSTAB_CL_ALPHA * alpha
///
/// At alpha > 0 (nose up), the h-stab produces positive CL (upward force),
/// which at the aft arm creates a nose-down restoring moment.
///
/// The table is extended to +/-180 deg via Viterna-Corrigan post-stall model,
/// so the h-stab stalls realistically at high alpha and produces flat-plate
/// drag when broadside to the wind. This prevents unrealistic pitch-locking
/// during tumbles and deep stalls.
pub fn hstab_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Table1D {
                    breakpoints: vec![-0.35, 0.0, 0.35],
                    values: vec![-0.35 * HSTAB_CL_ALPHA, 0.0, 0.35 * HSTAB_CL_ALPHA],
                },
                cd: AeroCoeff::Scalar(sourced!(
                    0.01,
                    "Estimate: symmetric airfoil profile drag at low alpha"
                )),
                area_m2: HSTAB_AREA_M2,
                chord_m: HSTAB_CHORD_M,
                ..Default::default()
            }
            .with_post_stall_extension(),
            collider,
            transform: Transform::from_xyz(-(H_TAIL_ARM_M as f32), 0.0, -0.10),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Elevator zone: pitch control surface.
///
/// Uses the physical elevator area (ELEVATOR_AREA_M2 = 1.07 m2) and an
/// effective CL per radian of deflection (ELEVATOR_CL_DELTA = -7.40/rad)
/// calibrated to match JSBSim CM_de.
///
/// Negative CL means: positive elevator (nose-up stick input) produces downward
/// force at the tail, creating a nose-up pitch moment via the tail arm.
pub fn elevator_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    // Elevator LE = h-stab TE = -(H_TAIL_ARM_M + hstab_chord/2)
    let hstab_te_x = -(H_TAIL_ARM_M + HSTAB_CHORD_M / 2.0);
    let elevator_center_x = hstab_te_x - ELEVATOR_CHORD_M / 2.0;
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(ELEVATOR_CL_DELTA),
                cd: AeroCoeff::Scalar(0.0),
                control_role: Some(ControlSurfaceRole::Elevator),
                area_m2: ELEVATOR_AREA_M2,
                chord_m: ELEVATOR_CHORD_M,
                ..Default::default()
            },
            collider,
            transform: Transform::from_xyz(elevator_center_x as f32, 0.0, -0.10),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Vertical tail zone: structural mass and weathercock stability.
///
/// Uses the physical fin planform area (VFIN_AREA_M2 = 0.425 m2) and an
/// effective CY per radian of sideslip (VFIN_CY_BETA = -7.01/rad) calibrated
/// to match JSBSim CN_beta.
///
/// Negative CY_beta: positive sideslip (wind from right) produces a leftward
/// force at the aft tail, generating a restoring (nose-right) yaw moment.
///
/// The CY table is extended to +/-180 deg via Viterna-Corrigan so the fin
/// stalls realistically in deep sideslip and does not lock the aircraft
/// into an unrealistic yaw pattern during tumbles.
pub fn vtail_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(0.0),
                cd: AeroCoeff::Scalar(sourced!(
                    0.01,
                    "Estimate: symmetric airfoil profile drag at low beta"
                )),
                cy: AeroCoeff::Table1D {
                    breakpoints: vec![-avian3d::math::FRAC_PI_2, 0.0, avian3d::math::FRAC_PI_2],
                    values: vec![
                        -VFIN_CY_BETA * avian3d::math::FRAC_PI_2,
                        0.0,
                        VFIN_CY_BETA * avian3d::math::FRAC_PI_2,
                    ],
                },
                area_m2: VFIN_AREA_M2,
                chord_m: VFIN_MEAN_CHORD_M,
                ..Default::default()
            }
            .with_post_stall_extension(),
            collider,
            transform: Transform::from_xyz(-(VFIN_ARM_M as f32), 0.0, -0.60),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Rudder zone: yaw control surface.
///
/// Uses the physical rudder planform area (RUDDER_AREA_M2 = 0.356 m2) and an
/// effective CY per radian of deflection (RUDDER_CY_DELTA = -7.86/rad)
/// calibrated to match JSBSim CN_dr.
///
/// Negative CY: positive rudder (nose-right) produces leftward force at the
/// tail, generating positive (nose-right) yaw torque.
pub fn rudder_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        AeroZoneBundle {
            zone: AeroZone {
                cl: AeroCoeff::Scalar(0.0),
                cd: AeroCoeff::Scalar(0.0),
                cy: AeroCoeff::Scalar(RUDDER_CY_DELTA),
                control_role: Some(ControlSurfaceRole::Rudder),
                area_m2: RUDDER_AREA_M2,
                chord_m: RUDDER_MEAN_CHORD_M,
                ..Default::default()
            },
            collider,
            // Rudder LE is at the fin TE. Fin center at -VFIN_ARM_M, fin extends
            // VFIN_MEAN_CHORD_M/2 aft, so rudder LE = -(VFIN_ARM_M + VFIN_MEAN_CHORD_M/2).
            // Rudder center = rudder LE - RUDDER_MEAN_CHORD_M/2.
            transform: Transform::from_xyz(
                -((VFIN_ARM_M + VFIN_MEAN_CHORD_M / 2.0 + RUDDER_MEAN_CHORD_M / 2.0) as f32),
                0.0,
                -0.45,
            ),
            global_transform: GlobalTransform::default(),
        },
        density,
    )
}

/// Engine zone: Continental A-65 piston engine with fixed-pitch propeller.
///
/// Max thrust ≈ 990 N (65 hp engine at sea level, actuator-disk estimate).
/// Propeller diameter: 74 in = 1.880 m (J3Cub FlightGear repo: prop_74in_2f_NACA).
/// Throttle curve is nonlinear to match JSBSim thrust response.
///
/// Position: 1.31 m forward of CG (J3Cub FlightGear repo: engine at JSBSim x = −37.52 in,
/// CG at x = 13.80 in → body x = (13.80 − (−37.52)) × 0.0254 = +1.306 m ≈ 1.31 m).
/// 0.04 m below CG: propeller shaft is slightly below the aircraft reference datum.
pub fn engine_zone(collider: Collider, density: ColliderDensity) -> impl Bundle {
    (
        EngineZone {
            max_thrust_n:    sourced!(990.0, "Calibration:JSBSim: Continental A-65 (65 hp); peak thrust calibrated to match JSBSim trim at 50 kts / 1000 ft"),
            throttle_curve:  sourced!(
                vec![[0.0, 0.0], [0.5, 0.42], [0.75, 0.64], [1.0, 1.0]],
                "Calibration:JSBSim: nonlinear throttle response matching JSBSim thrust vs throttle setting (prop efficiency drop at low opening)"
            ),
            thrust_axis_body: Vector::X, // +X = forward
            // prop_74in_2f_NACA, fixed-pitch ~22° cruise setting, maxrpm 2300.
            // J_zero ≈ 1.1 at 22° pitch: V_zero = J × n × D ≈ 1.1 × (2300/60) × 1.880 ≈ 79 m/s.
            zero_thrust_speed_ms: Some(sourced!(80.0, "Estimate:J3Cub FlightGear: prop_74in_2f_NACA; J_zero ≈ 1.1 at 22° pitch, maxrpm 2300 -> V = 1.1 × (2300/60) × 1.880 ≈ 79 m/s, rounded to 80")),
        },
        collider,
        density,
        Transform::from_xyz(1.31, 0.0, 0.04),
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
        Vec3::new(1.00, 0.0, -0.30),  // firewall top
        Vec3::new(0.60, 0.0, -0.34),  // cowl/windshield transition
        Vec3::new(0.20, 0.0, -0.38),  // windshield base
        Vec3::new(-0.20, 0.0, -0.40), // cabin peak
        Vec3::new(-0.60, 0.0, -0.38), // rear cabin
        Vec3::new(-1.00, 0.0, -0.34), // rear seat
    ];
    let bot_profile: Vec<Vec3> = vec![
        Vec3::new(1.00, 0.0, 0.35),  // firewall bottom
        Vec3::new(0.60, 0.0, 0.34),  // lower cowl
        Vec3::new(0.20, 0.0, 0.32),  // belly
        Vec3::new(-0.20, 0.0, 0.28), // belly taper
        Vec3::new(-0.60, 0.0, 0.24), // rear belly
        Vec3::new(-1.00, 0.0, 0.20), // rear seat
    ];

    // Cross-section rings at key stations.
    let rings = vec![
        ellipse_ring(1.00, 0.28, 0.32),  // firewall
        ellipse_ring(0.20, 0.30, 0.35),  // cabin front
        ellipse_ring(-0.60, 0.28, 0.31), // rear cabin
        ellipse_ring(-1.00, 0.24, 0.27), // rear seat (fwd/aft boundary)
    ];

    let mut lines = vec![top_profile, bot_profile];
    lines.extend(rings);
    GizmoContours { lines }
}

/// Aft fuselage (tail boom) contours, tapered profile from rear seat to tail.
///
/// Zone center is at aircraft x=−2.35, covers x=[−3.70, −1.00].
/// Local x: +1.35 = fwd end (−1.00 aircraft), −1.35 = aft end (−3.70 aircraft).
fn fuse_aft_contours() -> GizmoContours {
    let top_profile: Vec<Vec3> = vec![
        Vec3::new(1.35, 0.0, -0.27), // fwd end (matches fuse_fwd rear)
        Vec3::new(0.65, 0.0, -0.22),
        Vec3::new(0.00, 0.0, -0.18),
        Vec3::new(-0.65, 0.0, -0.14),
        Vec3::new(-1.10, 0.0, -0.10),
        Vec3::new(-1.35, 0.0, -0.08), // tail end
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
        ellipse_ring(1.35, 0.20, 0.24),  // fwd end
        ellipse_ring(0.00, 0.14, 0.15),  // mid boom
        ellipse_ring(-1.00, 0.08, 0.08), // near tail
        ellipse_ring(-1.35, 0.06, 0.06), // tail tip
    ];

    let mut lines = vec![top_profile, bot_profile];
    lines.extend(rings);
    GizmoContours { lines }
}

/// Cabin / windshield contours, the greenhouse profile above the fuselage.
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
    let windshield_r: Vec<Vec3> = windshield_l
        .iter()
        .map(|p| Vec3::new(p.x, -p.y, p.z))
        .collect();

    // Roof spine (centerline).
    let roof: Vec<Vec3> = vec![
        Vec3::new(0.30, 0.0, -0.22),  // front
        Vec3::new(-0.10, 0.0, -0.25), // peak
        Vec3::new(-0.50, 0.0, -0.18), // rear
    ];

    GizmoContours {
        lines: vec![windshield_l, windshield_r, roof],
    }
}

/// Engine contours, spinner cone and propeller disc.
///
/// Zone center at aircraft (1.65, 0, 0.04). Engine GizmoShape is a cylinder;
/// contours add the spinner and prop disc on top.
fn engine_contours() -> GizmoContours {
    // Spinner cone (linestrip profile, one side).
    let spinner: Vec<Vec3> = vec![
        Vec3::new(0.25, 0.0, 0.12),  // cylinder front bottom
        Vec3::new(0.50, 0.0, 0.00),  // spinner tip
        Vec3::new(0.25, 0.0, -0.12), // cylinder front top
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

/// H-stab planform, simple rectangle.
/// Zone-local coords: collider cuboid(0.60, 1.00, 0.02).
fn hstab_contours() -> GizmoContours {
    let hc = 0.30_f32;
    let hs = 0.50_f32;
    GizmoContours {
        lines: vec![vec![
            Vec3::new(-hc, -hs, 0.0),
            Vec3::new(-hc, hs, 0.0),
            Vec3::new(hc, hs, 0.0),
            Vec3::new(hc, -hs, 0.0),
            Vec3::new(-hc, -hs, 0.0),
        ]],
    }
}

/// Elevator planform, simple rectangle.
/// Zone-local coords: collider cuboid(0.35, 1.00, 0.02).
fn elevator_contours() -> GizmoContours {
    let hc = 0.175_f32;
    let hs = 0.50_f32;
    GizmoContours {
        lines: vec![vec![
            Vec3::new(-hc, -hs, 0.0),
            Vec3::new(-hc, hs, 0.0),
            Vec3::new(hc, hs, 0.0),
            Vec3::new(hc, -hs, 0.0),
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
        let fractions: [Scalar; 6] = [0.175, 0.175, 0.150, 0.175, 0.175, 0.150];
        let sum: Scalar = fractions.iter().sum();
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

    /// H-stab CL is zero at zero alpha (linear model).
    #[test]
    fn hstab_cl_zero_at_zero_alpha() {
        let coeff = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-0.35 * HSTAB_CL_ALPHA, 0.0, 0.35 * HSTAB_CL_ALPHA],
        };
        let cl = coeff.evaluate(0.0, RE_BP[0]);
        assert!(
            cl.abs() < 1e-10,
            "h-stab CL at alpha=0 should be 0, got {cl}"
        );
    }

    /// H-stab CL is **positive** at positive alpha -> upward tail force -> pitch-down restoring moment.
    #[test]
    fn hstab_cl_positive_at_positive_alpha() {
        let coeff = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-0.35 * HSTAB_CL_ALPHA, 0.0, 0.35 * HSTAB_CL_ALPHA],
        };
        let cl = coeff.evaluate(0.1, RE_BP[0]);
        assert!(
            cl > 0.0,
            "h-stab CL at positive alpha should be positive (upward force at tail), got {cl}"
        );
    }

    /// Aileron roll moment magnitude matches JSBSim Roll_aileron at full deflection.
    ///
    /// JSBSim: M_roll = Cl_da * q * S_ref * b = 0.3498 * q * S_ref * 10.742
    /// Our model: 2 * CL_ail * q * S_ail * y_arm
    ///          = 2 * 5.59 * q * 1.376 * 4.05
    #[test]
    fn aileron_roll_moment_matches_jsbsim() {
        let y_arm: Scalar = 4.05;
        let our_coeff = 2.0 * AILERON_CL_DELTA * AILERON_AREA_M2 * y_arm;

        let jsbsim_coeff = 0.3498 * WING_SPAN_M * WING_AREA_M2;
        assert!(
            (our_coeff - jsbsim_coeff).abs() / jsbsim_coeff < 0.01,
            "aileron moment mismatch: ours={our_coeff:.2}, jsbsim={jsbsim_coeff:.2}"
        );
    }

    /// Elevator pitch moment matches JSBSim CM_de at full deflection.
    ///
    /// JSBSim: M_pitch = CM_de * q * S_ref * c = -1.2004 * q * S_ref * 1.6
    /// Our model: CL_elev * q * S_elev * l_t = -7.40 * q * 1.07 * 4.023
    #[test]
    fn elevator_pitch_moment_matches_jsbsim() {
        let our_moment = ELEVATOR_CL_DELTA * ELEVATOR_AREA_M2 * H_TAIL_ARM_M;

        let cm_de: Scalar = -1.2004;
        let jsbsim_moment = cm_de * WING_AREA_M2 * CHORD_M;
        assert!(
            (our_moment - jsbsim_moment).abs() / jsbsim_moment.abs() < 0.01,
            "elevator moment: ours={our_moment:.2}, jsbsim={jsbsim_moment:.2}"
        );
    }

    /// Emergent pitch-damping derivative Cmq from h-stab zone geometry.
    ///
    /// The pitch damping arises because `zone_local_angles` adds
    /// `delta_alpha = -q * x / V` to each zone. For the h-stab at
    /// x = -l_t, a pitch-up (q > 0) increases tail alpha, producing
    /// a nose-down restoring moment. The standard form (Nelson 1998,
    /// eq 4.65) gives the non-dimensional Cmq from the tail:
    ///
    /// Cmq_tail = -2 * CL_alpha_tail * (S_tail / S_ref) * (l_t / c_bar)^2
    ///
    /// Datcom gives Cmq = -6/rad for the J3 Cub. Our model produces
    /// about -10/rad because we do not model downwash lag: during a
    /// pitch transient the wing's downwash field takes time to reach
    /// the tail, reducing the effective delta-alpha. Without that lag
    /// factor (typically 1 - d_epsilon/d_alpha ~ 0.6), our tail sees
    /// the full kinematic alpha increment.
    #[test]
    fn emergent_cmq_from_hstab_geometry() {
        let cmq = -2.0
            * HSTAB_CL_ALPHA
            * (HSTAB_AREA_M2 / WING_AREA_M2)
            * (H_TAIL_ARM_M / CHORD_M).powi(2);

        let datcom_cmq: Scalar = -6.0;

        // Our emergent value should be more negative than Datcom (no downwash lag).
        // Factor of ~1.7 is expected.
        assert!(
            cmq < datcom_cmq,
            "emergent Cmq ({cmq:.1}) should be more negative than Datcom ({datcom_cmq})"
        );
        assert!(
            cmq > datcom_cmq * 3.0,
            "emergent Cmq ({cmq:.1}) should not exceed 3x Datcom ({datcom_cmq})"
        );

        // The ratio tells us what downwash lag factor would reconcile the two.
        // expected: (1 - d_epsilon/d_alpha) ~ Datcom / emergent ~ 0.6
        let downwash_factor = datcom_cmq / cmq;
        assert!(
            downwash_factor > 0.4 && downwash_factor < 0.8,
            "implied downwash factor {downwash_factor:.2} outside [0.4, 0.8]"
        );
    }
}
