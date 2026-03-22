//! Aircraft-level components: geometry and the core spawn bundle.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use avian3d::prelude::{RigidBody, ConstantForce, ConstantTorque};

/// Whole-aircraft angular-rate damping derivatives, used as an **LOD fallback**.
///
/// At full fidelity, roll/pitch/yaw damping emerge naturally from per-zone local
/// α/β corrections (see `zone_local_angles`): the wing tips resist roll, the
/// h-stab resists pitch, the v-tail resists yaw — all from geometry alone.
///
/// Set `AircraftGeometry::lod_damping` to `Some(LodDamping { … })` only when
/// the aircraft has too few zones to produce realistic physical damping (e.g.
/// a single-zone missile body, or a low-fidelity background AI aircraft).
///
/// # Non-dimensional form
///
/// ```text
/// ΔL = Cl_p · (p·b / 2V) · q̄ · S · b     roll  damping → body X
/// ΔM = Cm_q · (q·c̄ / 2V) · q̄ · S · c̄    pitch damping → body Y
/// ΔN = Cn_r · (r·b / 2V) · q̄ · S · b     yaw   damping → body Z
/// ```
///
/// All derivatives should be negative (damping opposes motion).
/// Typical light GA values (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Serialize, Deserialize)]
pub struct LodDamping {
    /// Roll damping derivative ∂Cl/∂p̂, where p̂ = p·b/(2V).
    /// Typical range: −0.4 to −0.5 for light aircraft.
    pub cl_p: f64,
    /// Pitch damping derivative ∂Cm/∂q̂, where q̂ = q·c̄/(2V).
    /// Typical range: −10 to −20 for light aircraft.
    pub cm_q: f64,
    /// Yaw damping derivative ∂Cn/∂r̂, where r̂ = r·b/(2V).
    /// Typical range: −0.1 to −0.15 for light aircraft.
    pub cn_r: f64,
}

/// Wing and tail geometry constants used for aerodynamic non-dimensionalisation
/// (q̄·S, q̄·S·b, q̄·S·c̄).
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AircraftGeometry {
    /// Reference wing area S (m²).
    pub wing_area_m2: f64,
    /// Wing span b (m). Used to non-dimensionalise rolling/yawing moments.
    pub wing_span_m: f64,
    /// Mean aerodynamic chord c̄ (m). Used to non-dimensionalise pitching moment.
    pub chord_m: f64,

    // ── LOD damping (optional) ─────────────────────────────────────────────

    /// Whole-aircraft damping derivatives, applied as an LOD fallback.
    ///
    /// `None` (default) — damping comes entirely from per-zone local α/β
    /// physics.  Use this for any aircraft with a realistic zone layout.
    ///
    /// `Some(LodDamping { … })` — global derivatives are added on top of
    /// zone physics.  Use only for sparse-zone aircraft (single-zone bodies,
    /// low-fidelity AI) that cannot produce adequate damping from geometry.
    pub lod_damping: Option<LodDamping>,

    // ── Induced drag ─────────────────────────────────────────────────────

    /// Oswald span-efficiency factor *e*. Used to compute induced drag:
    ///
    /// ```text
    /// CD_i = CL² / (π · e · AR)
    /// ```
    ///
    /// where AR = b² / S. Typical values: 0.75–0.85 for low-wing monoplanes,
    /// 0.85–0.95 for high-wing (like J3 Cub). Set to 0.0 to disable induced
    /// drag (legacy behaviour).
    pub oswald_factor: f64,
}

/// Core bundle. Spawn on the aircraft root entity.
///
/// Mass, centre of gravity, and inertia are computed automatically by Avian
/// from child colliders' [`avian3d::prelude::ColliderDensity`] values.
///
/// Aerodynamic forces are accumulated each frame into the included
/// [`ConstantForce`] and [`ConstantTorque`] components, which Avian then
/// applies natively via `ForceSystems::ApplyConstantForces`.
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::*;
/// # use bevy::prelude::*;
/// // commands.spawn(AircraftCoreBundle { geometry: AircraftGeometry { wing_area_m2: 16.2, wing_span_m: 10.6, chord_m: 1.53 }, ..default() });
/// ```
#[derive(Bundle, Default)]
pub struct AircraftCoreBundle {
    /// Wing/tail geometry constants.
    pub geometry: AircraftGeometry,
    /// Control surface inputs — write each frame from your input system.
    pub controls: crate::components::ControlInputs,
    /// Derived flight-state quantities (written by the library).
    pub flight_state: crate::components::FlightState,
    /// ISA atmosphere at this entity's altitude (written by the library).
    pub atmosphere: crate::components::AtmosphereState,
    /// Avian rigid body type. Must be `RigidBody::Dynamic`.
    pub rigid_body: RigidBody,
    /// Accumulated aerodynamic + propulsive force (written by the library each
    /// frame). Avian applies this natively via `ForceSystems::ApplyConstantForces`.
    pub constant_force: ConstantForce,
    /// Accumulated aerodynamic + propulsive torque (written by the library each
    /// frame). Avian applies this natively via `ForceSystems::ApplyConstantForces`.
    pub constant_torque: ConstantTorque,
    /// World-space transform.
    pub transform: Transform,
    /// Required by Bevy for transform propagation.
    pub global_transform: GlobalTransform,
}
