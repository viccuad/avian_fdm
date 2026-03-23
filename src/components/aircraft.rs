//! Aircraft-level components: geometry and the core spawn bundle.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use avian3d::prelude::{RigidBody, ConstantForce, ConstantTorque};

/// Whole-aircraft angular-rate damping derivatives, used as an **LOD fallback**.
///
/// Add this component to the aircraft root entity when the zone layout is too
/// sparse to produce realistic physical damping from geometry alone (e.g. a
/// single-zone missile body, or a low-fidelity AI aircraft).
///
/// **Mutually exclusive with per-zone local α/β** — when this component is
/// present, [`compute_aero_forces`](crate::aerodynamics::compute_aero_forces)
/// evaluates all zones at the global α/β (no rate corrections) and uses these
/// derivatives as the sole source of angular damping.  When absent, per-zone
/// local angles run and damping emerges naturally from zone geometry.
///
/// # Non-dimensional form
///
/// ```text
/// ΔL = Cl_p · (p · b / 2V) · q̄ · S · b     roll  damping → body X
/// ΔM = Cm_q · (q · c̄ / 2V) · q̄ · S · c̄    pitch damping → body Y
/// ΔN = Cn_r · (r · b / 2V) · q̄ · S · b     yaw   damping → body Z
/// ```
///
/// All derivatives should be negative (damping opposes motion).
/// Typical light GA values (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct LodDamping {
    /// Roll damping derivative ∂Cl/∂p̂, where p̂ = p · b/(2V).
    /// Typical range: −0.4 to −0.5 for light aircraft.
    pub cl_p: f64,
    /// Pitch damping derivative ∂Cm/∂q̂, where q̂ = q · c̄/(2V).
    /// Typical range: −10 to −20 for light aircraft.
    pub cm_q: f64,
    /// Yaw damping derivative ∂Cn/∂r̂, where r̂ = r · b/(2V).
    /// Typical range: −0.1 to −0.15 for light aircraft.
    pub cn_r: f64,
}

/// Lift-induced drag model.
///
/// Add this component to the aircraft root entity to enable whole-aircraft
/// induced drag:
///
/// ```text
/// CD_i = CL² / (π · e · AR),   AR = b² / S
/// ```
///
/// **Omit this component** for:
/// - **Gliders** — induced drag is typically already embedded in the wing zone
///   CD tables from measured polar data.
/// - **Missiles and projectiles** — no significant spanwise lift distribution;
///   the bluff-body drag dominates.
/// - **Aircraft whose zone CD tables already include induced drag** — adding
///   this component would double-count.
///
/// **Include this component** for:
/// - Any conventional lifting aircraft whose zone CD comes from a 2-D profile
///   drag polar (JSBSim-style) that separates parasite and induced drag.
///
/// | Aircraft type | Typical *e* |
/// |---|---|
/// | High-wing light aircraft (J3 Cub) | 0.85–0.95 |
/// | Low-wing monoplane | 0.75–0.85 |
/// | Elliptical wing (theoretical ideal) | 1.0 |
/// | Delta / swept wing | 0.6–0.75 |
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct InducedDrag {
    /// Oswald span-efficiency factor *e* ∈ (0, 1].
    ///
    /// Accounts for non-elliptical span loading, wingtip vortex losses, and
    /// fuselage interference.  Higher is better; 1.0 is the theoretical
    /// elliptical ideal.
    pub oswald_factor: f64,
}

/// Wing and tail geometry constants used for aerodynamic non-dimensionalisation
/// (q̄ · S, q̄ · S · b, q̄ · S · c̄).
///
/// Lives on the **aircraft root entity** as part of [`AircraftCoreBundle`].
///
/// Optional components — add separately to the root entity as needed:
/// - [`LodDamping`] — explicit damping derivatives for sparse-zone LOD aircraft.
/// - [`InducedDrag`] — lift-induced drag for conventional lifting aircraft.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AircraftGeometry {
    /// Reference wing area S (m²).
    pub wing_area_m2: f64,
    /// Wing span b (m). Used to non-dimensionalise rolling/yawing moments.
    pub wing_span_m: f64,
    /// Mean aerodynamic chord c̄ (m). Used to non-dimensionalise pitching moment.
    pub chord_m: f64,
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
/// # Optional components (add to the same entity after spawning)
///
/// - [`InducedDrag`] — add for conventional lifting aircraft (most fixed-wing).
///   Omit for gliders with polar-based CDs, missiles, or LOD AI.
/// - [`LodDamping`] — add only for sparse-zone aircraft where zone geometry
///   cannot produce realistic roll/pitch/yaw damping.
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::*;
/// # use bevy::prelude::*;
/// // Full-fidelity aircraft with induced drag, zone-based damping:
/// // commands.spawn((
/// //     AircraftCoreBundle { geometry: AircraftGeometry { wing_area_m2: 16.2, wing_span_m: 10.6, chord_m: 1.53 }, ..default() },
/// //     InducedDrag { oswald_factor: 0.85 },
/// // ));
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
