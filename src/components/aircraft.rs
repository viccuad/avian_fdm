//! Aircraft-level components: geometry, mass properties, and aerodynamic aggregate.

use bevy::prelude::*;
use bevy::math::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use avian3d::prelude::RigidBody;

/// Wing and tail geometry constants. Used for all aerodynamic force scaling
/// and non-dimensionalisation (q̄·S, q̄·S·b, q̄·S·c̄).
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
}

/// Total aircraft mass properties. Written each frame by `aggregate_zones`
/// (when `damage` feature is on) or set once at spawn (when off).
///
/// **Do not write to this component directly** — it is owned by the library.
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
#[cfg(feature = "damage")]
pub struct AircraftMass {
    /// Total mass (kg). Sum of all zone mass contributions × health.
    pub mass_kg: f64,
    /// Inertia tensor in body frame (kg·m²). Recomputed via parallel-axis theorem.
    pub inertia_tensor: DMat3,
    /// Centre of gravity in body frame (m). Shifts as zones are damaged.
    pub cg_body_m: DVec3,
}

/// Summed, health-weighted aerodynamic coefficient totals, evaluated at the
/// current angle of attack and Reynolds number.
///
/// Written each frame by `aggregate_zones`. The aerodynamics system reads
/// these `f64` values and multiplies by q̄·S directly — no further table
/// lookup needed.
///
/// **Do not write to this component directly.**
///
/// Lives on the **aircraft root entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
#[cfg(feature = "damage")]
pub struct AircraftAggregate {
    /// Summed lift coefficient CL (dimensionless).
    pub cl_total: f64,
    /// Summed drag coefficient CD (dimensionless).
    pub cd_total: f64,
    /// Summed side-force coefficient CY (dimensionless).
    pub cy_total: f64,
    /// Summed pitching-moment coefficient CM (dimensionless, about c̄).
    pub cm_total: f64,
    /// Summed rolling-moment coefficient Cl (dimensionless, about b).
    pub croll_total: f64,
    /// Summed yawing-moment coefficient Cn (dimensionless, about b).
    pub cn_total: f64,
    /// Extra drag pressure from all damaged zones (Pa). Added as a constant drag force.
    pub structural_drag_pa: f64,
    /// Per-surface effectiveness scale factors (0–1).
    #[cfg(feature = "damage")]
    pub control_effectiveness: crate::components::aero_zone::ControlEffectiveness,
}

/// Core bundle. Always spawn this on the aircraft root entity.
///
/// Then add [`AircraftDamageBundle`] and/or [`crate::components::AircraftPropulsionBundle`]
/// as needed.
///
/// Forces are applied each frame via the `Forces` query data from avian3d —
/// no `ExternalForce`/`ExternalTorque` components are needed in this bundle.
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::*;
/// # use bevy::prelude::*;
/// // commands.spawn(AircraftCoreBundle { geometry: AircraftGeometry { ... }, ..default() });
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
    /// World-space transform.
    pub transform: Transform,
    /// Required by Bevy for transform propagation.
    pub global_transform: GlobalTransform,
}

/// Bundle for damage-model components. Add alongside [`AircraftCoreBundle`]
/// when `features = ["damage"]`.
#[derive(Bundle, Default)]
#[cfg(feature = "damage")]
pub struct AircraftDamageBundle {
    /// Total mass properties. Filled by `aggregate_zones`.
    pub mass_props: AircraftMass,
    /// Evaluated coefficient totals. Filled by `aggregate_zones`.
    pub aggregate: AircraftAggregate,
}
