//! Aircraft-level components: geometry and the core spawn bundle.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use avian3d::prelude::{RigidBody, ConstantForce, ConstantTorque};

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
