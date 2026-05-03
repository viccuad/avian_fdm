//! Plugin registration and FDM system wiring.
//!
//! ## Execution order within `PhysicsStepSystems::BroadPhase`
//!
//! ```text
//! Atmosphere -> FlightState -> Forces
//! ```
//!
//! The FDM chain runs in `BroadPhase` (not `First`) to avoid a Bevy static
//! ambiguity with Avian's `update_child_collider_position`, which writes
//! `Position`/`Rotation` for child colliders in `First`. Both systems operate
//! on disjoint entity sets at runtime, but the static checker cannot prove this.
//! `BroadPhase` runs after `First`, so the ordering is guaranteed.
//!
//! Avian's `ForceSystems::ApplyConstantForces` runs in `Solver` (after
//! `BroadPhase`), so forces are always written before they are read.
//!
//! ## Adding a custom system (e.g. autopilot)
//!
//! Use [`AircraftFdmSystem`] to schedule relative to named sets:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::*;
//! # use avian_fdm::plugin::AircraftFdmSystem;
//! // app.add_systems(PhysicsSchedule,
//! //     my_autopilot.after(AircraftFdmSystem::FlightState)
//! //                 .before(AircraftFdmSystem::Forces));
//! ```

use crate::_bevy::*;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};

use crate::aerodynamics::compute_aero_forces;
use crate::atmosphere::{update_atmosphere, update_flight_state};
use crate::propulsion::compute_engine_zone_forces;

/// Named system sets for the FDM pipeline. Use these to hook in custom systems.
///
/// Execution order: `Atmosphere` -> `FlightState` -> `Forces`
///
/// All sets run inside `PhysicsStepSystems::BroadPhase`.
///
/// Example: autopilot that reads `FlightState` and writes `ControlInputs`:
///
/// ```rust,no_run
/// # use bevy::prelude::*;
/// # use avian3d::prelude::*;
/// # use avian_fdm::plugin::AircraftFdmSystem;
/// // app.add_systems(PhysicsSchedule,
/// //     my_autopilot.after(AircraftFdmSystem::FlightState)
/// //                 .before(AircraftFdmSystem::Forces));
/// ```
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub enum AircraftFdmSystem {
    /// Update ISA atmosphere density, temperature, and dynamic pressure.
    Atmosphere,
    /// Derive FlightState (alpha, beta, airspeed, altitude) from physics state.
    FlightState,
    /// Compute all zone forces (engine + aerodynamic) and accumulate onto the root body.
    Forces,
}

/// Adds all FDM subsystems enabled by the active feature flags.
///
/// Add `PhysicsPlugins` before this plugin.
///
/// # Example
/// ```rust,no_run
/// use avian_fdm::plugin::AircraftFdmPlugin;
/// use bevy::prelude::*;
///
/// App::new()
///     .add_plugins(AircraftFdmPlugin::default())
///     .run();
/// ```
///
/// # Startup validation
///
/// In debug builds, a `PostStartup` system runs [`validate_aero_zones`] on
/// every [`AeroZone`](crate::components::AeroZone) entity, logging warnings
/// for table structure errors (unsorted breakpoints, dimension mismatches,
/// NaN/Inf) and placeholder coefficients. Disable by setting
/// `AircraftFdmPlugin { validate_on_startup: false }`.
pub struct AircraftFdmPlugin {
    /// Run [`validate_aero_zones`] in `PostStartup`. Default: `true` in debug
    /// builds, `false` in release.
    pub validate_on_startup: bool,
}

impl Default for AircraftFdmPlugin {
    fn default() -> Self {
        Self {
            validate_on_startup: cfg!(debug_assertions),
        }
    }
}

impl Plugin for AircraftFdmPlugin {
    fn build(&self, app: &mut App) {
        if app
            .get_schedule(avian3d::prelude::PhysicsSchedule)
            .is_none()
        {
            panic!(
                "Failed to build `AircraftFdmPlugin`: \
                Avian's `PhysicsSchedule` was not found. \
                Add `PhysicsPlugins` before `AircraftFdmPlugin`."
            );
        }

        use crate::components::{
            AeroZone, AircraftGeometry, AtmosphereState, ControlInputs, EngineZone, Failure,
            FlightState, GizmoContours, GizmoShape, InducedDrag, LodDamping, ZoneForce,
        };
        app.register_type::<AircraftGeometry>()
            .register_type::<ControlInputs>()
            .register_type::<FlightState>()
            .register_type::<AtmosphereState>()
            .register_type::<AeroZone>()
            .register_type::<ZoneForce>()
            .register_type::<GizmoShape>()
            .register_type::<GizmoContours>()
            .register_type::<Failure>()
            .register_type::<InducedDrag>()
            .register_type::<LodDamping>()
            .register_type::<EngineZone>();

        use crate::airfoil::{resolve_airfoil_names, AirfoilLibrary};
        app.init_resource::<AirfoilLibrary>()
            .add_systems(PreUpdate, resolve_airfoil_names);

        register_fdm_systems(app);

        if self.validate_on_startup {
            app.add_systems(PostStartup, (validate_rigid_bodies, validate_aero_zones));
        }
    }
}

fn register_fdm_systems(app: &mut App) {
    app.configure_sets(
        PhysicsSchedule,
        (
            AircraftFdmSystem::Atmosphere,
            AircraftFdmSystem::FlightState,
            AircraftFdmSystem::Forces,
        )
            .chain()
            .in_set(PhysicsStepSystems::BroadPhase),
    );

    app.add_systems(
        PhysicsSchedule,
        update_atmosphere.in_set(AircraftFdmSystem::Atmosphere),
    );
    app.add_systems(
        PhysicsSchedule,
        update_flight_state.in_set(AircraftFdmSystem::FlightState),
    );
    app.add_systems(
        PhysicsSchedule,
        (compute_engine_zone_forces, compute_aero_forces)
            .chain()
            .in_set(AircraftFdmSystem::Forces),
    );
}

/// Startup validation system: warns if any [`crate::components::AircraftGeometry`] root entity
/// does not have `RigidBody::Dynamic`.
///
/// A `RigidBody::Static` or `RigidBody::Kinematic` root will silently ignore
/// all accumulated forces, so the aircraft will never move under aerodynamics.
///
/// Registered automatically by [`AircraftFdmPlugin`] when
/// `validate_on_startup` is `true` (default in debug builds).
pub fn validate_rigid_bodies(
    query: Query<(Entity, &avian3d::prelude::RigidBody), With<crate::components::AircraftGeometry>>,
) {
    for (entity, rb) in &query {
        if !rb.is_dynamic() {
            warn!(
                "Entity {entity} has AircraftGeometry but RigidBody is not Dynamic. \
                 Aerodynamic forces will be ignored by Avian. \
                 Set RigidBody::Dynamic on the aircraft root entity."
            );
        }
    }
}

/// Startup validation system: checks every [`crate::components::AeroZone`] for table structure
/// errors and placeholder coefficients.
///
/// Runs in `PostStartup` (after all `Startup` systems have spawned entities).
/// Logs warnings via `warn!` for each problem found. Does not
/// panic; the aircraft will still run, but broken tables will produce garbage.
///
/// Registered automatically by [`AircraftFdmPlugin`] when
/// `validate_on_startup` is `true` (default in debug builds).
pub fn validate_aero_zones(query: Query<(Entity, &crate::components::AeroZone)>) {
    let mut total_problems = 0;
    for (entity, zone) in &query {
        let label = format!("Entity {entity}");
        let problems = zone.validate(&label);
        for p in &problems {
            warn!("AeroZone validation: {p}");
        }
        total_problems += problems.len();
    }
    if total_problems > 0 {
        warn!(
            "AeroZone validation found {total_problems} problem(s). \
             Fix these before trusting simulation results."
        );
    }
}
