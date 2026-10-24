//! Plugin registration and the schedule of the FDM systems.
//!
//! ## Execution order in `PhysicsStepSystems::BroadPhase`
//!
//! ```text
//! Atmosphere -> FlightState -> Forces
//! ```
//!
//! The FDM chain runs in `BroadPhase`, not in `First`. This prevents a static
//! Bevy ambiguity with the Avian system `update_child_collider_position`. That
//! system writes `Position` and `Rotation` for child colliders in `First`. At
//! runtime, the two systems use different entities, but the static checker
//! cannot prove this. `BroadPhase` runs after `First`, so the order is always
//! correct.
//!
//! The Avian set `ForceSystems::ApplyConstantForces` runs in `Solver`, after
//! `BroadPhase`. Thus, the FDM always writes the forces before Avian reads
//! them.
//!
//! ## Add a custom system (for example, an autopilot)
//!
//! Use the named sets in [`AircraftFdmSystems`] to put your system in the
//! correct position:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::*;
//! # use avian_fdm::plugin::AircraftFdmSystems;
//! // app.add_systems(PhysicsSchedule,
//! //     my_autopilot.after(AircraftFdmSystems::FlightState)
//! //                 .before(AircraftFdmSystems::Forces));
//! ```

use crate::_bevy::*;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};

use crate::aerodynamics::compute_aero_forces;
use crate::atmosphere::update_atmosphere;
use crate::kinematics::update_flight_state;
use crate::propulsion::compute_engine_zone_forces;

/// Named system sets for the FDM pipeline. Use them to add custom systems.
///
/// Execution order: `Atmosphere` -> `FlightState` -> `Forces`
///
/// All sets run in `PhysicsStepSystems::BroadPhase`.
///
/// Example: an autopilot that reads `FlightState` and writes `ControlInputs`:
///
/// ```rust,no_run
/// # use bevy::prelude::*;
/// # use avian3d::prelude::*;
/// # use avian_fdm::plugin::AircraftFdmSystems;
/// // app.add_systems(PhysicsSchedule,
/// //     my_autopilot.after(AircraftFdmSystems::FlightState)
/// //                 .before(AircraftFdmSystems::Forces));
/// ```
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub enum AircraftFdmSystems {
    /// Updates the ISA atmosphere: density, pressure, temperature, and speed of sound.
    Atmosphere,
    /// Calculates `FlightState` (alpha, beta, airspeed, altitude) from the physics state.
    FlightState,
    /// Calculates all zone forces (engine and aerodynamic) and adds them on the root entity.
    Forces,
}

/// Adds all FDM subsystems that the active feature flags enable.
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
/// In debug builds, a `PostStartup` system runs [`validate_aero_zones`] on each
/// [`AeroZone`](crate::components::AeroZone) entity. It also runs
/// [`validate_rigid_bodies`] on each aircraft root entity. The systems log a
/// warning for each table structure error (unsorted breakpoints, dimension
/// mismatches, NaN or Inf values). To disable these checks, set
/// `AircraftFdmPlugin { validate_on_startup: false }`.
pub struct AircraftFdmPlugin {
    /// Runs [`validate_aero_zones`] and [`validate_rigid_bodies`] in
    /// `PostStartup`. Default: `true` in debug builds, `false` in release
    /// builds.
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
            AircraftFdmSystems::Atmosphere,
            AircraftFdmSystems::FlightState,
            AircraftFdmSystems::Forces,
        )
            .chain()
            .in_set(PhysicsStepSystems::BroadPhase),
    );

    app.add_systems(
        PhysicsSchedule,
        update_atmosphere.in_set(AircraftFdmSystems::Atmosphere),
    );
    app.add_systems(
        PhysicsSchedule,
        update_flight_state.in_set(AircraftFdmSystems::FlightState),
    );
    app.add_systems(
        PhysicsSchedule,
        (compute_engine_zone_forces, compute_aero_forces)
            .chain()
            .in_set(AircraftFdmSystems::Forces),
    );
}

/// Startup validation system. It logs a warning for each
/// [`crate::components::AircraftGeometry`] root entity that does not have
/// `RigidBody::Dynamic`.
///
/// A `RigidBody::Static` or `RigidBody::Kinematic` root entity ignores all
/// forces without a warning. Thus, the aerodynamic forces cannot move the
/// aircraft.
///
/// [`AircraftFdmPlugin`] adds this system automatically when
/// `validate_on_startup` is `true` (the default in debug builds).
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

/// Startup validation system. It examines each [`crate::components::AeroZone`]
/// for table structure errors and negative dimensions.
///
/// It runs in `PostStartup`, after all `Startup` systems spawn their entities.
/// It logs a `warn!` for each problem that it finds. It does not panic. The
/// aircraft continues to run, but a broken table gives incorrect forces.
///
/// [`AircraftFdmPlugin`] adds this system automatically when
/// `validate_on_startup` is `true` (the default in debug builds).
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
