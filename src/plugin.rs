//! Plugin registration.

use crate::_bevy::*;

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
        if app.get_schedule(avian3d::prelude::PhysicsSchedule).is_none() {
            panic!(
                "Failed to build `AircraftFdmPlugin`: \
                Avian's `PhysicsSchedule` was not found. \
                Add `PhysicsPlugins` before `AircraftFdmPlugin`."
            );
        }

        use crate::components::{
            AeroZone, AircraftGeometry, AtmosphereState, ControlInputs,
            Failure, FlightState, GizmoShape, GizmoContours, InducedDrag,
            LodDamping, ZoneForce,
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
           .register_type::<LodDamping>();

        use crate::components::{EngineZone, PropwashState};
        app.register_type::<EngineZone>()
           .register_type::<PropwashState>();

        crate::systems::register_fdm_systems(app);

        if self.validate_on_startup {
            app.add_systems(PostStartup, (validate_rigid_bodies, validate_aero_zones));
        }
    }
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
