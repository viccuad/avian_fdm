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
#[derive(Default)]
#[non_exhaustive]
pub struct AircraftFdmPlugin;

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
            Failure, FlightState, GizmoShape, GizmoContours, ZoneForce,
        };

        app.register_type::<AircraftGeometry>()
           .register_type::<ControlInputs>()
           .register_type::<FlightState>()
           .register_type::<AtmosphereState>()
           .register_type::<AeroZone>()
           .register_type::<ZoneForce>()
           .register_type::<GizmoShape>()
           .register_type::<GizmoContours>()
           .register_type::<Failure>();

        #[cfg(feature = "propulsion")]
        {
            use crate::components::{EngineZone, PropwashState};
            app.register_type::<EngineZone>()
               .register_type::<PropwashState>();
        }

        crate::systems::register_fdm_systems(app);
    }
}
