//! Plugin registration.

use bevy::prelude::*;

/// Adds all FDM subsystems enabled by the active feature flags.
///
/// # Example
/// ```rust,no_run
/// use avian_fdm::plugin::AircraftFdmPlugin;
/// use bevy::prelude::*;
///
/// App::new()
///     .add_plugins(AircraftFdmPlugin)
///     .run();
/// ```
pub struct AircraftFdmPlugin;

impl Plugin for AircraftFdmPlugin {
    fn build(&self, app: &mut App) {
        use crate::components::{
            AeroZone, AircraftGeometry, AtmosphereState, ControlInputs,
            Damageable, FlightState, GizmoShape, GizmoContours, ZoneForce,
        };

        app.register_type::<AircraftGeometry>()
           .register_type::<ControlInputs>()
           .register_type::<FlightState>()
           .register_type::<AtmosphereState>()
           .register_type::<AeroZone>()
           .register_type::<GizmoShape>()
           .register_type::<GizmoContours>();

        #[cfg(feature = "damage")]
        app.register_type::<Damageable>();

        #[cfg(feature = "damage")]
        app.add_plugins(crate::detach::DetachPlugin);

        #[cfg(feature = "propulsion")]
        {
            use crate::components::{EngineZone, PropwashState};
            app.register_type::<EngineZone>()
               .register_type::<PropwashState>();
        }

        // ZoneForce is internal — register for inspector access but not public API.
        let _ = std::any::TypeId::of::<ZoneForce>(); // suppress unused-import lint

        crate::systems::register_fdm_systems(app);
    }
}
