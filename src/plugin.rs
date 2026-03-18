//! Plugin registration. Each subsystem is its own [`Plugin`] so consumers
//! can add only the pieces they need. [`AircraftFdmPlugin`] is a convenience
//! that adds all subsystems enabled by the active feature flags.

use bevy::prelude::*;

/// Adds all FDM subsystems enabled by the active feature flags.
///
/// Equivalent to adding each sub-plugin individually:
/// - [`AtmospherePlugin`] — ISA atmosphere model (always included)
/// - [`AerodynamicsPlugin`] — force/moment pipeline (always included)
/// - [`DamagePlugin`] — zone health + mass aggregation (`damage` feature)
/// - [`PropulsionPlugin`] — piston engine model (`propulsion` feature)
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

/// ISA atmosphere model. Computes [`crate::components::AtmosphereState`] and
/// [`crate::components::FlightState`] each physics frame.
pub struct AtmospherePlugin;

/// Aerodynamic force/moment pipeline. Requires [`AtmospherePlugin`].
pub struct AerodynamicsPlugin;

/// Zone health tracking, mass aggregation, and CG computation.
/// Only available with `features = ["damage"]`.
#[cfg(feature = "damage")]
pub struct DamagePlugin;

/// Piston engine thrust model and propwash. Requires [`AerodynamicsPlugin`].
/// Only available with `features = ["propulsion"]`.
#[cfg(feature = "propulsion")]
pub struct PropulsionPlugin;

impl Plugin for AircraftFdmPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(AtmospherePlugin)
           .add_plugins(AerodynamicsPlugin);

        #[cfg(feature = "damage")]
        app.add_plugins(DamagePlugin);

        #[cfg(feature = "propulsion")]
        app.add_plugins(PropulsionPlugin);

        // Wire all systems in the correct execution order.
        crate::systems::register_fdm_systems(app);
    }
}

impl Plugin for AtmospherePlugin {
    fn build(&self, app: &mut App) {
        use crate::components::{AtmosphereState, FlightState};
        app.register_type::<AtmosphereState>()
           .register_type::<FlightState>();
        // Systems registered in systems.rs after all subsystems are available.
    }
}

impl Plugin for AerodynamicsPlugin {
    fn build(&self, app: &mut App) {
        use crate::components::{AircraftGeometry, AircraftCoreBundle, ControlInputs};
        app.register_type::<AircraftGeometry>()
           .register_type::<ControlInputs>();
    }
}

#[cfg(feature = "damage")]
impl Plugin for DamagePlugin {
    fn build(&self, app: &mut App) {
        use crate::components::{AircraftMass, AircraftAggregate, AeroZone, AeroZoneHealth};
        app.register_type::<AircraftMass>()
           .register_type::<AircraftAggregate>()
           .register_type::<AeroZone>()
           .register_type::<AeroZoneHealth>();
    }
}

#[cfg(feature = "propulsion")]
impl Plugin for PropulsionPlugin {
    fn build(&self, app: &mut App) {
        use crate::components::{EngineConfig, PropwashState};
        app.register_type::<EngineConfig>()
           .register_type::<PropwashState>();
    }
}
