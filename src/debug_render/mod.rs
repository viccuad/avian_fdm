//! FDM debug rendering plugin.
//!
//! It draws Bevy gizmos for the aerodynamic force vectors, the moment arrows,
//! the zone health wireframes, and the angle-of-attack indicator.
//!
//! The crate compiles this module only with `features = ["debug-plugin"]`.
//! [`crate::plugin::AircraftFdmPlugin`] does not include it. Add it next to
//! that plugin:
//!
//! ```rust,no_run
//! use avian_fdm::plugin::AircraftFdmPlugin;
//! use avian_fdm::debug_render::AircraftFdmDebugPlugin;
//! use bevy::prelude::*;
//!
//! App::new()
//!     .add_plugins((AircraftFdmPlugin::default(), AircraftFdmDebugPlugin))
//!     .run();
//! ```
//!
//! To configure it for all entities, use [`FdmGizmos`] in [`GizmoConfigStore`]:
//!
//! ```rust,no_run
//! use avian_fdm::debug_render::FdmGizmos;
//! use bevy::prelude::*;
//!
//! App::new()
//!     .insert_gizmo_config(
//!         FdmGizmos::forces(),
//!         GizmoConfig::default(),
//!     );
//! ```
//!
//! To change the color of one zone entity, use [`FdmDebugRender`].

mod colliders;
mod configuration;
mod gizmos;

pub use crate::gizmo_shape::{GizmoContours, GizmoShape};
pub use colliders::ShowColliders;
pub use configuration::{FdmDebugRender, FdmGizmos};

use crate::_bevy::*;
use bevy_transform::TransformSystems;
use gizmos::*;

/// Plugin that adds all FDM debug gizmo overlays.
///
/// You must add it **separately** from [`crate::plugin::AircraftFdmPlugin`]. That plugin does not
/// include it. This is the same convention as the Avian `PhysicsDebugPlugin`.
///
/// [`AircraftFdmPlugin`]: crate::plugin::AircraftFdmPlugin
#[derive(Default)]
pub struct AircraftFdmDebugPlugin;

impl Plugin for AircraftFdmDebugPlugin {
    fn build(&self, app: &mut App) {
        colliders::plugin(app);

        app.init_gizmo_group::<FdmGizmos>();
        app.register_type::<FdmDebugRender>();

        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        store.config_mut::<FdmGizmos>().0.line.width = 1.5;

        app.add_systems(
            PostUpdate,
            (
                debug_render_cg,
                debug_render_ac,
                debug_render_zone_forces,
                debug_render_thrust,
                debug_render_resultant,
                debug_render_moments,
                debug_render_zones,
                debug_render_wind,
            )
                .after(TransformSystems::Propagate)
                .run_if(|store: Res<GizmoConfigStore>| store.config::<FdmGizmos>().0.enabled),
        );
    }
}
