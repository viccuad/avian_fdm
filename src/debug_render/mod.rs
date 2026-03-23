//! FDM debug rendering plugin.
//!
//! Renders aerodynamic force vectors, moment arcs, zone health wireframes, and
//! angle-of-attack indicators as Bevy gizmos.
//!
//! Only compiled with `features = ["debug-plugin"]`. Add alongside
//! [`AircraftFdmPlugin`] — not included in it by default:
//!
//! ```rust,no_run
//! use avian_fdm::plugin::AircraftFdmPlugin;
//! use avian_fdm::debug_render::AircraftFdmDebugPlugin;
//! use bevy::prelude::*;
//!
//! App::new()
//!     .add_plugins((AircraftFdmPlugin, AircraftFdmDebugPlugin))
//!     .run();
//! ```
//!
//! Configure globally via [`GizmoConfigStore`] using [`FdmGizmos`]:
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
//! Override per zone entity with [`FdmDebugRender`].

mod configuration;
mod gizmos;

pub use configuration::{FdmDebugRender, FdmGizmos};

use bevy::prelude::*;
use bevy::transform::TransformSystems;
use gizmos::*;

/// Plugin that adds all FDM debug gizmo overlays.
///
/// Must be added **separately** from [`AircraftFdmPlugin`] — it is not included
/// by default (following Avian's [`PhysicsDebugPlugin`] convention).
///
/// [`AircraftFdmPlugin`]: crate::plugin::AircraftFdmPlugin
/// [`PhysicsDebugPlugin`]: avian3d::debug_render::PhysicsDebugPlugin
#[derive(Default)]
pub struct AircraftFdmDebugPlugin;

impl Plugin for AircraftFdmDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<FdmGizmos>();
        app.register_type::<FdmDebugRender>();

        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        store.config_mut::<FdmGizmos>().0.line.width = 1.5;

        app.add_systems(
            PostUpdate,
            (
                debug_render_zone_forces,
                debug_render_thrust,
                debug_render_resultant,
                debug_render_moments,
                debug_render_zones,
                debug_render_wind,
            )
                .after(TransformSystems::Propagate)
                .run_if(|store: Res<GizmoConfigStore>| {
                    store.config::<FdmGizmos>().0.enabled
                }),
        );
    }
}
