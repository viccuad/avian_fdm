//! In-game debug visualisation plugin.
//!
//! Adds force-vector gizmos, moment arcs, zone health wireframes, AoA
//! overlays, and an egui HUD. All overlays are runtime-toggled via
//! [`FdmDebugConfig`] — no recompilation needed.
//!
//! Only compiled with `features = ["debug-viz"]`.

use bevy::prelude::*;

/// Runtime configuration for FDM debug overlays.
/// Insert this resource and mutate it freely (key bindings, pause menu, etc.).
#[derive(Resource, Reflect, Clone)]
#[reflect(Resource)]
pub struct FdmDebugConfig {
    /// Draw lift, drag, side force, thrust, weight, and resultant force arrows.
    pub show_forces: bool,
    /// Draw pitching / rolling / yawing moment arcs centred on the CG.
    pub show_moments: bool,
    /// Draw zone collider wireframes coloured green→red by health, plus a CG sphere.
    pub show_zones: bool,
    /// Draw relative-wind arrow, AoA arc, sideslip arc; tint stalling zones red.
    pub show_aoa: bool,
    /// Draw an egui panel showing TAS, ALT, AoA, q̄, Re, etc.
    pub show_hud: bool,
    /// World-space metres per Newton for force arrow length.
    pub force_scale: f32,
}

impl Default for FdmDebugConfig {
    fn default() -> Self {
        Self {
            show_forces: false,
            show_moments: false,
            show_zones: false,
            show_aoa: false,
            show_hud: false,
            force_scale: 0.001,
        }
    }
}

/// Optional plugin that adds all FDM debug overlays.
/// Add alongside [`crate::plugin::AircraftFdmPlugin`]:
/// ```rust,no_run
/// use avian_fdm::debug::AircraftFdmDebugPlugin;
/// # use bevy::prelude::*;
/// // app.add_plugins(AircraftFdmDebugPlugin);
/// ```
pub struct AircraftFdmDebugPlugin;

impl Plugin for AircraftFdmDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FdmDebugConfig>()
           .register_type::<FdmDebugConfig>();
        // TODO(debug-viz): add gizmo + egui systems
    }
}
