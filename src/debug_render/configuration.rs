//! [`FdmGizmos`] configuration group and per-entity [`FdmDebugRender`] component.

use bevy_color::palettes::css::*;
use crate::_bevy::*;

/// Gizmo configuration group for FDM debug rendering. See [`AircraftFdmDebugPlugin`].
///
/// Register globally via [`GizmoConfigStore`]:
///
/// ```rust,no_run
/// use avian_fdm::debug_render::FdmGizmos;
/// use bevy::prelude::*;
///
/// App::new()
///     .insert_gizmo_config(
///         FdmGizmos {
///             lift_color: Some(Color::srgb(0.0, 1.0, 0.0)),
///             ..FdmGizmos::none()
///         },
///         GizmoConfig::default(),
///     );
/// ```
///
/// Per-entity overrides are supported via the [`FdmDebugRender`] component.
///
/// [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin
#[derive(Reflect, GizmoConfigGroup)]
pub struct FdmGizmos {
    /// Color for per-zone lift force arrows. `None` = disabled.
    pub lift_color: Option<Color>,
    /// Color for per-zone drag force arrows. `None` = disabled.
    pub drag_color: Option<Color>,
    /// Color for per-zone side-force arrows. `None` = disabled.
    pub side_force_color: Option<Color>,
    /// Color for thrust arrows (engine zones). `None` = disabled.
    pub thrust_color: Option<Color>,
    /// Color for the total accumulated aero+thrust force arrow on the root. `None` = disabled.
    pub total_force_color: Option<Color>,
    /// Color for the weight (gravity) arrow on the root. `None` = disabled.
    pub weight_color: Option<Color>,
    /// Color for the net-force arrow (aero+thrust+weight) on the root. `None` = disabled.
    pub resultant_color: Option<Color>,
    /// Color for pitching-moment arcs. `None` = disabled.
    pub pitch_moment_color: Option<Color>,
    /// Color for rolling-moment arcs. `None` = disabled.
    pub roll_moment_color: Option<Color>,
    /// Color for yawing-moment arcs. `None` = disabled.
    pub yaw_moment_color: Option<Color>,
    /// Color for zone collider wireframes, tinted green to red by [`Failure::remaining`].
    /// `None` = disabled.
    ///
    /// [`Failure::remaining`]: crate::components::Failure
    pub zone_color: Option<Color>,
    /// Color for the relative-wind / angle-of-attack indicator arrow. `None` = disabled.
    pub wind_color: Option<Color>,
    /// Color for the CG (centre of gravity) sphere. `None` = disabled.
    pub cg_color: Option<Color>,
    /// Color for the aerodynamic-centre spheres on each zone. `None` = disabled.
    pub ac_color: Option<Color>,
    /// Radius (metres) of the CG and AC marker spheres.
    pub marker_radius: f32,
    /// World-space metres per Newton for force arrow length.
    pub force_scale: f32,
}

impl Default for FdmGizmos {
    fn default() -> Self {
        Self {
            lift_color: Some(LIME.into()),
            drag_color: Some(RED.into()),
            side_force_color: Some(YELLOW.into()),
            thrust_color: Some(AQUA.into()),
            total_force_color: Some(YELLOW.into()),
            weight_color: Some(RED.into()),
            resultant_color: Some(WHITE.into()),
            pitch_moment_color: Some(ORANGE.into()),
            roll_moment_color: Some(PINK.into()),
            yaw_moment_color: Some(VIOLET.into()),
            zone_color: Some(ORANGE.into()),
            wind_color: Some(LIGHT_CYAN.into()),
            cg_color: Some(SILVER.into()),
            ac_color: Some(AQUA.into()),
            marker_radius: 0.075,
            force_scale: 0.001,
        }
    }
}

impl FdmGizmos {
    /// All overlays enabled with default colours.
    pub fn all() -> Self {
        Self::default()
    }

    /// All overlays disabled. Use as a base for `..FdmGizmos::none()` patterns.
    pub fn none() -> Self {
        Self {
            lift_color: None,
            drag_color: None,
            side_force_color: None,
            thrust_color: None,
            total_force_color: None,
            weight_color: None,
            resultant_color: None,
            pitch_moment_color: None,
            roll_moment_color: None,
            yaw_moment_color: None,
            zone_color: None,
            wind_color: None,
            cg_color: None,
            ac_color: None,
            marker_radius: 0.075,
            force_scale: 0.001,
        }
    }

    /// Only force arrows (lift, drag, side force, thrust, resultant) enabled.
    pub fn forces() -> Self {
        Self {
            lift_color:        Some(LIME.into()),
            drag_color:        Some(RED.into()),
            side_force_color:  Some(YELLOW.into()),
            thrust_color:      Some(AQUA.into()),
            total_force_color: Some(YELLOW.into()),
            weight_color:      Some(RED.into()),
            resultant_color:   Some(WHITE.into()),
            ..Self::none()
        }
    }

    /// Only moment arcs (pitch, roll, yaw) enabled.
    pub fn moments() -> Self {
        Self {
            pitch_moment_color: Some(ORANGE.into()),
            roll_moment_color:  Some(PINK.into()),
            yaw_moment_color:   Some(VIOLET.into()),
            ..Self::none()
        }
    }

    /// Only zone health wireframes enabled.
    pub fn zones() -> Self {
        Self {
            zone_color: Some(ORANGE.into()),
            ..Self::none()
        }
    }
}

// ── Per-entity override ───────────────────────────────────────────────────────

/// Per-zone debug render override. Attach to an [`AeroZone`] entity to
/// override the global [`FdmGizmos`] colour for that zone's wireframe.
///
/// # Example
///
/// ```rust,no_run
/// use avian_fdm::debug_render::FdmDebugRender;
/// use bevy::prelude::*;
///
/// // commands.entity(zone_entity).insert(
/// //     FdmDebugRender::default().with_zone_color(Color::srgb(1.0, 0.0, 0.0))
/// // );
/// ```
///
/// [`AeroZone`]: crate::components::AeroZone
#[derive(Component, Reflect, Clone, Copy, PartialEq)]
#[reflect(Component, PartialEq)]
pub struct FdmDebugRender {
    /// Wireframe colour for this zone's collider. `None` = use global [`FdmGizmos::zone_color`].
    pub zone_color: Option<Color>,
}

impl Default for FdmDebugRender {
    fn default() -> Self {
        Self { zone_color: None }
    }
}

impl FdmDebugRender {
    /// Override the zone wireframe colour for this entity.
    pub fn with_zone_color(mut self, color: Color) -> Self {
        self.zone_color = Some(color);
        self
    }

    /// Disable zone wireframe rendering for this entity.
    pub fn without_zone(mut self) -> Self {
        self.zone_color = None;
        self
    }
}
