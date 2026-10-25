//! [`FdmGizmos`] configuration group and per-entity [`FdmDebugRender`] component.

use crate::_bevy::*;
use bevy_color::palettes::css::*;

/// Gizmo configuration group for the FDM debug rendering. Refer to [`AircraftFdmDebugPlugin`].
///
/// To register it for all entities, use [`GizmoConfigStore`]:
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
/// To change the zone color of one entity, use the [`FdmDebugRender`] component.
///
/// [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin
#[derive(Reflect, GizmoConfigGroup)]
pub struct FdmGizmos {
    /// Color for the aerodynamic force arrow of each zone. The arrow shows the
    /// sum of lift, drag, and side force. `None` = disabled.
    pub lift_color: Option<Color>,
    /// Color for separate drag arrows. The debug systems do not use it at this
    /// time. `lift_color` draws one combined arrow for each zone.
    pub drag_color: Option<Color>,
    /// Color for separate side-force arrows. The debug systems do not use it
    /// at this time. `lift_color` draws one combined arrow for each zone.
    pub side_force_color: Option<Color>,
    /// Color for the thrust arrows (engine zones). `None` = disabled.
    pub thrust_color: Option<Color>,
    /// Color for the arrow of the total aerodynamic and thrust force on the root entity. `None` = disabled.
    pub total_force_color: Option<Color>,
    /// Color for the weight (gravity) arrow on the root entity. `None` = disabled.
    pub weight_color: Option<Color>,
    /// Color for the net-force arrow (aerodynamic + thrust + weight) on the root entity. `None` = disabled.
    pub resultant_color: Option<Color>,
    /// Color for the pitching-moment arrow. `None` = disabled.
    pub pitch_moment_color: Option<Color>,
    /// Color for the rolling-moment arrow. `None` = disabled.
    pub roll_moment_color: Option<Color>,
    /// Color for the yawing-moment arrow. `None` = disabled.
    pub yaw_moment_color: Option<Color>,
    /// On/off switch for the zone wireframes. `None` = disabled.
    ///
    /// The value of the color is not used. Each zone gets a color from its
    /// zone type, or from [`FdmDebugRender`]. [`Failure::remaining`] then
    /// changes this color toward gray.
    ///
    /// [`Failure::remaining`]: crate::components::Failure
    pub zone_color: Option<Color>,
    /// Color for the relative-wind arrow and the nose arrow (angle-of-attack indicator). `None` = disabled.
    pub wind_color: Option<Color>,
    /// Color for the CG (center of gravity) sphere. `None` = disabled.
    pub cg_color: Option<Color>,
    /// Color for the aerodynamic-center cross on each zone. `None` = disabled.
    pub ac_color: Option<Color>,
    /// Radius (meters) of the CG sphere. Also the half-length of the arms of the AC cross.
    pub marker_radius: f32,
    /// Length of a force arrow in world space, in meters per newton.
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
    /// All overlays enabled, with the default colors.
    pub fn all() -> Self {
        Self::default()
    }

    /// All overlays disabled. Use it as a base for the `..FdmGizmos::none()` pattern.
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

    /// Only the force arrows enabled (zone force, thrust, total force, weight, resultant).
    pub fn forces() -> Self {
        Self {
            lift_color: Some(LIME.into()),
            drag_color: Some(RED.into()),
            side_force_color: Some(YELLOW.into()),
            thrust_color: Some(AQUA.into()),
            total_force_color: Some(YELLOW.into()),
            weight_color: Some(RED.into()),
            resultant_color: Some(WHITE.into()),
            ..Self::none()
        }
    }

    /// Only the moment arrows enabled (pitch, roll, yaw).
    pub fn moments() -> Self {
        Self {
            pitch_moment_color: Some(ORANGE.into()),
            roll_moment_color: Some(PINK.into()),
            yaw_moment_color: Some(VIOLET.into()),
            ..Self::none()
        }
    }

    /// Only the zone health wireframes enabled.
    pub fn zones() -> Self {
        Self {
            zone_color: Some(ORANGE.into()),
            ..Self::none()
        }
    }
}

//
// Per-entity override
//

/// Debug render override for one zone. Attach it to an [`AeroZone`] entity
/// to replace the zone-type color of its wireframe.
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
#[derive(Default)]
pub struct FdmDebugRender {
    /// Wireframe color for this zone. `None` = use the color of the zone type.
    ///
    /// The damage tint applies on top of this color. To disable all zone
    /// wireframes, set [`FdmGizmos::zone_color`] to `None`.
    pub zone_color: Option<Color>,
}

impl FdmDebugRender {
    /// Sets the wireframe color of this zone.
    pub fn with_zone_color(mut self, color: Color) -> Self {
        self.zone_color = Some(color);
        self
    }

    /// Removes the color override. The zone then uses the color of its zone type.
    pub fn without_zone(mut self) -> Self {
        self.zone_color = None;
        self
    }
}
