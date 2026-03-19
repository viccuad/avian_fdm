//! [`AeroZone`] and related types for the per-zone aerodynamic model.

use bevy::prelude::*;
use avian3d::prelude::Collider;
use serde::{Deserialize, Serialize};
use crate::components::aero_coeff::AeroCoeff;
use crate::components::zone_force::ZoneForce;

/// Per-zone aerodynamic coefficient contributions.
///
/// Attach to any child entity that has an Avian [`Collider`]. The FDM
/// system queries all entities with this component, evaluates the coefficients
/// at the current flight state, and calls `apply_force_at_point` on the
/// nearest `RigidBody` ancestor — Avian computes the moment arm automatically.
///
/// Damage is tracked separately via [`super::Damageable`]. When absent, the
/// zone is treated as fully intact.
///
/// Lives on each **AeroZone child entity** (child of the aircraft root).
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AeroZone {
    /// Partial contribution to CL (lift coefficient).
    pub cl: AeroCoeff,
    /// Partial contribution to CD (drag coefficient).
    pub cd: AeroCoeff,
    /// Partial contribution to CY (side-force coefficient).
    pub cy: AeroCoeff,
    /// Partial contribution to CM (pitching-moment coefficient, about c̄).
    pub cm: AeroCoeff,
    /// Partial contribution to Cl (rolling-moment coefficient, about b).
    pub croll: AeroCoeff,
    /// Partial contribution to Cn (yawing-moment coefficient, about b).
    pub cn: AeroCoeff,
    /// If `Some`, this zone acts as a control surface. Its coefficients are
    /// additionally scaled by the matching [`super::ControlInputs`] value.
    pub control_role: Option<ControlSurfaceRole>,
    /// Drag pressure (Pa) added per unit of damage while the zone is still
    /// attached (`health > 0`). Represents structural drag from deformation.
    ///
    /// ```text
    /// struct_drag = damage_drag_coeff × (1 − health)   when health > 0
    ///             = 0                                   when health == 0 (detached)
    /// ```
    pub damage_drag_coeff: f64,
}

/// Which flight control function this zone performs, if any.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[reflect(Serialize, Deserialize)]
pub enum ControlSurfaceRole {
    /// Horizontal tail elevator.
    Elevator,
    /// Left aileron.
    AileronLeft,
    /// Right aileron.
    AileronRight,
    /// Vertical tail rudder.
    Rudder,
}

/// Common structural material densities (kg/m³) for use with Avian's
/// [`avian3d::prelude::ColliderDensity`].
///
/// # Example
/// ```rust
/// use avian_fdm::components::materials;
/// // entity.insert(ColliderDensity(materials::ALUMINIUM as f32));
/// let _ = materials::ALUMINIUM;
/// ```
pub mod materials {
    /// Aluminium alloy (6061-T6).
    pub const ALUMINIUM: f64 = 2_700.0;
    /// Structural steel.
    pub const STEEL: f64 = 7_800.0;
    /// Titanium alloy (Ti-6Al-4V).
    pub const TITANIUM: f64 = 4_500.0;
    /// Carbon fibre reinforced polymer (CFRP).
    pub const CARBON_FIBRE: f64 = 1_600.0;
    /// Glass fibre reinforced polymer (GFRP).
    pub const GLASS_FIBRE: f64 = 1_800.0;
    /// Balsa wood — used in RC aircraft ribs and formers.
    pub const BALSA: f64 = 150.0;
    /// Aircraft-grade plywood.
    pub const PLYWOOD: f64 = 600.0;
    /// Expanded polystyrene (EPS) foam.
    pub const FOAM: f64 = 30.0;
    /// Rubber — tyres, seals.
    pub const RUBBER: f64 = 1_200.0;
    /// Perspex / acrylic — canopy glazing.
    pub const PERSPEX: f64 = 1_190.0;
}

/// Bundle for one aerodynamic zone child entity.
///
/// Spawn as a child of the aircraft root entity.
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::*;
/// # use avian3d::prelude::*;
/// # use bevy::prelude::*;
/// // commands.spawn(AeroZoneBundle { zone: AeroZone { ... }, collider: Collider::cuboid(1.0, 0.1, 2.0), ..default() });
/// ```
#[derive(Bundle, Default)]
pub struct AeroZoneBundle {
    /// Aerodynamic coefficients and control role.
    pub zone: AeroZone,
    /// Per-frame force output (written by FDM, read by accumulation system).
    pub zone_force: ZoneForce,
    /// Avian collider — required for hit detection and for Avian to include
    /// this zone's mass (via [`avian3d::prelude::ColliderDensity`]) in the
    /// parent rigid body's [`avian3d::prelude::ComputedMass`].
    pub collider: Collider,
    /// Position/orientation relative to the aircraft root.
    pub transform: Transform,
    /// Required by Bevy for transform propagation.
    pub global_transform: GlobalTransform,
}

impl Default for AeroZone {
    fn default() -> Self {
        Self {
            cl: AeroCoeff::Scalar(0.0),
            cd: AeroCoeff::Scalar(0.0),
            cy: AeroCoeff::Scalar(0.0),
            cm: AeroCoeff::Scalar(0.0),
            croll: AeroCoeff::Scalar(0.0),
            cn: AeroCoeff::Scalar(0.0),
            control_role: None,
            damage_drag_coeff: 0.0,
        }
    }
}
