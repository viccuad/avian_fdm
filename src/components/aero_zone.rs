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
/// at the current flight state, and accumulates force + torque onto the
/// nearest `RigidBody` ancestor.
///
/// ## Aerodynamic centre offset
///
/// By default, aerodynamic forces are applied at the zone entity's origin
/// (its [`Transform`] position). When building aircraft from 3D models
/// (e.g. in Blender), it is often more convenient to place the zone entity
/// at the geometric centre of its mesh, then specify where the aerodynamic
/// centre is relative to that origin via [`ac_offset`](Self::ac_offset).
///
/// ```text
///   Zone origin (mesh centre)
///        │
///        ├── ac_offset ──▶ Aerodynamic Centre
///        │                  (force application point)
/// ```
///
/// The moment-coefficient data (CM, Croll, Cn) is assumed to be referenced
/// to the aerodynamic centre.
///
/// Failure state is tracked separately via [`super::Failure`]. When absent, the
/// zone is treated as fully intact.
///
/// ## Coefficient presence semantics
///
/// Primary coefficients (`cl`, `cd`) are always present — use
/// [`AeroCoeff::Scalar`]`(0.0)` (with a [`crate::sourced!`] note) for intentional zeros.
///
/// Secondary coefficients (`cy`, `cm`, `croll`, `cn`) are `Option`:
///
/// | Field value | Meaning |
/// |---|---|
/// | `None` (default) | Absent by design — symmetric section, no contribution |
/// | `Some(Placeholder)` | Contribution expected but not yet modelled |
/// | `Some(Scalar(0.0))` | Explicitly zero (document with `sourced!`) |
/// | `Some(Table1D/2D)` | Fully modelled |
///
/// Lives on each **AeroZone child entity** (child of the aircraft root).
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AeroZone {
    /// Partial contribution to CL (lift coefficient).
    ///
    /// Always present. Defaults to [`AeroCoeff::Placeholder`] so unset zones warn.
    pub cl: AeroCoeff,
    /// Partial contribution to CD (drag coefficient).
    ///
    /// Always present. Defaults to [`AeroCoeff::Placeholder`] so unset zones warn.
    pub cd: AeroCoeff,
    /// Partial contribution to CY (side-force coefficient).
    ///
    /// `None` = absent by design (most symmetric zones).
    /// `Some(Placeholder)` = expected but not yet modelled.
    pub cy: Option<AeroCoeff>,
    /// Partial contribution to CM (pitching-moment coefficient, about c̄).
    ///
    /// `None` = absent by design (moment handled via tail geometry).
    /// `Some(Placeholder)` = expected but not yet modelled.
    pub cm: Option<AeroCoeff>,
    /// Partial contribution to Cl (rolling-moment coefficient, about b).
    ///
    /// `None` = absent by design (handled emergently by zone geometry).
    /// `Some(Placeholder)` = expected but not yet modelled.
    pub croll: Option<AeroCoeff>,
    /// Partial contribution to Cn (yawing-moment coefficient, about b).
    ///
    /// `None` = absent by design (handled emergently by zone geometry).
    /// `Some(Placeholder)` = expected but not yet modelled.
    pub cn: Option<AeroCoeff>,
    /// Offset from the zone entity's local origin to the aerodynamic centre,
    /// in the zone's local coordinate frame (metres).
    ///
    /// When `Vec3::ZERO` (default), the entity origin *is* the AC — i.e. the
    /// zone's [`Transform`] position is both mesh centre and force application
    /// point (the legacy behaviour).
    ///
    /// For a wing panel whose mesh is centred at mid-chord, a typical value
    /// is `Vec3::new(0.25 * chord, 0.0, 0.0)` to place the AC at the
    /// quarter-chord point (body-frame X = forward).
    pub ac_offset: Vec3,
    /// If `Some`, this zone acts as a control surface. Its coefficients are
    /// additionally scaled by the matching [`super::ControlInputs`] value.
    pub control_role: Option<ControlSurfaceRole>,
    /// Extra drag added when the zone is partially failed but still attached
    /// (`remaining > 0`). Represents structural drag from deformation.
    ///
    /// `None` (the default) means this zone has no damage-drag model — the
    /// common case for most zones. `Some(coeff)` enables the calculation:
    ///
    /// ```text
    /// CD_extra = coeff × (1 − remaining) / q̄   when remaining > 0
    ///          = 0                              when remaining == 0 (detached)
    /// ```
    ///
    /// Only set this on zones where partial failure causes visible structural
    /// deformation that increases drag (e.g. a bent wing panel, torn fabric).
    pub damage_drag_coeff: Option<f64>,
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
            cl: AeroCoeff::Placeholder,
            cd: AeroCoeff::Placeholder,
            cy: None,
            cm: None,
            croll: None,
            cn: None,
            ac_offset: Vec3::ZERO,
            control_role: None,
            damage_drag_coeff: None,
        }
    }
}
