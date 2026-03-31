//! [`AeroZone`] and related types for the per-zone aerodynamic model.

use crate::components::aero_coeff::AeroCoeff;
use crate::components::zone_force::ZoneForce;
use avian3d::prelude::Collider;
use crate::_bevy::*;
use serde::{Deserialize, Serialize};

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
/// All fields are [`AeroCoeff`]. Three variants carry distinct meaning:
///
/// | Value | Meaning | Runtime |
/// |---|---|---|
/// | `Absent` (default for `cy/cm/croll/cn`) | Not applicable by design: symmetric section, no contribution | Silent 0.0 |
/// | `Placeholder` (default for `cl/cd`) | Should exist but not yet modelled | `warn_once!` + 0.0 |
/// | `Scalar(0.0)` with [`crate::sourced!`] | Explicitly zero | Silent 0.0 |
/// | `Table1D` / `Table2D` | Fully modelled | Interpolated value |
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
    /// Defaults to [`AeroCoeff::Absent`]. Most symmetric zones produce no side force.
    /// Set to [`AeroCoeff::Placeholder`] if this zone should contribute CY but data is pending.
    pub cy: AeroCoeff,
    /// Partial contribution to CM (pitching-moment coefficient, about c̄).
    ///
    /// Defaults to [`AeroCoeff::Absent`]. Pitching moment is often handled via tail geometry.
    /// Set to [`AeroCoeff::Placeholder`] if this zone should contribute CM but data is pending.
    pub cm: AeroCoeff,
    /// Partial contribution to Cl (rolling-moment coefficient, about b).
    ///
    /// Defaults to [`AeroCoeff::Absent`]. Roll is often handled emergently by zone geometry.
    /// Set to [`AeroCoeff::Placeholder`] if this zone should contribute Cl but data is pending.
    pub croll: AeroCoeff,
    /// Partial contribution to Cn (yawing-moment coefficient, about b).
    ///
    /// Defaults to [`AeroCoeff::Absent`]. Yaw is often handled emergently by zone geometry.
    /// Set to [`AeroCoeff::Placeholder`] if this zone should contribute Cn but data is pending.
    pub cn: AeroCoeff,
    /// Offset from the zone entity's local origin to the Aerodynamic Centre,
    /// in the zone's local coordinate frame (metres).
    ///
    /// When `Vec3::ZERO` (default), the entity origin *is* the AC, i.e. the
    /// zone's [`Transform`] position is both mesh centre and force application
    /// point.
    ///
    /// For a wing panel whose mesh is centered at mid-chord, a typical value
    /// is `Vec3::new(0.25 * chord, 0.0, 0.0)` to place the AC at the
    /// quarter-chord point (body-frame X = forward).
    pub ac_offset: Vec3,
    /// If `Some`, this zone acts as a control surface. Its coefficients are
    /// additionally scaled by the matching [`super::ControlInputs`] value.
    pub control_role: Option<ControlSurfaceRole>,
    /// Extra drag added when the zone is partially failed. Represents structural drag from
    /// deformation.
    ///
    /// `None` (the default) means this zone has no damage-drag model. This is
    /// the common case for most zones. `Some(coeff)` enables the calculation.
    /// **Extra drag = damage coefficient × fraction destroyed ÷ dynamic pressure.
    /// Peaks at intermediate failure; zero when fully intact or fully detached.**
    ///
    /// ```text
    /// CD_extra = coeff × (1 − remaining) / q̄   when remaining > 0
    ///          = 0                              when remaining == 0 (detached)
    /// ```
    ///
    /// Only set this on zones where partial failure causes visible structural
    /// deformation that increases drag (e.g. a bent wing panel, torn fabric).
    pub damage_drag_coeff: Option<f64>,

    /// Aerodynamic planform area of this zone (m²).
    ///
    /// Force is computed as `coeff * q_bar * area_m2`, so each zone is
    /// self-contained: its CL/CD tables hold the true airfoil coefficients and
    /// the area scales them to the correct force magnitude. This replaces the
    /// old pattern of scaling whole-aircraft coefficients by area fraction and
    /// multiplying by the aircraft reference area.
    ///
    /// For wing zones, set this to the physical planform area of the panel.
    /// For tail and control surface zones whose coefficients were derived from
    /// whole-aircraft stability derivatives (e.g. CM_alpha, CN_beta), set this
    /// to the aircraft reference wing area so that the derived coefficients
    /// produce the correct force.
    ///
    /// Defaults to 0.0, meaning the zone produces no aerodynamic force. This is
    /// correct for mass-only zones (fuselage, struts, gear) that have zero
    /// CL/CD.
    pub area_m2: f64,

    /// Reference chord for this zone (m), used to dimensionalize pitching-moment
    /// coefficients: `M_pitch = CM * q_bar * area_m2 * chord_m`.
    ///
    /// For wing zones, use the mean aerodynamic chord. Defaults to 0.0 (no
    /// pitching moment contribution, which is correct when CM is Absent).
    pub chord_m: f64,
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
    /// Avian collider, required for Avian to include this zone's mass (via
    /// [`avian3d::prelude::ColliderDensity`]) in the parent rigid body's
    /// [`avian3d::prelude::ComputedMass`].
    /// One can use it also for hit detection, but it's better to use a Sensor over a real model
    /// part.
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
            cy: AeroCoeff::Absent,
            cm: AeroCoeff::Absent,
            croll: AeroCoeff::Absent,
            cn: AeroCoeff::Absent,
            ac_offset: Vec3::ZERO,
            control_role: None,
            damage_drag_coeff: None,
            area_m2: 0.0,
            chord_m: 0.0,
        }
    }
}
