//! [`AeroZone`], [`AeroZoneHealth`], and related types for the zone-based
//! damage model.
//!
//! Only compiled when `features = ["damage"]`.

use bevy::prelude::*;
use bevy::math::DVec3;
use serde::{Deserialize, Serialize};
use avian3d::prelude::Collider;
use crate::components::aero_coeff::AeroCoeff;

/// Per-zone aerodynamic coefficient contributions and material properties.
///
/// Authored in Blender/Skein and exported as part of the scene. This is the
/// primary aircraft configuration surface — designers specify all aerodynamic
/// coefficients here, per zone.
///
/// Lives on each **AeroZone child entity**.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AeroZone {
    /// This zone's partial contribution to CL at full health.
    pub cl: AeroCoeff,
    /// This zone's partial contribution to CD at full health.
    pub cd: AeroCoeff,
    /// This zone's partial contribution to CY (side force) at full health.
    pub cy: AeroCoeff,
    /// This zone's partial contribution to CM (pitching moment) at full health.
    pub cm: AeroCoeff,
    /// This zone's partial contribution to Cl (rolling moment) at full health.
    pub croll: AeroCoeff,
    /// This zone's partial contribution to Cn (yawing moment) at full health.
    pub cn: AeroCoeff,
    /// If `Some`, this zone acts as a control surface of the given type.
    /// Its derivatives are also scaled by the matching [`ControlInputs`] value
    /// and by zone health.
    pub control_role: Option<ControlSurfaceRole>,
    /// How this zone's mass is determined at `PostStartup`. See [`ZoneMass`].
    pub zone_mass: ZoneMass,
    /// Extra drag coefficient added per unit of damage (1 − health).
    /// Represents structural drag from deformation and exposed internals.
    pub damage_drag_coeff: f64,
}

/// Runtime health and cached mass/volume of one aerodynamic zone.
///
/// `value` is written by the game's hit/damage system.
/// `collider_volume_m3` and `mass_kg` are computed once at `PostStartup`
/// and cached — never recomputed per frame.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AeroZoneHealth {
    /// Zone health: 1.0 = fully intact, 0.0 = completely destroyed.
    /// Write this from your projectile/collision system.
    pub value: f64,
    /// Collider volume in m³, computed at `PostStartup` from `Collider::volume()`.
    /// Zero if the collider volume API is unavailable (see implementation note).
    pub collider_volume_m3: f64,
    /// Mass at full health (kg), computed at `PostStartup` from [`ZoneMass`]:
    /// - `FromDensity(d)` → `d × collider_volume_m3`
    /// - `Direct(kg)` → `kg`
    ///
    /// Current frame mass contribution = `mass_kg × health.value`.
    pub mass_kg: f64,
}

impl Default for AeroZoneHealth {
    fn default() -> Self {
        Self { value: 1.0, collider_volume_m3: 0.0, mass_kg: 0.0 }
    }
}

/// How the mass of an [`AeroZone`] is determined once at `PostStartup`.
///
/// Choose based on how accurately the zone's Avian [`Collider`] represents
/// the actual structural member geometry.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug, PartialEq)]
#[reflect(Serialize, Deserialize)]
pub enum ZoneMass {
    /// `mass = material_density × collider_volume_m3`
    ///
    /// Use when the collider accurately represents the structural member
    /// geometry — e.g. a spar modelled as a thin box, a skin panel as a flat
    /// cuboid, a foam core as a solid block. Pass a value from the
    /// [`materials`] module or any custom density in kg/m³.
    FromDensity(f64),

    /// `mass = the specified value in kg`
    ///
    /// Use when the collider is a bounding approximation (not the actual
    /// material volume), or when the part mass is known from published
    /// weight-and-balance data.
    Direct(f64),
}

impl Default for ZoneMass {
    fn default() -> Self {
        ZoneMass::Direct(0.0)
    }
}

/// Common structural material densities (kg/m³) for use with
/// [`ZoneMass::FromDensity`].
///
/// # Example
/// ```rust
/// use avian_fdm::components::ZoneMass;
/// use avian_fdm::components::materials;
///
/// let spar_mass = ZoneMass::FromDensity(materials::ALUMINIUM);
/// ```
pub mod materials {
    /// Aluminium alloy (7075-T6: 2 800 kg/m³; 6061-T6: 2 700 kg/m³).
    pub const ALUMINIUM: f64 = 2_700.0;
    /// Structural steel.
    pub const STEEL: f64 = 7_800.0;
    /// Titanium alloy (Ti-6Al-4V).
    pub const TITANIUM: f64 = 4_500.0;
    /// Carbon fibre reinforced polymer (CFRP), unidirectional layup.
    pub const CARBON_FIBRE: f64 = 1_600.0;
    /// Glass fibre reinforced polymer (GFRP).
    pub const GLASS_FIBRE: f64 = 1_800.0;
    /// Balsa wood — used in RC aircraft ribs and formers.
    pub const BALSA: f64 = 150.0;
    /// Aircraft-grade plywood.
    pub const PLYWOOD: f64 = 600.0;
    /// Expanded polystyrene (EPS) foam — RC fuselages and wings.
    pub const FOAM: f64 = 30.0;
    /// Rubber — tyres, seals.
    pub const RUBBER: f64 = 1_200.0;
    /// Perspex / acrylic — canopy glazing.
    pub const PERSPEX: f64 = 1_190.0;
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

/// Per-surface control effectiveness scale factors (0–1).
///
/// Computed by `aggregate_zones` from the health of zones with matching
/// [`ControlSurfaceRole`]. A fully damaged elevator gives `elevator = 0.0`.
/// Multiplicative accumulation: two 50%-health elevator zones → 0.25.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Serialize, Deserialize)]
pub struct ControlEffectiveness {
    /// Elevator effectiveness (0–1).
    pub elevator: f64,
    /// Left aileron effectiveness (0–1).
    pub aileron_left: f64,
    /// Right aileron effectiveness (0–1).
    pub aileron_right: f64,
    /// Rudder effectiveness (0–1).
    pub rudder: f64,
}

impl Default for ControlEffectiveness {
    fn default() -> Self {
        Self { elevator: 1.0, aileron_left: 1.0, aileron_right: 1.0, rudder: 1.0 }
    }
}

/// Bundle for one aerodynamic zone child entity.
///
/// Spawn as a child of the aircraft root entity. Also requires an Avian
/// [`Collider`] (included in this bundle) for hit detection and volume
/// computation.
#[derive(Bundle)]
pub struct AeroZoneBundle {
    /// Aerodynamic coefficient contributions and material properties.
    pub zone: AeroZone,
    /// Runtime health and cached mass. `value` starts at 1.0.
    pub health: AeroZoneHealth,
    /// Avian collider — used for hit detection and (with `FromDensity`) volume.
    pub collider: Collider,
    /// Position in the aircraft body frame.
    pub transform: Transform,
    /// Required by Bevy for transform propagation.
    pub global_transform: GlobalTransform,
}
