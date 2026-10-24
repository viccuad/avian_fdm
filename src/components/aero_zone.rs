//! [`AeroZone`] and related types for the aerodynamic model of each zone.

use crate::_bevy::*;
use crate::components::aero_coeff::AeroCoeff;
use avian3d::math::Scalar;
use avian3d::prelude::Collider;
use serde::{Deserialize, Serialize};

/// Aerodynamic coefficients of one zone.
///
/// Attach this component to a child entity that has an Avian [`Collider`].
/// In each physics step, the FDM system finds all entities with this
/// component. It evaluates their coefficients at the current flight state.
/// Then it adds the force and the torque on the aircraft root entity.
///
/// It is on each **AeroZone child entity** (a child of the aircraft root
/// entity).
///
/// The damage state is in a separate component, [`super::Failure`].
///
/// All coefficient fields are [`AeroCoeff`].
///
/// ## Offset of the aerodynamic center
///
/// The aerodynamic center (AC) is the point on a wing section where the
/// pitching moment does not change with the angle of attack. By default, the
/// forces act at the origin of the zone entity. To move the point where the
/// forces act, set [`ac_offset`](Self::ac_offset).
///
/// ## Required components
///
/// - ZoneForce: the FDM writes it in each physics step. Do not change it.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
#[require(crate::components::zone_force::ZoneForce)]
pub struct AeroZone {
    /// Name of a registered airfoil. At spawn time, the crate copies the
    /// coefficient tables of this airfoil into the zone. Use a name that you
    /// registered with
    /// [`RegisterAirfoil::register_airfoil`](crate::airfoil::RegisterAirfoil::register_airfoil)
    /// (for example, `"NACA2412"` or `"USA35B"`).
    ///
    /// If the name is not empty, the airfoil resolution system finds it in
    /// [`AirfoilLibrary`](crate::airfoil::AirfoilLibrary). Then it replaces
    /// each [`AeroCoeff::Placeholder`] field in `cl`, `cd`, and `cm` with the
    /// library data. It never replaces an explicit (non-`Placeholder`) field.
    ///
    /// Default: `""`. An empty string means "no named airfoil", and the
    /// system skips the zone.
    pub airfoil_name: String,
    /// Contribution of this zone to CL (lift coefficient).
    ///
    /// Always present. Default: [`AeroCoeff::Placeholder`], so a zone without
    /// data gives a warning.
    pub cl: AeroCoeff,
    /// Contribution of this zone to CD (drag coefficient).
    ///
    /// Always present. Default: [`AeroCoeff::Placeholder`], so a zone without
    /// data gives a warning.
    pub cd: AeroCoeff,
    /// Contribution of this zone to CY (side-force coefficient).
    ///
    /// Default: [`AeroCoeff::Absent`]. Most symmetric zones make no side force.
    /// If this zone needs CY but you do not have the data yet, set
    /// [`AeroCoeff::Placeholder`].
    pub cy: AeroCoeff,
    /// Contribution of this zone to CM (pitching-moment coefficient, about c_bar).
    ///
    /// Default: [`AeroCoeff::Absent`]. Usually, the tail geometry gives the pitching moment.
    /// If this zone needs CM but you do not have the data yet, set
    /// [`AeroCoeff::Placeholder`].
    pub cm: AeroCoeff,
    /// Contribution of this zone to Croll (rolling-moment coefficient, about b).
    ///
    /// The name is `croll`, not `cl`, because `cl` is the lift coefficient field on
    /// [`AirfoilData`](crate::airfoil::AirfoilData). The standard ICAS and NASA notation
    /// for the rolling moment is `Cl`.
    ///
    /// Default: [`AeroCoeff::Absent`]. Usually, the roll comes from the zone geometry.
    /// If this zone needs Croll but you do not have the data yet, set
    /// [`AeroCoeff::Placeholder`].
    pub croll: AeroCoeff,
    /// Contribution of this zone to Cn (yawing-moment coefficient, about b).
    ///
    /// Default: [`AeroCoeff::Absent`]. Usually, the yaw comes from the zone geometry.
    /// If this zone needs Cn but you do not have the data yet, set
    /// [`AeroCoeff::Placeholder`].
    pub cn: AeroCoeff,
    /// Offset from the local origin of the zone entity to the aerodynamic
    /// center, in the local frame of the zone (meters).
    ///
    /// If the value is `Vec3::ZERO` (the default), the entity origin *is* the
    /// AC. That is, the [`Transform`] position of the zone is both the mesh
    /// center and the point where the forces act.
    ///
    /// Example: a wing panel has its mesh center at mid-chord. To put the AC
    /// at the quarter-chord point, use `Vec3::new(0.25 * chord, 0.0, 0.0)`
    /// (body-frame X is forward).
    pub ac_offset: Vec3,
    /// If `Some`, this zone is a control surface. The matching
    /// [`super::ControlInputs`] value also scales its coefficients.
    pub control_role: Option<ControlSurfaceRole>,
    /// Extra drag from structural deformation when the zone has partial
    /// damage. `None` (the default) disables it. Before the scale by
    /// `remaining`, the extra drag coefficient is
    /// `damage_drag_coeff * (1 - remaining) / qbar`. The extra drag is zero
    /// for an intact zone and for a fully detached zone.
    pub damage_drag_coeff: Option<Scalar>,

    /// Aerodynamic planform area of this zone (m²).
    ///
    /// Force = coefficient * qbar * area_m2. Thus, each zone is
    /// self-contained. For a wing zone, use the physical planform area.
    /// Default: 0.0 (no aerodynamic force). This is correct for zones that
    /// only add mass.
    pub area_m2: Scalar,

    /// Reference chord of this zone (m). The crate uses it to change the
    /// pitching-moment coefficient into a moment:
    /// `M_pitch = CM * qbar * area_m2 * chord_m`. It also uses it to calculate
    /// the Reynolds number of the zone.
    ///
    /// For a wing zone, use the mean aerodynamic chord. Default: 0.0. Then the
    /// zone adds no pitching moment, and its Reynolds number is 0.
    pub chord_m: Scalar,
}

impl AeroZone {
    /// Extends the CL, CD, and CY tables to +/-180 deg with the
    /// Viterna-Corrigan post-stall model.
    ///
    /// The method calculates the aspect ratio from the zone geometry as
    /// AR = area_m2 / chord_m^2. This value is exact for rectangular surfaces
    /// (wings, stabilizers, and fins with a constant chord). For a tapered or
    /// swept surface, the true AR is span^2 / area. This value is different
    /// from area / chord^2 when the chord changes along the span. For these
    /// surfaces, call the lower-level methods
    /// [`AeroCoeff::with_post_stall_lift`] and
    /// [`AeroCoeff::with_post_stall_drag`] directly, with the correct AR.
    ///
    /// The method applies `with_post_stall_lift` to CL and CY, and
    /// `with_post_stall_drag` to CD. It does not change the other coefficients
    /// (CM, Croll, Cn). It also does not change the variants that the
    /// extension does not apply to (Absent, Placeholder).
    ///
    /// If all tables cover the full +/-180 deg range, the evaluation never
    /// clamps the angle. This is true for all local angles of attack, also in
    /// post-stall flight or in a tumble.
    ///
    /// If the zone has a zero chord or a zero area, the method returns it
    /// with no change. No meaningful AR exists for such a zone.
    ///
    /// # Example
    ///
    /// ```
    /// # use avian_fdm::components::*;
    /// # use avian_fdm::components::aero_coeff::AeroCoeff;
    /// let zone = AeroZone {
    ///     cl: AeroCoeff::Table1D {
    ///         breakpoints: vec![-0.35, 0.0, 0.35],
    ///         values: vec![-2.5, 0.0, 2.5],
    ///     },
    ///     cd: AeroCoeff::Scalar(0.01),
    ///     area_m2: 3.0,
    ///     chord_m: 1.0,
    ///     ..Default::default()
    /// }.with_post_stall_extension();
    /// // AR = 3.0 / 1.0^2 = 3.0. The method calculates it automatically.
    /// // CL now covers +/-180 deg (Viterna lift model).
    /// // CD is now a Table1D with flat-plate drag at high angles.
    /// ```
    pub fn with_post_stall_extension(mut self) -> Self {
        if self.chord_m <= 0.0 || self.area_m2 <= 0.0 {
            return self;
        }
        let ar = self.area_m2 / (self.chord_m * self.chord_m);
        self.cl = self.cl.with_post_stall_lift(ar);
        self.cd = self.cd.with_post_stall_drag(ar);
        self.cy = self.cy.with_post_stall_lift(ar);
        self
    }

    /// Validates all coefficients and fields. Returns a list of problems. An
    /// empty list means that the zone is valid.
    ///
    /// The method examines each [`AeroCoeff`] field for table structure
    /// errors (unsorted breakpoints, mismatched dimensions, NaN or Inf
    /// values). It also reports a negative `area_m2` or `chord_m`.
    ///
    /// Call it at spawn time to find data-entry mistakes early. Refer to
    /// [`AeroCoeff::validate`] for the checks on each coefficient.
    pub fn validate(&self, zone_name: &str) -> Vec<String> {
        let mut problems = Vec::new();
        let fields: &[(&str, &AeroCoeff)] = &[
            ("cl", &self.cl),
            ("cd", &self.cd),
            ("cy", &self.cy),
            ("cm", &self.cm),
            ("croll", &self.croll),
            ("cn", &self.cn),
        ];
        for (field, coeff) in fields {
            let label = format!("{zone_name}.{field}");
            problems.extend(coeff.validate(&label));
        }
        if self.area_m2 < 0.0 {
            problems.push(format!(
                "{zone_name}: area_m2 is negative ({:.4})",
                self.area_m2
            ));
        }
        if self.chord_m < 0.0 {
            problems.push(format!(
                "{zone_name}: chord_m is negative ({:.4})",
                self.chord_m
            ));
        }
        problems
    }
}

/// The flight control function of a zone, if it has one.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[reflect(Serialize, Deserialize)]
pub enum ControlSurfaceRole {
    /// Elevator on the horizontal tail.
    Elevator,
    /// Left aileron.
    AileronLeft,
    /// Right aileron. The crate inverts the sign of the aileron input for this role.
    AileronRight,
    /// Rudder on the vertical tail.
    Rudder,
}

/// Bundle for one aerodynamic zone child entity.
///
/// Spawn it as a child of the aircraft root entity.
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
    /// Avian collider that sets the shape and volume of this zone.
    ///
    /// Avian uses it to calculate the mass contribution of the zone (with
    /// [`avian3d::prelude::ColliderDensity`]) to the
    /// [`avian3d::prelude::ComputedMass`] of the parent rigid body.
    ///
    /// Zone colliders are **real physics colliders**. By default, they touch
    /// all other colliders in the world (terrain, obstacles). For
    /// aerodynamics only, with no collision response, give each zone entity
    /// [`avian3d::prelude::CollisionLayers`]. Use a layer that does not
    /// interact with the world. Do not use [`avian3d::prelude::Sensor`].
    /// Avian does not include sensor colliders in the mass calculation, so the
    /// aircraft gets an incorrect inertia.
    pub collider: Collider,
    /// Position and orientation relative to the aircraft root entity.
    pub transform: Transform,
    /// Bevy needs it for transform propagation.
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
            airfoil_name: String::new(),
            control_role: None,
            damage_drag_coeff: None,
            area_m2: 0.0,
            chord_m: 0.0,
        }
    }
}
