//! Components for the full aircraft: geometry, damping, induced drag, and the core spawn bundle.

use crate::_bevy::*;
use avian3d::prelude::{AngularVelocity, ConstantForce, ConstantTorque, LinearVelocity, RigidBody};
use serde::{Deserialize, Serialize};

/// Core bundle. Spawn it on the aircraft root entity.
///
/// # Mass properties (automatic by design)
///
/// Avian calculates the mass, the center of gravity, and the full
/// angular-inertia tensor automatically. It uses the shapes and the
/// [`avian3d::prelude::ColliderDensity`] values of the child zone colliders.
/// You can read the results in [`avian3d::prelude::ComputedMass`],
/// [`avian3d::prelude::ComputedCenterOfMass`], and
/// [`avian3d::prelude::ComputedAngularInertia`] on the root entity.
///
/// **Do not add [`avian3d::prelude::NoAutoMass`],
/// [`avian3d::prelude::NoAutoAngularInertia`], or
/// [`avian3d::prelude::NoAutoCenterOfMass`]** to an aircraft root entity,
/// unless you also add the matching [`avian3d::prelude::Mass`],
/// [`avian3d::prelude::AngularInertia`], or
/// [`avian3d::prelude::CenterOfMass`] component. These markers remove the
/// contribution of the child colliders. The recommended preset workflow (for
/// example, `avian_fdm_j3cub_jsbsim::presets::j3cub`) adjusts the
/// `ColliderDensity` of each zone to get the target inertia values. If you add
/// a `NoAuto*` marker without the matching explicit value, the aircraft gets
/// the default unit mass and identity inertia. No warning occurs.
///
/// ## Changes at runtime
///
/// Avian calculates the mass properties once, when the rigid body and its
/// colliders spawn together. After the first frame, you can change a child
/// [`avian3d::prelude::ColliderDensity`], replace a
/// [`avian3d::prelude::Collider`], or attach new zone children. If you do one
/// of these changes, insert
/// [`avian3d::dynamics::rigid_body::mass_properties::components::RecomputeMassProperties`]
/// on the root entity. Then the `Computed*` values update in the next physics
/// step.
///
/// # Forces
///
/// In each physics step, the crate adds the aerodynamic forces into the
/// [`ConstantForce`] and [`ConstantTorque`] components of this bundle. Avian
/// then applies them with `ForceSystems::ApplyConstantForces`.
///
/// A `RigidBody::Dynamic` has [`LinearVelocity`] and [`AngularVelocity`]. This
/// bundle includes them with zero values. To set an initial velocity, replace
/// them after the spawn.
///
/// # Collision filtering
///
/// Zone colliders are real physics colliders. By default, they touch terrain
/// and other objects in the scene. For a simulation with aerodynamics only (no
/// collision response), put [`avian3d::prelude::CollisionLayers`] on each zone
/// child entity. Then the zones do not touch world geometry. Do not use
/// [`avian3d::prelude::Sensor`]. Avian does not include a sensor collider in
/// the mass calculation.
///
/// # Optional components (add them to the same entity after the spawn)
///
/// - [`InducedDrag`]: add it for conventional lifting aircraft (most
///   fixed-wing aircraft). Do not add it for gliders with polar-based CD
///   tables, for missiles, or for low-detail AI aircraft.
/// - [`LodDamping`]: add it only for sparse-zone aircraft. In these aircraft,
///   the zone geometry cannot make realistic roll, pitch, and yaw damping.
/// - [`avian3d::prelude::SweptCcd`] with [`avian3d::prelude::CcdFilter::DYNAMIC`]:
///   add it if aircraft can collide with each other. By default, Avian sweeps
///   fast bodies only against static and kinematic bodies. Refer to
///   [Numeric Precision](crate#precision-is-not-tunneling).
///
/// # Example
/// ```rust,no_run
/// # use avian_fdm::components::*;
/// # use bevy::prelude::*;
/// // Full-detail aircraft with induced drag and zone-based damping:
/// // commands.spawn((
/// //     AircraftCoreBundle { geometry: AircraftGeometry { wing_area_m2: 16.2, wing_span_m: 10.6, chord_m: 1.53 }, ..default() },
/// //     InducedDrag { oswald_factor: 0.85 },
/// // ));
/// ```
#[derive(Bundle, Default)]
pub struct AircraftCoreBundle {
    /// Reference geometry of the aircraft.
    pub geometry: AircraftGeometry,
    /// Control inputs. Write them in each frame from your input system.
    pub controls: crate::components::ControlInputs,
    /// Flight state values that the crate calculates (the crate writes them).
    pub flight_state: crate::components::FlightState,
    /// ISA atmosphere at the altitude of this entity (the crate writes it).
    pub atmosphere: crate::components::AtmosphereState,
    /// Avian rigid body type. It must be `RigidBody::Dynamic`.
    pub rigid_body: RigidBody,
    /// Sum of the aerodynamic and propulsive forces. The crate writes it in
    /// each physics step. Avian applies it with `ForceSystems::ApplyConstantForces`.
    pub constant_force: ConstantForce,
    /// Sum of the aerodynamic and propulsive torques. The crate writes it in
    /// each physics step. Avian applies it with `ForceSystems::ApplyConstantForces`.
    pub constant_torque: ConstantTorque,
    /// Linear velocity in world space. `update_flight_state` needs it.
    /// To start with an initial airspeed, set a non-zero value after the spawn.
    pub linear_velocity: LinearVelocity,
    /// Angular velocity in world space. `update_flight_state` needs it.
    pub angular_velocity: AngularVelocity,
    /// Transform in world space.
    pub transform: Transform,
    /// Bevy needs it for transform propagation.
    pub global_transform: GlobalTransform,
}

/// Reference geometry of the wing. The crate uses these values to change
/// forces into dimensionless coefficients (non-dimensionalization) and back.
///
/// It is on the **aircraft root entity**, as part of [`AircraftCoreBundle`].
///
/// Optional components for the root entity:
/// - [`LodDamping`]: explicit damping for sparse-zone aircraft.
/// - [`InducedDrag`]: lift-induced drag for conventional aircraft.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug, Default)]
#[reflect(Component, Serialize, Deserialize)]
pub struct AircraftGeometry {
    /// Reference wing area S (m²).
    pub wing_area_m2: f32,
    /// Wing span b (m). The reference length for rolling and yawing moments.
    pub wing_span_m: f32,
    /// Mean aerodynamic chord c_bar (m). The reference length for the pitching
    /// moment in [`LodDamping`].
    pub chord_m: f32,
}

/// Damping derivatives for the angular rates of the full aircraft. LOD means
/// "level of detail": this is a fallback for simple models.
///
/// A damping derivative is a coefficient. It sets how strongly the aircraft
/// resists a rotation about one axis. Add this component to the aircraft root
/// entity when the zone layout is too sparse to make realistic damping from
/// geometry. Examples are single-zone missiles and low-detail aircraft.
///
/// **This component and the per-zone local alpha and beta are mutually
/// exclusive.** If this component is present, `compute_aero_forces` evaluates
/// all zones at the global alpha and beta, with no rate corrections. These
/// derivatives are then the only source of angular damping. If this component
/// is absent, each zone uses its local angles. Damping then comes from the
/// zone geometry.
///
/// # Non-dimensional form
///
/// **Damping moment = damping derivative × normalized angular rate × dynamic
/// pressure × wing area × reference length.** The normalized rate (for
/// example, roll rate × wing span ÷ (2 × airspeed)) is dimensionless. It
/// compares the rotational speed at the tip to the forward speed. A negative
/// derivative gives damping that opposes the motion.
///
/// ```text
/// delta_L = Cl_p * (p * b / 2V)     * qbar * S * b       roll damping (body X)
/// delta_M = Cm_q * (q * c_bar / 2V) * qbar * S * c_bar   pitch damping (body Y)
/// delta_N = Cn_r * (r * b / 2V)     * qbar * S * b       yaw damping (body Z)
/// ```
///
/// All derivatives must be negative, so that the damping opposes the motion.
/// Typical values for light general-aviation aircraft (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct LodDamping {
    /// Roll damping derivative ∂Cl/∂p_hat, where p_hat = p * b / (2V).
    /// Typical range for light aircraft: −0.4 to −0.5.
    pub cl_p: f32,
    /// Pitch damping derivative ∂Cm/∂q_hat, where q_hat = q * c_bar / (2V).
    /// Typical range for light aircraft: −10 to −20.
    pub cm_q: f32,
    /// Yaw damping derivative ∂Cn/∂r_hat, where r_hat = r * b / (2V).
    /// Typical range for light aircraft: −0.1 to −0.15.
    pub cn_r: f32,
}

/// Model of the lift-induced drag.
///
/// Induced drag is the extra drag that lift causes. The wing-tip vortices
/// turn the airflow downward behind the wing.
///
/// Add this component to the aircraft root entity to add induced drag for
/// the full aircraft. Related topics: induced drag, drag polar, Oswald
/// efficiency.
///
/// **Do not add this component** for:
/// - **Gliders**. Usually, the CD tables of the wing zones come from measured
///   polar data, and they include the induced drag.
/// - **Missiles and projectiles**. They have no significant lift distribution
///   along the span. The drag of the blunt body is the largest drag.
/// - **Aircraft with zone CD tables that include induced drag**. This
///   component then counts the induced drag two times.
///
/// **Add this component** for:
/// - Each conventional lifting aircraft with zone CD values from a 2-D profile
///   drag polar (JSBSim style). This type of polar keeps the parasite drag and
///   the induced drag separate.
///
/// Induced drag coefficient = lift coefficient squared ÷ (π × Oswald
/// efficiency × aspect ratio):
///
/// ```text
/// CD_i = CL^2 / (pi * e * AR),   AR = b^2 / S
/// ```
///
/// Typical Oswald efficiency values: high-wing light aircraft 0.85–0.95,
/// low-wing monoplane 0.75–0.85, elliptical wing (ideal) 1.0, delta or swept
/// wing 0.6–0.75.
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct InducedDrag {
    /// Oswald span-efficiency factor *e*, range (0, 1].
    ///
    /// It shows how near the lift distribution of the wing is to the ideal
    /// elliptical shape. A value of 1.0 is perfect (no lost energy). Lower
    /// values include real losses from the wing shape and from fuselage
    /// interference.
    pub oswald_factor: f32,
}
