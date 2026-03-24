//! # avian_fdm, 6-DoF Flight Dynamics Model for Bevy + Avian
//!
//! `avian_fdm` is a Bevy plugin that turns an Avian rigid-body hierarchy into
//! a physically plausible aircraft. Each physics step it evaluates
//! aerodynamic and propulsive forces on every [`components::AeroZone`] child
//! entity and accumulates them into Avian's [`avian3d::prelude::ConstantForce`]
//! and [`avian3d::prelude::ConstantTorque`] on the root body. Avian's
//! integrator then advances position, velocity, orientation, and angular
//! velocity, `avian_fdm` never touches those directly.
//!
//! Mass, centre of gravity, and the full inertia tensor are computed
//! automatically by Avian from the [`avian3d::prelude::ColliderDensity`] on
//! each child collider. Failing or destroying a zone (setting
//! [`components::Failure::remaining`] to 0) instantly updates the physics
//! without any bookkeeping on the game's part.
//!
//! ---
//!
//! ## Table of Contents
//!
//! 1. [Quick Start](#quick-start)
//! 2. [What is a Flight Dynamics Model?](#what-is-a-flight-dynamics-model)
//! 3. [Coordinate Frames](#coordinate-frames)
//! 4. [The Equations of Motion](#the-equations-of-motion)
//! 5. [The Atmosphere](#the-atmosphere)
//! 6. [Aerodynamic Forces and Moments](#aerodynamic-forces-and-moments)
//! 7. [Propulsion Coupling](#propulsion-coupling)
//! 8. [Zone Decomposition and Damage](#zone-decomposition-and-damage)
//!    - [Collider strategy](#collider-strategy)
//! 9. [Reading Simulation Output](#reading-simulation-output)
//! 10. [Data Flow](#data-flow)
//! 11. [Feature Flags](#feature-flags)
//! 12. [Further Reading](#further-reading)
//!
//! ---
//!
//! ## Quick Start
//!
//! Add `avian_fdm` and `avian3d` to `Cargo.toml`:
//!
//! ```toml
//! avian_fdm = { version = "0.1", features = ["presets"] }
//! avian3d   = { version = "0.6" }
//! bevy      = { version = "0.18" }
//! ```
//!
//! Spawn the reference J-3 Cub aircraft:
//!
//! ```rust,no_run
//! use avian_fdm::prelude::*;
//! use avian_fdm::presets::j3cub;
//! use avian3d::prelude::{LinearVelocity, PhysicsPlugins};
//! use bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .add_plugins(PhysicsPlugins::default())
//!         .add_plugins(AircraftFdmPlugin::default())
//!         .add_systems(Startup, spawn)
//!         .run();
//! }
//!
//! fn spawn(mut commands: Commands) {
//!     let root = j3cub::spawn(
//!         &mut commands,
//!         Transform::from_xyz(0.0, 300.0, 0.0)
//!             .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
//!     );
//!     // Give it cruise airspeed so it doesn't fall before lift builds up.
//!     commands.entity(root).insert(LinearVelocity(Vec3::new(27.0, 0.0, 0.0)));
//! }
//! ```
//!
//! Build your own aircraft by assembling [`components::AeroZone`] children
//! around an [`components::AircraftCoreBundle`] root (see
//! [Zone Decomposition](#zone-decomposition-and-damage)).
//!
//! ---
//!
//! ## What is a Flight Dynamics Model?
//!
//! A **Flight Dynamics Model** (FDM) is the mathematical subsystem that
//! computes all forces and moments acting on an aircraft at each instant, given
//! its current state (position, velocity, attitude, angular rate) and the
//! pilot's control inputs.
//!
//! The output of an FDM feeds Newton's laws: the rigid-body integrator advances
//! the state forward in time. `avian_fdm` is the FDM; [Avian] is the
//! integrator.
//!
//! ### Scope of this library for now
//!
//! **In scope:**
//! - ISA atmosphere (0–20 km)
//! - Lift, drag, and side-force per zone
//! - Pitch, roll, and yaw damping derivatives
//! - Piston engine + fixed-pitch propeller model
//! - Failure degradation (performance loss as `remaining` reaches 0)
//! - 6-DoF integration (via Avian)
//!
//! **Out of scope** (or game's responsibility):
//! - Ground contact and landing gear forces
//! - Structural elasticity / aeroelasticity
//! - Compressibility effects (transonic / supersonic)
//! - Turbine and jet engine cycles
//! - Fuel burn and weight change over time
//! - Autopilot and stability augmentation
//! - Physical detachment of damaged zones
//!
//! ### Stability-derivative approach
//!
//! `avian_fdm` uses the **small-perturbation stability-derivative method**
//! (sometimes called the *linear aerodynamic model*). Aerodynamic coefficients
//! C_L, C_D, C_Y are expressed as tabulated functions of angle of attack α
//! and Reynolds number Re, then multiplied by dynamic pressure q̄ and
//! reference area S. **Aerodynamic force = coefficient × dynamic pressure × wing area:**
//!
//! ```text
//! Lift  = C_L(α, Re) · q̄ · S
//! Drag  = C_D(α, Re) · q̄ · S
//! Side  = C_Y(α, Re) · q̄ · S
//! ```
//!
//! This is the same method used by [JSBSim], [FlightGear], and most
//! fixed-wing game simulators. It captures realistic stall behaviour,
//! Reynolds-number effects, and control-surface authority without requiring a
//! full CFD solver.
//!
//! ---
//!
//! ## Coordinate Frames
//!
//! Two frames appear throughout the codebase. Understanding both is essential
//! when reading zone transforms or force vectors.
//!
//! ### Body frame (SAE aerospace convention)
//!
//! ```text
//!         X ──► (forward / nose)
//!        ╱
//!       ╱
//!      ╱──── Y (right wing / starboard)
//!      │
//!      ▼  Z (down / belly)
//! ```
//!
//! | Axis | Points toward | Positive rotation (right-hand rule) |
//! |------|---------------|--------------------------------------|
//! | X    | Nose          | Roll right (starboard wing down)     |
//! | Y    | Right wing    | Pitch nose up                        |
//! | Z    | Belly / down  | Yaw nose right                       |
//!
//! This matches the SAE J670 standard used in JSBSim and most aerospace
//! textbooks (Stevens & Lewis, Etkin & Reid, Nelson).
//!
//! Zone transforms in the presets are authored in this frame. Example:
//! a wing zone at `Transform::from_xyz(-0.10, -2.82, -0.58)` is 0.10 m aft of
//! the CG datum, 2.82 m to port (−Y), and 0.58 m above datum (−Z = up).
//!
//! ### Stability frame
//!
//! The stability frame is the body frame rotated by −α about body Y, aligning
//! its X axis with the velocity vector. Lift is defined as perpendicular to the
//! velocity (−Z_stability), drag as opposing it (−X_stability).
//! **Force in stability axes, then rotated to body frame, then to world frame:**
//!
//! ```text
//! force_stability = (−C_D · q̄ · S,  C_Y · q̄ · S,  −C_L · q̄ · S)
//! force_body      = R_y(−α) · force_stability
//! force_world     = q_root  · force_body
//! ```
//!
//! See [`aerodynamics`] for the full derivation.
//!
//! ### World frame (Bevy / Avian, Y-up right-handed)
//!
//! Bevy uses a Y-up, right-handed world frame. The aircraft must be spawned
//! with a rotation that aligns body X (forward) to world X and body Z (down) to
//! world −Y. `Quat::from_rotation_x(FRAC_PI_2)` achieves this.
//!
//! All internal computation uses `f64` for numerical stability. The only
//! `f64`-to-`f32` cast is when writing to Avian's `f32` force/torque components.
//!
//! ---
//!
//! ## The Equations of Motion
//!
//! An aircraft in free flight has **6 degrees of freedom** (6-DoF): three
//! translational (x, y, z) and three rotational (roll φ, pitch θ, yaw ψ).
//! Avian integrates all six, so this section explains what Avian is solving and
//! what `avian_fdm` must supply.
//!
//! ### Translational dynamics
//!
//! Newton's second law in vector form, **net force equals mass times acceleration:**
//!
//! ```text
//! F_net = m · (dV/dt)
//! ```
//!
//! where **V** is the centre-of-mass velocity in world coordinates. `avian_fdm`
//! supplies **F_net** via [`avian3d::prelude::ConstantForce`]; Avian advances
//! **V** each substep.
//!
//! The forces that `avian_fdm` contributes are:
//! - Aerodynamic: lift, drag, side-force from each [`components::AeroZone`]
//! - Propulsive: thrust from each [`components::EngineZone`]
//!
//! Gravity is applied separately by Avian's own gravity system.
//!
//! ### Rotational dynamics (Euler's equations)
//!
//! In **body frame**, with principal axes close to body X/Y/Z, Euler's
//! equations of motion are. **Rolling, pitching, and yawing moments equal
//! inertia times angular acceleration plus gyroscopic cross-coupling terms:**
//!
//! ```text
//! L = I_xx · ṗ + (I_zz − I_yy) · q · r   (roll  equation)
//! M = I_yy · q̇ + (I_xx − I_zz) · p · r   (pitch equation)
//! N = I_zz · ṙ + (I_yy − I_xx) · p · q   (yaw   equation)
//! ```
//!
//! where (p, q, r) are body-frame roll/pitch/yaw rates, and (L, M, N) are the
//! roll/pitch/yaw moments. Avian evaluates this system internally; `avian_fdm`
//! supplies (L, M, N) via [`avian3d::prelude::ConstantTorque`] by computing the
//! cross product of each zone force with its moment arm.
//! **Torque = lever arm × force (cross product):**
//!
//! ```text
//! τ_zone = (r_zone − r_CG) × F_zone
//! ```
//!
//! where **r_zone** is the zone's world position and **r_CG** is the
//! world-space centre of mass (read from
//! [`avian3d::prelude::ComputedCenterOfMass`]).
//!
//! ### Why Avian handles integration
//!
//! Implementing a numerically stable 6-DoF integrator is non-trivial, it
//! requires careful handling of quaternion renormalisation, sub-stepping for
//! stiff systems, and correct coupling between translation and rotation.
//! Avian provides all of this, tested and optimised. `avian_fdm` stays in its
//! lane: force and moment computation only.
//!
//! ---
//!
//! ## The Atmosphere
//!
//! See [`atmosphere`] for the full implementation.
//!
//! ### Why density drives everything
//!
//! Every aerodynamic force scales with **dynamic pressure**, the kinetic energy
//! of the airflow per unit volume. **Dynamic pressure q-bar = half × air density (kg/m³) × airspeed² (m/s):**
//!
//! ```text
//! q̄ = ½ · ρ · V²
//! ```
//!
//! where ρ is air density (kg/m³) and V is true airspeed (m/s). At sea level
//! ρ₀ = 1.225 kg/m³; at 2 500 m it drops to roughly 0.98 kg/m³, a 20%
//! reduction that directly cuts lift and drag by 20% at the same airspeed.
//! An aircraft must fly faster at altitude to generate the same lift.
//!
//! Dynamic pressure also controls Reynolds number.
//! **Reynolds number = (density × speed × chord) ÷ viscosity, a dimensionless ratio
//! of inertial to viscous forces that determines whether airflow is smooth or turbulent:**
//!
//! ```text
//! Re = ρ · V · c̄ / μ
//! ```
//!
//! where c̄ is mean aerodynamic chord and μ is dynamic viscosity. Reynolds
//! number governs boundary-layer behaviour: at low Re the flow separates
//! earlier (sharper stall, higher C_D₀), so the FDM uses Re as the second
//! dimension of its C_L/C_D lookup tables.
//!
//! ### International Standard Atmosphere (ISA)
//!
//! The [`atmosphere`] module implements ICAO Doc 7488 for 0–20 km:
//!
//! **Troposphere (h ≤ 11 000 m)**: temperature drops linearly with altitude (lapse rate),
//! pressure follows a power law, density is derived from the ideal gas law:
//! ```text
//! T = 288.15 − 0.0065 · h          (K)
//! p = 101 325 · (T / 288.15)^5.256 (Pa)
//! ρ = p / (287.053 · T)             (kg/m³)
//! ```
//!
//! **Stratosphere (11 000 m < h ≤ 20 000 m)**: temperature is constant (isothermal layer),
//! pressure decays exponentially with altitude (barometric formula):
//! ```text
//! T = 216.65                                     (K, isothermal)
//! p = p₁₁ · exp(−g · (h − 11000) / (R · T₁₁))  (Pa)
//! ρ = p / (R · T)                                (kg/m³)
//! ```
//!
//! Dynamic viscosity μ uses **Sutherland's law**: the gas-kinetic model that
//! correctly predicts viscosity *increasing* with temperature (opposite to
//! liquids). **Viscosity scales as temperature to the 3/2 power, corrected by
//! Sutherland's constant (110.4 K) for real-gas behaviour:**
//!
//! ```text
//! μ = 1.716×10⁻⁵ · (T/273.15)^(3/2) · (273.15 + 110.4) / (T + 110.4)
//! ```
//!
//! Every frame, [`atmosphere::update_atmosphere`] writes a fresh
//! [`components::AtmosphereState`] to the root entity; the aerodynamics and
//! propulsion systems read from it.
//!
//! ---
//!
//! ## Aerodynamic Forces and Moments
//!
//! See [`aerodynamics`] for the full implementation.
//!
//! ### Stability derivatives: a Taylor series in disguise
//!
//! Aerodynamic coefficients are measured quantities, not derived from first
//! principles in real-time. The stability-derivative method represents them as
//! a **Taylor series** around a trim condition (small perturbations).
//! **Lift coefficient grows linearly with angle of attack up to stall; drag
//! follows a parabolic polar (increases with lift squared):**
//!
//! ```text
//! C_L(α) ≈ C_L₀ + C_Lα · α + C_Lα² · α² + …
//! C_D(α) ≈ C_D₀ + k · C_L²        (parabolic drag polar)
//! ```
//!
//! For large-α behaviour (stall, post-stall), a lookup table is more accurate
//! than a truncated series. `avian_fdm` uses [`components::aero_coeff::AeroCoeff`],
//! which supports three representations:
//!
//! | Variant | Use case |
//! |---------|----------|
//! | `Scalar(f64)` | Constant coefficient (e.g. fuselage parasitic drag) |
//! | `Table1D` | C_L(α) for simple surfaces |
//! | `Table2D` | C_L(α, Re) for wings where boundary-layer state matters |
//!
//! The J3Cub preset uses `Table2D` for wing C_L and C_D, with α breakpoints
//! from −20° to +20° and two Re columns (1.7 × 10⁶ and 3.7 × 10⁶) derived
//! from JSBSim's J3Cub.xml and USA-35B airfoil measurements.
//!
//! ### Force construction pipeline
//!
//! For each [`components::AeroZone`] child:
//!
//! 1. Read α, q̄, Re from [`components::FlightState`] on the root entity.
//! 2. Evaluate C_L(α, Re), C_D(α, Re), C_Y(α, Re) via bilinear interpolation.
//! 3. Multiply by the zone's share of reference area (`fraction × S_ref`).
//! 4. Scale by `Failure.remaining` ∈ [0, 1]: zones at zero remaining contribute nothing.
//! 5. Construct the force vector in **stability axes**. **Force along each axis
//!    = aerodynamic coefficient × dynamic pressure × wing area:**
//!    ```text
//!    F_stab = (−C_D · q̄ · S,  C_Y · q̄ · S,  −C_L · q̄ · S)
//!    ```
//! 6. Rotate to world: `F_world = q_root · R_y(−α) · F_stab`
//! 7. Write to [`components::ZoneForce`] on the zone entity.
//!
//! All of this happens in [`aerodynamics::compute_aero_forces`], which also
//! evaluates per-zone pure torques from CM/Croll/Cn coefficients and sums
//! everything into [`avian3d::prelude::ConstantForce`] /
//! [`avian3d::prelude::ConstantTorque`] on the root.
//!
//! ### Dynamic damping
//!
//! Angular-rate damping is applied once per root as whole-aircraft moment
//! increments (not per zone). The three damping derivatives are from
//! Nelson (1998), Table B1, for the J3Cub.
//! **Damping moment = damping derivative × normalised angular rate × dynamic pressure × area × length.
//! The normalised rate (e.g. p·b/2V) is dimensionless: angular rate scaled by wingspan
//! and divided by airspeed:**
//!
//! ```text
//! ΔM = C_Mq · (q · c̄/2V) · q̄ · S · c̄   (pitch damping,  C_Mq = −12)
//! ΔL = C_lp · (p · b/2V) · q̄ · S · b    (roll  damping,  C_lp = −0.45)
//! ΔN = C_nr · (r · b/2V) · q̄ · S · b    (yaw   damping,  C_nr = −0.12)
//! ```
//!
//! These provide the rate-dependent restoring moments that prevent unrealistic
//! divergent oscillations. Without pitch damping, for example, a phugoid
//! perturbation would not decay.
//!
//! ### Control surfaces
//!
//! Each zone can be tagged with a [`components::ControlSurfaceRole`]:
//!
//! | Role | Input read | Effect |
//! |------|-----------|--------|
//! | `Elevator` | `ControlInputs::elevator` | Scales zone C_L linearly |
//! | `AileronLeft` | `ControlInputs::aileron` | C_L scaled +aileron |
//! | `AileronRight` | `ControlInputs::aileron` | C_L scaled −aileron |
//! | `Rudder` | `ControlInputs::rudder` | Scales zone C_Y linearly |
//!
//! Deflection also increases drag slightly (C_D scaled by `|input|`).
//! The moment arm from zone position to CG generates roll/pitch/yaw moments
//! automatically. No separate moment coefficient needed.
//!
//! ---
//!
//! ## Propulsion Coupling
//!
//! *Only compiled with `features = ["propulsion"]`.* See [`propulsion`].
//!
//! ### Piston engine model
//!
//! Thrust at altitude follows the **Gagg-Ferrar correction**: **maximum thrust
//! scaled by throttle position and air density ratio raised to the 0.7 power
//! (empirical constant for naturally-aspirated piston engines):**
//!
//! ```text
//! T = T_max · η_throttle · (ρ/ρ₀)^0.7
//! ```
//!
//! where `η_throttle` is read from a configurable thrust-fraction lookup table
//! (allowing non-linear throttle response curves). The density exponent 0.7 is
//! empirically validated for naturally-aspirated piston engines.
//!
//! ### Actuator disk: propwash velocity
//!
//! The induced velocity behind the propeller (used later for propwash coupling)
//! is estimated with **actuator disk theory**: **propeller-induced airspeed =
//! square root of (thrust ÷ (2 × air density × disk area)), where disk area = π × radius²:**
//!
//! ```text
//! V_ind = √(T / (2 · ρ · A_disk))    A_disk = π · (d/2)²
//! ```
//!
//! This is stored in [`components::PropwashState`] on the root entity. Future
//! work (Group B, v2) will use V_ind to augment the lift of zones in the
//! propwash stream (elevator and horizontal stabiliser on a tractor aircraft).
//!
//! ### Thrust axis
//!
//! Each [`components::EngineZone`] specifies `thrust_axis_body: DVec3`, the
//! thrust direction in body frame. For a normal tractor aircraft this is
//! `DVec3::X` (forward); for a pusher or a tilted engine it can point elsewhere.
//! The body-frame axis is rotated to world space by the root quaternion before
//! writing to [`components::ZoneForce`].
//!
//! ---
//!
//! ## Zone Decomposition and Damage
//!
//! ### Decomposing an aircraft into zones
//!
//! The key design insight: **each structural part of the aircraft is a separate
//! ECS entity** (an [`components::AeroZone`]) child of the root rigid body.
//! Each zone owns:
//! - A [`avian3d::prelude::Collider`] that gives it physical volume and mass
//!   (via [`avian3d::prelude::ColliderDensity`])
//! - A [`avian3d::prelude::Transform`] that places it in body-frame coordinates
//! - An [`components::AeroZone`] that describes its aerodynamic contribution
//! - Optionally a [`components::Failure`] for degraded-performance tracking
//!
//! Avian **automatically** computes total mass, CG, and inertia tensor from
//! all child colliders. No explicit bookkeeping required.
//!
//! **Typical zone layout:**
//!
//! | Zone | Aerodynamic role | Dominant coefficients |
//! |------|------------------|-----------------------|
//! | Wing panels (×6) | Lift + roll | C_L(α, Re) table |
//! | Ailerons (×2) | Roll control | C_L scaled by aileron input |
//! | Fuselage | Parasitic drag | C_D only |
//! | H-stabiliser | Pitch stability | C_L(α) restorative |
//! | Elevator | Pitch control | C_L scaled by elevator input |
//! | V-tail | Yaw stability (mass placeholder) | C_Y = 0 until v2 |
//! | Rudder | Yaw control | C_Y scaled by rudder input |
//! | Engine zone | Thrust + mass |, |
//!
//! ### Collider strategy
//!
//! Zone colliders serve two purposes: **mass/inertia** (via `ColliderDensity`)
//! and **debug visualisation**. The right collider type depends on the zone's role:
//!
//! | Collider type | Mass | Hit detection | When to use |
//! |---|---|---|---|
//! | Primitive (`cuboid`, `ball`, `cylinder`) | ✅ exact analytic | Approximate | Aero surfaces, structural parts. Tune `ColliderDensity` to match target mass |
//! | `ConvexHull` (from mesh) | ✅ from hull volume | Good convex approx | Volumetric parts where hull ≈ real shape (engine cowl, fuselage bulkhead) |
//! | `TriMesh` | ❌ none (static only) | Exact | Never on AeroZones; use only for terrain or static scenery |
//!
//! In practice, **aero zones use primitives** and the detailed 3D model is a
//! separate visual-only child entity (no `Collider`, no `RigidBody`). The
//! primitive wireframes are diagnostic; the player sees the mesh.
//!
//! For accurate **hit detection** in a combat game, add a `Sensor` collider as a
//! child of the AeroZone: either a `ConvexHull` or `TriMesh` of the visual mesh.
//! Sensors fire `CollisionStarted`/`CollisionEnded` events without contributing
//! mass or exerting forces, so a bullet can detect which zone it struck and reduce
//! `Failure::remaining` accordingly. This hit-detection layer is **game code**. The
//! `avian_fdm` only defines `Failure` as the damage target; how damage is delivered
//! is outside the library's scope.
//!
//! ### How zone contributions compose
//!
//! Total lift = Σ C_L_zone · q̄ · S_zone (summed over all zones with remaining > 0)
//!
//! Each zone's `fraction` field controls its share of the reference area S_ref.
//! For the J3Cub wing, six panels each take 15–17.5% of total wing area; they
//! sum to 100%. The fuselage and tail zones have their own `wing_area_m2` (via
//! the root [`components::AircraftGeometry`]) so their coefficients scale correctly.
//!
//! ### Failure degradation
//!
//! When [`components::Failure::remaining`] is set to a value ∈ (0, 1):
//!
//! ```text
//! C_L_effective = C_L · remaining
//! C_D_effective = C_D · remaining + C_D_damage · (1 − remaining) / q̄
//! ```
//!
//! A failed zone produces less lift AND more drag (deformation increases
//! induced drag). At `remaining = 0`, the zone contributes zero force. It has
//! effectively separated from the airframe and produces no net aerodynamic effect.
//!
//! ### Physical detachment
//!
//! `avian_fdm` does **not** implement detachment logic. That is the game's
//! responsibility. When `remaining` reaches `0.0`, the zone silently contributes
//! zero force. The game may then choose to:
//!
//! - Remove the zone entity entirely
//! - Detach it by removing [`bevy::prelude::ChildOf`] and inserting
//!   [`avian3d::prelude::RigidBody::Dynamic`] (Avian recomputes mass/CG automatically)
//! - Replace it with a particle effect
//! - Leave it in place (zero-force zone costs very little)
//!
//! ---
//!
//! ## Reading Simulation Output
//!
//! [`components::FlightState`] is updated every physics frame on the root entity.
//! Use it in systems or for HUD display:
//!
//! ```rust,no_run
//! # use avian_fdm::prelude::*;
//! # use bevy::prelude::*;
//! fn print_flight_state(query: Query<&FlightState, With<AircraftGeometry>>) {
//!     for fs in &query {
//!         println!(
//!             "alt={:.0}m  TAS={:.1}m/s  AoA={:.2}°  Re={:.2e}",
//!             fs.altitude_m,
//!             fs.airspeed_ms,
//!             fs.alpha_rad.to_degrees(),
//!             fs.reynolds_number,
//!         );
//!     }
//! }
//! ```
//!
//! | Field | Units | Notes |
//! |-------|-------|-------|
//! | `altitude_m` | m | World Y position (positive up) |
//! | `airspeed_ms` | m/s | True airspeed (TAS) = body-frame velocity magnitude |
//! | `alpha_rad` | rad | Angle of attack; positive = nose above velocity vector |
//! | `beta_rad` | rad | Sideslip angle; positive = wind from starboard |
//! | `dynamic_pressure_pa` | Pa | q̄ = ½ρV² |
//! | `reynolds_number` |, | Re = ρVc̄/μ; drives C_L/C_D table column |
//! | `mach` |, | V / speed of sound; informational (no compressibility yet) |
//! | `p_rads`, `q_rads`, `r_rads` | rad/s | Body-frame roll/pitch/yaw rates |
//!
//! **Interpreting trim quality:**
//! - Level cruise: `alpha_rad` ≈ 2–5° (positive, small), `airspeed_ms` stable
//! - Phugoid: `airspeed_ms` and `alpha_rad` oscillate slowly (period ≈ 2πV/g);
//!   this is a natural mode and indicates the pitch model is working
//! - Divergence: `alpha_rad` grows unbounded: check elevator C_L sign
//!   convention and h-stab moment arm
//!
//! ---
//!
//! ## Data Flow
//!
//! All FDM systems run in `PhysicsStepSystems::BroadPhase`: after Avian's
//! child-collider position propagation (First), but before the constraint
//! solver. Forces are written to `ConstantForce`/`ConstantTorque` and read by
//! Avian's `ForceSystems::ApplyConstantForces` in the Solver phase.
//!
//! ```text
//! ┌── PhysicsStepSystems::BroadPhase (each physics substep) ─────────────────┐
//! │  1. update_atmosphere    reads: Position(root)                           │
//! │                          writes: AtmosphereState{ρ, p, T, a, μ}          │
//! │                                                                          │
//! │  2. update_flight_state  reads: AtmosphereState, LinearVelocity(root),   │
//! │                                 Rotation(root)                           │
//! │                          writes: FlightState{α, β, V, q̄  , Re, Mach, p,  │
//! │                                              q, r}                       │
//! │                                                                          │
//! │  3. compute_engine_zone_forces  [propulsion feature]                     │
//! │                          reads: FlightState, AtmosphereState,            │
//! │                                 ControlInputs, EngineZone                │
//! │                          writes: ZoneForce(engine), PropwashState        │
//! │                                                                          │
//! │  4. compute_aero_forces                                                  │
//! │                          reads: FlightState, AircraftGeometry,           │
//! │                                 ControlInputs, AeroZone, Failure,        │
//! │                                 GlobalTransform(zone), Children,         │
//! │                                 Position(root), Rotation(root),          │
//! │                                 ComputedCenterOfMass(root)               │
//! │                          writes: ZoneForce per child (side-effect),      │
//! │                                  ConstantForce(root),                    │
//! │                                  ConstantTorque(root)                    │
//! └──────────────────────────────────────────────────────────────────────────┘
//!         │
//!         v  PhysicsStepSystems::Solver , Avian integrates forces
//!            Position, LinearVelocity, Rotation, AngularVelocity updated
//!         │
//!         v  PostUpdate , Bevy propagates Transform to GlobalTransform
//! ```
//!
//! ### Inserting a custom system (e.g. autopilot)
//!
//! Place it between `update_flight_state` and `compute_aero_forces`:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};
//! # use avian_fdm::prelude::*;
//! # use avian_fdm::atmosphere::update_flight_state;
//! # use avian_fdm::aerodynamics::compute_aero_forces;
//! # fn my_autopilot() {}
//! // app.add_systems(
//! //     PhysicsSchedule,
//! //     my_autopilot
//! //         .after(update_flight_state)
//! //         .before(compute_aero_forces)
//! //         .in_set(PhysicsStepSystems::BroadPhase),
//! // );
//! ```
//!
//! The autopilot reads [`components::FlightState`] and writes
//! [`components::ControlInputs`]; both are available at that point in the chain.
//!
//! ---
//!
//! ## Feature Flags
//!
//! | Feature      | Default | Description |
//! |--------------|---------|-------------|
//! | `propulsion` | **on**  | Piston engine ([`components::EngineZone`]) and propwash model |
//! | `debug-plugin` | off   | Bevy Gizmo overlays for forces, moments, and zones ([`debug_render`]) |
//! | `presets`    | off     | Reference aircraft presets ([`presets`], e.g. J-3 Cub) |
//!
//! [`components::Failure`] is always available. No feature gate needed.
//! Detachment behaviour when `remaining = 0` is the game's responsibility.
//! Disable `propulsion` with `default-features = false` for gliders and
//! unpowered aircraft.
//!
//! ---
//!
//! ## Further Reading
//!
//! The following references were used in designing and validating `avian_fdm`.
//! They are listed roughly in order of accessibility.
//!
//! ### Introductory
//!
//! - **Robert C. Nelson, *Flight Stability and Automatic Control*, 2nd ed.
//!   (McGraw-Hill, 1998).**
//!   The clearest introduction to the stability-derivative approach. Table B1
//!   lists J3Cub-like GA derivatives used for the damping coefficients in
//!   [`aerodynamics`]. Chapters 2–4 cover the EoM derivation in body frame.
//!
//! - **John D. Anderson, *Introduction to Flight*, 8th ed. (McGraw-Hill, 2015).**
//!   Excellent coverage of aerodynamics fundamentals (lift, drag, Reynolds
//!   number, boundary layers) before tackling dynamics. Chapters 5 and 7 are
//!   especially relevant.
//!
//! ### Intermediate
//!
//! - **Bernard Etkin & Lloyd D. Reid, *Dynamics of Flight: Stability and
//!   Control*, 3rd ed. (Wiley, 1996).**
//!   Rigorous derivation of the 6-DoF Newton-Euler equations in body frame.
//!   The stability axis transformation (body to stability to wind axes) used in
//!   [`aerodynamics`] follows Etkin & Reid Chapter 4.
//!
//! - **Brian L. Stevens, Frank L. Lewis & Eric N. Johnson,
//!   *Aircraft Simulation and Control*, 3rd ed. (Wiley, 2016).**
//!   The standard reference for simulation architecture. The coordinate frame
//!   conventions in this library match Stevens & Lewis Appendix A. Also covers
//!   numerical integration and digital flight control systems.
//!
//! ### Atmospheric model
//!
//! - **ICAO Doc 7488/3, *Manual of the ICAO Standard Atmosphere*, 3rd ed.
//!   (International Civil Aviation Organization, 1993).**
//!   The authoritative definition of the International Standard Atmosphere
//!   implemented in [`atmosphere`]. Free download from the ICAO store.
//!
//! ### Open-source simulators
//!
//! - **[JSBSim](https://github.com/JSBSim-Team/jsbsim)**. Mature open-source
//!   FDM used by FlightGear and the US military. The J3Cub aerodynamic tables
//!   in [`presets::j3cub`] are derived from JSBSim's `J3Cub.xml` and cross-
//!   checked against USA-35B airfoil data. JSBSim's documentation explains the
//!   stability-derivative XML format in detail.
//!
//! - **[FlightGear](https://www.flightgear.org/)**. Open-source flight
//!   simulator using JSBSim as its default FDM. Useful for cross-validating
//!   flight behaviour and visualising aerodynamic data.
//!
//! - **[OpenPilot / ArduPilot](https://ardupilot.org/)**. Real autopilot
//!   firmware with extensive FDM documentation. Relevant if adding a stability-
//!   augmentation system on top of `avian_fdm`.
//!
//! [Avian]: https://crates.io/crates/avian3d
//! [JSBSim]: https://jsbsim.sourceforge.net/

#![deny(missing_docs)]

/// Annotates a value with its data provenance.
///
/// The source description is a string literal passed as the second argument.
/// It is **discarded by the macro**. Only the value expression is emitted.
/// The string never becomes a `&str` static and adds nothing to the binary.
///
/// Use this in preset files to record where each number came from, so future
/// maintainers can trace any value back to its origin without digging through
/// commit history or external references.
///
/// # Source prefix conventions
///
/// | Prefix | Meaning |
/// |---|---|
/// | `"JSBSim:J3Cub.xml"` | Directly transcribed from a JSBSim aircraft XML |
/// | `"Calibration:JSBSim"` | Tuned to match JSBSim behaviour experimentally |
/// | `"Literature:..."` | Derived from a published paper or textbook |
/// | `"Geometry"` | Computed analytically from aircraft dimensions |
/// | `"Estimate"` | Engineering judgement; no primary source |
/// | `"Guesswork"` | Placeholder; should be replaced with measured data |
///
/// # Example
///
/// ```rust
/// # use avian_fdm::sourced;
/// // Zero runtime cost. Expands to exactly `0.94f64`:
/// let e: f64 = sourced!(0.94, "JSBSim:J3Cub.xml: CD_i = CL²×0.0485, e≈0.94");
/// assert_eq!(e, 0.94);
/// ```
#[macro_export]
macro_rules! sourced {
    ($value:expr, $source:literal) => {
        $value
    };
}

/// Internal re-exports of bevy sub-crates.
/// All crate-internal modules import from here instead of the `bevy` meta-crate.
pub(crate) mod _bevy {
    pub(crate) use bevy_app::prelude::*;
    pub(crate) use bevy_ecs::prelude::*;
    pub(crate) use bevy_math::prelude::*;
    pub(crate) use bevy_transform::prelude::*;
    pub(crate) use bevy_reflect::prelude::*;
    pub(crate) use bevy_log::{warn, warn_once};
    #[cfg(feature = "debug-plugin")]
    pub(crate) use bevy_color::prelude::*;
    #[cfg(feature = "debug-plugin")]
    pub(crate) use bevy_gizmos::prelude::*;
}

pub mod aerodynamics;
pub mod atmosphere;
pub mod components;
pub mod math;
pub mod plugin;
pub mod systems;

#[cfg(feature = "propulsion")]
pub mod propulsion;

#[cfg(feature = "debug-plugin")]
pub mod debug_render;

#[cfg(feature = "presets")]
pub mod presets;

/// Re-exports for convenient glob import: `use avian_fdm::prelude::*;`
pub mod prelude {
    pub use crate::components::{
        aero_coeff::AeroCoeff, AeroZone, AeroZoneBundle, AircraftCoreBundle, AircraftGeometry,
        AtmosphereState, ControlInputs, ControlSurfaceRole, Failure, FlightState, GizmoContours,
        GizmoShape, ZoneForce,
    };
    pub use crate::plugin::AircraftFdmPlugin;
    pub use crate::sourced;
    pub use crate::systems::AircraftFdmSystem;

    #[cfg(feature = "debug-plugin")]
    pub use crate::debug_render::{AircraftFdmDebugPlugin, FdmDebugRender, FdmGizmos};

    #[cfg(feature = "propulsion")]
    pub use crate::components::{EngineZone, PropwashState};
}
