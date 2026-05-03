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
//! each child collider.
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
//! 8. [Emergent Behavior](#emergent-behavior)
//! 9. [Zone Decomposition and Damage](#zone-decomposition-and-damage)
//!    - [Collider strategy](#collider-strategy)
//! 10. [Reading Simulation Output](#reading-simulation-output)
//! 11. [Data Flow](#data-flow)
//! 12. [Feature Flags](#feature-flags)
//! 13. [Further Reading](#further-reading)
//!
//! ---
//!
//! ## Quick Start
//!
//! Add `avian_fdm` and `avian3d` to `Cargo.toml`:
//!
//! ```toml
//! avian_fdm          = "0.1"
//! avian_fdm_j3cub_jsbsim = { version = "0.1" }
//! avian3d            = { version = "0.6" }
//! bevy               = { version = "0.18" }
//! ```
//!
//! Spawn the reference J-3 Cub aircraft from `avian_fdm_j3cub_jsbsim`:
//!
//! ```rust,ignore
//! use avian_fdm::prelude::*;
//! use avian_fdm_j3cub_jsbsim::presets::j3cub;
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
//!     // Override the default zero velocity to start at cruise airspeed.
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
//! A **Flight Dynamics Model** (FDM) computes all forces and moments acting on
//! an aircraft at each instant, given its state (position, velocity, attitude,
//! angular rate) and the pilot's control inputs. The output feeds a rigid-body
//! integrator that advances the state forward in time. `avian_fdm` is the FDM;
//! [Avian] is the integrator.
//!
//! ### Scope of this library for now
//!
//! **In scope:**
//! - ISA atmosphere (0–20 km)
//! - Lift, drag, and side-force per zone
//! - Pitch, roll, and yaw damping derivatives
//! - Piston engine + fixed-pitch propeller model
//! - Failure degradation (performance loss with damage)
//! - 6-DoF integration (via Avian)
//!
//! **Out of scope** (future, or game's responsibility):
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
//! (sometimes called the *linear aerodynamic model*).
//!
//! Aerodynamic coefficients (C_L, C_D, C_Y) are dimensionless numbers that
//! describe how much lift, drag, or side-force an airfoil produces. They are
//! stored as lookup tables indexed by angle of attack and Reynolds number Re,
//! then multiplied by dynamic pressure q̄  (q-bar) and wing area S to obtain forces.
//!
//! ```text
//! Lift  = C_L(alpha, Re) * q-bar * S
//! Drag  = C_D(alpha, Re) * q-bar * S
//! ```
//!
//! This is the same method used by [JSBSim], FlightGear, and most
//! fixed-wing game simulators. It captures realistic stall behaviour,
//! Reynolds-number effects, and control-surface authority without requiring a
//! full CFD solver.
//!
//! ---
//!
//! ## Coordinate Frames
//!
//! Two frames appear throughout the codebase.
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
//! Zone transforms in the presets are authored in this frame. Example:
//! a wing zone at `Transform::from_xyz(-0.10, -2.82, -0.58)` is 0.10 m aft of
//! the CG datum, 2.82 m to port (-Y), and 0.58 m above datum (-Z = up).
//! Zone transforms in the presets are authored in this frame.
//!
//! ### Stability frame
//!
//! The stability frame is the body frame rotated by -alpha about body Y, aligning
//! its X axis with the velocity vector. Lift is defined as perpendicular to the
//! velocity (-Z_stability), drag as opposing it (-X_stability).
//!
//! See the `aerodynamics` module for the full derivation.
//!
//! ### World frame (Bevy / Avian, Y-up right-handed)
//!
//! Bevy uses a Y-up, right-handed world frame. The aircraft must be spawned
//! with a rotation that aligns body X (forward) to world X and body Z (down) to
//! world -Y. `Quat::from_rotation_x(FRAC_PI_2)` achieves this.
//!
//! ### Result
//!
//! **Force in stability axes, then rotated to body frame, then to world frame:**
//!
//! ```text
//! force_stability = (-C_D · q̄ · S,  C_Y · q̄ · S,  -C_L · q̄ · S)
//! force_body      = R_y(-alpha) · force_stability
//! force_world     = q_root  · force_body
//! ```
//!
//! All internal computation uses avian3d's native precision (`Scalar`),
//! which is `f32` or `f64` depending on the active feature flag.
//!
//! ---
//!
//! ## The Equations of Motion
//!
//! An aircraft in free flight has **6 degrees of freedom** (6-DoF): three
//! translational (x, y, z) and three rotational (roll φ, pitch θ, yaw ψ).
//!
//! ### Translational dynamics
//!
//! **Net force = mass * acceleration** (Newton's second law):
//!
//! ```text
//! F_net = m * (dV/dt)
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
//! equations of motion are **Rolling, pitching, and yawing moments equal
//! inertia times angular acceleration plus gyroscopic cross-coupling terms**:
//!
//! ```text
//! L = I_xx · ṗ  + (I_zz − I_yy) · q · r   (roll  equation)
//! M = I_yy · q̇  + (I_xx − I_zz) · p · r   (pitch equation)
//! N = I_zz · ṙ  + (I_yy − I_xx) · p · q   (yaw   equation)
//! ```
//!
//! where (p, q, r) are body-frame roll/pitch/yaw rates, and (L, M, N) are the
//! roll/pitch/yaw moments. Avian evaluates this system internally; `avian_fdm`
//! supplies (L, M, N) via [`avian3d::prelude::ConstantTorque`] by computing the
//! cross product of each zone force with its moment arm.
//! **Torque = lever arm x force (cross product):**
//!
//! ```text
//! tau_zone = (r_zone - r_CG) x F_zone
//! ```
//!
//! where **r_zone** is the zone's world position and **r_CG** is the
//! world-space centre of mass (read from
//! [`avian3d::prelude::ComputedCenterOfMass`]).
//! Avian handles quaternion renormalisation, sub-stepping, and
//! translation-rotation coupling internally.
//!
//! ---
//!
//! ## The Atmosphere
//!
//! See the `atmosphere` module for the full implementation.
//!
//! Every aerodynamic force scales with dynamic pressure, which is the
//! kinetic energy of the airflow per unit volume.
//! **Dynamic pressure = half * air density * airspeed squared**:
//!
//! ```text
//! q-bar = 0.5 * rho * V^2
//! ```
//!
//! At sea level, rho = 1.225 kg/m^3. At 2500 m it drops about 20%, directly
//! cutting lift and drag by 20% at the same airspeed. An aircraft must fly
//! faster at altitude to generate the same lift.
//!
//! Dynamic pressure also controls Reynolds number.
//! **Reynolds number = (density × speed × chord) ÷ viscosity**, a dimensionless ratio
//! of inertial to viscous forces that determines whether airflow is smooth or turbulent:
//!
//! ```text
//! Re = ρ · V · c̄  / μ
//! ```
//!
//! where c̄  is mean aerodynamic chord and μ is dynamic viscosity. Reynolds
//! number governs boundary-layer behaviour: at low Re the flow separates
//! earlier (sharper stall, higher C_D0), so the FDM uses Re as the second
//! dimension of its C_L/C_D lookup tables.
//! The `atmosphere` module implements the International Standard Atmosphere
//! (ICAO Doc 7488) for 0-20 km. Each frame,
//! `atmosphere::update_atmosphere` writes a fresh
//! [`components::AtmosphereState`] to the root entity.
//!
//! ---
//!
//! ## Aerodynamic Forces and Moments
//!
//! See the `aerodynamics` module for the full implementation.
//!
//! The aircraft is decomposed into zones. For each [`components::AeroZone`]
//! child, the system evaluates coefficient tables at the current flight
//! state, multiplies by dynamic pressure and zone area, and writes the
//! resulting force and torque. Coefficients support three representations:
//! constant (`Scalar`), 1-D table over angle of attack (`Table1D`), and
//! 2-D table over angle of attack and Reynolds number (`Table2D`).
//!
//! ### Dynamic damping: emergent vs explicit
//!
//! For full-zone aircraft (default), damping emerges from per-zone
//! local-angle corrections. Each zone gets its own effective angle of attack
//! based on its position and the body's angular rates:
//!
//! ```text
//! alpha_local = alpha + (p*y - q*x) / V (roll and pitch rate corrections)
//! beta_local  = beta  + (r*x)       / V (yaw rate correction)
//! ```
//!
//! The tail automatically produces restoring forces during pitch, the wings
//! produce differential lift during roll, and the fin produces side force
//! during yaw. No explicit damping coefficient is needed.
//!
//! For sparse models (missiles, simple drones), attach
//! [`components::LodDamping`] to the root for explicit damping derivatives.
//!
//! ### Control surfaces
//!
//! Each zone can be tagged with a [`components::ControlSurfaceRole`]. The
//! moment arm from zone position to CG generates roll/pitch/yaw moments
//! automatically. Deflection also increases drag slightly.
//!
//! ## Propulsion Coupling
//!
//! Thrust follows the Gagg-Ferrar altitude correction. Maximum thrust is
//! scaled by throttle position and air density ratio:
//!
//! ```text
//! T = T_max * throttle_fraction * (rho / rho_0)^0.7
//! ```
//!
//! Each [`components::EngineZone`] specifies a thrust direction in body frame.
//!
//! ---
//!
//! ## Emergent Behavior
//!
//! The zone-based architecture produces physically correct behaviors without
//! explicit global coefficients. They arise because forces are computed per
//! zone at each zone's position, the moment arm to the CG is computed each
//! step, and Avian recomputes mass/CG/inertia from surviving colliders.
//!
//! ### Stall and high angle-of-attack behavior
//!
//! **Stall:**
//! When alpha exceeds the CL table's stall angle, CL drops. If the tables
//! cover the post-stall regime, the wing zones produce less lift, the aircraft
//! pitches down or departs, and recovery follows normal stall physics.
//!
//! **Wing drop at stall:**
//! Under any roll rate at near-stall alpha, one wing tip sees higher local
//! alpha than the other (alpha_local += p*y/V). The higher-alpha tip stalls
//! first while the other keeps flying, driving an uncommanded roll. This is the
//! physical cause of wing drop.
//!
//! **Snap roll:**
//! An abrupt aileron input at near-stall alpha instantly pushes one wing zone
//! over the stall angle via the local-angle correction. That zone loses lift,
//! the other side keeps flying, and the aircraft rolls rapidly and
//! uncontrollably.
//!
//! **High-AoA damping reversal:**
//! Beyond the stall angle, the CL table slope becomes negative (dCL/dalpha < 0).
//! The per-zone local-angle correction now produces a force increment that
//! aids the angular rate rather than opposing it - damping reverses sign. This
//! is what drives spin autorotation. No separate damping-reversal model needed.
//!
//! **Deep stall:**
//! If wing zones stall but the tail is still flying (tail has a lower alpha
//! due to its longitudinal position and the pitch-rate correction), the tail
//! keeps generating lift. Because the tail is aft, this creates a nose-up
//! moment that sustains the high alpha and prevents recovery. Emerges directly
//! from the tables if they include the deep-stall regime.
//!
//! **Spin dynamics:**
//! In a spin, the descending wing has higher local alpha (from the roll rate)
//! and is stalled, while the rising wing has lower alpha and is still producing
//! lift. The differential lift drives autorotation. The yaw rate correction
//! adds a beta shift across the span. Together they produce a steady spin
//! state with correct rotation rates and altitude loss without any spin model.
//!
//! ### Cross-coupling
//!
//! **Adverse yaw (Cnp - yaw from roll rate):**
//! Under roll rate p, the descending wing produces more lift and more induced
//! drag than the rising wing. The induced-drag asymmetry yaws the nose toward
//! the descending wing. This emerges from per-zone drag tables evaluated at
//! different local angles. The angle correction drives part of it; adding
//! per-zone qbar scaling (see below) would complete it.
//!
//! **Proverse yaw from yaw damping (Clr - roll from yaw rate):**
//! Under yaw rate r, the advancing wing moves faster and produces more lift.
//! The differential lift rolls the aircraft toward the advancing wing. The
//! angle correction captures this partially; the full effect requires per-zone
//! qbar scaling.
//!
//! **Dutch roll:**
//! Yaw and roll couple through Clr and Cnp. The fin's yaw restoring moment
//! combined with the wing's differential lift produces the oscillatory yaw-roll
//! coupling that characterizes Dutch roll. The character of the mode (damped,
//! neutral, divergent) depends on the fin volume and wing dihedral.
//!
//! **Pitch-roll-yaw departure at high AoA:**
//! When wing zones begin to stall unevenly in a combined maneuver, all three
//! axes couple simultaneously. The asymmetric stall across the span drives
//! roll; the drag asymmetry drives yaw; the tail's reduced effectiveness drives
//! pitch up. These reinforce each other without any special departure model.
//!
//! ### Control authority
//!
//! **Roll authority (Cl_da):**
//! Ailerons are at large spanwise offsets. Their lift (scaled by aileron input)
//! x the moment arm to CG produces roll torque. Cl_da is never specified.
//!
//! **Pitch authority (Cm_de):**
//! The elevator is aft. Its lift x tail arm produces pitch torque.
//!
//! **Yaw authority (Cn_dr):**
//! The rudder produces side force at the fin. The moment arm to CG gives yaw.
//!
//! **Thrust pitching, rolling, and yawing moments:**
//! An engine zone offset from the aircraft centerline (y, z, or x from CG)
//! creates pitching and yawing moments automatically. A tilted thrust axis
//! (pusher, tilt-rotor) produces the correct coupled moments.
//!
//! **Propulsion torque reaction:**
//! A rotating propeller or engine imparts angular momentum to the airframe in
//! the opposite direction. We will model this in the future by adding a pure
//! torque on the engine zone in the axis oppositek to prop rotation. The moment
//! propagates to the root via Avian's constraint solver.
//!
//! ### Aerodynamic configuration effects
//!
//! **Flaps and slats:**
//! Add a flap zone at the wing trailing edge (or use a control surface role).
//! Give it a CL table that represents the cambered section at flap deflection,
//! scaled by the flap's share of wing area. Deploying flaps (setting the zone's
//! deflection input to 1) applies the cambered CL and the higher CD. No
//! dedicated flap model. Slats work the same way on the leading edge.
//!
//! **External stores and fuel tanks:**
//! Add an AeroZone with CL = 0 and a CD representing the store's drag area.
//! Place it at the store's position. The drag force applied at that position
//! creates a moment arm to the CG, affecting trim. Dropping stores
//! (Failure::remaining = 0) removes both their mass and their drag instantly.
//!
//! ### Flight dynamics modes
//!
//! **Phugoid oscillation:**
//! The aircraft pitches up, lift increases, it climbs and slows. Reduced speed
//! reduces lift, it pitches down and descends, accelerates again. This
//! long-period oscillation emerges from pitch stability and the lift/speed
//! relationship. No phugoid frequency coefficient exists anywhere.
//!
//! **Short-period mode:**
//! Rapid pitch oscillation emerges from pitch stiffness (tail volume) and pitch
//! damping (Cm_q). Frequency and damping ratio are set by geometry.
//!
//! **Spiral mode:**
//! When banked, the aircraft yaws toward the low wing (from sideslip). If yaw
//! stability is weaker than roll stability, the bank increases and the aircraft
//! spirals. This mode and its stability (damped vs divergent) emerge from the
//! balance between fin yaw stiffness and wing dihedral effect.
//!
//! ### Damage effects
//!
//! **CG shift:**
//! When a zone's Failure::remaining reaches zero, Avian removes its mass from
//! the compound collider. The CG shifts toward the surviving structure. All
//! moment arms recompute. Losing a wingtip shifts the CG toward the root.
//!
//! **Inertia tensor change:**
//! Avian recomputes the full tensor from surviving colliders. Losing a wing
//! panel reduces roll inertia. The aircraft becomes more responsive in roll.
//!
//! **Asymmetric roll from one-sided damage:**
//! The surviving wing's lift has no counterpart on the other side. The
//! resulting moment arm x lift drives a continuous roll. No "damage roll rate"
//! parameter exists.
//!
//! **Loss of control authority:**
//! Destroying a control surface zone (aileron, elevator, rudder) removes its
//! force contribution exactly. Partial damage (Failure::remaining = 0.5)
//! halves the zone's forces, giving half authority. The other surfaces are
//! unaffected.
//!
//! **Reduced damping from structural damage:**
//! Losing fin area reduces Cn_r. Losing wing area reduces Cl_p. The aircraft
//! oscillates more after perturbations. All follow directly from zone survival.
//!
//! **Changed departure characteristics after damage:**
//! After losing a wing panel, the asymmetric roll under roll rate is larger
//! (less opposing lift on the damaged side), making wing-drop at stall more
//! aggressive on that side.
//!
//! ### What does NOT yet emerge (gaps and planned work)
//!
//! The following require either additional per-zone data or structural changes
//! to the force computation pipeline:
//!
//! - **Clr and Cnp fully:** The angle-only correction captures part of these.
//!   The dominant term is the differential dynamic pressure (V +/- r*y)^2
//!   per zone. Planned: extend `zone_local_angles` to also scale each zone's
//!   local q-bar. With that change, Clr and Cnp would be fully emergent and
//!   automatically damage-aware.
//!
//! - **Cm_alphadot:** Requires tracking the lag between wing downwash and tail
//!   response over time. Cannot emerge from instantaneous angle corrections.
//!   Planned: scale a global coefficient by h-stab Failure::remaining.
//!
//! - **Wake turbulence and vortex interaction:** When two aircraft fly close,
//!   the trailing wingtip vortices from one aircraft add induced velocity to
//!   the other's wing zones. Planned as a separate VortexWake component per
//!   aircraft that injects a velocity field into the zone local-angle
//!   computation.
//!
//! - **Propeller slipstream and P-factor:** The asymmetric disk loading at
//!   high alpha and the accelerated slipstream over the inner wing both require
//!   modeling the propeller wake as a distributed velocity field, not a point
//!   force.
//!
//! - **Ground effect:** Increased lift and reduced induced drag near the ground
//!   requires a height-dependent correction to the local qbar or induced angle.
//!
//! - **Wing-fuselage interference:** At the wing root junction, the fuselage
//!   boundary layer and the wing's leading-edge pressure gradient form a
//!   horseshoe vortex that induces a local downwash on the inboard panels,
//!   reducing their effective angle of attack. Inboard panels produce less lift
//!   than the same section would in free air. Currently all panels use the same
//!   CL table. To model this, give root panels a slightly degraded CL table
//!   derived from wind tunnel or CFD data, or apply a scalar interference factor
//!   to the root panel's lift.
//!
//! - **Ice accretion and contamination:** These change the CL/CD tables of
//!   the affected zones. Planned as a table-modifier applied to zone
//!   coefficients at runtime.
//!
//! ---
//!
//! ## Zone Decomposition and Damage
//!
//! Each structural part of the aircraft is a separate ECS entity
//! (an [`components::AeroZone`]) child of the root rigid body.
//! Each zone owns:
//! - A [`avian3d::prelude::Collider`] that gives it physical volume and mass
//!   (via [`avian3d::prelude::ColliderDensity`])
//! - A `Transform` that places it in body-frame coordinates
//! - An [`components::AeroZone`] that describes its aerodynamic contribution
//! - Optionally a [`components::Failure`] for degraded-performance tracking
//!
//! Avian **automatically** computes total mass, CG, and inertia tensor from
//! all child colliders. No explicit bookkeeping required.
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
//! ### Failure degradation
//!
//! When [`components::Failure::remaining`] is set to a value in (0, 1):
//!
//! ```text
//!
//! C_L_effective = C_L * remaining
//! C_D_effective = C_D * remaining + C_D_damage * (1 - remaining) / q-bar
//! ```
//!
//! A failed [`components::AeroZone`] produces less lift AND more drag (deformation
//! increases induced drag). At `remaining = 0`, the zone contributes zero force. It has
//! effectively separated from the airframe and produces no net aerodynamic effect.
//! Physical detachment (removing the entity or re-parenting it) is the game's
//! responsibility.
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
//!             "alt={:.0}m  TAS={:.1}m/s  AoA={:.2}deg",
//!             fs.altitude_m,
//!             fs.airspeed_ms,
//!             fs.alpha_rad.to_degrees(),
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
//! | `dynamic_pressure_pa` | Pa | q-bar = 0.5 * rho * V^2 |
//! | `mach` | - | V / speed of sound; informational (no compressibility yet) |
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
//! ```text
//! +-- PhysicsStepSystems::BroadPhase (each physics substep) ----------------+
//! |  1. update_atmosphere    reads: Position(root)                          |
//! |                          writes: AtmosphereState                        |
//! |                                                                         |
//! |  2. update_flight_state  reads: AtmosphereState, LinearVelocity,        |
//! |                                 Rotation(root)                          |
//! |                          writes: FlightState                            |
//! |                                                                         |
//! |  3. compute_engine_zone_forces  reads: FlightState, AtmosphereState,    |
//! |                                 ControlInputs, EngineZone               |
//! |                          writes: ZoneForce(engine)                      |
//! |                                                                         |
//! |  4. compute_aero_forces                                                 |
//! |                          reads: FlightState, AircraftGeometry,          |
//! |                                 ControlInputs, AeroZone, Failure,       |
//! |                                 Transform(zone), Children               |
//! |                          writes: ZoneForce, ConstantForce(root),        |
//! |                                  ConstantTorque(root)                   |
//! +-------------------------------------------------------------------------+
//!         |
//!         v  Solver: Avian integrates forces
//!         |
//!         v  PostUpdate: Bevy propagates GlobalTransform from Position/Rotation
//! ```
//!
//! ### Why zone positions use Transform, not GlobalTransform
//!
//! Avian writes the root entity's physics state into `Position` and `Rotation`.
//! Bevy propagates those into `GlobalTransform` only in `PostUpdate`, which runs
//! after the physics schedule. Reading `GlobalTransform` on zone children during
//! `BroadPhase` would give values from the *previous* frame.
//!
//! Instead, `compute_aero_forces` reads `Position`/`Rotation` on the root and
//! local `Transform` on each zone (which is authored at spawn time and never
//! changes at runtime). Zone world position is reconstructed manually:
//! `world_pos = root_position + root_rotation * zone_transform.translation`.
//! This is always one physics step ahead of `GlobalTransform`.
//!
//! ### Inserting a custom system (e.g. autopilot)
//!
//!
//! The autopilot reads [`components::FlightState`] and writes
//! [`components::ControlInputs`]. Place it after `AircraftFdmSystem::FlightState`
//! and before `AircraftFdmSystem::Forces`:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};
//! # use avian_fdm::prelude::*;
//! # fn my_autopilot() {}
//! // app.add_systems(
//! //     PhysicsSchedule,
//! //     my_autopilot
//! //         .after(AircraftFdmSystem::FlightState)
//! //         .before(AircraftFdmSystem::Forces)
//! //         .in_set(PhysicsStepSystems::BroadPhase),
//! // );
//! ```
//!
//! ---
//!
//! ## Feature Flags
//!
//! | Feature        | Default | Description |
//! |----------------|---------|-------------|
//! | `f32`          | yes     | Enable avian3d f32 backend and collider shapes. |
//! | `f64`          | --      | Enable avian3d f64 backend (mutually exclusive with f32). |
//! | `debug-plugin` | yes     | Bevy Gizmo overlays for forces, moments, and zones ([`debug_render`]) |
//!
//! JSBSim-derived reference aircraft (J-3 Cub) are in the separate
//! `avian_fdm_j3cub_jsbsim` crate (GPL-3.0-only).
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
//!   the `aerodynamics` module. Chapters 2–4 cover the EoM derivation in body frame.
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
//!   the `aerodynamics` module follows Etkin & Reid Chapter 4.
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
//!   implemented in the `atmosphere` module. Free download from the ICAO store.
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
    #[cfg(feature = "debug-plugin")]
    pub(crate) use bevy_color::prelude::*;
    pub(crate) use bevy_ecs::prelude::*;
    #[cfg(feature = "debug-plugin")]
    pub(crate) use bevy_gizmos::prelude::*;
    pub(crate) use bevy_log::{warn, warn_once};
    pub(crate) use bevy_math::prelude::*;
    pub(crate) use bevy_reflect::prelude::*;
    pub(crate) use bevy_transform::prelude::*;
}

pub(crate) mod aerodynamics;
pub(crate) mod atmosphere;
pub mod airfoil;
pub mod components;
pub(crate) mod math;
pub mod plugin;
pub(crate) mod propulsion;
pub mod systems;

#[cfg(feature = "debug-plugin")]
pub mod debug_render;

/// Re-exports for convenient glob import: `use avian_fdm::prelude::*;`
pub mod prelude {
    pub use crate::airfoil::{AirfoilData, AirfoilLibrary, RegisterAirfoil};
    pub use crate::components::{
        aero_coeff::AeroCoeff, AeroZone, AeroZoneBundle, AircraftCoreBundle, AircraftGeometry,
        AtmosphereState, ControlInputs, ControlSurfaceRole, Failure, FlightState, GizmoContours,
        GizmoShape,
    };
    pub use crate::plugin::AircraftFdmPlugin;
    pub use crate::sourced;
    pub use crate::systems::AircraftFdmSystem;

    #[cfg(feature = "debug-plugin")]
    pub use crate::debug_render::{
        AircraftFdmDebugPlugin, FdmDebugRender, FdmGizmos, ShowColliders,
    };

    pub use crate::components::EngineZone;
}
