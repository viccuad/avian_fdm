//! # avian_fdm: a 6-DoF flight dynamics model for Bevy and Avian
//!
//! `avian_fdm` is a Bevy plugin. It turns an Avian rigid-body hierarchy into
//! an aircraft that flies in a physically plausible way.
//!
//! In each physics step, the plugin calculates the aerodynamic and propulsive
//! forces on each [`components::AeroZone`] child entity. It adds these forces
//! into the [`avian3d::prelude::ConstantForce`] and
//! [`avian3d::prelude::ConstantTorque`] of the root entity. Then the Avian
//! integrator calculates the new position, velocity, orientation, and angular
//! velocity. `avian_fdm` does not write these values directly.
//!
//! Avian calculates the mass, the center of gravity (CG), and the full inertia
//! tensor. It uses the shape and the [`avian3d::prelude::ColliderDensity`] of
//! each child collider.
//!
//! This documentation has two purposes. It tells you how to use the crate. It
//! also explains the physics of a flight dynamics model (FDM), step by step.
//!
//! ---
//!
//! ## Table of Contents
//!
//! 1. [Quick Start](#quick-start)
//! 2. [What is a Flight Dynamics Model?](#what-is-a-flight-dynamics-model)
//! 3. [Terms and Symbols](#terms-and-symbols)
//! 4. [Coordinate Frames](#coordinate-frames)
//! 5. [The Equations of Motion](#the-equations-of-motion)
//! 6. [The Atmosphere](#the-atmosphere)
//! 7. [Aerodynamic Forces and Moments](#aerodynamic-forces-and-moments)
//! 8. [Propulsion Coupling](#propulsion-coupling)
//! 9. [Emergent Behavior](#emergent-behavior)
//! 10. [Zone Decomposition and Damage](#zone-decomposition-and-damage)
//!     - [Collider strategy](#collider-strategy)
//! 11. [Reading Simulation Output](#reading-simulation-output)
//! 12. [Data Flow](#data-flow)
//! 13. [Feature Flags](#feature-flags)
//! 14. [Further Reading](#further-reading)
//!
//! ---
//!
//! ## Quick Start
//!
//! Add these dependencies to `Cargo.toml`:
//!
//! ```toml
//! avian_fdm              = "0.2"
//! avian_fdm_j3cub_jsbsim = { version = "0.2" }
//! avian3d                = { version = "0.7" }
//! bevy                   = { version = "0.19" }
//! ```
//!
//! Spawn the reference J-3 Cub aircraft from `avian_fdm_j3cub_jsbsim`:
//!
//! ```rust,ignore
//! use std::f32::consts::FRAC_PI_2;
//!
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
//!     // Turn body Z (down) to world -Y. Refer to "World frame" below.
//!     let level = Quat::from_rotation_x(FRAC_PI_2);
//!     let root = j3cub::spawn(
//!         &mut commands,
//!         Transform::from_xyz(0.0, 300.0, 0.0).with_rotation(level),
//!     );
//!     // The default velocity is zero. Start at cruise airspeed.
//!     commands.entity(root).insert(LinearVelocity(Vec3::new(27.0, 0.0, 0.0)));
//! }
//! ```
//!
//! To build your own aircraft, add [`components::AeroZone`] children to a
//! root entity that has an [`components::AircraftCoreBundle`]. Refer to
//! [Zone Decomposition and Damage](#zone-decomposition-and-damage).
//!
//! ---
//!
//! ## What is a Flight Dynamics Model?
//!
//! A **flight dynamics model** (FDM) calculates all forces and moments on an
//! aircraft at one instant. It has two inputs: the aircraft state and the
//! control inputs of the pilot. The state is the position, velocity,
//! attitude, and angular rate. A rigid-body integrator uses the forces and
//! moments to calculate the state at the next instant. `avian_fdm` is the
//! FDM. [Avian] is the integrator.
//!
//! ### Scope of this crate at this time
//!
//! **In scope:**
//! - ISA atmosphere (0–20 km)
//! - Lift, drag, and side force for each zone
//! - Pitch, roll, and yaw damping derivatives
//! - Piston engine and fixed-pitch propeller model
//! - Performance loss from damage
//! - 6-DoF integration (by Avian)
//!
//! **Out of scope** (planned work, or the responsibility of the game):
//! - Ground contact and landing gear forces
//! - Structural elasticity and aeroelasticity
//! - Compressibility effects (transonic and supersonic flight)
//! - Turbine and jet engine cycles
//! - Fuel burn and the change of weight over time
//! - Autopilot and stability augmentation
//! - Physical detachment of damaged zones
//!
//! ### Stability-derivative approach
//!
//! `avian_fdm` uses the **small-perturbation stability-derivative method**.
//! Another name for this method is the *linear aerodynamic model*.
//!
//! Aerodynamic coefficients (CL, CD, CY) are dimensionless numbers. They tell
//! how much lift, drag, or yaw side force a surface makes. The crate keeps them in
//! tables. The table inputs are the angle of attack (alpha) and the Reynolds
//! number (Re). To get a force, the crate multiplies the coefficient by the
//! dynamic pressure (qbar) and the reference area (S):
//!
//! ```text
//! Lift = CL(alpha, Re) * qbar * S
//! Drag = CD(alpha, Re) * qbar * S
//! ```
//!
//! [JSBSim], FlightGear, and most fixed-wing game simulators use the same
//! method. The method gives realistic stall behavior, Reynolds-number effects,
//! and control-surface authority. It does not need a full computational fluid
//! dynamics (CFD) solver.
//!
//! ---
//!
//! ## Terms and Symbols
//!
//! The documentation uses these symbols in formulas:
//!
//! | Symbol | Meaning | Unit |
//! |---|---|---|
//! | `alpha` | Angle of attack: the angle between body X and the velocity, in the body X-Z plane | rad |
//! | `beta` | Sideslip angle: the angle between the velocity and the body X-Z plane | rad |
//! | `V` | True airspeed | m/s |
//! | `rho` | Air density | kg/m³ |
//! | `mu` | Dynamic viscosity of the air | kg/(m·s) |
//! | `qbar` | Dynamic pressure, `0.5 * rho * V^2` | Pa |
//! | `Re` | Reynolds number | — |
//! | `S` | Reference area (wing area, or the area of one zone) | m² |
//! | `b` | Wing span | m |
//! | `c_bar` | Mean aerodynamic chord | m |
//! | `AR` | Aspect ratio, `b^2 / S` | — |
//! | `p`, `q`, `r` | Roll, pitch, and yaw rate in the body frame | rad/s |
//! | `L`, `M`, `N` | Rolling, pitching, and yawing moment | N·m |
//! | `CL`, `CD`, `CY` | Lift, drag, and side-force coefficient | — |
//! | `CM`, `Croll`, `Cn` | Pitching, rolling, and yawing-moment coefficient | — |
//!
//! Textbooks write the rolling-moment coefficient as `Cl`. This crate writes
//! `Croll` (field `croll`) to keep it apart from the lift coefficient `CL`.
//! Derivative names keep the textbook form. For example, `Cl_p` is the change
//! of the rolling-moment coefficient with the roll rate.
//!
//! ---
//!
//! ## Coordinate Frames
//!
//! Four frames appear in the code: the body frame, the zone frame, the
//! stability frame, and the world frame.
//!
//! ### Body frame (SAE aerospace convention)
//!
//! ```text
//!         X (forward / nose)
//!        ╱
//!       ╱
//!      ╱──── Y (right wing / starboard)
//!      │
//!      │  Z (down / belly)
//! ```
//!
//! | Axis | Points toward | Positive rotation (right-hand rule) |
//! |------|---------------|--------------------------------------|
//! | X    | Nose          | Roll right (starboard wing down)     |
//! | Y    | Right wing    | Pitch nose up                        |
//! | Z    | Belly / down  | Yaw nose right                       |
//!
//! This frame agrees with the SAE J670 standard. JSBSim and most aerospace
//! textbooks use it (Stevens & Lewis, Etkin & Reid, Nelson). The presets set
//! the zone transforms in this frame. For example, a wing zone at
//! `Transform::from_xyz(-0.10, -2.82, -0.58)` is 0.10 m aft of the CG datum.
//! It is also 2.82 m to port (-Y) and 0.58 m above the datum (-Z is up).
//!
//! ### Zone frame
//!
//! Each zone can have its own rotation in its `Transform`. This rotation sets
//! the zone frame. For example, a wing panel with dihedral has a small
//! rotation about body X. The crate projects the velocity into the zone frame
//! before it calculates alpha and beta for the zone. As a result, dihedral and
//! sweep change the local angles without a special correction term. If a zone
//! has no rotation, the zone frame and the body frame are the same.
//!
//! ### Stability frame
//!
//! The stability frame is the zone frame turned by -alpha about its Y axis.
//! Its X axis points along the velocity, projected on the X-Z plane. Lift is
//! perpendicular to the velocity (along -Z of the stability frame). Drag is
//! opposite to the velocity.
//!
//! The `aerodynamics` module in the source code has the full derivation.
//!
//! ### World frame (Bevy and Avian, Y-up, right-handed)
//!
//! Bevy uses a Y-up, right-handed world frame. Spawn the aircraft with a
//! rotation that aligns body X (forward) with world X. This rotation must also
//! align body Z (down) with world -Y. `Quat::from_rotation_x(FRAC_PI_2)` gives
//! this rotation. If the root entity has no rotation, body Y (right wing)
//! points up, and the lift is horizontal.
//!
//! ### From coefficients to a world force
//!
//! Drag opposes the full 3-D velocity. Lift and side force use the stability
//! frame. The crate then turns the sum into the world frame:
//!
//! ```text
//! F_drag_zone      = -CD * qbar * S * v_zone           (v_zone: unit velocity in the zone frame)
//! F_lift_side_stab = (0,  CY * qbar * S,  -CL * qbar * S)
//! F_world          = zone_to_world * (F_drag_zone + R_y(-alpha) * F_lift_side_stab)
//! zone_to_world    = body_to_world * zone_rotation
//! ```
//!
//! All FDM calculations use `f32`, in both precision modes. The FDM converts
//! values only where it reads from or writes to Avian components.
//!
//! ---
//!
//! ## The Equations of Motion
//!
//! An aircraft in free flight has **6 degrees of freedom** (6-DoF). Three are
//! translations (x, y, z). Three are rotations: roll (phi), pitch (theta), and
//! yaw (psi).
//!
//! ### Translational dynamics
//!
//! Newton's second law gives the translation. **The net force is equal to the
//! mass times the acceleration:**
//!
//! ```text
//! F_net = m * (dV/dt)
//! ```
//!
//! Here, **V** is the velocity of the center of mass in world coordinates.
//! `avian_fdm` supplies **F_net** through [`avian3d::prelude::ConstantForce`].
//! Avian calculates the new **V** in each substep.
//!
//! `avian_fdm` adds these forces:
//! - Aerodynamic: lift, drag, and side force from each [`components::AeroZone`]
//! - Propulsive: thrust from each [`components::EngineZone`]
//!
//! Avian applies gravity with its own gravity system.
//!
//! ### Rotational dynamics (Euler's equations)
//!
//! Euler's equations give the rotation. The equations below use the body
//! frame. They assume that the principal axes of inertia are near body X, Y,
//! and Z. **Each moment is equal to the inertia times the angular
//! acceleration, plus a gyroscopic cross-coupling term:**
//!
//! ```text
//! L = I_xx * p_dot + (I_zz - I_yy) * q * r   (roll  equation)
//! M = I_yy * q_dot + (I_xx - I_zz) * p * r   (pitch equation)
//! N = I_zz * r_dot + (I_yy - I_xx) * p * q   (yaw   equation)
//! ```
//!
//! Here, (p, q, r) are the roll, pitch, and yaw rates in the body frame.
//! (L, M, N) are the rolling, pitching, and yawing moments. Avian solves these
//! equations internally. `avian_fdm` supplies (L, M, N) through
//! [`avian3d::prelude::ConstantTorque`]. **For each zone, the torque is the
//! cross product of the moment arm and the zone force:**
//!
//! ```text
//! tau_zone = (r_zone - r_CG) x F_zone
//! ```
//!
//! Here, **r_zone** is the world position of the aerodynamic center of the
//! zone. **r_CG** is the world position of the center of mass. The crate reads
//! it from [`avian3d::prelude::ComputedCenterOfMass`]. A zone can also add a
//! pure torque from its moment coefficients (CM, Croll, Cn). Avian does the
//! quaternion normalization, the substeps, and the coupling between
//! translation and rotation.
//!
//! ---
//!
//! ## The Atmosphere
//!
//! The `atmosphere` module in the source code has the full implementation.
//!
//! Each aerodynamic force is proportional to the dynamic pressure. The dynamic
//! pressure is the kinetic energy of the airflow per unit volume. **Dynamic
//! pressure = half × air density × airspeed squared:**
//!
//! ```text
//! qbar = 0.5 * rho * V^2
//! ```
//!
//! At sea level, rho = 1.225 kg/m³. At 2500 m, rho is approximately 20% less.
//! At the same airspeed, lift and drag also decrease by approximately 20%.
//! Thus, an aircraft must fly faster at altitude to make the same lift.
//!
//! Air density and airspeed also set the Reynolds number. **Reynolds number =
//! (density × speed × chord) ÷ viscosity.** It is a dimensionless ratio of
//! inertial forces to viscous forces. It tells if the airflow is smooth or
//! turbulent:
//!
//! ```text
//! Re = rho * V * c_bar / mu
//! ```
//!
//! Here, c_bar is the mean aerodynamic chord and mu is the dynamic viscosity.
//! Each zone uses its own chord (`AeroZone::chord_m`) to calculate Re. The
//! Reynolds number controls the boundary layer. At low Re, the flow separates
//! earlier. The stall is sharper, and the zero-lift drag (CD0) is higher. For
//! this reason, Re is the second input of a 2-D coefficient table.
//!
//! The `atmosphere` module implements the International Standard Atmosphere
//! (ISA, ICAO Doc 7488) from 0 to 20 km. In each physics step,
//! `update_atmosphere` writes a new [`components::AtmosphereState`] to the
//! root entity.
//!
//! ---
//!
//! ## Aerodynamic Forces and Moments
//!
//! The `aerodynamics` module in the source code has the full implementation.
//!
//! The crate divides the aircraft into zones. For each
//! [`components::AeroZone`] child, the system does these steps:
//!
//! 1. It evaluates the coefficient tables at the local flight state of the
//!    zone.
//! 2. It multiplies the coefficients by the dynamic pressure and the zone
//!    area.
//! 3. It writes the force and the torque.
//!
//! A coefficient has one of three forms: a constant (`Scalar`), a 1-D table
//! over the angle of attack (`Table1D`), or a 2-D table over the angle of
//! attack and Re (`Table2D`).
//!
//! ### Damping: emergent or explicit
//!
//! Damping is a moment that opposes a rotation. For a full-zone aircraft (the
//! default), damping comes from a local-angle correction in each zone. Each
//! zone gets its own angle of attack and sideslip angle. These angles come
//! from the zone position and the angular rates of the body:
//!
//! ```text
//! alpha_local = alpha + (p*y - q*x) / V   (roll-rate and pitch-rate corrections)
//! beta_local  = beta  + (r*x)       / V   (yaw-rate correction)
//! ```
//!
//! As a result, the tail makes a restoring force during a pitch motion. The
//! wings make different lift on each side during a roll. The fin makes a side
//! force during a yaw. No explicit damping coefficient is necessary.
//!
//! For sparse models (missiles, simple drones), add
//! [`components::LodDamping`] to the root entity. It gives explicit damping
//! derivatives.
//!
//! ### Control surfaces
//!
//! You can give each zone a [`components::ControlSurfaceRole`]. The matching
//! value in [`components::ControlInputs`] then scales the lift, side-force,
//! and moment coefficients of the zone. The moment arm from the zone to the CG
//! turns this force into a roll, pitch, or yaw moment. The drag coefficient of
//! the zone scales with the absolute value of the input. Thus, a deflected
//! surface always adds drag.
//!
//! ## Propulsion Coupling
//!
//! Thrust uses the Gagg-Ferrar altitude correction. The zone health, the
//! throttle curve, the air density ratio, and the airspeed scale the maximum
//! thrust:
//!
//! ```text
//! T            = T_max * remaining * throttle_curve(throttle) * (rho / rho_0)^0.7 * speed_factor
//! speed_factor = max(0, 1 - (V / V_zero)^2)    (1.0 if zero_thrust_speed_ms is None)
//! ```
//!
//! Each [`components::EngineZone`] has a thrust direction in the body frame.
//! The position of the engine sets the moment arm to the CG.
//!
//! ---
//!
//! ## Emergent Behavior
//!
//! The zone model makes physically correct behaviors without global
//! coefficients for them. These behaviors have three causes:
//!
//! - The crate calculates the forces of each zone at the position of that
//!   zone.
//! - The crate calculates the moment arm to the CG in each physics step.
//! - Avian calculates the mass, the CG, and the inertia from the colliders.
//!
//! ### Stall and high angle-of-attack behavior
//!
//! **Stall:**
//! If alpha is more than the stall angle of the CL table, CL decreases. If the
//! tables include the post-stall range, the wing zones make less lift. The
//! aircraft pitches down or departs. The recovery obeys normal stall physics.
//!
//! **Wing drop at stall:**
//! Near the stall alpha, a roll rate gives one wing tip a higher local alpha
//! than the other tip (alpha_local += p*y/V). The tip with the higher alpha
//! stalls first. The other tip continues to fly. The result is a roll that the
//! pilot did not command. This is the physical cause of wing drop.
//!
//! **Snap roll:**
//! Near the stall alpha, a sudden aileron input pushes one wing zone past the
//! stall angle. The local-angle correction causes this. That zone loses lift,
//! and the other side continues to fly. The aircraft rolls quickly and out of
//! control.
//!
//! **Damping reversal at high angle of attack:**
//! After the stall angle, the slope of the CL table is negative
//! (dCL/dalpha < 0). The local-angle correction now makes a force that helps
//! the angular rate, instead of a force that opposes it. Thus, the damping
//! changes sign. This drives the autorotation of a spin. No separate model for
//! damping reversal is necessary.
//!
//! **Deep stall:**
//! Sometimes the wing zones stall, but the tail continues to fly. The tail has
//! a lower alpha because of its longitudinal position and the pitch-rate
//! correction. The tail is aft, so its lift makes a nose-up moment. This
//! moment keeps the alpha high and prevents recovery. If the tables include
//! the deep-stall range, this behavior comes directly from the tables.
//!
//! **Spin dynamics:**
//! In a spin, the descending wing has a higher local alpha from the roll rate,
//! and it is stalled. The rising wing has a lower alpha and continues to make
//! lift. The difference in lift drives the autorotation. The yaw-rate
//! correction changes beta for the zones forward and aft of the CG. Together,
//! these effects make a steady spin. The spin has correct rotation rates and
//! loss of altitude without a spin model.
//!
//! ### Cross-coupling
//!
//! **Adverse yaw (Cnp: yaw from roll rate):**
//! During a roll rate p, the relative wind at the descending wing comes more
//! from below. Its local alpha increases, and its lift vector tilts forward.
//! At the rising wing, the lift vector tilts aft. These tilted lift vectors
//! yaw the nose toward the rising wing, opposite to the roll. Each zone
//! calculates its lift perpendicular to its local velocity. Thus, the
//! local-angle correction gives this effect.
//!
//! **Roll from yaw rate (Clr):**
//! During a yaw rate r, the advancing wing moves faster and makes more lift.
//! The difference in lift raises the advancing wing, and the aircraft rolls
//! toward the retreating wing. The local-angle correction gives a part of
//! this effect. The full effect needs a per-zone qbar scaling.
//!
//! **Dutch roll:**
//! Yaw and roll couple through Clr and Cnp. The yaw restoring moment of the
//! fin and the different lift of the two wings together make an oscillation
//! in yaw and roll. This oscillation is the Dutch roll. The fin volume and the
//! wing dihedral set the type of the mode: damped, neutral, or divergent.
//!
//! **Departure in pitch, roll, and yaw at high angle of attack:**
//! In a combined maneuver, the wing zones can stall unevenly. Then all three
//! axes couple at the same time. The asymmetric stall along the span drives
//! roll. The drag asymmetry drives yaw. The tail loses effectiveness, and the
//! nose pitches up. These effects increase each other. No special departure
//! model is necessary.
//!
//! ### Control authority
//!
//! **Roll authority (Cl_da):**
//! The ailerons are far from the centerline. Their lift, scaled by the aileron
//! input, times the moment arm to the CG gives a roll torque. You do not set
//! Cl_da anywhere.
//!
//! **Pitch authority (Cm_de):**
//! The elevator is aft. Its lift times the tail arm gives a pitch torque.
//!
//! **Yaw authority (Cn_dr):**
//! The rudder makes a side force at the fin. The moment arm to the CG turns
//! this force into a yaw torque.
//!
//! **Pitching, rolling, and yawing moments from thrust:**
//! If an engine zone is not on the CG (offset in x, y, or z), its thrust makes
//! pitching and yawing moments automatically. A tilted thrust axis (pusher,
//! tilt-rotor) gives the correct coupled moments.
//!
//! **Torque reaction of the propeller:**
//! A turning propeller or engine applies an opposite angular momentum to the
//! airframe. The crate does not model this effect yet. The plan is to add a
//! pure torque on the engine zone, about the axis of the propeller, opposite
//! to its rotation. The torque then goes to the root entity through the Avian
//! constraint solver.
//!
//! ### Aerodynamic configuration effects
//!
//! **Flaps and slats:**
//! Add a flap zone at the trailing edge of the wing. Give it a CL table for
//! the cambered section at flap deflection. Scale the table by the fraction of
//! wing area that the flap has. [`components::ControlInputs`] has no flap
//! input, and [`components::ControlSurfaceRole`] has no flap role. To deploy
//! the flaps, change `cl` and `cd` of the flap zone from game code. Then the
//! zone applies the cambered CL and the higher CD. No special flap model is
//! necessary. Slats use the same method on the leading edge.
//!
//! **External stores and fuel tanks:**
//! Add an AeroZone with CL = 0 and a CD for the drag area of the store. Put
//! the zone at the position of the store. The drag force at that position has
//! a moment arm to the CG, so it changes the trim. To drop a store, set
//! `Failure::remaining` to 0. This removes its drag. To also remove its mass,
//! despawn the zone entity or remove its collider.
//!
//! ### Flight dynamics modes
//!
//! **Phugoid oscillation:**
//! The aircraft pitches up. The lift increases, and the aircraft climbs and
//! slows. The lower speed decreases the lift. The aircraft pitches down,
//! descends, and accelerates again. This long-period oscillation comes from
//! pitch stability and the relation between lift and speed. No phugoid
//! frequency coefficient exists in the crate.
//!
//! **Short-period mode:**
//! A fast pitch oscillation comes from pitch stiffness (tail volume) and pitch
//! damping (Cm_q). The geometry sets the frequency and the damping ratio.
//!
//! **Spiral mode:**
//! In a bank, the aircraft slips toward the low wing. The fin turns the nose
//! into the slip, and the dihedral effect of the wing rolls the aircraft back
//! to level. If the yaw stiffness of the fin is strong compared to the
//! dihedral effect, the bank increases, and the aircraft spirals. The balance
//! between these two effects sets the stability of the mode (damped or
//! divergent).
//!
//! ### Damage effects
//!
//! **CG shift:**
//! If game code removes the collider of a zone (for example, it despawns the
//! zone entity), Avian removes the mass of that zone. The CG moves toward the
//! remaining structure, and all moment arms change. For example, the loss of
//! a wing tip moves the CG away from that tip. `Failure::remaining = 0` alone
//! does not change the mass.
//!
//! **Inertia tensor change:**
//! After the removal of a collider, Avian calculates the full tensor again
//! from the remaining colliders. The loss of a wing panel decreases the roll
//! inertia. The aircraft then responds faster in roll.
//!
//! **Asymmetric roll from damage on one side:**
//! The lift of the intact wing has no opposite lift on the other side. This
//! lift times its moment arm makes a continuous roll. No "damage roll rate"
//! parameter exists.
//!
//! **Loss of control authority:**
//! If a control-surface zone (aileron, elevator, or rudder) has
//! `Failure::remaining = 0`, its force contribution is zero. Partial damage
//! (`Failure::remaining = 0.5`) halves the forces of the zone. The surface
//! then has half authority. The other surfaces do not change.
//!
//! **Less damping from structural damage:**
//! The loss of fin area decreases Cn_r. The loss of wing area decreases Cl_p.
//! After a disturbance, the aircraft oscillates more. All these effects come
//! directly from the zones that remain.
//!
//! **Changed departure behavior after damage:**
//! After the loss of a wing panel, the asymmetric roll under a roll rate is
//! larger. The damaged side has less lift to oppose the roll. Thus, a wing
//! drop at stall is more aggressive on that side.
//!
//! ### What does NOT emerge yet (gaps and planned work)
//!
//! These effects need more data for each zone, or changes to the force
//! calculation:
//!
//! - **Clr and Cnp (full effect):** The angle-only correction gives a part of
//!   these terms. The largest term is the difference in dynamic pressure,
//!   (V +/- r*y)^2, between zones. The plan is to make `zone_local_angles`
//!   also scale the local qbar of each zone. With that change, Clr and Cnp
//!   will be fully emergent, and damage will change them automatically.
//!
//! - **Cm_alphadot:** This term needs the time lag between the wing downwash
//!   and the response of the tail. Instantaneous angle corrections cannot give
//!   it. The plan is to scale a global coefficient by the
//!   `Failure::remaining` of the horizontal stabilizer.
//!
//! - **Wake turbulence and vortex interaction:** When two aircraft fly near
//!   each other, the wing-tip vortices of one aircraft add induced velocity at
//!   the wing zones of the other. The plan is a separate `VortexWake`
//!   component for each aircraft. It will add a velocity field to the
//!   local-angle calculation of the zones.
//!
//! - **Propeller slipstream and P-factor:** At high alpha, the propeller disk
//!   has asymmetric loading. Also, the slipstream accelerates the air over the
//!   inner wing. Both effects need a model of the propeller wake as a
//!   distributed velocity field, not as a point force.
//!
//! - **Ground effect:** Near the ground, lift increases and induced drag
//!   decreases. This effect needs a correction to the local qbar or to the
//!   induced angle. The correction changes with the height.
//!
//! - **Wing-fuselage interference:** At the wing root, the fuselage boundary
//!   layer and the pressure gradient at the wing leading edge make a
//!   horseshoe vortex. This vortex causes a local downwash on the inboard
//!   panels and decreases their angle of attack. Thus, inboard panels make
//!   less lift than the same section in free air. At this time, all panels
//!   use the same CL table. To model this effect, give the root panels a
//!   slightly lower CL table from wind-tunnel or CFD data. Alternatively,
//!   apply a scalar interference factor to the lift of the root panel.
//!
//! - **Ice accretion and contamination:** Ice and contamination change the
//!   CL and CD tables of the zones that they affect. The plan is a table
//!   modifier that changes the zone coefficients at runtime.
//!
//! ---
//!
//! ## Zone Decomposition and Damage
//!
//! Each structural part of the aircraft is a separate ECS entity: an
//! [`components::AeroZone`]. It is a child of the root rigid body. Each zone
//! has these parts:
//! - A [`avian3d::prelude::Collider`] that gives it volume and mass (with
//!   [`avian3d::prelude::ColliderDensity`])
//! - A `Transform` that sets its position and rotation in the body frame
//! - An [`components::AeroZone`] that describes its aerodynamic contribution
//! - An optional [`components::Failure`] that records damage
//!
//! Avian **automatically** calculates the total mass, the CG, and the inertia
//! tensor from all child colliders. You do not need to calculate them.
//!
//! ### Collider strategy
//!
//! Zone colliders have two purposes: **mass and inertia** (with
//! `ColliderDensity`) and **debug visualization**. The correct collider type
//! depends on the role of the zone:
//!
//! | Collider type | Mass | Hit detection | When to use |
//! |---|---|---|---|
//! | Primitive (`cuboid`, `ball`, `cylinder`) | ✅ exact, analytic | Approximate | Aerodynamic surfaces and structural parts. Adjust `ColliderDensity` to get the target mass. |
//! | `ConvexHull` (from mesh) | ✅ from hull volume | Good convex approximation | Parts with volume, where the hull is near the real shape (engine cowl, fuselage bulkhead) |
//! | `TriMesh` | ❌ none (static only) | Exact | Never on an AeroZone. Use it only for terrain or static scenery. |
//!
//! Usually, **aero zones use primitives**. The detailed 3-D model is a
//! separate child entity for visuals only (no `Collider`, no `RigidBody`). The
//! primitive wireframes are for diagnosis. The player sees the mesh.
//!
//! For accurate **hit detection** in a combat game, add a `Sensor` collider as
//! a child of the AeroZone. Use a `ConvexHull` or a `TriMesh` of the visual
//! mesh. Sensors send `CollisionStarted` and `CollisionEnded` events. They add
//! no mass and apply no forces. Thus, game code can find the zone that a
//! bullet hit, and decrease `Failure::remaining` for that zone. This
//! hit-detection layer is **game code**. `avian_fdm` only defines `Failure` as
//! the damage target. The method that applies damage is outside the scope of
//! the crate.
//!
//! ### Performance loss from damage
//!
//! If [`components::Failure::remaining`] is between 0 and 1, the coefficients
//! of the zone change as follows:
//!
//! ```text
//! CL_effective = CL * input * remaining
//! CD_effective = (CD * |input| + damage_drag_coeff * (1 - remaining) / qbar) * remaining
//! ```
//!
//! For a control surface, `input` is the matching control input. For all
//! other zones, `input` is 1.
//!
//! A damaged [`components::AeroZone`] makes less lift. If `damage_drag_coeff`
//! is set, it also makes extra drag from structural deformation. This extra
//! drag is zero for an intact zone and for a zone with `remaining = 0`. It is
//! largest between these two values. At `remaining = 0`, the zone makes no
//! force. Its aerodynamic effect is the same as that of a zone that separated
//! from the airframe. Physical detachment (a despawn, or a new parent) is the
//! responsibility of the game.
//!
//! ---
//!
//! ## Reading Simulation Output
//!
//! The crate updates [`components::FlightState`] on the root entity in each
//! physics step. Use it in your systems or for a HUD:
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
//! | `airspeed_ms` | m/s | True airspeed (TAS): the magnitude of the velocity relative to the air |
//! | `alpha_rad` | rad | Angle of attack. Positive when the nose is above the velocity vector. |
//! | `beta_rad` | rad | Sideslip angle. Positive when the wind comes from starboard. |
//! | `dynamic_pressure_pa` | Pa | qbar = 0.5 * rho * V^2 |
//! | `mach` | - | V divided by the speed of sound. For information only (no compressibility model yet). |
//! | `p_rads`, `q_rads`, `r_rads` | rad/s | Roll, pitch, and yaw rates in the body frame |
//!
//! **How to read the trim quality:**
//! - Level cruise: `alpha_rad` is approximately 2–5° (small and positive).
//!   `airspeed_ms` is stable.
//! - Phugoid: `airspeed_ms` and `alpha_rad` oscillate slowly (period
//!   approximately π·√2·V/g, the Lanchester approximation). This is a natural
//!   mode. It shows that the pitch model works.
//! - Divergence: `alpha_rad` increases without limit. If this occurs, examine
//!   the sign convention of the elevator CL. Also examine the moment arm of
//!   the horizontal stabilizer.
//!
//! ---
//!
//! ## Data Flow
//!
//! ```text
//! +-- PhysicsStepSystems::BroadPhase (once per physics step) ---------------+
//! |  1. update_atmosphere          reads:  Position(root)                   |
//! |                                writes: AtmosphereState                  |
//! |                                                                         |
//! |  2. update_flight_state        reads:  Position(root), Rotation(root),  |
//! |                                        LinearVelocity, AngularVelocity, |
//! |                                        AtmosphereState, WindResource    |
//! |                                writes: FlightState                      |
//! |                                                                         |
//! |  3. compute_engine_zone_forces reads:  ControlInputs, AtmosphereState,  |
//! |                                        FlightState, EngineZone,         |
//! |                                        Failure, Position(root),         |
//! |                                        Rotation(root),                  |
//! |                                        Transform(engine)                |
//! |                                writes: ZoneForce(engine)                |
//! |                                                                         |
//! |  4. compute_aero_forces        reads:  Position(root), Rotation(root),  |
//! |                                        ComputedCenterOfMass,            |
//! |                                        FlightState, AtmosphereState,    |
//! |                                        AircraftGeometry, ControlInputs, |
//! |                                        AeroZone, Failure,               |
//! |                                        Transform(zone, engine),         |
//! |                                        Children,                        |
//! |                                        ZoneForce(engine), LodDamping,   |
//! |                                        InducedDrag                      |
//! |                                writes: ZoneForce(aero),                 |
//! |                                        ConstantForce(root),             |
//! |                                        ConstantTorque(root)             |
//! +-------------------------------------------------------------------------+
//!         |
//!         v  Solver: Avian integrates the forces
//!         |
//!         v  PostUpdate: Bevy propagates GlobalTransform from Position and Rotation
//! ```
//!
//! ### Why the FDM reads Position, Rotation, and Transform, not GlobalTransform
//!
//! Avian writes the physics state of the root entity into `Position` and
//! `Rotation`. Bevy copies these values into `GlobalTransform` only in
//! `PostUpdate`, after the physics schedule. Thus, during `BroadPhase`, the
//! `GlobalTransform` of the root and of each zone child has the value from
//! the *previous* frame. With a floating origin, `GlobalTransform` is also not
//! the physics position.
//!
//! For this reason, all FDM systems read `Position` and `Rotation` on the root
//! entity. `compute_aero_forces` and `compute_engine_zone_forces` also read
//! the local `Transform` of each aero zone and each engine zone. The zone
//! `Transform` is set at spawn time and does not change at runtime.
//! `compute_aero_forces` calculates the moment arm of each zone in body frame
//! and rotates it to world orientation:
//! `arm = root_rotation * (zone_transform.translation - center_of_mass)`.
//! This value is current in the physics step. It does not lose precision far
//! from the origin, because it does not subtract two world positions.
//!
//! ### Add a custom system (for example, an autopilot)
//!
//! An autopilot reads [`components::FlightState`] and writes
//! [`components::ControlInputs`]. Put it after
//! `AircraftFdmSystems::FlightState` and before `AircraftFdmSystems::Forces`:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};
//! # use avian_fdm::prelude::*;
//! # fn my_autopilot() {}
//! // app.add_systems(
//! //     PhysicsSchedule,
//! //     my_autopilot
//! //         .after(AircraftFdmSystems::FlightState)
//! //         .before(AircraftFdmSystems::Forces)
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
//! | `f32`          | yes     | Enables the avian3d f32 backend and collider shapes. |
//! | `f64`          | --      | Enables the avian3d f64 backend. You cannot use it together with `f32`. |
//! | `debug-plugin` | yes     | Bevy gizmo overlays for forces, moments, and zones ([`debug_render`]) |
//!
//! The JSBSim-derived reference aircraft (J-3 Cub) is in the separate
//! `avian_fdm_j3cub_jsbsim` crate (GPL-3.0-only).
//!
//! ---
//!
//! ## Further Reading
//!
//! The references below helped to design and validate `avian_fdm`. The list
//! starts with the easiest texts.
//!
//! ### Introductory
//!
//! - **Robert C. Nelson, *Flight Stability and Automatic Control*, 2nd ed.
//!   (McGraw-Hill, 1998).**
//!   The clearest introduction to the stability-derivative approach. Table B1
//!   gives derivatives for light general-aviation (GA) aircraft similar to the
//!   J-3 Cub. The damping coefficients in the `aerodynamics` module come from
//!   this table. Chapters 2–4 derive the equations of motion in the body
//!   frame.
//!
//! - **John D. Anderson, *Introduction to Flight*, 8th ed. (McGraw-Hill, 2015).**
//!   Very good coverage of the basics of aerodynamics (lift, drag, Reynolds
//!   number, boundary layers) before the study of dynamics. Chapters 5 and 7
//!   are the most relevant.
//!
//! ### Intermediate
//!
//! - **Bernard Etkin & Lloyd D. Reid, *Dynamics of Flight: Stability and
//!   Control*, 3rd ed. (Wiley, 1996).**
//!   A rigorous derivation of the 6-DoF Newton-Euler equations in the body
//!   frame. The axis transformation in the `aerodynamics` module (body to
//!   stability to wind axes) follows Etkin & Reid, Chapter 4.
//!
//! - **Brian L. Stevens, Frank L. Lewis & Eric N. Johnson,
//!   *Aircraft Simulation and Control*, 3rd ed. (Wiley, 2016).**
//!   The standard reference for simulation architecture. The frame
//!   conventions of this crate agree with Stevens & Lewis, Appendix A. The
//!   book also covers numerical integration and digital flight control
//!   systems.
//!
//! ### Atmospheric model
//!
//! - **ICAO Doc 7488/3, *Manual of the ICAO Standard Atmosphere*, 3rd ed.
//!   (International Civil Aviation Organization, 1993).**
//!   The official definition of the International Standard Atmosphere. The
//!   `atmosphere` module implements it. You can download it free of charge
//!   from the ICAO store.
//!
//! [Avian]: https://crates.io/crates/avian3d
//! [JSBSim]: https://jsbsim.sourceforge.net/

#![deny(missing_docs)]

/// Records the data source of a value.
///
/// The second argument is a string literal that describes the source. The
/// macro **discards** this string and emits only the value expression. The
/// string does not become a `&str` static, and it adds nothing to the binary.
///
/// Use this macro in preset files to record the origin of each number. Then a
/// maintainer can find the source of a value without a search through the
/// commit history or external references.
///
/// # Conventions for the source prefix
///
/// | Prefix | Meaning |
/// |---|---|
/// | `"JSBSim:J3Cub.xml"` | Copied directly from a JSBSim aircraft XML file |
/// | `"Calibration:JSBSim"` | Adjusted by experiment to match the JSBSim behavior |
/// | `"Literature:..."` | Taken from a published paper or textbook |
/// | `"Geometry"` | Calculated analytically from the aircraft dimensions |
/// | `"Estimate"` | Engineering judgment. No primary source. |
/// | `"Guesswork"` | Placeholder. Replace it with measured data. |
///
/// # Example
///
/// ```rust
/// # use avian_fdm::sourced;
/// // No runtime cost. The macro expands to exactly `0.94`:
/// let e: f64 = sourced!(0.94, "JSBSim:J3Cub.xml: CD_i = CL²×0.0485, e≈0.94");
/// assert_eq!(e, 0.94);
/// ```
#[macro_export]
macro_rules! sourced {
    ($value:expr, $source:literal) => {
        $value
    };
}

/// Internal re-exports of the Bevy sub-crates.
/// All internal modules import from here, not from the `bevy` meta-crate.
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
pub mod airfoil;
pub(crate) mod atmosphere;
pub mod components;
// The crate declares this module without the `debug-plugin` gate. Thus,
// `AircraftFdmPlugin` can call `.register_type::<GizmoShape>()` in all
// builds, and these components stay scene-serializable when `debug-plugin`
// is disabled. The file is in `src/debug_render/`, because it belongs there
// by its topic.
#[path = "debug_render/gizmo_shape.rs"]
pub(crate) mod gizmo_shape;
pub(crate) mod kinematics;
pub(crate) mod math;
pub mod plugin;
pub(crate) mod propulsion;

#[cfg(feature = "debug-plugin")]
pub mod debug_render;

/// Re-exports for a glob import: `use avian_fdm::prelude::*;`
pub mod prelude {
    pub use crate::airfoil::{AirfoilData, AirfoilLibrary, RegisterAirfoil};
    pub use crate::components::{
        aero_coeff::AeroCoeff, AeroZone, AeroZoneBundle, AircraftCoreBundle, AircraftGeometry,
        AtmosphereState, ControlInputs, ControlSurfaceRole, EngineZone, Failure, FlightState,
        GizmoContours, GizmoShape,
    };
    pub use crate::plugin::AircraftFdmPlugin;
    pub use crate::plugin::AircraftFdmSystems;
    pub use crate::sourced;

    #[cfg(feature = "debug-plugin")]
    pub use crate::debug_render::{
        AircraftFdmDebugPlugin, FdmDebugRender, FdmGizmos, ShowColliders,
    };
}
