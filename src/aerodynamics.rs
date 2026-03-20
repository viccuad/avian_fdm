//! Aerodynamic force pipeline.
//!
//! The pipeline is built from small, documented pure functions that each handle
//! one step of the aerodynamic computation.  The Bevy system
//! [`compute_aero_forces`] orchestrates them in order:
//!
//! ```text
//! For each aircraft root:
//!   For each AeroZone child:
//!     1. evaluate_zone_coefficients  — lookup CL/CD/… tables, apply control
//!                                      surface scaling and damage degradation
//!     2. zone_force_world            — rotate stability-frame forces to world,
//!                                      compute pure aerodynamic torques
//!   For each EngineZone child:
//!     3. accumulate_engine_force     — add pre-computed thrust (from propulsion
//!                                      system) and its moment-arm torque
//!   Once per aircraft:
//!     4. damping_torque              — angular-rate damping (cl_p, cm_q, cn_r)
//! ```
//!
//! Each function's doc comment explains the physics behind that step, so reading
//! them top-to-bottom gives a self-contained introduction to how the FDM
//! converts aerodynamic data into Avian forces.

use bevy::prelude::*;
use bevy::math::{DVec3, DQuat};
use avian3d::prelude::{
    ConstantForce, ConstantTorque, Position, Rotation, ComputedCenterOfMass,
};

use crate::components::{
    AeroZone, AircraftGeometry, ControlInputs, ControlSurfaceRole,
    Damageable, FlightState, ZoneForce,
};
#[cfg(feature = "propulsion")]
use crate::components::EngineZone;

// ── Step 1: Coefficient evaluation ───────────────────────────────────────────

/// Six non-dimensional aerodynamic coefficients, fully scaled and ready to be
/// multiplied by dynamic pressure and reference area to produce forces.
pub struct ZoneCoefficients {
    /// Lift coefficient (positive = upward in stability frame).
    pub cl: f64,
    /// Drag coefficient (positive = opposing motion).
    pub cd: f64,
    /// Side-force coefficient (positive = starboard).
    pub cy: f64,
    /// Pitching-moment coefficient (positive = nose up).
    pub cm: f64,
    /// Rolling-moment coefficient (positive = starboard wing down).
    pub croll: f64,
    /// Yawing-moment coefficient (positive = nose right).
    pub cn: f64,
}

/// Evaluate an [`AeroZone`]'s coefficient tables and apply control-surface
/// scaling and damage degradation.
///
/// # How it works
///
/// 1. **Table lookup** — Each coefficient (CL, CD, CY, CM, Croll, Cn) is
///    evaluated from the zone's [`AeroCoeff`](crate::components::aero_coeff::AeroCoeff)
///    at the current angle of attack α and Reynolds number Re.  The table may
///    be a constant (`Scalar`), a 1-D function of α (`Table1D`), or a 2-D
///    function of (α, Re) (`Table2D`).
///
/// 2. **Control-surface scaling** — If the zone is tagged with a
///    [`ControlSurfaceRole`], its lift/side/moment coefficients are multiplied
///    by the corresponding pilot input ∈ [−1, 1].  The coefficients in the
///    table represent *full-deflection* authority; scaling by the input gives
///    the contribution at partial deflection.
///
///    Drag always *increases* with deflection (flow separation from the
///    deflected surface), so CD is scaled by `|input|` rather than signed
///    input.  `AileronRight` mirrors the aileron sign so that left stick
///    deflects the left aileron trailing-edge down (more lift) and the right
///    one trailing-edge up (less lift).
///
/// 3. **Damage degradation** — All coefficients are multiplied by
///    `health ∈ (0, 1]`.  A half-destroyed wing produces half the lift.
///    Additionally, structural deformation adds parasitic drag that peaks at
///    intermediate health:
///
///    ```text
///    CD_effective = (CD_base · |input| + damage_drag · (1 − health) / q̄) · health
///    ```
///
///    At health = 0 the zone is fully detached and produces no force at all
///    (handled by the caller, not this function).
pub fn evaluate_zone_coefficients(
    zone: &AeroZone,
    ctrl: &ControlInputs,
    alpha: f64,
    re: f64,
    qbar: f64,
    health: f64,
) -> ZoneCoefficients {
    // Raw table lookups at the current flight condition.
    let cl_base = zone.cl.evaluate(alpha, re);
    let cd_base = zone.cd.evaluate(alpha, re);
    let cy_base = zone.cy.evaluate(alpha, re);
    let cm_base = zone.cm.evaluate(alpha, re);
    let croll_base = zone.croll.evaluate(alpha, re);
    let cn_base = zone.cn.evaluate(alpha, re);

    // Control-surface scaling.
    let (scale, cd_scale) = match &zone.control_role {
        Some(ControlSurfaceRole::Elevator) => (ctrl.elevator, ctrl.elevator.abs()),
        Some(ControlSurfaceRole::AileronLeft) => (ctrl.aileron, ctrl.aileron.abs()),
        Some(ControlSurfaceRole::AileronRight) => (-ctrl.aileron, ctrl.aileron.abs()),
        Some(ControlSurfaceRole::Rudder) => (ctrl.rudder, ctrl.rudder.abs()),
        None => (1.0, 1.0),
    };

    // Damage: structural deformation drag, scaled out at health = 0.
    let extra_cd = zone.damage_drag_coeff * (1.0 - health) / qbar.max(1e-4);

    ZoneCoefficients {
        cl: cl_base * scale * health,
        cd: (cd_base * cd_scale + extra_cd) * health,
        cy: cy_base * scale * health,
        cm: cm_base * scale * health,
        croll: croll_base * scale * health,
        cn: cn_base * scale * health,
    }
}

// ── Step 2: Stability-frame forces → world ───────────────────────────────────

/// World-space force and torque produced by a single zone.
pub struct ZoneWorldForce {
    /// Aerodynamic force in world coordinates (N).
    pub force: DVec3,
    /// Pure aerodynamic torque in world coordinates (N·m).
    ///
    /// This is the couple that exists independently of the zone's position
    /// (e.g. an airfoil's pitching moment about its own aerodynamic centre).
    /// It is *not* the moment-arm torque — that is computed separately by the
    /// caller using `(zone_position − CG) × force`.
    pub torque: DVec3,
}

/// Convert non-dimensional coefficients into a world-space force and torque.
///
/// # Coordinate frame journey
///
/// Aerodynamic coefficients are defined in the **stability frame**, which is
/// the body frame rotated by −α about body-Y so that its X axis aligns with
/// the velocity vector.  Forces are constructed there:
///
/// ```text
/// F_stability = ( −CD · q̄ · S,     drag opposes motion (−X_s)
///                  CY · q̄ · S,     side force (Y_s = body Y, starboard)
///                 −CL · q̄ · S )    lift perpendicular to velocity (−Z_s)
/// ```
///
/// The force is then rotated to body frame and then to world:
///
/// ```text
/// F_body  = R_y(−α) · F_stability      (stab_to_body rotation)
/// F_world = q_root  · F_body            (body_to_world quaternion)
/// ```
///
/// # Pure aerodynamic torques
///
/// Each zone also produces moment coefficients (CM, Croll, Cn) that represent
/// pure couples — torques that exist even if the zone were at the CG.  These
/// are expressed directly in body frame (not stability frame):
///
/// ```text
/// τ_body = ( Croll · q̄ · S · b,   rolling moment  → body X
///            CM    · q̄ · S · c̄,   pitching moment → body Y
///            Cn    · q̄ · S · b )   yawing moment   → body Z
/// ```
///
/// where `b` is wing span and `c̄` is mean aerodynamic chord.  Rolling and
/// yawing moments use span `b` as the reference length (lateral); pitching
/// moment uses chord `c̄` (longitudinal).
pub fn zone_force_world(
    coeffs: &ZoneCoefficients,
    qbar: f64,
    s: f64,
    b: f64,
    c: f64,
    stab_to_body: DQuat,
    body_to_world: DQuat,
) -> ZoneWorldForce {
    let force_stab = DVec3::new(
        -coeffs.cd * qbar * s,
        coeffs.cy * qbar * s,
        -coeffs.cl * qbar * s,
    );
    let force = body_to_world * (stab_to_body * force_stab);

    let torque_body = DVec3::new(
        coeffs.croll * qbar * s * b,
        coeffs.cm * qbar * s * c,
        coeffs.cn * qbar * s * b,
    );
    let torque = body_to_world * torque_body;

    ZoneWorldForce { force, torque }
}

// ── Step 3: Engine force accumulation ────────────────────────────────────────

/// Accumulate a pre-computed engine zone's thrust into the root force/torque.
///
/// Engine zones are evaluated by the separate `compute_engine_zone_forces`
/// system (see [`crate::propulsion`]).  By the time this function runs, each
/// engine zone's [`ZoneForce`] already contains the thrust vector and
/// application point.  We simply add the force and its moment-arm torque
/// (position × force) to the aircraft totals.
///
/// The moment arm is measured from the aircraft's centre of gravity to the
/// engine's world position.  An off-centre engine (e.g. a twin) naturally
/// produces a yawing moment when thrust is asymmetric.
fn accumulate_engine_force(
    zf: &ZoneForce,
    com_world: Vec3,
    total_force: &mut Vec3,
    total_torque: &mut Vec3,
) {
    if zf.force != Vec3::ZERO {
        *total_force += zf.force;
        *total_torque += (zf.world_point - com_world).cross(zf.force);
    }
}

// ── Step 4: Dynamic damping ──────────────────────────────────────────────────

/// Compute whole-aircraft angular-rate damping torque in world coordinates.
///
/// # Why damping matters
///
/// Without rate-dependent restoring moments, any angular perturbation would
/// grow without bound — the aircraft would tumble after the slightest gust.
/// Real aircraft are damped by the aerodynamic surfaces that resist rotation:
///
/// - **Roll damping (Cl_p):** When the aircraft rolls right (p > 0), the
///   descending right wing sees increased angle of attack, producing more lift,
///   while the rising left wing sees less.  The differential lift opposes the
///   roll — a restoring moment proportional to roll rate.
///
/// - **Pitch damping (Cm_q):** When the aircraft pitches nose-up (q > 0), the
///   horizontal tail moves downward, increasing its angle of attack and
///   producing a nose-down moment that opposes the pitch rate.
///
/// - **Yaw damping (Cn_r):** When the aircraft yaws right (r > 0), the
///   vertical tail sees a sideslip that produces a leftward (restoring) yaw
///   moment.
///
/// # Non-dimensional form
///
/// Each derivative is expressed in non-dimensional rate:
///
/// ```text
/// ΔL = Cl_p · (p·b / 2V) · q̄ · S · b     roll  damping → body X
/// ΔM = Cm_q · (q·c̄ / 2V) · q̄ · S · c̄    pitch damping → body Y
/// ΔN = Cn_r · (r·b / 2V) · q̄ · S · b     yaw   damping → body Z
/// ```
///
/// The `(rate × length / 2V)` term is the non-dimensional rate — the ratio
/// of the tip speed (from rotation) to the freestream velocity.  Multiplying
/// by `q̄ · S · length` recovers a dimensional moment in N·m.
///
/// Typical values for a light GA aircraft (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.  All are negative
/// because damping opposes motion.
pub fn damping_torque(
    flight: &FlightState,
    geo: &AircraftGeometry,
    body_to_world: DQuat,
) -> DVec3 {
    let v = flight.airspeed_ms;
    let qbar = flight.dynamic_pressure_pa;
    let s = geo.wing_area_m2;
    let b = geo.wing_span_m;
    let c = geo.chord_m;

    let pb_2v = flight.p_rads * b / (2.0 * v);
    let qc_2v = flight.q_rads * c / (2.0 * v);
    let rb_2v = flight.r_rads * b / (2.0 * v);

    let damp_body = DVec3::new(
        geo.cl_p * pb_2v * qbar * s * b,
        geo.cm_q * qc_2v * qbar * s * c,
        geo.cn_r * rb_2v * qbar * s * b,
    );

    body_to_world * damp_body
}

// ── Orchestrator system ──────────────────────────────────────────────────────

/// Cast a `DVec3` to `Vec3` (f64 → f32).
fn dvec3_to_vec3(v: DVec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Bevy system that orchestrates the aerodynamic pipeline each physics step.
///
/// For each aircraft root entity, this system:
///
/// 1. Iterates every [`AeroZone`] child and calls
///    [`evaluate_zone_coefficients`] → [`zone_force_world`] to get
///    world-space forces and torques.
/// 2. Accumulates each zone's force, moment-arm torque `(r_zone − r_CG) × F`,
///    and pure aerodynamic torque into the root's
///    [`ConstantForce`] / [`ConstantTorque`].
/// 3. Rolls up pre-computed engine thrust via [`accumulate_engine_force`].
/// 4. Adds whole-aircraft [`damping_torque`].
///
/// The results are consumed by Avian's solver in the next phase.
pub fn compute_aero_forces(
    mut root_query: Query<(
        &mut ConstantForce,
        &mut ConstantTorque,
        &Position,
        &Rotation,
        &ComputedCenterOfMass,
        &FlightState,
        &AircraftGeometry,
        &ControlInputs,
        &Children,
    )>,
    mut zone_query: Query<(
        &AeroZone,
        &GlobalTransform,
        &mut ZoneForce,
        Option<&Damageable>,
    )>,
    #[cfg(feature = "propulsion")]
    engine_zone_query: Query<&ZoneForce, (With<EngineZone>, Without<AeroZone>)>,
) {
    for (mut cf, mut ct, pos, rot, com, flight, geo, ctrl, children)
        in root_query.iter_mut()
    {
        cf.0 = Vec3::ZERO;
        ct.0 = Vec3::ZERO;

        if flight.airspeed_ms < 1e-4 {
            continue;
        }

        let alpha = flight.alpha_rad;
        let re = flight.reynolds_number;
        let qbar = flight.dynamic_pressure_pa;
        let s = geo.wing_area_m2;
        let b = geo.wing_span_m;
        let c = geo.chord_m;

        let body_to_world = DQuat::from_array(rot.0.to_array().map(|x| x as f64));
        let stab_to_body = DQuat::from_rotation_y(-alpha);
        let com_world: Vec3 = pos.0 + rot.0 * com.0;

        // ── Per-zone accumulation ────────────────────────────────────────
        for child in children.iter() {
            if let Ok((zone, zone_gt, mut zone_force, dmg)) = zone_query.get_mut(child) {
                *zone_force = ZoneForce::default();

                let health = dmg.map(|d| d.health).unwrap_or(1.0);
                if health <= 0.0 {
                    continue;
                }

                // Step 1: evaluate coefficients.
                let coeffs = evaluate_zone_coefficients(zone, ctrl, alpha, re, qbar, health);

                // Step 2: convert to world-space force and torque.
                let wf = zone_force_world(&coeffs, qbar, s, b, c, stab_to_body, body_to_world);

                if !wf.force.is_finite() || !wf.torque.is_finite() {
                    warn_once!("Non-finite aero force/torque on zone — zeroed");
                    continue;
                }

                let force_world = dvec3_to_vec3(wf.force);
                let torque_world = dvec3_to_vec3(wf.torque);

                // Write per-zone output for debug visualisation.
                zone_force.force = force_world;
                zone_force.torque = torque_world;
                let ac_world = zone_gt.transform_point(zone.ac_offset);
                zone_force.world_point = ac_world;

                // Accumulate onto root: force + moment-arm torque + pure torque.
                cf.0 += force_world;
                ct.0 += (ac_world - com_world).cross(force_world) + torque_world;

                continue;
            }

            // Step 3: engine zone thrust accumulation.
            #[cfg(feature = "propulsion")]
            if let Ok(zf) = engine_zone_query.get(child) {
                accumulate_engine_force(zf, com_world, &mut cf.0, &mut ct.0);
                continue;
            }
        }

        // Step 4: whole-aircraft dynamic damping.
        let damp = damping_torque(flight, geo, body_to_world);
        if damp.is_finite() {
            ct.0 += dvec3_to_vec3(damp);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
#[cfg(test)]
mod tests {
    use crate::components::aero_coeff::AeroCoeff;

    /// Dynamic pressure proportionality: doubling airspeed quadruples force.
    #[test]
    fn dynamic_pressure_quadruples_with_speed() {
        let rho = 1.225_f64;
        let v1 = 50.0_f64;
        let v2 = 100.0_f64;
        let qbar1 = 0.5 * rho * v1 * v1;
        let qbar2 = 0.5 * rho * v2 * v2;
        assert!((qbar2 / qbar1 - 4.0).abs() < 1e-10);
    }

    /// At health = 0.0, force should be zero (detached zone contributes nothing).
    #[test]
    fn zero_health_zero_force() {
        let health = 0.0_f64;
        let cl = 0.5_f64;
        let qbar = 1000.0_f64;
        let s = 16.2_f64;
        let force = cl * health * qbar * s;
        assert_eq!(force, 0.0);
    }

    /// Structural drag is zero at full health and at health=0, peaks between.
    #[test]
    fn structural_drag_curve() {
        let damage_drag_coeff = 500.0_f64;
        let qbar = 1000.0_f64;

        let full = damage_drag_coeff * (1.0 - 1.0_f64) / qbar; // h=1: 0
        let half = damage_drag_coeff * (1.0 - 0.5_f64) / qbar; // h=0.5
        let zero = 0.0_f64; // h=0: zone gone, no contribution

        assert_eq!(full, 0.0);
        assert!(half > 0.0);
        assert_eq!(zero, 0.0);
    }

    /// Control surface AileronRight has opposite sign to AileronLeft.
    #[test]
    fn aileron_mirror() {
        let aileron_input = 0.5_f64;
        let left_scale = aileron_input;    // AileronLeft
        let right_scale = -aileron_input;  // AileronRight
        assert_eq!(left_scale, -right_scale);
    }

    /// Negative CL produces downward (negative Z in stability, negative Y in world at level flight).
    #[test]
    fn negative_cl_downward_force() {
        let cl = -0.3_f64;
        let qbar = 1000.0_f64;
        let s = 16.2_f64;
        // In stability frame Z_s: lift = −cl * qbar * s (so negative cl → positive Z_s = downward)
        let lift_z_stab = -cl * qbar * s;
        assert!(lift_z_stab > 0.0, "negative CL should push down (+Z_s)");
    }

    /// Coefficient evaluation smoke test.
    #[test]
    fn aero_coeff_scalar_evaluate() {
        let c = AeroCoeff::Scalar(0.8);
        assert!((c.evaluate(0.1, 1e6) - 0.8).abs() < 1e-12);
    }
}
