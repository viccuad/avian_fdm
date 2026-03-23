//! Aerodynamic force pipeline.
//!
//! The pipeline is built from small, documented pure functions that each handle
//! one step of the aerodynamic computation.  The Bevy system
//! [`compute_aero_forces`] orchestrates them in order:
//!
//! ```text
//! For each aircraft root:
//!   For each AeroZone child:
//!     0. zone_local_angles           — per-zone effective α/β from body rates
//!                                      (roll/pitch/yaw-rate corrections)
//!     1. evaluate_zone_coefficients  — lookup CL/CD/… tables at local α/β,
//!                                      apply control-surface scaling and damage
//!     2. zone_force_world            — rotate stability-frame forces to world,
//!                                      compute pure aerodynamic torques
//!   For each EngineZone child:
//!     3. accumulate_engine_force     — add pre-computed thrust (from propulsion
//!                                      system) and its moment-arm torque
//!   Once per aircraft:
//!     4. induced_drag                — whole-aircraft CD_i = CL²/(π · e · AR)
//!     5. damping_torque (LOD fallback) — only when `lod_damping` is `Some`;
//!                                        skipped for full-fidelity zone layouts
//! ```
//!
//! ## Why per-zone local angles matter
//!
//! With per-zone local α/β (step 0), asymmetric stall, snap rolls, spins,
//! adverse yaw, Dutch roll, and pitch-rate tail authority limits **emerge
//! naturally** from zone geometry — no special-case logic needed.
//!
//! The two modes are **mutually exclusive**:
//!
//! | `lod_damping` | Step 0 | Step 5 | Best for |
//! |---|---|---|---|
//! | `None` (default) | per-zone local α/β | skipped | full-zone aircraft |
//! | `Some(LodDamping)` | global α/β only | derivatives applied | sparse-zone bodies |
//!
//! Each function's doc comment explains the physics behind that step, so reading
//! them top-to-bottom gives a self-contained introduction to how the FDM
//! converts aerodynamic data into Avian forces.

use avian3d::prelude::{ComputedCenterOfMass, ConstantForce, ConstantTorque, Position, Rotation};
use bevy::math::{DQuat, DVec3};
use bevy::prelude::*;

#[cfg(feature = "propulsion")]
use crate::components::EngineZone;
use crate::components::{
    get_remaining, AeroZone, AircraftGeometry, ControlInputs, ControlSurfaceRole, Failure,
    FlightState, InducedDrag, LodDamping, ZoneForce,
};
use crate::math::to_dvec3;

// ── Step 0: Per-zone local angle of attack and sideslip ──────────────────────

/// Compute the local effective angle of attack and sideslip for a zone.
///
/// Body angular rates create local velocity increments at each zone that shift
/// its effective angle of attack and sideslip beyond the whole-aircraft values.
/// Three additive correction layers are applied:
///
/// **Layer 1 — Roll-rate Δα (asymmetric stall, snap rolls, spins)**
///
/// A zone at spanwise station `y` (metres, positive starboard) sees a
/// body-Z velocity increment `Δw = p · y` from roll rate `p` (rad/s).
///
/// ```text
/// Δα_roll = p · y / V
/// ```
///
/// At p = 1 rad/s, V = 50 m/s, y = +4.57 m (J3 Cub wing tip): Δα ≈ +5.2°.
/// When the aircraft is near stall, the descending tip stalls while the rising
/// tip keeps flying — producing uncommanded roll that can steepen into a spin.
///
/// **Layer 2 — Pitch-rate Δα (tail authority limits, CG sensitivity)**
///
/// A zone at longitudinal station `x` (metres, positive forward from CG) sees
/// a body-Z velocity increment from pitch rate `q` (rad/s):
///
/// ```text
/// Δα_pitch = q · x / V
/// ```
///
/// The horizontal stabiliser at x ≈ −4 m sees *reduced* AoA during a pull
/// (q > 0), limiting tail authority at high pitch rates and reproducing the
/// pitch-up departure tendency naturally.
///
/// **Layer 3 — Yaw-rate Δβ (adverse yaw, Dutch roll)**
///
/// A zone at spanwise station `y` sees a body-Y velocity increment from yaw
/// rate `r` (rad/s):
///
/// ```text
/// Δβ_yaw = r · y / V
/// ```
///
/// Creates asymmetric drag across the span (adverse yaw from differential
/// profile drag) and is the primary driver of Dutch-roll dynamics.
///
/// # Arguments
///
/// - `alpha`: whole-aircraft angle of attack (rad).
/// - `beta`: whole-aircraft sideslip (rad).
/// - `p`, `q`, `r`: body-axis angular rates (rad/s).
/// - `x`: zone body-frame longitudinal station (m) from CG; positive forward.
/// - `y`: zone body-frame spanwise station (m) from CG; positive starboard.
/// - `v`: true airspeed (m/s); must be > 0.
///
/// # Returns
///
/// `(alpha_local, beta_local)` — both in radians.
pub fn zone_local_angles(
    alpha: f64,
    beta: f64,
    p: f64,
    q: f64,
    r: f64,
    x: f64,
    y: f64,
    v: f64,
) -> (f64, f64) {
    let alpha_local = alpha + (p * y + q * x) / v;
    let beta_local = beta + (r * y) / v;
    (alpha_local, beta_local)
}

// ── Step 1: Coefficient evaluation ───────────────────────────────────────────

/// Six non-dimensional aerodynamic coefficients, fully scaled and ready to be
/// multiplied by dynamic pressure and reference area to produce forces.
pub(crate) struct ZoneCoefficients {
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
/// 1. **Table lookup** — CL, CD, CM, Croll, Cn are evaluated at `alpha_local`
///    (the per-zone effective angle of attack, already corrected for body rates
///    via [`zone_local_angles`]).  CY (side force) is evaluated at `beta_local`
///    (the per-zone effective sideslip).  Tables may be a constant (`Scalar`),
///    a 1-D function of the primary angle (`Table1D`), or a 2-D function of
///    (angle, Re) (`Table2D`).
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
/// 3. **Failure degradation** — All coefficients are multiplied by
///    `remaining ∈ (0, 1]`.  A wing at 0.5 remaining produces half the lift.
///    Additionally, structural deformation adds parasitic drag that peaks at
///    intermediate failure:
///
///    ```text
///    CD_effective = (CD_base · |input| + damage_drag · (1 − remaining) / q̄) · remaining
///    ```
///
///    At `remaining = 0` the zone is fully detached and produces no force at all
///    (handled by the caller, not this function).
pub(crate) fn evaluate_zone_coefficients(
    zone: &AeroZone,
    ctrl: &ControlInputs,
    alpha_local: f64,
    beta_local: f64,
    re: f64,
    qbar: f64,
    remaining: f64,
) -> ZoneCoefficients {
    // Raw table lookups at the current flight condition.
    // CL, CD depend on local angle of attack.
    // CY (side force) depends on local sideslip.
    // Absent/Placeholder/data variants are all handled by .evaluate() directly.
    let cl_base = zone.cl.evaluate(alpha_local, re);
    let cd_base = zone.cd.evaluate(alpha_local, re);
    let cy_base = zone.cy.evaluate(beta_local, re);
    let cm_base = zone.cm.evaluate(alpha_local, re);
    let croll_base = zone.croll.evaluate(alpha_local, re);
    let cn_base = zone.cn.evaluate(alpha_local, re);

    // Control-surface scaling.
    let (scale, cd_scale) = match &zone.control_role {
        Some(ControlSurfaceRole::Elevator) => (ctrl.elevator, ctrl.elevator.abs()),
        Some(ControlSurfaceRole::AileronLeft) => (ctrl.aileron, ctrl.aileron.abs()),
        Some(ControlSurfaceRole::AileronRight) => (-ctrl.aileron, ctrl.aileron.abs()),
        Some(ControlSurfaceRole::Rudder) => (ctrl.rudder, ctrl.rudder.abs()),
        None => (1.0, 1.0),
    };

    // Failure degradation: structural deformation drag grows as remaining → 0.
    // Only computed when the zone has a damage-drag model (most zones don't).
    let extra_cd = zone
        .damage_drag_coeff
        .map(|coeff| coeff * (1.0 - remaining) / qbar.max(1e-4))
        .unwrap_or(0.0);

    ZoneCoefficients {
        cl: cl_base * scale * remaining,
        cd: (cd_base * cd_scale + extra_cd) * remaining,
        cy: cy_base * scale * remaining,
        cm: cm_base * scale * remaining,
        croll: croll_base * scale * remaining,
        cn: cn_base * scale * remaining,
    }
}

// ── Step 2: Stability-frame forces → world ───────────────────────────────────

/// World-space force and torque produced by a single zone.
pub(crate) struct ZoneWorldForce {
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
pub(crate) fn zone_force_world(
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
/// This is an **LOD fallback** for aircraft with too few zones to produce
/// realistic physical damping from geometry alone.  At full fidelity, call
/// [`zone_local_angles`] per zone instead — the differential forces from wing
/// and tail zones naturally oppose body rates without any explicit derivatives.
///
/// # When to use
///
/// Supply a [`LodDamping`](crate::components::LodDamping) value only for:
/// - Single-zone bodies (missiles, pylons)
/// - Low-fidelity AI aircraft with minimal zone layouts
///
/// For any aircraft with realistic wing, h-stab, and v-tail zones, leave
/// `lod_damping = None` and let zone physics do the work.
///
/// # How it works
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
/// ```text
/// ΔL = Cl_p · (p · b / 2V) · q̄ · S · b     roll  damping → body X
/// ΔM = Cm_q · (q · c̄ / 2V) · q̄ · S · c̄    pitch damping → body Y
/// ΔN = Cn_r · (r · b / 2V) · q̄ · S · b     yaw   damping → body Z
/// ```
///
/// The `(rate × length / 2V)` term is the non-dimensional rate.  Multiplying
/// by `q̄ · S · length` recovers a dimensional moment in N·m.
///
/// Typical values for a light GA aircraft (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.  All are negative
/// because damping opposes motion.
pub fn damping_torque(
    flight: &FlightState,
    lod: &crate::components::LodDamping,
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
        lod.cl_p * pb_2v * qbar * s * b,
        lod.cm_q * qc_2v * qbar * s * c,
        lod.cn_r * rb_2v * qbar * s * b,
    );

    body_to_world * damp_body
}

// ── Orchestrator system ──────────────────────────────────────────────────────
/// Bevy system that orchestrates the aerodynamic pipeline each physics step.
///
/// The two fidelity modes are **mutually exclusive**, selected by whether
/// [`AircraftGeometry::lod_damping`] is `Some`:
///
/// **Full-fidelity mode** (`lod_damping = None`):
/// - Step 0 applies per-zone local α/β corrections from body rates.
/// - Roll/pitch/yaw damping emerges from differential zone forces.
/// - Step 5 is skipped.
///
/// **LOD mode** (`lod_damping = Some(…)`):
/// - Step 0 is skipped; all zones evaluate at the global α/β.
/// - Step 5 applies explicit `Cl_p`/`Cm_q`/`Cn_r` derivatives as the sole
///   source of damping.
///
/// Steps common to both modes:
/// 1. [`evaluate_zone_coefficients`] → [`zone_force_world`] per zone.
/// 2. Moment-arm torques `(r_zone − r_CG) × F` accumulated.
/// 3. Pre-computed engine thrust via [`accumulate_engine_force`].
/// 4. Whole-aircraft induced drag CD_i = CL²/(π · e · AR).
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
        Option<&LodDamping>,
        Option<&InducedDrag>,
    )>,
    mut zone_query: Query<(
        &AeroZone,
        &GlobalTransform,
        &mut ZoneForce,
        Option<&Failure>,
    )>,
    #[cfg(feature = "propulsion")] engine_zone_query: Query<
        &ZoneForce,
        (With<EngineZone>, Without<AeroZone>),
    >,
) {
    for (mut cf, mut ct, pos, rot, com, flight, geo, ctrl, children, lod_damping, induced_drag) in
        root_query.iter_mut()
    {
        cf.0 = Vec3::ZERO;
        ct.0 = Vec3::ZERO;

        if flight.airspeed_ms < 1e-4 {
            continue;
        }

        let alpha = flight.alpha_rad;
        let beta = flight.beta_rad;
        let re = flight.reynolds_number;
        let qbar = flight.dynamic_pressure_pa;
        let v = flight.airspeed_ms;
        let p = flight.p_rads;
        let q = flight.q_rads;
        let r = flight.r_rads;
        let s = geo.wing_area_m2;
        let b = geo.wing_span_m;
        let c = geo.chord_m;

        let body_to_world = DQuat::from_array(rot.0.to_array().map(|x| x as f64));
        // Global stab_to_body (at whole-aircraft α) — used for induced drag and LOD mode.
        let stab_to_body_global = DQuat::from_rotation_y(-alpha);
        let com_world: Vec3 = pos.0 + rot.0 * com.0;
        // CG in world space as DVec3, used to measure zone moment arms.
        let cg_world = to_dvec3(com_world);

        // LOD mode: when the `LodDamping` component is present, zones evaluate
        // at the global α/β (no per-zone corrections) and `damping_torque` owns
        // all damping.  When absent, per-zone local angles run and damping
        // emerges from differential zone forces.
        let use_lod = lod_damping.is_some();

        // ── Per-zone accumulation ────────────────────────────────────────
        let mut total_cl = 0.0_f64; // sum of zone CL contributions (for induced drag)

        for child in children.iter() {
            if let Ok((zone, zone_gt, mut zone_force, opt_failure)) = zone_query.get_mut(child) {
                *zone_force = ZoneForce::default();

                let remaining = get_remaining(opt_failure);
                if remaining <= 0.0 {
                    continue;
                }

                // Step 0: per-zone local α/β — skipped in LOD mode.
                // In full-fidelity mode, measure the zone's body-frame position
                // relative to the CG and apply roll/pitch/yaw-rate corrections.
                let (alpha_local, beta_local, stab_to_body_local) = if use_lod {
                    (alpha, beta, stab_to_body_global)
                } else {
                    let zone_world_pos = zone_gt.translation();
                    let zone_rel_world = to_dvec3(zone_world_pos) - cg_world;
                    let zone_body = body_to_world.inverse() * zone_rel_world;
                    let (al, bl) =
                        zone_local_angles(alpha, beta, p, q, r, zone_body.x, zone_body.y, v);
                    (al, bl, DQuat::from_rotation_y(-al))
                };

                // Step 1: evaluate coefficients at local α/β.
                let coeffs = evaluate_zone_coefficients(
                    zone,
                    ctrl,
                    alpha_local,
                    beta_local,
                    re,
                    qbar,
                    remaining,
                );
                total_cl += coeffs.cl;

                // Step 2: convert to world-space force and torque.
                let wf =
                    zone_force_world(&coeffs, qbar, s, b, c, stab_to_body_local, body_to_world);

                if !wf.force.is_finite() || !wf.torque.is_finite() {
                    warn_once!("Non-finite aero force/torque on zone — zeroed");
                    continue;
                }

                let force_world = wf.force.as_vec3();
                let torque_world = wf.torque.as_vec3();

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

        // Step 4: induced drag — CD_i = CL² / (π · e · AR).
        //
        // Only applied when the `InducedDrag` component is present.
        // Absent for gliders (CD already in polar), missiles, or aircraft
        // whose zone CDs already include induced drag.
        if let Some(id) = induced_drag {
            let ar = b * b / s;
            let cd_i = total_cl * total_cl / (std::f64::consts::PI * id.oswald_factor * ar);
            let drag_i = cd_i * qbar * s;
            let drag_stab = DVec3::new(-drag_i, 0.0, 0.0);
            let drag_world = body_to_world * (stab_to_body_global * drag_stab);
            cf.0 += drag_world.as_vec3();
        }

        // Step 5: global damping torque — LOD mode only.
        // Mutually exclusive with per-zone local angles (step 0): when the
        // `LodDamping` component is present, zones evaluated at global α/β
        // produce no emergent damping, so the derivatives here are the sole source.
        if let Some(lod) = lod_damping {
            let damp = damping_torque(flight, lod, geo, body_to_world);
            if damp.is_finite() {
                ct.0 += damp.as_vec3();
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::aero_coeff::AeroCoeff;

    /// Build a minimal AeroZone with the given CL/CD scalars and no control role.
    fn simple_zone(cl: f64, cd: f64) -> AeroZone {
        AeroZone {
            cl: AeroCoeff::Scalar(cl),
            cd: AeroCoeff::Scalar(cd),
            ..Default::default()
        }
    }

    fn neutral_controls() -> ControlInputs {
        ControlInputs {
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
            throttle: 0.0,
        }
    }

    // ── evaluate_zone_coefficients ────────────────────────────────────────

    #[test]
    fn zero_remaining_produces_zero_coefficients() {
        let zone = simple_zone(0.5, 0.03);
        let coeffs =
            evaluate_zone_coefficients(&zone, &neutral_controls(), 0.1, 0.0, 1e6, 1000.0, 0.0);
        assert_eq!(coeffs.cl, 0.0);
        assert_eq!(coeffs.cd, 0.0);
    }

    #[test]
    fn half_remaining_halves_lift() {
        let zone = simple_zone(1.0, 0.0);
        let full =
            evaluate_zone_coefficients(&zone, &neutral_controls(), 0.1, 0.0, 1e6, 1000.0, 1.0);
        let half =
            evaluate_zone_coefficients(&zone, &neutral_controls(), 0.1, 0.0, 1e6, 1000.0, 0.5);
        assert!((half.cl - full.cl * 0.5).abs() < 1e-12);
    }

    #[test]
    fn damage_drag_peaks_at_intermediate_remaining() {
        let mut zone = simple_zone(0.0, 0.03);
        zone.damage_drag_coeff = Some(500.0);
        let qbar = 1000.0;

        let full = evaluate_zone_coefficients(&zone, &neutral_controls(), 0.0, 0.0, 1e6, qbar, 1.0);
        let half = evaluate_zone_coefficients(&zone, &neutral_controls(), 0.0, 0.0, 1e6, qbar, 0.5);
        // At remaining=1: no damage drag.  At remaining=0.5: extra drag present.
        assert!(
            half.cd > full.cd * 0.5,
            "damage drag should add extra at intermediate remaining"
        );
    }

    #[test]
    fn elevator_scales_lift_by_input() {
        let mut zone = simple_zone(1.0, 0.0);
        zone.control_role = Some(ControlSurfaceRole::Elevator);

        let ctrl_half = ControlInputs {
            elevator: 0.5,
            ..neutral_controls()
        };
        let ctrl_full = ControlInputs {
            elevator: 1.0,
            ..neutral_controls()
        };

        let c_half = evaluate_zone_coefficients(&zone, &ctrl_half, 0.1, 0.0, 1e6, 1000.0, 1.0);
        let c_full = evaluate_zone_coefficients(&zone, &ctrl_full, 0.1, 0.0, 1e6, 1000.0, 1.0);
        assert!((c_half.cl - c_full.cl * 0.5).abs() < 1e-12);
    }

    #[test]
    fn aileron_right_mirrors_left() {
        let cl_val = 0.8;
        let mut zone_l = simple_zone(cl_val, 0.0);
        zone_l.control_role = Some(ControlSurfaceRole::AileronLeft);
        let mut zone_r = simple_zone(cl_val, 0.0);
        zone_r.control_role = Some(ControlSurfaceRole::AileronRight);

        let ctrl = ControlInputs {
            aileron: 0.6,
            ..neutral_controls()
        };
        let cl_left = evaluate_zone_coefficients(&zone_l, &ctrl, 0.1, 0.0, 1e6, 1000.0, 1.0).cl;
        let cl_right = evaluate_zone_coefficients(&zone_r, &ctrl, 0.1, 0.0, 1e6, 1000.0, 1.0).cl;
        assert!(
            (cl_left + cl_right).abs() < 1e-12,
            "ailerons should produce opposite CL"
        );
    }

    #[test]
    fn control_deflection_always_increases_drag() {
        let mut zone = simple_zone(0.0, 0.05);
        zone.control_role = Some(ControlSurfaceRole::Elevator);

        let neutral =
            evaluate_zone_coefficients(&zone, &neutral_controls(), 0.0, 0.0, 1e6, 1000.0, 1.0);
        let ctrl_pos = ControlInputs {
            elevator: 0.8,
            ..neutral_controls()
        };
        let deflected = evaluate_zone_coefficients(&zone, &ctrl_pos, 0.0, 0.0, 1e6, 1000.0, 1.0);
        assert!(
            deflected.cd >= neutral.cd * 0.8 - 1e-12,
            "deflection should not reduce drag"
        );
    }

    // ── zone_force_world ─────────────────────────────────────────────────

    #[test]
    fn lift_opposes_gravity_at_zero_alpha() {
        let coeffs = ZoneCoefficients {
            cl: 1.0,
            cd: 0.0,
            cy: 0.0,
            cm: 0.0,
            croll: 0.0,
            cn: 0.0,
        };
        // At α=0 and level flight: stab_to_body is identity, body_to_world
        // rotates body-Z(down) to world-−Y(down), so lift (−Z_s) → world +Y.
        let stab_to_body = DQuat::IDENTITY;
        let body_to_world = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);

        let wf = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            stab_to_body,
            body_to_world,
        );
        assert!(
            wf.force.y > 0.0,
            "lift should point up (+Y world), got {}",
            wf.force.y
        );
    }

    #[test]
    fn drag_opposes_forward_motion() {
        let coeffs = ZoneCoefficients {
            cl: 0.0,
            cd: 1.0,
            cy: 0.0,
            cm: 0.0,
            croll: 0.0,
            cn: 0.0,
        };
        let stab_to_body = DQuat::IDENTITY;
        let body_to_world = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);

        let wf = zone_force_world(
            &coeffs,
            1000.0,
            16.0,
            10.0,
            1.6,
            stab_to_body,
            body_to_world,
        );
        // Body X = forward = world X after the 90° rotation.  Drag is −X_s.
        assert!(
            wf.force.x < 0.0,
            "drag should oppose forward motion, got {}",
            wf.force.x
        );
    }

    #[test]
    fn force_scales_with_dynamic_pressure() {
        let coeffs = ZoneCoefficients {
            cl: 0.5,
            cd: 0.0,
            cy: 0.0,
            cm: 0.0,
            croll: 0.0,
            cn: 0.0,
        };
        let stab = DQuat::IDENTITY;
        let b2w = DQuat::IDENTITY;

        let wf1 = zone_force_world(&coeffs, 500.0, 16.0, 10.0, 1.6, stab, b2w);
        let wf2 = zone_force_world(&coeffs, 2000.0, 16.0, 10.0, 1.6, stab, b2w);
        // q̄ ratio is 4:1, so force ratio should be 4:1.
        let ratio = wf2.force.length() / wf1.force.length();
        assert!(
            (ratio - 4.0).abs() < 1e-10,
            "force should scale with q̄, ratio = {ratio}"
        );
    }

    #[test]
    fn pitching_moment_uses_chord_not_span() {
        let coeffs = ZoneCoefficients {
            cl: 0.0,
            cd: 0.0,
            cy: 0.0,
            cm: 1.0,
            croll: 0.0,
            cn: 0.0,
        };
        let b2w = DQuat::IDENTITY;
        let stab = DQuat::IDENTITY;

        let wf_a = zone_force_world(&coeffs, 1000.0, 1.0, 10.0, 2.0, stab, b2w);
        let wf_b = zone_force_world(&coeffs, 1000.0, 1.0, 10.0, 4.0, stab, b2w);
        // Pitching moment ∝ chord; doubling chord should double torque.
        let ratio = wf_b.torque.y / wf_a.torque.y;
        assert!(
            (ratio - 2.0).abs() < 1e-10,
            "CM should scale with chord, ratio = {ratio}"
        );
    }

    // ── damping_torque ───────────────────────────────────────────────────

    #[test]
    fn roll_damping_opposes_roll_rate() {
        let flight = FlightState {
            p_rads: 1.0,
            q_rads: 0.0,
            r_rads: 0.0,
            airspeed_ms: 50.0,
            dynamic_pressure_pa: 1531.0,
            alpha_rad: 0.0,
            beta_rad: 0.0,
            mach: 0.15,
            reynolds_number: 3e6,
            altitude_m: 0.0,
        };
        let lod = crate::components::LodDamping {
            cl_p: -0.45,
            cm_q: 0.0,
            cn_r: 0.0,
        };
        let geo = AircraftGeometry {
            wing_span_m: 10.0,
            chord_m: 1.6,
            wing_area_m2: 16.0,
        };
        let damp = damping_torque(&flight, &lod, &geo, DQuat::IDENTITY);
        // p > 0 and cl_p < 0 → roll damping moment should be negative (opposes roll).
        assert!(
            damp.x < 0.0,
            "roll damping should oppose positive p, got {}",
            damp.x
        );
    }

    #[test]
    fn zero_rates_produce_zero_damping() {
        let flight = FlightState {
            p_rads: 0.0,
            q_rads: 0.0,
            r_rads: 0.0,
            airspeed_ms: 50.0,
            dynamic_pressure_pa: 1531.0,
            ..Default::default()
        };
        let lod = crate::components::LodDamping {
            cl_p: -0.45,
            cm_q: -12.0,
            cn_r: -0.12,
        };
        let geo = AircraftGeometry {
            wing_span_m: 10.0,
            chord_m: 1.6,
            wing_area_m2: 16.0,
        };
        let damp = damping_torque(&flight, &lod, &geo, DQuat::IDENTITY);
        assert!(
            damp.length() < 1e-10,
            "zero rates should produce zero damping"
        );
    }

    // ── AeroCoeff (kept from original) ───────────────────────────────────

    #[test]
    fn aero_coeff_scalar_evaluate() {
        let c = AeroCoeff::Scalar(0.8);
        assert!((c.evaluate(0.1, 1e6) - 0.8).abs() < 1e-12);
    }

    // ── zone_local_angles ────────────────────────────────────────────────

    /// Layer 1: positive roll rate + positive spanwise station → increased local α.
    #[test]
    fn roll_rate_increases_alpha_at_positive_y() {
        // p = 1 rad/s, y = 4 m (starboard tip), V = 50 m/s → Δα = +0.08 rad
        let (alpha_l, beta_l) = zone_local_angles(0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0, 50.0);
        let expected = 0.1 + 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < 1e-12,
            "Δα_roll should be p · y/V, got {alpha_l}"
        );
        assert!(
            (beta_l - 0.0).abs() < 1e-12,
            "roll rate should not affect β"
        );
    }

    /// Layer 1: negative spanwise station (port side) gets reduced α under roll.
    #[test]
    fn roll_rate_decreases_alpha_at_negative_y() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 1.0, 0.0, 0.0, 0.0, -4.0, 50.0);
        let expected = 0.1 - 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < 1e-12,
            "port wing tip should see reduced α under positive roll"
        );
    }

    /// Layer 2: positive pitch rate + negative longitudinal station (tail) → decreased α.
    /// This reproduces tail authority limits: pulling hard reduces tail restoring moment.
    #[test]
    fn pitch_rate_decreases_alpha_at_tail() {
        // q = 1 rad/s, x = -4 m (aft of CG), V = 50 m/s → Δα = -0.08 rad
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 0.0, 1.0, 0.0, -4.0, 0.0, 50.0);
        let expected = 0.1 - 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < 1e-12,
            "tail should see reduced α during pull (q > 0), got {alpha_l}"
        );
    }

    /// Layer 2: positive pitch rate + positive longitudinal station (nose) → increased α.
    #[test]
    fn pitch_rate_increases_alpha_at_nose() {
        let (alpha_l, _) = zone_local_angles(0.1, 0.0, 0.0, 1.0, 0.0, 4.0, 0.0, 50.0);
        let expected = 0.1 + 4.0 / 50.0;
        assert!(
            (alpha_l - expected).abs() < 1e-12,
            "nose should see increased α during pull (q > 0), got {alpha_l}"
        );
    }

    /// Layer 3: yaw rate shifts sideslip proportional to spanwise station.
    #[test]
    fn yaw_rate_shifts_beta_at_spanwise_station() {
        // r = 1 rad/s, y = 3 m, V = 50 m/s → Δβ = +0.06 rad
        let (alpha_l, beta_l) = zone_local_angles(0.0, 0.05, 0.0, 0.0, 1.0, 0.0, 3.0, 50.0);
        let expected_beta = 0.05 + 3.0 / 50.0;
        assert!(
            (beta_l - expected_beta).abs() < 1e-12,
            "Δβ_yaw should be r · y/V, got {beta_l}"
        );
        assert!(
            (alpha_l - 0.0).abs() < 1e-12,
            "yaw rate should not affect α"
        );
    }

    /// Zero body rates → local angles equal global angles.
    #[test]
    fn zero_rates_leave_angles_unchanged() {
        let (a, b) = zone_local_angles(0.2, 0.1, 0.0, 0.0, 0.0, -2.0, 3.0, 30.0);
        assert!((a - 0.2).abs() < 1e-12, "zero rates should not alter α");
        assert!((b - 0.1).abs() < 1e-12, "zero rates should not alter β");
    }

    /// All three layers combine additively.
    #[test]
    fn all_layers_combine_additively() {
        // p=1, q=1, r=1; x=2, y=3; V=10
        // α_local = α + (p · y + q · x)/V = 0.1 + (3+2)/10 = 0.6
        // β_local = β + r · y/V = 0.05 + 3/10 = 0.35
        let (a, b) = zone_local_angles(0.1, 0.05, 1.0, 1.0, 1.0, 2.0, 3.0, 10.0);
        assert!((a - 0.6).abs() < 1e-12, "combined α layers, got {a}");
        assert!((b - 0.35).abs() < 1e-12, "combined β layers, got {b}");
    }

    /// Emergent roll damping: symmetric zones at ±y with a linear-CL table
    /// produce differential lift under roll rate that opposes the roll.
    #[test]
    fn emergent_roll_damping_from_symmetric_zones() {
        // Wing zone with a simple linear CL curve: CL = 5 · α (slope ~0.088/deg)
        let zone = AeroZone {
            cl: AeroCoeff::Table1D {
                breakpoints: vec![-0.5, 0.5],
                values: vec![-2.5, 2.5],
            },
            cd: AeroCoeff::Scalar(0.0),
            ..Default::default()
        };
        let ctrl = neutral_controls();
        let alpha = 0.1_f64; // 5.7° — well below stall
        let v = 50.0_f64;
        let p = 1.0_f64; // positive roll rate (right wing down)
        let re = 1e6;
        let qbar = 1500.0;

        // Starboard zone at y = +4 m
        let (al_r, bl_r) = zone_local_angles(alpha, 0.0, p, 0.0, 0.0, 0.0, 4.0, v);
        let cr = evaluate_zone_coefficients(&zone, &ctrl, al_r, bl_r, re, qbar, 1.0);

        // Port zone at y = −4 m
        let (al_l, bl_l) = zone_local_angles(alpha, 0.0, p, 0.0, 0.0, 0.0, -4.0, v);
        let cl = evaluate_zone_coefficients(&zone, &ctrl, al_l, bl_l, re, qbar, 1.0);

        // Starboard (descending) tip sees more lift; port (ascending) tip sees less.
        assert!(
            cr.cl > cl.cl,
            "starboard tip should have higher CL under positive roll: {:.3} vs {:.3}",
            cr.cl,
            cl.cl
        );

        // Both zones produce a lift force in world frame.
        // World: body_to_world = identity; stab_to_body = identity (α_local ≈ 0 for small Δ)
        let stab_r = DQuat::from_rotation_y(-al_r);
        let stab_l = DQuat::from_rotation_y(-al_l);
        let b2w = DQuat::IDENTITY;
        let wf_r = zone_force_world(&cr, qbar, 16.0, 10.0, 1.6, stab_r, b2w);
        let wf_l = zone_force_world(&cl, qbar, 16.0, 10.0, 1.6, stab_l, b2w);

        // Moment arm: starboard at +y=4, port at -y=4 (in body frame = world here).
        let arm_r = DVec3::new(0.0, 4.0, 0.0);
        let arm_l = DVec3::new(0.0, -4.0, 0.0);
        let roll_r = arm_r.cross(wf_r.force).x;
        let roll_l = arm_l.cross(wf_l.force).x;
        let net_roll = roll_r + roll_l;

        // Net roll moment should oppose positive roll rate (i.e. be negative about X).
        assert!(
            net_roll < 0.0,
            "emergent roll damping should oppose p>0, net roll moment = {net_roll:.2} N·m"
        );
    }

    // ── accumulate_engine_force ──────────────────────────────────────────

    /// An on-centre engine (at CG) produces pure force and no moment.
    #[test]
    fn engine_at_cg_no_moment() {
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::ZERO, // coincides with CG
            torque: Vec3::ZERO,
        };
        let mut total_force = Vec3::ZERO;
        let mut total_torque = Vec3::ZERO;
        accumulate_engine_force(&zf, Vec3::ZERO, &mut total_force, &mut total_torque);

        assert!((total_force - Vec3::new(500.0, 0.0, 0.0)).length() < 1e-5);
        assert!(
            total_torque.length() < 1e-5,
            "on-axis engine must not produce torque"
        );
    }

    /// An off-centre engine (e.g. starboard twin) produces a yawing moment.
    #[test]
    fn engine_offset_right_produces_yaw_torque() {
        // Engine 2 m to the right of CG, thrusting forward (+X).
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::new(0.0, 2.0, 0.0), // +Y = starboard
            torque: Vec3::ZERO,
        };
        let com = Vec3::ZERO;
        let mut total_force = Vec3::ZERO;
        let mut total_torque = Vec3::ZERO;
        accumulate_engine_force(&zf, com, &mut total_force, &mut total_torque);

        // moment arm = (0,2,0) × (500,0,0) = (0·0 − 0·0, 0·500 − 2·0, 2·0 − 0·500)
        //            = (0, 0, -1000)  → yaw left (nose-left) torque about world -Z
        assert!(
            (total_torque.z - (-1000.0)).abs() < 1e-4,
            "starboard engine should produce nose-left yaw torque, got z={}",
            total_torque.z
        );
    }

    /// A zero-force engine produces no force and no moment (short-circuit).
    #[test]
    fn engine_zero_force_no_accumulation() {
        let zf = ZoneForce {
            force: Vec3::ZERO,
            world_point: Vec3::new(0.0, 5.0, 0.0),
            torque: Vec3::ZERO,
        };
        let mut total_force = Vec3::new(100.0, 0.0, 0.0);
        let mut total_torque = Vec3::new(0.0, 50.0, 0.0);
        accumulate_engine_force(&zf, Vec3::ZERO, &mut total_force, &mut total_torque);

        // Totals must be unchanged — zero force is the short-circuit path.
        assert!((total_force - Vec3::new(100.0, 0.0, 0.0)).length() < 1e-5);
        assert!((total_torque - Vec3::new(0.0, 50.0, 0.0)).length() < 1e-5);
    }

    // ── evaluate_zone_coefficients: missing coverage ─────────────────────

    /// Rudder scales CY (side force), not CL — the unique control axis test.
    #[test]
    fn rudder_scales_side_force() {
        let mut zone = AeroZone {
            cy: AeroCoeff::Scalar(1.0),
            ..Default::default()
        };
        zone.control_role = Some(ControlSurfaceRole::Rudder);

        let ctrl_full = ControlInputs {
            rudder: 1.0,
            ..neutral_controls()
        };
        let ctrl_half = ControlInputs {
            rudder: 0.5,
            ..neutral_controls()
        };

        let c_full = evaluate_zone_coefficients(&zone, &ctrl_full, 0.0, 0.0, 1e6, 1000.0, 1.0);
        let c_half = evaluate_zone_coefficients(&zone, &ctrl_half, 0.0, 0.0, 1e6, 1000.0, 1.0);

        // CY should be scaled by rudder input.
        assert!((c_full.cy - 1.0).abs() < 1e-12, "full rudder → CY=1");
        assert!((c_half.cy - 0.5).abs() < 1e-12, "half rudder → CY=0.5");
        // CL must be unaffected by rudder.
        assert_eq!(c_full.cl, 0.0, "rudder must not affect CL");
    }

    /// Half-damaged zone with partial control deflection: both scalings apply.
    #[test]
    fn combined_damage_and_control_deflection() {
        let mut zone = simple_zone(1.0, 0.1);
        zone.control_role = Some(ControlSurfaceRole::Elevator);
        let ctrl = ControlInputs {
            elevator: 0.5,
            ..neutral_controls()
        };

        let intact = evaluate_zone_coefficients(&zone, &ctrl, 0.0, 0.0, 1e6, 1000.0, 1.0);
        let damaged = evaluate_zone_coefficients(&zone, &ctrl, 0.0, 0.0, 1e6, 1000.0, 0.5);

        // CL: base * elevator_scale * remaining = 1.0 * 0.5 * remaining
        assert!((intact.cl - 0.5).abs() < 1e-12, "intact CL");
        assert!(
            (damaged.cl - 0.25).abs() < 1e-12,
            "half-remaining halves CL further"
        );

        // Damaged should always produce less lift than intact at same deflection.
        assert!(damaged.cl < intact.cl);
    }

    // ── damping_torque: all three axes simultaneously ─────────────────────

    #[test]
    fn all_axes_damping_combine_independently() {
        let flight = FlightState {
            p_rads: 1.0,
            q_rads: 1.0,
            r_rads: 1.0,
            airspeed_ms: 50.0,
            dynamic_pressure_pa: 1531.0,
            ..Default::default()
        };
        let lod = crate::components::LodDamping {
            cl_p: -0.45,
            cm_q: -12.0,
            cn_r: -0.12,
        };
        let geo = AircraftGeometry {
            wing_span_m: 10.0,
            chord_m: 1.6,
            wing_area_m2: 16.0,
        };
        let damp = damping_torque(&flight, &lod, &geo, DQuat::IDENTITY);

        // All three rates positive, all derivatives negative → all moments negative.
        assert!(
            damp.x < 0.0,
            "roll damping should oppose positive p, got x={}",
            damp.x
        );
        assert!(
            damp.y < 0.0,
            "pitch damping should oppose positive q, got y={}",
            damp.y
        );
        assert!(
            damp.z < 0.0,
            "yaw damping should oppose positive r, got z={}",
            damp.z
        );

        // Yaw damping (cn_r=-0.12) should be weaker than roll (cl_p=-0.45) at equal rates
        // because |cn_r| < |cl_p| and both use span b as reference length.
        assert!(
            damp.z.abs() < damp.x.abs(),
            "yaw damp should be weaker than roll (|cn_r| < |cl_p|), z={}, x={}",
            damp.z,
            damp.x
        );
    }

    /// Damping torque with non-identity rotation — moment direction rotates with aircraft.
    #[test]
    fn damping_torque_rotates_with_body() {
        let flight = FlightState {
            p_rads: 1.0,
            q_rads: 0.0,
            r_rads: 0.0,
            airspeed_ms: 50.0,
            dynamic_pressure_pa: 1531.0,
            ..Default::default()
        };
        let lod = crate::components::LodDamping {
            cl_p: -0.45,
            cm_q: 0.0,
            cn_r: 0.0,
        };
        let geo = AircraftGeometry {
            wing_span_m: 10.0,
            chord_m: 1.6,
            wing_area_m2: 16.0,
        };
        let damp_identity = damping_torque(&flight, &lod, &geo, DQuat::IDENTITY);

        // 90° roll about world X: body X stays the same, so roll damping about body X
        // should produce a torque of the same magnitude but in world X direction.
        let rot_90x = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let damp_rotated = damping_torque(&flight, &lod, &geo, rot_90x);

        // Magnitude should be equal regardless of rotation.
        let tol = 1e-5;
        assert!(
            (damp_rotated.length() - damp_identity.length()).abs() < tol,
            "rotation should not change damping magnitude"
        );
    }

    // ── AeroCoeff::Absent variant ─────────────────────────────────────────────

    #[test]
    fn absent_evaluates_to_zero_silently() {
        assert_eq!(AeroCoeff::Absent.evaluate(0.3, 1e6), 0.0);
        assert_eq!(AeroCoeff::Absent.evaluate(-0.5, 2e6), 0.0);
    }

    #[test]
    fn absent_is_absent_true() {
        assert!(AeroCoeff::Absent.is_absent());
    }

    #[test]
    fn scalar_is_absent_false() {
        assert!(!AeroCoeff::Scalar(0.0).is_absent());
    }

    #[test]
    fn placeholder_is_absent_false() {
        assert!(!AeroCoeff::Placeholder.is_absent());
    }

    #[test]
    fn absent_secondary_fields_produce_no_moment() {
        // Default AeroZone has cy/cm/croll/cn = Absent — all produce zero silently.
        let zone = AeroZone {
            cl: AeroCoeff::Scalar(1.0),
            cd: AeroCoeff::Scalar(0.1),
            ..Default::default() // cy/cm/croll/cn = Absent
        };
        let ctrl = neutral_controls();
        let coeffs = evaluate_zone_coefficients(&zone, &ctrl, 0.2, 0.0, 1e6, 1000.0, 1.0);
        assert_eq!(coeffs.cy, 0.0, "Absent cy → 0");
        assert_eq!(coeffs.cm, 0.0, "Absent cm → 0");
        assert_eq!(coeffs.croll, 0.0, "Absent croll → 0");
        assert_eq!(coeffs.cn, 0.0, "Absent cn → 0");
    }
}
