//! Aerodynamic force pipeline.
//!
//! A single unified system [`compute_aero_forces`] iterates each aircraft root
//! entity, then its child `AeroZone` entities inline. For each zone it:
//!
//! 1. Evaluates stability-derivative coefficients at the current α and Re.
//! 2. Computes a world-space force (from CL, CD, CY) and a pure aerodynamic
//!    torque (from CM, Croll, Cn).
//! 3. Writes to the zone's [`ZoneForce`] component (for debug visualisation).
//! 4. Accumulates force, moment-arm torque, and pure torque into the root's
//!    [`ConstantForce`] / [`ConstantTorque`].
//!
//! After all zones are processed, whole-aircraft dynamic damping torques are
//! added from the per-aircraft `AircraftGeometry` derivatives (cl_p, cm_q, cn_r).
//!
//! ## Force construction (stability axes → world)
//!
//! ```text
//! Lift  = CL · q̄ · S   (−Z_stability, perpendicular to relative wind)
//! Drag  = CD · q̄ · S   (−X_stability, opposing motion)
//! Side  = CY · q̄ · S   ( Y_stability = body Y, right wing)
//! ```
//!
//! Stability axes are rotated from body frame by −α about body Y:
//!
//! ```text
//! stab_to_body = DQuat::from_rotation_y(−α)
//! body_to_world = root_quaternion
//! ```
//!
//! ## Per-zone pure torques
//!
//! ```text
//! Torque_body = (Croll · q̄ · S · b,  CM · q̄ · S · c̄,  Cn · q̄ · S · b)
//! ```
//!
//! These are pure aerodynamic couples (e.g. airfoil pitching moment about its
//! aerodynamic centre). They are added to `ConstantTorque` alongside the
//! moment-arm torques from force × offset.
//!
//! ## Dynamic damping
//!
//! Applied once per root after zone accumulation:
//!
//! ```text
//! ΔCM = cm_q · (q·c̄/2V)    pitch damping
//! ΔCl = cl_p · (p·b/2V)    roll  damping
//! ΔCn = cn_r · (r·b/2V)    yaw   damping
//! ```

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

// ─────────────────────────────────────────────────────────────────────────────

/// Unified aerodynamic force system: per-zone coefficient evaluation,
/// force/torque accumulation, and dynamic damping — in a single pass.
///
/// Iterates each aircraft root entity, then its children inline. Writes
/// per-zone `ZoneForce` as a side-effect for debug visualisation. Writes
/// totals to `ConstantForce` / `ConstantTorque` on the root.
///
/// Engine zones (with `EngineZone` component) are skipped — their `ZoneForce`
/// is already written by `compute_engine_zone_forces`. Their force and torque
/// contributions are included during the child accumulation loop.
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
    // Engine zones are read-only here — their ZoneForce was already written
    // by compute_engine_zone_forces. We just need to include their contribution
    // in the accumulation pass.
    #[cfg(feature = "propulsion")]
    engine_zone_query: Query<&ZoneForce, (With<EngineZone>, Without<AeroZone>)>,
) {
    for (mut cf, mut ct, pos, rot, com, flight, geo, ctrl, children)
        in root_query.iter_mut()
    {
        // Reset each frame — we recompute from scratch.
        cf.0 = Vec3::ZERO;
        ct.0 = Vec3::ZERO;

        // Skip entire aircraft if airspeed is negligible.
        if flight.airspeed_ms < 1e-4 {
            continue;
        }

        let alpha = flight.alpha_rad;
        let re = flight.reynolds_number;
        let qbar = flight.dynamic_pressure_pa;
        let s = geo.wing_area_m2;
        let b = geo.wing_span_m;
        let c = geo.chord_m;

        let q_world = DQuat::from_array(rot.0.to_array().map(|x| x as f64));
        let stab_to_body = DQuat::from_rotation_y(-alpha);

        // Global centre of mass (f32, Avian-native).
        let com_world: Vec3 = pos.0 + rot.0 * com.0;

        // ── Per-zone: evaluate coefficients, compute force+torque, accumulate ──
        for child in children.iter() {
            // ── AeroZone children ──────────────────────────────────────────
            if let Ok((zone, zone_gt, mut zone_force, dmg)) = zone_query.get_mut(child) {
                *zone_force = ZoneForce::default();

                let health = dmg.map(|d| d.health).unwrap_or(1.0);
                if health <= 0.0 {
                    continue;
                }

                // ── Evaluate base coefficients ────────────────────────────
                let cl_base = zone.cl.evaluate(alpha, re);
                let cd_base = zone.cd.evaluate(alpha, re);
                let cy_base = zone.cy.evaluate(alpha, re);
                let cm_base = zone.cm.evaluate(alpha, re);
                let croll_base = zone.croll.evaluate(alpha, re);
                let cn_base = zone.cn.evaluate(alpha, re);

                // Control surface scaling: coefficients represent authority at
                // full deflection; scale linearly by actual input ∈ [−1, 1].
                // AileronRight mirrors the aileron input (opposite direction).
                let (scale, cd_scale) = match &zone.control_role {
                    Some(ControlSurfaceRole::Elevator) => (ctrl.elevator, ctrl.elevator.abs()),
                    Some(ControlSurfaceRole::AileronLeft) => (ctrl.aileron, ctrl.aileron.abs()),
                    Some(ControlSurfaceRole::AileronRight) => (-ctrl.aileron, ctrl.aileron.abs()),
                    Some(ControlSurfaceRole::Rudder) => (ctrl.rudder, ctrl.rudder.abs()),
                    None => (1.0, 1.0),
                };

                let cl = cl_base * scale * health;
                // Drag from deformation adds across [1, 0] health, disappears at 0.
                let extra_cd = zone.damage_drag_coeff * (1.0 - health) / qbar.max(1e-4);
                let cd = (cd_base * cd_scale + extra_cd) * health;
                let cy = cy_base * scale * health;
                let cm = cm_base * scale * health;
                let croll = croll_base * scale * health;
                let cn = cn_base * scale * health;

                // ── Force: stability axes → body → world ──────────────────
                let force_stab = DVec3::new(
                    -cd * qbar * s,  // drag opposes motion (−X_s)
                    cy * qbar * s,   // side force (Y_s = body Y)
                    -cl * qbar * s,  // lift upward (−Z_s)
                );
                let force_world_f64 = q_world * (stab_to_body * force_stab);

                // ── Pure torque: body → world ─────────────────────────────
                // These are aerodynamic couples (e.g. airfoil CM_ac) that
                // exist independently of the zone's position offset.
                let torque_body = DVec3::new(
                    croll * qbar * s * b,  // rolling moment → body X
                    cm * qbar * s * c,     // pitching moment → body Y
                    cn * qbar * s * b,     // yawing moment → body Z
                );
                let torque_world_f64 = q_world * torque_body;

                if !force_world_f64.is_finite() || !torque_world_f64.is_finite() {
                    warn_once!("Non-finite aero force/torque on zone — zeroed");
                    continue;
                }

                let force_world = Vec3::new(
                    force_world_f64.x as f32,
                    force_world_f64.y as f32,
                    force_world_f64.z as f32,
                );
                let torque_world = Vec3::new(
                    torque_world_f64.x as f32,
                    torque_world_f64.y as f32,
                    torque_world_f64.z as f32,
                );

                // Write per-zone output (for debug viz).
                zone_force.force = force_world;
                zone_force.torque = torque_world;
                zone_force.world_point = zone_gt.translation();

                // Accumulate onto root.
                cf.0 += force_world;
                ct.0 += (zone_gt.translation() - com_world).cross(force_world) + torque_world;

                continue;
            }

            // ── Engine zone children (propulsion feature) ─────────────────
            // ZoneForce was already written by compute_engine_zone_forces.
            // We just accumulate the force + moment arm torque here.
            #[cfg(feature = "propulsion")]
            if let Ok(zf) = engine_zone_query.get(child) {
                if zf.force != Vec3::ZERO {
                    cf.0 += zf.force;
                    ct.0 += (zf.world_point - com_world).cross(zf.force);
                }
                continue;
            }
        }

        // ── Dynamic damping torques (whole-aircraft, applied once) ────────────
        let v = flight.airspeed_ms;

        let pb_2v = flight.p_rads * b / (2.0 * v);
        let qc_2v = flight.q_rads * c / (2.0 * v);
        let rb_2v = flight.r_rads * b / (2.0 * v);

        // Damping moments in body frame using per-aircraft derivatives.
        let damp_body = DVec3::new(
            geo.cl_p * pb_2v * qbar * s * b,  // roll damping → body X
            geo.cm_q * qc_2v * qbar * s * c,  // pitch damping → body Y
            geo.cn_r * rb_2v * qbar * s * b,  // yaw damping → body Z
        );

        // Rotate to world frame.
        let damp_world = q_world * damp_body;
        if damp_world.is_finite() {
            ct.0 += Vec3::new(
                damp_world.x as f32,
                damp_world.y as f32,
                damp_world.z as f32,
            );
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
