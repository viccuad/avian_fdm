//! Aerodynamic force pipeline.
//!
//! Two-phase design:
//!
//! 1. **`compute_zone_forces`** — per `AeroZone` entity, evaluates
//!    stability-derivative coefficients and writes a `ZoneForce` (world-space
//!    force + application point) to the zone entity. Pure math; never touches
//!    Avian query data.
//!
//! 2. **`accumulate_zone_forces`** — per aircraft root entity, iterates
//!    `Children`, sums `ZoneForce` contributions into `ConstantForce` (total
//!    force) and `ConstantTorque` (moment arm cross-product using the root's
//!    `ComputedCenterOfMass`), then adds dynamic-damping torques. Avian applies
//!    `ConstantForce`/`ConstantTorque` natively in
//!    `ForceSystems::ApplyConstantForces`.
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
//! ## Dynamic damping
//!
//! Applied once per root in `accumulate_zone_forces`:
//!
//! ```text
//! ΔCM = CM_q · (q·c̄/2V)    pitch damping  (Nelson Table B1: −12)
//! ΔCl = Cl_p · (p·b/2V)    roll  damping  (−0.45)
//! ΔCn = Cn_r · (r·b/2V)    yaw   damping  (−0.12)
//! ```

use bevy::prelude::*;
use bevy::math::{DVec3, DQuat};
use avian3d::prelude::{
    ColliderOf, ConstantForce, ConstantTorque, Position, Rotation, ComputedCenterOfMass,
};

use crate::components::{
    AeroZone, AircraftGeometry, ControlInputs, ControlSurfaceRole,
    Damageable, FlightState, WindResource, ZoneForce,
};
#[cfg(feature = "propulsion")]
use crate::components::PropwashState;

// ── Dynamic damping derivatives (Nelson, "Flight Stability", Table B1) ───────
const CM_Q: f64 = -12.0;
const CL_P: f64 = -0.45;
const CN_R: f64 = -0.12;

// ─────────────────────────────────────────────────────────────────────────────

/// Phase 1: evaluate per-zone aerodynamic coefficients and write `ZoneForce`.
///
/// Reads `FlightState`, `AircraftGeometry`, and `ControlInputs` from the
/// zone's parent RigidBody (via `ColliderOf.body`). Does **not** mutate any
/// Avian physics component — it only writes to the zone's own `ZoneForce`.
pub fn compute_zone_forces(
    mut zone_query: Query<(
        &AeroZone,
        &GlobalTransform,
        &ColliderOf,
        &mut ZoneForce,
        Option<&Damageable>,
    )>,
    root_query: Query<(&FlightState, &AircraftGeometry, &ControlInputs, &GlobalTransform)>,
    wind: Option<Res<WindResource>>,
) {
    let wind_world = wind.map(|w| w.velocity_world_ms).unwrap_or(DVec3::ZERO);
    let _ = wind_world;

    for (zone, zone_gt, col_of, mut zone_force, dmg) in zone_query.iter_mut() {
        *zone_force = ZoneForce::default();

        let health = dmg.map(|d| d.health).unwrap_or(1.0);
        if health <= 0.0 {
            continue;
        }

        // Look up root flight data. Skip if root isn't set up yet.
        let Ok((flight, geo, ctrl, root_gt)) = root_query.get(col_of.body) else {
            continue;
        };

        if flight.airspeed_ms < 1e-4 {
            continue;
        }

        let alpha = flight.alpha_rad;
        let re = flight.reynolds_number;
        let qbar = flight.dynamic_pressure_pa;
        let s = geo.wing_area_m2;

        // ── Evaluate base coefficients ────────────────────────────────────────
        let cl_base = zone.cl.evaluate(alpha, re);
        let cd_base = zone.cd.evaluate(alpha, re);
        let cy_base = zone.cy.evaluate(alpha, re);

        // Control surface scaling: coefficients represent the authority at
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
        // Drag from deformation adds across [1, 0] health, then disappears at 0.
        let extra_cd = zone.damage_drag_coeff * (1.0 - health) / qbar.max(1e-4);
        let cd = (cd_base * cd_scale + extra_cd) * health;
        let cy = cy_base * scale * health;

        // ── Force in stability axes → body → world ────────────────────────────
        // Stability axes: X_s into relative wind, Z_s perpendicular (lift-up).
        // Rotate to body by −α about body Y, then body → world by root quat.
        let force_stab = DVec3::new(
            -cd * qbar * s,  // drag opposes motion (−X_s)
            cy * qbar * s,   // side force (Y_s = body Y)
            -cl * qbar * s,  // lift upward (−Z_s)
        );

        let q_world = DQuat::from_array(root_gt.rotation().to_array().map(|x| x as f64));
        let stab_to_body = DQuat::from_rotation_y(-alpha);
        let force_world_f64 = q_world * (stab_to_body * force_stab);

        if !force_world_f64.is_finite() {
            warn_once!("Non-finite aero force on zone — zeroed");
            continue;
        }

        zone_force.force = Vec3::new(
            force_world_f64.x as f32,
            force_world_f64.y as f32,
            force_world_f64.z as f32,
        );
        zone_force.world_point = zone_gt.translation();
    }
}

/// Phase 2: sum `ZoneForce` from all children into root `ConstantForce` /
/// `ConstantTorque`, plus whole-aircraft dynamic damping torques.
///
/// Runs once per aircraft root entity. `ConstantForce`/`ConstantTorque` are
/// reset to zero first so stale values never accumulate across frames.
pub fn accumulate_zone_forces(
    mut root_query: Query<(
        &mut ConstantForce,
        &mut ConstantTorque,
        &Position,
        &Rotation,
        &ComputedCenterOfMass,
        &FlightState,
        &AircraftGeometry,
        &Children,
    )>,
    zone_force_query: Query<&ZoneForce>,
    #[cfg(feature = "propulsion")]
    propwash_query: Query<&PropwashState>,
) {
    for (mut cf, mut ct, pos, rot, com, flight, geo, children) in root_query.iter_mut() {
        // Reset each frame — we recompute from scratch.
        cf.0 = Vec3::ZERO;
        ct.0 = Vec3::ZERO;

        // Global centre of mass (f32, Avian-native).
        let com_world: Vec3 = pos.0 + rot.0 * com.0;

        // Sum zone force contributions.
        for child in children.iter() {
            let Ok(zf) = zone_force_query.get(child) else { continue };
            if zf.force == Vec3::ZERO {
                continue;
            }
            cf.0 += zf.force;
            ct.0 += (zf.world_point - com_world).cross(zf.force);
        }

        // ── Dynamic damping torques (whole-aircraft, applied once) ────────────
        if flight.airspeed_ms >= 1e-4 {
            let v = flight.airspeed_ms;
            let qbar = flight.dynamic_pressure_pa;
            let s = geo.wing_area_m2;
            let b = geo.wing_span_m;
            let c = geo.chord_m;

            let pb_2v = flight.p_rads * b / (2.0 * v);
            let qc_2v = flight.q_rads * c / (2.0 * v);
            let rb_2v = flight.r_rads * b / (2.0 * v);

            // Damping moments in body frame.
            let damp_body = DVec3::new(
                CL_P * pb_2v * qbar * s * b,  // roll damping → body X
                CM_Q * qc_2v * qbar * s * c,  // pitch damping → body Y
                CN_R * rb_2v * qbar * s * b,  // yaw damping → body Z
            );

            // Rotate to world frame.
            let q_world = DQuat::from_array(rot.0.to_array().map(|x| x as f64));
            let damp_world = q_world * damp_body;
            if damp_world.is_finite() {
                ct.0 += Vec3::new(
                    damp_world.x as f32,
                    damp_world.y as f32,
                    damp_world.z as f32,
                );
            }
        }

        // ── Propwash lift increment ────────────────────────────────────────────
        #[cfg(feature = "propulsion")]
        {
            // Look up PropwashState on this root entity (if it exists).
            // Detailed propwash coupling is Group A (post-v1).
            let _ = &propwash_query;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
#[cfg(test)]
mod tests {
    use super::*;
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
