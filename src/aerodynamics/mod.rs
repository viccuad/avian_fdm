//! Aerodynamic force pipeline.
//!
//! The pipeline is built from small, documented pure functions that each handle
//! one step of the aerodynamic computation.  The Bevy system
//! [`compute_aero_forces`] orchestrates them in order:
//!
//! ```text
//! For each aircraft root:
//!   For each AeroZone child:
//!     0. zone_local_angles           - per-zone effective alpha/beta from body rates
//!     1. evaluate_zone_coefficients  - lookup CL/CD/... tables, apply scaling and damage
//!     2. zone_force_world            - rotate stability-frame forces to world
//!   For each EngineZone child:
//!     3. accumulate_engine_force     - add pre-computed thrust and its moment-arm torque
//!   Once per aircraft:
//!     4. induced_drag                - whole-aircraft CD_i = CL^2/(pi * e * AR)
//!     5. damping_torque (LOD fallback) - only when LodDamping is present
//! ```
//!
//! ## Fidelity modes
//!
//! | `LodDamping` | Step 0 | Step 5 | Best for |
//! |---|---|---|---|
//! | `None` (default) | per-zone local α/β | skipped | full-zone aircraft |
//! | `Some(LodDamping)` | global α/β only | derivatives applied | sparse-zone bodies |
//!
//! Each step's physics are documented in its own module.

pub(crate) mod coefficients;
pub(crate) mod damping;
pub(crate) mod local_angles;
pub(crate) mod world_forces;

// Re-export the public API so callers use `aerodynamics::foo`.
pub(crate) use coefficients::evaluate_zone_coefficients;
pub use damping::damping_torque;
pub use local_angles::zone_local_angles;
pub(crate) use world_forces::zone_force_world;

use crate::_bevy::*;
use crate::math::{quat_to_quaternion, vec3_to_vector, vector_to_vec3};
use avian3d::math::{Scalar, Vector};
use avian3d::prelude::{ComputedCenterOfMass, ConstantForce, ConstantTorque, Position, Rotation};

use crate::components::EngineZone;
use crate::components::{
    get_remaining, AeroZone, AircraftGeometry, AtmosphereState, ControlInputs, Failure,
    FlightState, InducedDrag, LodDamping, ZoneForce,
};

#[allow(clippy::unnecessary_cast)]
const PI_VAL: Scalar = std::f64::consts::PI as Scalar;

//
// Step 3: Engine force accumulation
//

/// Accumulate a pre-computed engine zone's thrust into the root force/torque.
///
/// The moment arm is measured from the aircraft's CG to the engine's world
/// position.  An off-centre engine naturally produces a yawing moment when
/// thrust is asymmetric.
fn accumulate_engine_force(
    zf: &ZoneForce,
    com_world: Vector,
    total_force: &mut Vector,
    total_torque: &mut Vector,
) {
    if zf.force != Vec3::ZERO {
        let force = vec3_to_vector(zf.force);
        *total_force += force;
        // ZoneForce.world_point is Vec3 (always f32 for visualization).
        *total_torque += (vec3_to_vector(zf.world_point) - com_world).cross(force);
    }
}

//
// Orchestrator system
//

/// Bevy system that orchestrates the aerodynamic pipeline each physics step.
#[allow(clippy::type_complexity)]
pub fn compute_aero_forces(
    mut root_query: Query<(
        &mut ConstantForce,
        &mut ConstantTorque,
        &Position,
        &Rotation,
        &ComputedCenterOfMass,
        &FlightState,
        &AtmosphereState,
        &AircraftGeometry,
        &ControlInputs,
        &Children,
        Option<&LodDamping>,
        Option<&InducedDrag>,
    )>,
    mut zone_query: Query<(&AeroZone, &Transform, &mut ZoneForce, Option<&Failure>)>,
    engine_zone_query: Query<&ZoneForce, (With<EngineZone>, Without<AeroZone>)>,
) {
    for (
        mut cf,
        mut ct,
        pos,
        rot,
        com,
        flight,
        atm,
        geo,
        ctrl,
        children,
        lod_damping,
        induced_drag,
    ) in root_query.iter_mut()
    {
        cf.0 = Vector::ZERO;
        ct.0 = Vector::ZERO;

        if flight.airspeed_ms < 1e-4 {
            continue;
        }

        let alpha = flight.alpha_rad;
        let beta = flight.beta_rad;
        let qbar = flight.dynamic_pressure_pa;
        let v = flight.airspeed_ms;
        let p = flight.p_rads;
        let q = flight.q_rads;
        let r = flight.r_rads;
        let s = geo.wing_area_m2;
        let b = geo.wing_span_m;

        // Pre-compute viscosity once per aircraft for per-zone Reynolds number.
        let mu = crate::atmosphere::sutherland_viscosity(atm.temperature_k);
        let rho = atm.density_kgm3;

        let body_to_world = rot.0;
        let (sa, ca) = (alpha.sin(), alpha.cos());
        let (sb, cb) = (beta.sin(), beta.cos());
        let vel_body_unit_global = Vector::new(ca * cb, sb, sa * cb);
        let com_world: Vector = pos.0 + body_to_world * com.0;
        let use_lod = lod_damping.is_some();

        // Area-weighted CL sum for induced drag: CL_aircraft = total / S_ref.
        let mut total_cl_x_area: Scalar = 0.0;

        for child in children.iter() {
            if let Ok((zone, zone_transform, mut zone_force, opt_failure)) =
                zone_query.get_mut(child)
            {
                *zone_force = ZoneForce::default();

                let remaining = get_remaining(opt_failure);
                if remaining <= 0.0 {
                    continue;
                }

                // Zone body position relative to CG.
                // Local Transform is used instead of GlobalTransform to avoid
                // the one-frame propagation lag (PostUpdate runs after physics).
                let zone_body_from_cg: Vector = vec3_to_vector(zone_transform.translation) - com.0;

                // Zone rotation in body frame.
                let zone_q = quat_to_quaternion(zone_transform.rotation);

                // Step 0: per-zone local α/β (skipped in LOD mode).
                //
                // In full mode: project the body-frame velocity unit vector into
                // the zone's local frame. This naturally captures any geometric
                // orientation effect (dihedral, sweep, anhedral, etc.) without
                // needing explicit correction parameters. Angular-rate corrections
                // (roll/pitch/yaw) are then added on top.
                //
                // In LOD mode: use whole-aircraft alpha/beta and body-to-world rotation
                // directly (zone geometry is ignored as an approximation).
                let (alpha_local, beta_local, vel_zone_unit_local, zone_to_world) = if use_lod {
                    (alpha, beta, vel_body_unit_global, body_to_world)
                } else {
                    // Velocity in zone's local frame.
                    let vel_zone = zone_q.inverse() * vel_body_unit_global;
                    let alpha_zone = vel_zone.z.atan2(vel_zone.x);
                    let beta_zone = vel_zone
                        .y
                        .atan2((vel_zone.x * vel_zone.x + vel_zone.z * vel_zone.z).sqrt());
                    let (al, bl) = zone_local_angles(
                        alpha_zone,
                        beta_zone,
                        p,
                        q,
                        r,
                        zone_body_from_cg.x,
                        zone_body_from_cg.y,
                        v,
                    );
                    let (sal, cal) = (al.sin(), al.cos());
                    let (sbl, cbl) = (bl.sin(), bl.cos());
                    (
                        al,
                        bl,
                        Vector::new(cal * cbl, sbl, sal * cbl),
                        body_to_world * zone_q,
                    )
                };

                // Step 1: coefficient evaluation.
                // Per-zone Reynolds number: Re = rho * V * chord_zone / mu.
                // Zones with chord_m = 0 (mass-only) get Re = 0; their
                // coefficients are Absent so Re is never used.
                let re_zone = if zone.chord_m > 0.0 {
                    rho * v * zone.chord_m / mu
                } else {
                    0.0
                };
                let coeffs = evaluate_zone_coefficients(
                    zone,
                    ctrl,
                    alpha_local,
                    beta_local,
                    re_zone,
                    qbar,
                    remaining,
                );
                total_cl_x_area += coeffs.cl * zone.area_m2;

                // Step 2: world-space force and torque.
                let wf = zone_force_world(
                    &coeffs,
                    qbar,
                    zone.area_m2,
                    b,
                    zone.chord_m,
                    alpha_local,
                    vel_zone_unit_local,
                    zone_to_world,
                );

                if !wf.force.is_finite() || !wf.torque.is_finite() {
                    warn_once!("Non-finite aero force/torque on zone: zeroed");
                    continue;
                }

                let force = wf.force;
                let torque = wf.torque;
                // ac_world: aerodynamic centre in world space. zone_transform
                // and zone.ac_offset are always Vec3 (Bevy Transform is f32).
                let ac_world = pos.0
                    + body_to_world * vec3_to_vector(zone_transform.translation + zone.ac_offset);

                zone_force.force = vector_to_vec3(force);
                zone_force.torque = vector_to_vec3(torque);
                zone_force.world_point = vector_to_vec3(ac_world);

                cf.0 += force;
                ct.0 += (ac_world - com_world).cross(force) + torque;
                continue;
            }

            // Step 3: engine zone thrust accumulation.
            if let Ok(zf) = engine_zone_query.get(child) {
                let mut ef = Vector::ZERO;
                let mut et = Vector::ZERO;
                accumulate_engine_force(zf, com_world, &mut ef, &mut et);
                cf.0 += ef;
                ct.0 += et;
                continue;
            }
        }

        // Step 4: induced drag. CD_i = CL² / (π · e · AR).
        // CL_aircraft is the area-weighted sum of per-zone CLs divided by S_ref.
        if let Some(id) = induced_drag {
            let ar = b * b / s;
            let cl_aircraft = total_cl_x_area / s;
            let cd_i = cl_aircraft * cl_aircraft / (PI_VAL * id.oswald_factor * ar);
            let drag_world = body_to_world * (vel_body_unit_global * (-cd_i * qbar * s));
            cf.0 += drag_world;
        }

        // Step 5: LOD damping (mutually exclusive with step 0).
        if let Some(lod) = lod_damping {
            let damp = damping_torque(flight, lod, geo, body_to_world);
            if damp.is_finite() {
                ct.0 += damp;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::ZoneForce;

    /// On-centre engine produces pure thrust, no torque.
    #[test]
    fn engine_at_cg_no_moment() {
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::ZERO,
            torque: Vec3::ZERO,
        };
        let (mut f, mut t) = (Vector::ZERO, Vector::ZERO);
        accumulate_engine_force(&zf, Vector::ZERO, &mut f, &mut t);
        assert!((f - Vector::new(500.0, 0.0, 0.0)).length() < 1e-5);
        assert!(t.length() < 1e-5, "on-axis engine must not produce torque");
    }

    /// Starboard off-centre engine produces nose-left yaw torque.
    #[test]
    fn engine_offset_right_produces_yaw_torque() {
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::new(0.0, 2.0, 0.0),
            torque: Vec3::ZERO,
        };
        let (mut f, mut t) = (Vector::ZERO, Vector::ZERO);
        accumulate_engine_force(&zf, Vector::ZERO, &mut f, &mut t);
        // arm=(0,2,0) × thrust=(500,0,0) → torque=(0,0,-1000)
        assert!(
            (t.z - (-1000.0)).abs() < 1e-4,
            "starboard engine → nose-left yaw, got z={}",
            t.z
        );
    }

    /// Zero-force engine short-circuits: totals unchanged.
    #[test]
    fn engine_zero_force_no_accumulation() {
        let zf = ZoneForce {
            force: Vec3::ZERO,
            world_point: Vec3::new(0.0, 5.0, 0.0),
            torque: Vec3::ZERO,
        };
        let (mut f, mut t) = (Vector::new(100.0, 0.0, 0.0), Vector::new(0.0, 50.0, 0.0));
        accumulate_engine_force(&zf, Vector::ZERO, &mut f, &mut t);
        assert!((f - Vector::new(100.0, 0.0, 0.0)).length() < 1e-5);
        assert!((t - Vector::new(0.0, 50.0, 0.0)).length() < 1e-5);
    }
}
