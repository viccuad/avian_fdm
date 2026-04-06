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
use avian3d::prelude::{ComputedCenterOfMass, ConstantForce, ConstantTorque, Position, Rotation};
use bevy_math::{DQuat, DVec3};

use crate::components::EngineZone;
use crate::components::{
    get_remaining, AeroZone, AircraftGeometry, AtmosphereState, ControlInputs, Failure,
    FlightState, InducedDrag, LodDamping, ZoneForce,
};
use crate::math::to_dvec3;

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
    com_world: Vec3,
    total_force: &mut Vec3,
    total_torque: &mut Vec3,
) {
    if zf.force != Vec3::ZERO {
        *total_force += zf.force;
        *total_torque += (zf.world_point - com_world).cross(zf.force);
    }
}

//
// Orchestrator system
//

/// Bevy system that orchestrates the aerodynamic pipeline each physics step.
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
    engine_zone_query: Query<
        &ZoneForce,
        (With<EngineZone>, Without<AeroZone>),
    >,
) {
    for (mut cf, mut ct, pos, rot, com, flight, atm, geo, ctrl, children, lod_damping, induced_drag) in
        root_query.iter_mut()
    {
        cf.0 = Vec3::ZERO;
        ct.0 = Vec3::ZERO;

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

        let body_to_world = DQuat::from_array(rot.0.to_array().map(|x| x as f64));
        // Unit velocity vector in body frame, reconstructed from alpha and beta.
        // Used for drag direction: drag must always oppose the full 3D velocity,
        // including the sideslip component, not just the in-plane component.
        let (sa, ca) = (alpha.sin(), alpha.cos());
        let (sb, cb) = (beta.sin(), beta.cos());
        let vel_body_unit_global = DVec3::new(ca * cb, sb, sa * cb);
        let com_world: Vec3 = pos.0 + rot.0 * com.0;
        let use_lod = lod_damping.is_some();

        // Area-weighted CL sum for induced drag: CL_aircraft = total / S_ref.
        let mut total_cl_x_area = 0.0_f64;

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
                let zone_body_from_cg: DVec3 =
                    to_dvec3(zone_transform.translation) - to_dvec3(com.0);

                // Zone rotation in body frame as a double-precision quaternion.
                let zone_q = DQuat::from_array(zone_transform.rotation.to_array().map(|x| x as f64));

                // Step 0: per-zone local α/β (skipped in LOD mode).
                //
                // In full mode: project the body-frame velocity unit vector into
                // the zone's local frame. This naturally captures any geometric
                // orientation effect (dihedral, sweep, anhedral, etc.) without
                // needing explicit correction parameters. Angular-rate corrections
                // (roll/pitch/yaw) are then added on top.
                //
                // In LOD mode: use whole-aircraft α/β and body-to-world rotation
                // directly (zone geometry is ignored as an approximation).
                let (alpha_local, beta_local, vel_zone_unit_local, zone_to_world) = if use_lod {
                    (alpha, beta, vel_body_unit_global, body_to_world)
                } else {
                    // Velocity in zone's local frame.
                    let vel_zone = zone_q.inverse() * vel_body_unit_global;
                    let alpha_zone = f64::atan2(vel_zone.z, vel_zone.x);
                    let beta_zone = f64::atan2(
                        vel_zone.y,
                        (vel_zone.x * vel_zone.x + vel_zone.z * vel_zone.z).sqrt(),
                    );
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
                    (al, bl, DVec3::new(cal * cbl, sbl, sal * cbl), body_to_world * zone_q)
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
                let wf =
                    zone_force_world(&coeffs, qbar, zone.area_m2, b, zone.chord_m, alpha_local, vel_zone_unit_local, zone_to_world);

                if !wf.force.is_finite() || !wf.torque.is_finite() {
                    warn_once!("Non-finite aero force/torque on zone: zeroed");
                    continue;
                }

                let force_world = wf.force.as_vec3();
                let torque_world = wf.torque.as_vec3();
                let ac_world = pos.0 + rot.0 * (zone_transform.translation + zone.ac_offset);

                zone_force.force = force_world;
                zone_force.torque = torque_world;
                zone_force.world_point = ac_world;

                cf.0 += force_world;
                ct.0 += (ac_world - com_world).cross(force_world) + torque_world;
                continue;
            }

            // Step 3: engine zone thrust accumulation.
            if let Ok(zf) = engine_zone_query.get(child) {
                accumulate_engine_force(zf, com_world, &mut cf.0, &mut ct.0);
                continue;
            }
        }

        // Step 4: induced drag. CD_i = CL² / (π · e · AR).
        // CL_aircraft is the area-weighted sum of per-zone CLs divided by S_ref.
        if let Some(id) = induced_drag {
            let ar = b * b / s;
            let cl_aircraft = total_cl_x_area / s;
            let cd_i = cl_aircraft * cl_aircraft / (std::f64::consts::PI * id.oswald_factor * ar);
            let drag_world = body_to_world * (vel_body_unit_global * (-cd_i * qbar * s));
            cf.0 += drag_world.as_vec3();
        }

        // Step 5: LOD damping (mutually exclusive with step 0).
        if let Some(lod) = lod_damping {
            let damp = damping_torque(flight, lod, geo, body_to_world);
            if damp.is_finite() {
                ct.0 += damp.as_vec3();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::ZoneForce;
    use bevy_math::Vec3;

    /// On-centre engine produces pure thrust, no torque.
    #[test]
    fn engine_at_cg_no_moment() {
        let zf = ZoneForce {
            force: Vec3::new(500.0, 0.0, 0.0),
            world_point: Vec3::ZERO,
            torque: Vec3::ZERO,
        };
        let (mut f, mut t) = (Vec3::ZERO, Vec3::ZERO);
        accumulate_engine_force(&zf, Vec3::ZERO, &mut f, &mut t);
        assert!((f - Vec3::new(500.0, 0.0, 0.0)).length() < 1e-5);
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
        let (mut f, mut t) = (Vec3::ZERO, Vec3::ZERO);
        accumulate_engine_force(&zf, Vec3::ZERO, &mut f, &mut t);
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
        let (mut f, mut t) = (Vec3::new(100.0, 0.0, 0.0), Vec3::new(0.0, 50.0, 0.0));
        accumulate_engine_force(&zf, Vec3::ZERO, &mut f, &mut t);
        assert!((f - Vec3::new(100.0, 0.0, 0.0)).length() < 1e-5);
        assert!((t - Vec3::new(0.0, 50.0, 0.0)).length() < 1e-5);
    }
}
