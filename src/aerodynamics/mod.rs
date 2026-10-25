//! Aerodynamic force pipeline.
//!
//! The pipeline has small, documented, pure functions. Each function does one
//! step of the aerodynamic calculation. The Bevy system
//! [`compute_aero_forces`] calls them in this order:
//!
//! ```text
//! For each aircraft root entity:
//!   For each AeroZone child:
//!     0. zone_local_angles             - local alpha and beta of the zone, from the body rates
//!     1. evaluate_zone_coefficients    - evaluate the CL, CD, ... tables, apply scaling and damage
//!     2. zone_force_world              - turn the stability-frame forces into the world frame
//!   For each EngineZone child:
//!     3. accumulate_engine_force       - add the thrust and the torque from its moment arm
//!   Once per aircraft:
//!     4. induced_drag                  - full-aircraft CD_i = CL^2 / (pi * e * AR)
//!     5. damping_torque (LOD fallback) - only when LodDamping is present
//! ```
//!
//! ## Detail modes
//!
//! | `LodDamping` | Step 0 | Step 5 | Best for |
//! |---|---|---|---|
//! | `None` (default) | local alpha and beta for each zone | skipped | full-zone aircraft |
//! | `Some(LodDamping)` | global alpha and beta only | derivatives applied | sparse-zone bodies |
//!
//! The module of each step documents the physics of that step.

pub(crate) mod coefficients;
pub(crate) mod damping;
pub(crate) mod engine_coupling;
pub(crate) mod induced_drag;
pub(crate) mod local_angles;
pub(crate) mod world_forces;

pub(crate) use coefficients::evaluate_zone_coefficients;
pub use damping::damping_torque;
pub use local_angles::zone_local_angles;
pub(crate) use world_forces::zone_force_world;

use crate::_bevy::*;
use crate::math::{f32_to_scalar, quat_to_quaternion, vec3_to_vector, vector_to_vec3};
use avian3d::math::{Scalar, Vector};
use avian3d::prelude::{ComputedCenterOfMass, ConstantForce, ConstantTorque, Position, Rotation};

use crate::components::EngineZone;
use crate::components::{
    get_remaining, AeroZone, AircraftGeometry, AtmosphereState, ControlInputs, Failure,
    FlightState, InducedDrag, LodDamping, ZoneForce,
};

use engine_coupling::accumulate_engine_force;
use induced_drag::apply_induced_drag;

/// Bevy system that runs the aerodynamic pipeline in each physics step.
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
    engine_zone_query: Query<(&ZoneForce, &Transform), (With<EngineZone>, Without<AeroZone>)>,
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

        let mu = f32_to_scalar(crate::atmosphere::sutherland_viscosity(atm.temperature_k));
        let rho = f32_to_scalar(atm.density_kgm3);

        let body_to_world = rot.0;
        let (sa, ca) = (alpha.sin(), alpha.cos());
        let (sb, cb) = (beta.sin(), beta.cos());
        let vel_body_unit_global = Vector::new(ca * cb, sb, sa * cb);
        let use_lod = lod_damping.is_some();

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

                let zone_body_from_cg: Vector = vec3_to_vector(zone_transform.translation) - com.0;
                let zone_q = quat_to_quaternion(zone_transform.rotation);

                // Step 0: local alpha and beta for the zone (skipped in LOD mode).
                let (alpha_local, beta_local, vel_zone_unit_local, zone_to_world) = if use_lod {
                    (alpha, beta, vel_body_unit_global, body_to_world)
                } else {
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

                // Step 2: force and torque in world space.
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
                let ac_body = vec3_to_vector(zone_transform.translation + zone.ac_offset);
                // The arm is calculated in body frame and then rotated. A
                // difference of two world positions loses precision far from
                // the origin.
                let ac_from_cg_world = body_to_world * (ac_body - com.0);

                zone_force.force = vector_to_vec3(force);
                zone_force.world_point = vector_to_vec3(pos.0 + body_to_world * ac_body);

                cf.0 += force;
                ct.0 += ac_from_cg_world.cross(force) + torque;
                continue;
            }

            // Step 3: adds the thrust of the engine zone.
            if let Ok((zf, engine_transform)) = engine_zone_query.get(child) {
                let arm_world =
                    body_to_world * (vec3_to_vector(engine_transform.translation) - com.0);
                let mut ef = Vector::ZERO;
                let mut et = Vector::ZERO;
                accumulate_engine_force(zf, arm_world, &mut ef, &mut et);
                cf.0 += ef;
                ct.0 += et;
                continue;
            }
        }

        // Step 4: induced drag.
        if let Some(id) = induced_drag {
            cf.0 += apply_induced_drag(id, total_cl_x_area, s, b, qbar, vel_body_unit_global, *rot);
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
