//! Engine thrust computation system (Gagg-Ferrar altitude model).

use crate::_bevy::*;
use crate::math::{quat_to_quaternion, vector_to_vec3};
use avian3d::math::Scalar;
use avian3d::prelude::ColliderOf;

use super::engine_zone::EngineZone;
use super::throttle::interp_curve;
use crate::components::{
    get_remaining, AtmosphereState, ControlInputs, Failure, FlightState, ZoneForce,
};

/// Sea-level standard density (kg/m³).
pub(super) const RHO_0: Scalar = 1.225;

/// Phase 1: compute engine thrust and write `ZoneForce`.
#[allow(clippy::type_complexity)]
pub fn compute_engine_zone_forces(
    mut engine_query: Query<(
        &EngineZone,
        &GlobalTransform,
        &ColliderOf,
        &mut ZoneForce,
        Option<&Failure>,
    )>,
    mut root_query: Query<(
        &ControlInputs,
        &AtmosphereState,
        &FlightState,
        &GlobalTransform,
    )>,
) {
    for (engine, engine_gt, col_of, mut zone_force, opt_failure) in engine_query.iter_mut() {
        *zone_force = ZoneForce::default();

        let remaining = get_remaining(opt_failure);
        if remaining <= 0.0 {
            continue;
        }

        let Ok((ctrl, atmos, flight, root_gt)) = root_query.get_mut(col_of.body) else {
            continue;
        };

        // 1. Throttle to thrust fraction.
        let throttle = ctrl.throttle.clamp(0.0, 1.0);
        let thrust_fraction = interp_curve(&engine.throttle_curve, throttle);

        // 2. Gagg–Ferrar altitude correction.
        let rho = atmos.density_kgm3;
        let density_ratio = (rho / RHO_0).max(0.0);

        // 3. Speed-dependent thrust decay for fixed-pitch propellers.
        let speed_factor = if let Some(v_zero) = engine.zero_thrust_speed_ms {
            let ratio = flight.airspeed_ms / v_zero;
            (1.0 - ratio * ratio).max(0.0)
        } else {
            1.0
        };

        let thrust_n = engine.max_thrust_n
            * remaining
            * thrust_fraction
            * density_ratio.powf(0.7)
            * speed_factor;

        // 4. Rotate thrust axis body to world and write ZoneForce.
        let q = quat_to_quaternion(root_gt.rotation());
        let thrust_world = q * (engine.thrust_axis_body.normalize_or_zero() * thrust_n);

        if !thrust_world.is_finite() {
            warn_once!("Non-finite thrust force, zeroed");
            continue;
        }

        zone_force.force = vector_to_vec3(thrust_world);
        zone_force.world_point = engine_gt.translation();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gagg_ferrar_sea_level() {
        let density_ratio = (RHO_0 / RHO_0).max(0.0);
        let thrust = 500.0 * 0.8 * density_ratio.powf(0.7);
        assert!((thrust - 400.0).abs() < 1e-6);
    }

    #[test]
    fn gagg_ferrar_altitude_reduces_thrust() {
        let rho_alt = 0.9;
        let ratio = (rho_alt / RHO_0).powf(0.7);
        assert!(ratio < 1.0);
    }

    #[test]
    fn zero_remaining_zero_thrust() {
        let remaining = 0.0_f32;
        let max_thrust = 500.0_f32;
        let thrust = max_thrust * remaining;
        assert_eq!(thrust, 0.0);
    }
}
