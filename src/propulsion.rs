//! Piston engine thrust model.
//!
//! # Overview
//!
//! Gagg-Ferrar altitude correction: thrust = max thrust x health fraction x
//! throttle position x air density ratio raised to the 0.7 power. The 0.7
//! exponent is empirical for naturally-aspirated piston engines: thrust falls
//! with altitude as air thins.
//!
//! ```text
//! T = T_max * remaining * throttle_fraction * (rho / rho_0)^0.7
//! ```
//!
//! An optional speed-dependent factor models fixed-pitch propeller efficiency
//! drop: thrust scales by max(0, 1 - (V / V_zero)^2), reaching zero at the
//! windmilling speed.

use avian3d::math::Scalar;
use crate::_bevy::*;
use crate::math::{quat_to_quaternion, vector_to_vec3};
use avian3d::prelude::ColliderOf;

use crate::components::{
    AtmosphereState, ControlInputs, Failure, EngineZone, FlightState, ZoneForce,
    get_remaining,
};

/// Sea-level standard density (kg/m³).
const RHO_0: Scalar = 1.225;

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

        let Ok((ctrl, atmos, flight, root_gt)) =
            root_query.get_mut(col_of.body) else { continue };

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

        let thrust_n = engine.max_thrust_n * remaining * thrust_fraction
            * density_ratio.powf(0.7)
            * speed_factor;

        // 3. Rotate thrust axis body to world and write ZoneForce.
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

/// Linear interpolation over a `[[throttle, fraction]; N]` lookup table.
///
/// Clamps to the boundary values when `x` is outside the table range.
pub(crate) fn interp_curve(curve: &[[Scalar; 2]], x: Scalar) -> Scalar {
    use crate::components::aero_coeff::lerp_1d;
    if curve.is_empty() { return 0.0; }
    if curve.len() == 1 { return curve[0][1]; }
    // Clamp to table bounds (lerp_1d assumes in-range input).
    let x = x.clamp(curve[0][0], curve[curve.len() - 1][0]);
    let bp: Vec<Scalar> = curve.iter().map(|p| p[0]).collect();
    let vals: Vec<Scalar> = curve.iter().map(|p| p[1]).collect();
    lerp_1d(x, &bp, &vals)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_throttle_curve_midpoint() {
        let curve = vec![[0.0, 0.0], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.5) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn throttle_curve_clamp_above() {
        let curve = vec![[0.0, 0.0], [1.0, 0.9]];
        assert!((interp_curve(&curve, 1.5) - 0.9).abs() < 1e-10);
    }

    #[test]
    fn throttle_curve_clamp_below() {
        let curve = vec![[0.2, 0.1], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.0) - 0.1).abs() < 1e-10);
    }

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
        let remaining = 0.0;
        let max_thrust = 500.0;
        let thrust = max_thrust * remaining;
        assert_eq!(thrust, 0.0);
    }

    #[test]
    fn interp_curve_empty_returns_zero() {
        assert_eq!(interp_curve(&[], 0.5), 0.0);
    }

    #[test]
    fn interp_curve_single_entry_returns_that_value() {
        let curve = vec![[0.5, 0.8]];
        assert!((interp_curve(&curve, 0.0) - 0.8).abs() < 1e-12, "below");
        assert!((interp_curve(&curve, 0.5) - 0.8).abs() < 1e-12, "exact");
        assert!((interp_curve(&curve, 1.0) - 0.8).abs() < 1e-12, "above");
    }

    #[test]
    fn interp_curve_three_breakpoints() {
        // Idle to mid: 0.0 to 0.5 maps to 0.0 to 0.6; mid to full: 0.5 to 1.0 maps to 0.6 to 1.0
        let curve = vec![[0.0, 0.0], [0.5, 0.6], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.0)  - 0.0).abs() < 1e-12, "lower clamp");
        assert!((interp_curve(&curve, 0.25) - 0.3).abs() < 1e-12, "lower segment mid");
        assert!((interp_curve(&curve, 0.5)  - 0.6).abs() < 1e-12, "breakpoint");
        assert!((interp_curve(&curve, 0.75) - 0.8).abs() < 1e-12, "upper segment mid");
        assert!((interp_curve(&curve, 1.0)  - 1.0).abs() < 1e-12, "upper clamp");
    }

    #[test]
    fn interp_curve_clamps_outside_range() {
        let curve = vec![[0.2, 0.1], [0.8, 0.9]];
        assert!((interp_curve(&curve, 0.0) - 0.1).abs() < 1e-12, "below range clamps to first value");
        assert!((interp_curve(&curve, 1.0) - 0.9).abs() < 1e-12, "above range clamps to last value");
    }
}
