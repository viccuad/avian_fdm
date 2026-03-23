//! Piston engine thrust model.
//!
//! # Overview
//!
//! Gagg–Ferrar altitude correction — **thrust = max thrust × health fraction ×
//! throttle position × air density ratio raised to the 0.7 power. The 0.7
//! exponent is empirical for naturally-aspirated piston engines: thrust falls
//! with altitude as air thins. See: Gagg-Ferrar piston engine model.**
//!
//! ```text
//! T = T_max · remaining · throttle_fraction · (ρ / ρ₀)^0.7
//! ```
//!
//! Propeller induced velocity via **actuator disk theory** — **induced airspeed
//! behind the propeller = square root of (thrust ÷ (2 × air density × disk area)).
//! Disk area = π × propeller radius². See: actuator disk theory,
//! momentum theory propeller.**
//!
//! ```text
//! V_ind = √(T / (2 · ρ · A))    A = π · (d/2)²
//! ```
//!
//! `V_ind` is stored in [`PropwashState`] on the root entity and read by
//! `compute_aero_forces` for propwash effects (Group A, post-v1).
//!
//! Only compiled with `features = ["propulsion"]`.

use std::f64::consts::PI;
use bevy::prelude::*;
use bevy::math::DQuat;
use avian3d::prelude::ColliderOf;

use crate::components::{
    AtmosphereState, ControlInputs, Failure, EngineZone, FlightState, PropwashState, ZoneForce,
    get_remaining,
};

/// Sea-level standard density (kg/m³).
const RHO_0: f64 = 1.225;

/// Phase 1: compute engine thrust and write `ZoneForce` + update `PropwashState`.
pub fn compute_engine_zone_forces(
    mut engine_query: Query<(
        &EngineZone,
        &GlobalTransform,
        &ColliderOf,
        &mut ZoneForce,
        &mut PropwashState,
        Option<&Failure>,
    )>,
    mut root_query: Query<(
        &ControlInputs,
        &AtmosphereState,
        &FlightState,
        &GlobalTransform,
    )>,
) {
    for (engine, engine_gt, col_of, mut zone_force, mut propwash, opt_failure) in engine_query.iter_mut() {
        *zone_force = ZoneForce::default();

        let remaining = get_remaining(opt_failure);
        if remaining <= 0.0 {
            continue;
        }

        let Ok((ctrl, atmos, flight, root_gt)) =
            root_query.get_mut(col_of.body) else { continue };

        // 1. Throttle → thrust fraction.
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

        // 3. Actuator disk induced velocity.
        let radius = engine.prop_diameter_m * 0.5;
        let disk_area = PI * radius * radius;
        let v_ind = if thrust_n > 0.0 && rho > 0.0 {
            (thrust_n / (2.0 * rho * disk_area)).sqrt()
        } else {
            0.0
        };
        propwash.induced_velocity_ms = v_ind;
        propwash.direction_body = engine.thrust_axis_body.normalize_or_zero();

        // 4. Rotate thrust axis body → world and write ZoneForce.
        let q = DQuat::from_array(root_gt.rotation().to_array().map(|x| x as f64));
        let thrust_world = q * (engine.thrust_axis_body.normalize_or_zero() * thrust_n);

        if !thrust_world.is_finite() {
            warn_once!("Non-finite thrust force — zeroed");
            continue;
        }

        zone_force.force = Vec3::new(
            thrust_world.x as f32,
            thrust_world.y as f32,
            thrust_world.z as f32,
        );
        zone_force.world_point = engine_gt.translation();
    }
}

/// Linear interpolation over a `[[throttle, fraction]; N]` lookup table.
///
/// Clamps to the boundary values when `x` is outside the table range.
pub(crate) fn interp_curve(curve: &[[f64; 2]], x: f64) -> f64 {
    if curve.is_empty() { return 0.0; }
    if curve.len() == 1 { return curve[0][1]; }
    if x <= curve[0][0] { return curve[0][1]; }
    if x >= curve[curve.len() - 1][0] { return curve[curve.len() - 1][1]; }
    for i in 0..curve.len() - 1 {
        let x0 = curve[i][0];
        let x1 = curve[i + 1][0];
        if x >= x0 && x <= x1 {
            let t = (x - x0) / (x1 - x0);
            return curve[i][1] + t * (curve[i + 1][1] - curve[i][1]);
        }
    }
    // Every in-range x is covered by the loop above; the boundary checks at the
    // top of the function prevent reaching here.
    unreachable!("interp_curve: x={x} not found in curve with {} entries", curve.len())
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
        let rho_alt = 0.9_f64;
        let ratio = (rho_alt / RHO_0).powf(0.7);
        assert!(ratio < 1.0);
    }

    #[test]
    fn induced_velocity_positive_at_nonzero_thrust() {
        let thrust = 400.0_f64;
        let rho = 1.225_f64;
        let radius = 0.9_f64;
        let disk_area = PI * radius * radius;
        let v_ind = (thrust / (2.0 * rho * disk_area)).sqrt();
        assert!(v_ind > 0.0);
    }

    #[test]
    fn zero_remaining_zero_thrust() {
        let remaining = 0.0_f64;
        let max_thrust = 500.0_f64;
        let thrust = max_thrust * remaining;
        assert_eq!(thrust, 0.0);
    }

    #[test]
    fn interp_curve_empty_returns_zero() {
        assert_eq!(interp_curve(&[], 0.5), 0.0);
    }

    #[test]
    fn interp_curve_single_entry_returns_that_value() {
        let curve = vec![[0.5_f64, 0.8_f64]];
        assert!((interp_curve(&curve, 0.0) - 0.8).abs() < 1e-12, "below");
        assert!((interp_curve(&curve, 0.5) - 0.8).abs() < 1e-12, "exact");
        assert!((interp_curve(&curve, 1.0) - 0.8).abs() < 1e-12, "above");
    }

    #[test]
    fn interp_curve_three_breakpoints() {
        // Idle→mid: 0.0→0.5 maps to 0.0→0.6; mid→full: 0.5→1.0 maps to 0.6→1.0
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
        assert!((interp_curve(&curve, 0.0) - 0.1).abs() < 1e-12, "below range → first value");
        assert!((interp_curve(&curve, 1.0) - 0.9).abs() < 1e-12, "above range → last value");
    }
}
