//! Piston engine thrust model.
//!
//! # Overview
//!
//! Gagg–Ferrar altitude correction:
//!
//! ```text
//! T = T_max · health · throttle_fraction · (ρ / ρ₀)^0.7
//! ```
//!
//! Propeller induced velocity (actuator disk):
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
    AtmosphereState, ControlInputs, Damageable, EngineZone, FlightState, PropwashState, ZoneForce,
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
        Option<&Damageable>,
    )>,
    mut root_query: Query<(
        &ControlInputs,
        &AtmosphereState,
        &FlightState,
        &GlobalTransform,
    )>,
) {
    for (engine, engine_gt, col_of, mut zone_force, mut propwash, dmg) in engine_query.iter_mut() {
        *zone_force = ZoneForce::default();

        let health = dmg.map(|d| d.health).unwrap_or(1.0);
        if health <= 0.0 {
            continue;
        }

        let Ok((ctrl, atmos, _flight, root_gt)) =
            root_query.get_mut(col_of.body) else { continue };

        // 1. Throttle → thrust fraction.
        let throttle = ctrl.throttle.clamp(0.0, 1.0);
        let thrust_fraction = interp_curve(&engine.throttle_curve, throttle);

        // 2. Gagg–Ferrar altitude correction.
        let rho = atmos.density_kgm3;
        let density_ratio = (rho / RHO_0).max(0.0);
        let thrust_n = engine.max_thrust_n * health * thrust_fraction * density_ratio.powf(0.7);

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
    curve[curve.len() - 1][1]
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
    fn zero_health_zero_thrust() {
        let health = 0.0_f64;
        let max_thrust = 500.0_f64;
        let thrust = max_thrust * health;
        assert_eq!(thrust, 0.0);
    }
}
