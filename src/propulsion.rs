//! Piston engine thrust model.
//!
//! # Model overview
//!
//! A fixed-pitch piston engine with Gagg–Ferrar altitude correction:
//!
//! ```text
//! T = T_max · throttle_fraction · (ρ / ρ₀)^0.7
//! ```
//!
//! The 0.7 exponent is an empirical fit to piston-engine test data
//! (Gagg & Ferrar, 1934) and matches JSBSim's `FGPiston` model to within 2%
//! across the troposphere.
//!
//! ## Propeller induced velocity (actuator disk theory)
//!
//! At static thrust the propeller accelerates an air column of area A:
//!
//! ```text
//! V_ind = √(T / (2 · ρ · A))      A = π · (d/2)²
//! ```
//!
//! `V_ind` is stored in [`PropwashState`] and read by `compute_aerodynamics`
//! to compute the incremental lift from propwash over the inner wing.
//!
//! ## Thrust axis
//!
//! Thrust is applied in the direction of `EngineConfig::thrust_axis_body`,
//! rotated to world frame by the aircraft's current orientation quaternion.
//!
//! Only compiled with `features = ["propulsion"]`.

use std::f64::consts::PI;

use bevy::prelude::*;
use bevy::math::DQuat;
use avian3d::dynamics::rigid_body::forces::{Forces, WriteRigidBodyForces};

use crate::components::{AtmosphereState, ControlInputs, EngineConfig, FlightState, PropwashState};

/// Sea-level standard density (kg/m³).
const RHO_0: f64 = 1.225;

/// Compute engine thrust and propwash each frame.
///
/// Runs in `PhysicsSet::Prepare`, after `update_flight_state` and before
/// `compute_aerodynamics` (so aerodynamics can read the updated
/// [`PropwashState`]).
///
/// # What this system writes
/// - [`PropwashState::induced_velocity_ms`] — actuator-disk induced velocity
/// - [`PropwashState::direction_body`] — copy of `EngineConfig::thrust_axis_body`
/// - Thrust force via Avian [`Forces`] (world-frame)
pub fn compute_propulsion(
    mut query: Query<(
        Forces,
        &EngineConfig,
        &ControlInputs,
        &AtmosphereState,
        &FlightState,
        &mut PropwashState,
        &Transform,
    )>,
) {
    for (mut forces, engine, ctrl, atmos, _flight, mut propwash, transform) in &mut query {
        // 1. Look up throttle → thrust fraction via linear interpolation.
        let throttle = ctrl.throttle.clamp(0.0, 1.0);
        let thrust_fraction = interp_curve(&engine.throttle_curve, throttle);

        // 2. Gagg–Ferrar altitude correction: T = T_max · f · (ρ/ρ₀)^0.7
        let rho = atmos.density_kgm3;
        let density_ratio = (rho / RHO_0).max(0.0);
        let thrust_n = engine.max_thrust_n * thrust_fraction * density_ratio.powf(0.7);

        // 3. Actuator disk induced velocity: V_ind = √(T / (2ρA))
        let radius = engine.prop_diameter_m * 0.5;
        let disk_area = PI * radius * radius;
        let v_ind = if thrust_n > 0.0 && rho > 0.0 {
            (thrust_n / (2.0 * rho * disk_area)).sqrt()
        } else {
            0.0
        };

        // 4. Write PropwashState.
        propwash.induced_velocity_ms = v_ind;
        propwash.direction_body = engine.thrust_axis_body.normalize_or_zero();

        // 5. Rotate thrust axis body → world and apply force.
        let q = DQuat::from_array(transform.rotation.to_array().map(|x| x as f64));
        let thrust_world = q * (engine.thrust_axis_body.normalize_or_zero() * thrust_n);

        debug_assert!(
            thrust_world.is_finite(),
            "NaN/Inf in thrust force: {thrust_world:?}"
        );

        if thrust_world.is_finite() {
            forces.apply_force(bevy::math::Vec3::new(
                thrust_world.x as f32,
                thrust_world.y as f32,
                thrust_world.z as f32,
            ));
        } else {
            warn!("Non-finite thrust force — zeroed this frame");
        }
    }
}

/// Linear interpolation over a `[[throttle, fraction]; N]` curve.
///
/// Clamps output to the range `[first_fraction, last_fraction]` when the
/// input is outside the table bounds.
fn interp_curve(curve: &[[f64; 2]], x: f64) -> f64 {
    if curve.is_empty() {
        return 0.0;
    }
    if curve.len() == 1 {
        return curve[0][1];
    }
    if x <= curve[0][0] {
        return curve[0][1];
    }
    if x >= curve[curve.len() - 1][0] {
        return curve[curve.len() - 1][1];
    }
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
        let f = interp_curve(&curve, 0.5);
        assert!((f - 0.5).abs() < 1e-10);
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
        // At sea level density ratio = 1.0, so thrust = T_max * fraction.
        let rho = RHO_0;
        let density_ratio = (rho / RHO_0).max(0.0);
        let thrust = 500.0 * 0.8 * density_ratio.powf(0.7);
        assert!((thrust - 400.0).abs() < 1e-6);
    }

    #[test]
    fn gagg_ferrar_altitude_reduces_thrust() {
        // At altitude, rho < rho_0, so thrust should be less than sea-level thrust.
        let rho_alt = 0.9_f64; // ~1000 m
        let density_ratio = rho_alt / RHO_0;
        let thrust_alt = 500.0 * density_ratio.powf(0.7);
        let thrust_sl = 500.0_f64;
        assert!(thrust_alt < thrust_sl);
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
}
