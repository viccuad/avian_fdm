//! Flight kinematics: calculates [`FlightState`] from the velocity, attitude, wind, and atmosphere.
//!
//! This module connects two sides. One side is the rigid-body state that
//! Avian owns (position, linear and angular velocity, orientation). The
//! other side is the aerodynamic state that the FDM uses (alpha, beta, V,
//! qbar, Mach, and the body rates p, q, r).
//!
//! It is intentionally *not* part of [`crate::atmosphere`]. It uses only two
//! atmospheric inputs: the density (for qbar) and the speed of sound (for
//! Mach). It reads both from an [`AtmosphereState`] that another system
//! calculated before. All other calculations are pure kinematics, quaternion
//! rotations and `atan2` calls. They do not depend on an atmosphere model.

use crate::_bevy::*;
use crate::atmosphere::WindResource;
use crate::components::{AtmosphereState, FlightState};
use avian3d::math::{Scalar, Vector};

/// Updates [`FlightState`] on each aircraft from the velocity and the atmosphere.
///
/// It reads [`avian3d::prelude::LinearVelocity`] and
/// [`avian3d::prelude::AngularVelocity`], and subtracts the wind from
/// [`WindResource`] (if present). It turns the result into the body frame.
/// Then it calculates alpha, beta, V, qbar, Mach, and the body rates.
#[allow(clippy::unnecessary_cast)]
pub fn update_flight_state(
    mut query: Query<(
        &GlobalTransform,
        &avian3d::prelude::LinearVelocity,
        &avian3d::prelude::AngularVelocity,
        &AtmosphereState,
        &mut FlightState,
    )>,
    wind: Option<Res<WindResource>>,
) {
    use crate::math::{quat_to_quaternion, world_to_body};

    let wind_world = wind.map(|w| w.velocity_world_ms).unwrap_or(Vector::ZERO);

    for (transform, lin_vel, ang_vel, atm, mut fs) in &mut query {
        let altitude_m = transform.translation().y as Scalar;

        // Body angular rates: turns the world AngularVelocity into the body frame.
        let q = quat_to_quaternion(transform.rotation());
        let av_world = ang_vel.0;
        let omega_body = q.inverse() * av_world;
        let p_rads = omega_body.x;
        let q_rads = omega_body.y;
        let r_rads = omega_body.z;

        // Velocity in world space, relative to the air mass.
        let vel_world = lin_vel.0 - wind_world;

        let airspeed_ms = vel_world.length();

        // Guard for zero airspeed: skip the derived values, and keep the old FlightState values.
        if airspeed_ms < 1e-4 {
            fs.airspeed_ms = airspeed_ms;
            fs.altitude_m = altitude_m;
            fs.dynamic_pressure_pa = 0.0;
            fs.p_rads = p_rads;
            fs.q_rads = q_rads;
            fs.r_rads = r_rads;
            continue;
        }

        // Turns the velocity into the body frame.
        let vel_body = world_to_body(q, vel_world);

        // Angle of attack, alpha: atan2(w, u)
        let u = vel_body.x; // forward
        let v = vel_body.y; // right
        let w = vel_body.z; // down
        let alpha_rad = w.atan2(u);

        // Sideslip angle, beta: atan2(v, sqrt(u^2 + w^2))
        let beta_rad = v.atan2((u * u + w * w).sqrt());

        let dynamic_pressure_pa = 0.5 * atm.density_kgm3 * airspeed_ms * airspeed_ms;
        let mach = airspeed_ms / atm.speed_of_sound_ms;

        *fs = FlightState {
            alpha_rad,
            beta_rad,
            airspeed_ms,
            mach,
            dynamic_pressure_pa,
            altitude_m,
            p_rads,
            q_rads,
            r_rads,
        };
    }
}
