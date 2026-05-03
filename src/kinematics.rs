//! Flight kinematics: derive [`FlightState`] from velocity, attitude, wind, and atmosphere.
//!
//! This module is the bridge between the rigid-body state owned by Avian
//! (position, linear/angular velocity, orientation) and the aerodynamic
//! reference frame consumed by the FDM (α, β, V, q̄, Mach, body rates p/q/r).
//!
//! It is deliberately *not* part of [`crate::atmosphere`]: the only atmospheric
//! inputs are density (for q̄) and speed-of-sound (for Mach), both read from a
//! pre-computed [`AtmosphereState`]. Everything else is pure kinematics —
//! quaternion rotations and `atan2` calls — independent of any atmosphere model.

use crate::_bevy::*;
use crate::atmosphere::WindResource;
use crate::components::{AtmosphereState, FlightState};
use avian3d::math::{Scalar, Vector};

/// Updates [`FlightState`] on each aircraft from velocity and atmosphere.
///
/// Reads [`avian3d::prelude::LinearVelocity`] and [`avian3d::prelude::AngularVelocity`],
/// converts to body frame, and derives α, β, V, q̄, Re, Mach.
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

        // Body angular rates, rotate world AngularVelocity to body frame.
        let q = quat_to_quaternion(transform.rotation());
        let av_world = ang_vel.0;
        let omega_body = q.inverse() * av_world;
        let p_rads = omega_body.x;
        let q_rads = omega_body.y;
        let r_rads = omega_body.z;

        // World-frame velocity relative to air mass.
        let vel_world = lin_vel.0 - wind_world;

        let airspeed_ms = vel_world.length();

        // Zero-airspeed guard: skip derived quantities, leave stale FlightState values.
        if airspeed_ms < 1e-4 {
            fs.airspeed_ms = airspeed_ms;
            fs.altitude_m = altitude_m;
            fs.dynamic_pressure_pa = 0.0;
            fs.p_rads = p_rads;
            fs.q_rads = q_rads;
            fs.r_rads = r_rads;
            continue;
        }

        // Rotate velocity to body frame.
        let vel_body = world_to_body(q, vel_world);

        // Angle of attack α: atan2(w, u)
        let u = vel_body.x; // forward
        let v = vel_body.y; // right
        let w = vel_body.z; // down
        let alpha_rad = w.atan2(u);

        // Sideslip β: atan2(v, sqrt(u²+w²))
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
