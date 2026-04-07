//! ISA atmosphere model and wind resource.
//!
//! Implements the International Standard Atmosphere (ICAO Doc 7488) for the
//! troposphere (0-11 km) and lower stratosphere (11-20 km), plus Sutherland's
//! law for dynamic viscosity.
//!
//! ## Why density matters
//!
//! Every aerodynamic force scales with dynamic pressure (q-bar = half *
//! density * airspeed^2). A 20% drop in density (roughly 2 500 m altitude)
//! means 20% less lift and drag at the same speed. Reynolds number (density *
//! speed * chord / viscosity) controls how air flows over the surface: at low
//! Reynolds numbers the flow separates earlier, causing sharper stall and
//! higher drag.
//!
//! ## ISA formulas (ICAO Doc 7488)
//!
//! **Troposphere (h <= 11 000 m)**: temperature drops linearly (6.5 K/km), pressure
//! follows a power law from the temperature, density rho from the ideal gas law:
//!
//! ```text
//! T = 288.15 - 0.0065 * h          (K)
//! p = 101325 * (T / 288.15)^5.256  (Pa)
//! rho = p / (287.053 * T)           (kg/m^3)
//! ```
//!
//! **Stratosphere (11 000 m < h <= 20 000 m)**: temperature is constant (isothermal
//! layer at -56.5 C), pressure decays exponentially with altitude (barometric formula),
//! density again from the ideal gas law:
//!
//! ```text
//! T = 216.65                                    (K, isothermal)
//! p = p₁₁ * exp(-g * (h−h₁₁) / (R * T₁₁))       (Pa)
//! rho = p / (R * T)                             (kg/m^3)
//! ```
//!
//! ## Sutherland's law (dynamic viscosity)
//!
//! Gas viscosity *increases* with temperature (unlike liquids, gas molecules
//! collide more at higher T, transferring more momentum across flow layers).
//! **Dynamic viscosity = reference viscosity × (T/273)^(3/2) corrected by
//! Sutherland's constant (110.4 K) for real-gas behaviour. See:
//! Sutherland's law viscosity.**
//!
//! ```text
//! mu = μ = μ_ref * (T/T_ref)^(3/2) * (T_ref + S) / (T + S)
//! μ_ref = 1.716×10⁻⁵ kg/(m·s),  T_ref = 273.15 K,  S = 110.4 K
//! ```

use crate::_bevy::*;
use crate::components::{AtmosphereState, FlightState, WindResource};
use avian3d::math::{Scalar, Vector};

//
// ISA constants
//

/// Sea-level temperature (K).
const T0: Scalar = 288.15;
/// Sea-level pressure (Pa).
const P0: Scalar = 101_325.0;
/// Tropospheric lapse rate (K/m).
const L: Scalar = 0.006_5;
/// Tropopause altitude (m).
const H_TROP: Scalar = 11_000.0;
/// Tropopause temperature (K).
const T_TROP: Scalar = 216.65;
/// Specific gas constant for dry air (J/(kg·K)).
const R_AIR: Scalar = 287.052_87;
/// Standard gravity (m/s²).
const G: Scalar = 9.806_65;
/// Adiabatic index of air.
const GAMMA: Scalar = 1.4;

/// Pressure exponent in the troposphere: g / (R · L).
const TROP_EXPONENT: Scalar = G / (R_AIR * L); // ~ 5.2559

/// Sutherland reference temperature (K).
const T_REF_SUTH: Scalar = 273.15;
/// Sutherland reference dynamic viscosity (kg/(m·s)).
const MU_REF_SUTH: Scalar = 1.716e-5;
/// Sutherland constant S (K).
const S_SUTH: Scalar = 110.4;

//
// Pure ISA functions (public for testing and external use)
//

/// Compute ISA temperature (K) at geometric altitude `h` (m).
pub fn isa_temperature(h: Scalar) -> Scalar {
    if h <= H_TROP {
        T0 - L * h
    } else {
        T_TROP
    }
}

/// Compute ISA static pressure (Pa) at geometric altitude `h` (m).
pub fn isa_pressure(h: Scalar) -> Scalar {
    if h <= H_TROP {
        let t = isa_temperature(h);
        P0 * (t / T0).powf(TROP_EXPONENT)
    } else {
        // Pressure at the tropopause boundary.
        let p_trop = P0 * (T_TROP / T0).powf(TROP_EXPONENT);
        p_trop * ((-G * (h - H_TROP)) / (R_AIR * T_TROP)).exp()
    }
}

/// Compute ISA air density (kg/m³) at geometric altitude `h` (m).
#[cfg(test)]
pub(crate) fn isa_density(h: Scalar) -> Scalar {
    let t = isa_temperature(h);
    let p = isa_pressure(h);
    p / (R_AIR * t)
}

/// Compute speed of sound (m/s) at temperature `t` (K).
pub fn speed_of_sound(t: Scalar) -> Scalar {
    (GAMMA * R_AIR * t).sqrt()
}

/// Compute dynamic viscosity (kg/(m·s)) via Sutherland's law at temperature `t` (K).
pub fn sutherland_viscosity(t: Scalar) -> Scalar {
    MU_REF_SUTH * (t / T_REF_SUTH).powf(1.5) * (T_REF_SUTH + S_SUTH) / (t + S_SUTH)
}

/// Populate an [`AtmosphereState`] for the given altitude (m).
pub fn atmosphere_at(h: Scalar) -> AtmosphereState {
    let temperature_k = isa_temperature(h);
    let pressure_pa = isa_pressure(h);
    let density_kgm3 = pressure_pa / (R_AIR * temperature_k);
    let speed_of_sound_ms = speed_of_sound(temperature_k);
    AtmosphereState {
        density_kgm3,
        pressure_pa,
        temperature_k,
        speed_of_sound_ms,
    }
}

//
// Bevy systems
//

/// Updates [`AtmosphereState`] on each aircraft from its world-space altitude.
///
/// Reads `GlobalTransform.translation().y` as geometric altitude above sea level.
#[allow(clippy::unnecessary_cast)]
pub fn update_atmosphere(mut query: Query<(&GlobalTransform, &mut AtmosphereState)>) {
    for (transform, mut atm) in &mut query {
        let altitude_m = transform.translation().y as Scalar;
        *atm = atmosphere_at(altitude_m);
    }
}

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

//
// Unit tests
//

#[cfg(test)]
mod tests {
    use super::*;

    /// Tolerance for ISA validation: ±0.1% on density, ±0.5% on pressure.
    /// Reference values from ICAO Doc 7488 / standard ISA tables.
    const DENS_TOL: Scalar = 0.001;
    const PRES_TOL: Scalar = 0.005;
    const TEMP_TOL: Scalar = 0.001;

    fn pct_err(got: Scalar, expected: Scalar) -> Scalar {
        (got - expected).abs() / expected
    }

    #[test]
    fn isa_sea_level() {
        // ICAO: T=288.15 K, p=101325 Pa, ρ=1.2250 kg/m³
        let atm = atmosphere_at(0.0);
        assert!(
            pct_err(atm.temperature_k, 288.15) < TEMP_TOL,
            "T₀ {}",
            atm.temperature_k
        );
        assert!(
            pct_err(atm.pressure_pa, 101_325.0) < PRES_TOL,
            "p₀ {}",
            atm.pressure_pa
        );
        assert!(
            pct_err(atm.density_kgm3, 1.2250) < DENS_TOL,
            "ρ₀ {}",
            atm.density_kgm3
        );
    }

    #[test]
    fn isa_1000m() {
        // ICAO: T=281.65 K, p=89874 Pa, ρ=1.1117 kg/m³
        let atm = atmosphere_at(1_000.0);
        assert!(pct_err(atm.temperature_k, 281.65) < TEMP_TOL);
        assert!(pct_err(atm.pressure_pa, 89_874.0) < PRES_TOL);
        assert!(pct_err(atm.density_kgm3, 1.1117) < DENS_TOL);
    }

    #[test]
    fn isa_5000m() {
        // ICAO: T=255.65 K, p=54048 Pa, ρ=0.7364 kg/m³
        let atm = atmosphere_at(5_000.0);
        assert!(pct_err(atm.temperature_k, 255.65) < TEMP_TOL);
        assert!(pct_err(atm.pressure_pa, 54_048.0) < PRES_TOL);
        assert!(pct_err(atm.density_kgm3, 0.7364) < DENS_TOL);
    }

    #[test]
    fn isa_11000m_tropopause() {
        // ICAO: T=216.65 K, p=22632 Pa, ρ=0.3639 kg/m³
        let atm = atmosphere_at(11_000.0);
        assert!(pct_err(atm.temperature_k, 216.65) < TEMP_TOL);
        assert!(pct_err(atm.pressure_pa, 22_632.0) < PRES_TOL);
        assert!(pct_err(atm.density_kgm3, 0.3639) < DENS_TOL);
    }

    #[test]
    fn isa_stratosphere_is_isothermal() {
        // Above 11 km temperature must stay at 216.65 K.
        assert!((isa_temperature(15_000.0) - 216.65).abs() < 0.01);
        assert!((isa_temperature(20_000.0) - 216.65).abs() < 0.01);
    }

    #[test]
    fn speed_of_sound_sea_level() {
        // ICAO: a₀ = 340.294 m/s at sea level
        let a = speed_of_sound(288.15);
        assert!((a - 340.294).abs() < 0.01, "a₀ = {a}");
    }

    #[test]
    fn sutherland_sea_level() {
        // Standard value: μ ≈ 1.789×10⁻⁵ kg/(m·s) at 288.15 K
        let mu = sutherland_viscosity(288.15);
        assert!((mu - 1.789e-5).abs() / 1.789e-5 < 0.005, "μ = {mu:.4e}");
    }

    #[test]
    fn density_decreases_with_altitude() {
        assert!(isa_density(0.0) > isa_density(5_000.0));
        assert!(isa_density(5_000.0) > isa_density(11_000.0));
        assert!(isa_density(11_000.0) > isa_density(15_000.0));
    }
}
