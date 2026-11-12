//! ISA (International Standard Atmosphere) model, as pure functions.
//!
//! This module implements the International Standard Atmosphere (ICAO Doc
//! 7488) for the troposphere (0–11 km) and the lower stratosphere (11–20 km).
//! It also implements Sutherland's law for the dynamic viscosity. It contains
//! no Bevy types. Refer to [`super::systems`] for the ECS systems that use
//! these functions.
//!
//! ## Geometric and geopotential altitude
//!
//! The geometric altitude `h` is the true height above sea level. The crate
//! uses it everywhere, for example in
//! [`FlightState::altitude_m`](crate::components::FlightState::altitude_m).
//!
//! Real gravity decreases with height. The ISA formulas use one constant
//! gravity `g0`. Thus ICAO defines the ISA in the geopotential altitude `H`.
//!
//! The geopotential altitude `H` is the height in a fictional world with the
//! constant gravity `g0`, where a body has the same potential energy as at
//! the geometric height `h`:
//!
//! ```text
//! H = r0 * h / (r0 + h)        r0 = 6 356 766 m (ICAO)
//! ```
//!
//! [`atmosphere_at`] takes `h` and converts it to `H` with
//! [`geopotential_altitude`]. The other ISA functions take `H`. For example,
//! the tropopause is at `H` = 11 000 m, that is `h` = 11 019 m.
//!
//! `r0` is part of the ISA definition. It is not the radius of the app world
//! ([`PlanetResource::radius_m`](crate::components::PlanetResource::radius_m)).
//! A change of the world radius does not change the standard atmosphere.
//!
//! ## Why density is important
//!
//! Each aerodynamic force is proportional to the dynamic pressure (qbar =
//! 0.5 * density * airspeed^2). If the density decreases by 20% (at
//! approximately 2 500 m altitude), lift and drag also decrease by 20% at the
//! same speed. The Reynolds number (density * speed * chord / viscosity)
//! controls how the air flows over the surface. At low Reynolds numbers, the
//! flow separates earlier. The stall is then sharper, and the drag is higher.
//!
//! ## ISA formulas (ICAO Doc 7488)
//!
//! All altitudes in the formulas are geopotential.
//!
//! **Troposphere (H <= 11 000 m)**: the temperature decreases linearly
//! (6.5 K/km). The pressure obeys a power law of the temperature. The density
//! rho comes from the ideal gas law:
//!
//! ```text
//! T = 288.15 - 0.0065 * H          (K)
//! p = 101325 * (T / 288.15)^5.256  (Pa)
//! rho = p / (287.053 * T)           (kg/m^3)
//! ```
//!
//! **Stratosphere (11 000 m < H <= 20 000 m)**: the temperature is constant
//! (isothermal layer at -56.5 °C). The pressure decreases exponentially with
//! altitude (barometric formula). The density again comes from the ideal gas
//! law:
//!
//! ```text
//! T = 216.65                                    (K, isothermal)
//! p = p_11 * exp(-g * (H - H_11) / (R * T_11))  (Pa)
//! rho = p / (R * T)                             (kg/m^3)
//! ```
//!
//! ## Sutherland's law (dynamic viscosity)
//!
//! The viscosity of a gas *increases* with the temperature. Liquids are
//! different. At a higher T, gas molecules collide more frequently. Thus, they
//! move more momentum between the flow layers. **Dynamic viscosity =
//! reference viscosity × (T/T_ref)^(3/2), with a correction by Sutherland's
//! constant (110.4 K) for real-gas behavior.** Related topic: Sutherland's law
//! for viscosity.
//!
//! ```text
//! mu     = mu_ref * (T/T_ref)^(3/2) * (T_ref + S) / (T + S)
//! mu_ref = 1.716e-5 kg/(m·s),  T_ref = 273.15 K,  S = 110.4 K
//! ```

use crate::components::AtmosphereState;

//
// ISA constants
//

/// Sea-level temperature (K).
const T0: f32 = 288.15;
/// Sea-level pressure (Pa).
const P0: f32 = 101_325.0;
/// Tropospheric lapse rate (K/m).
const L: f32 = 0.006_5;
/// Tropopause altitude (geopotential m).
const H_TROP: f32 = 11_000.0;
/// Tropopause temperature (K).
const T_TROP: f32 = 216.65;
/// Specific gas constant for dry air (J/(kg·K)).
const R_AIR: f32 = 287.052_87;
/// Standard gravity (m/s²).
const G: f32 = 9.806_65;
/// Earth radius of the geopotential altitude conversion (m). At this radius,
/// gravity is `G`. This is not the radius of the app world.
const R0_ISA: f32 = sourced!(
    6_356_766.0,
    "ICAO Doc 7488 r0; also JSBSim FGStandardAtmosphere::EarthRadius"
);
/// Adiabatic index of air.
const GAMMA: f32 = 1.4;

/// Pressure exponent in the troposphere: g / (R · L).
const TROP_EXPONENT: f32 = G / (R_AIR * L); // ~ 5.2559

/// Sutherland reference temperature (K).
const T_REF_SUTH: f32 = 273.15;
/// Sutherland reference dynamic viscosity (kg/(m·s)).
const MU_REF_SUTH: f32 = 1.716e-5;
/// Sutherland constant S (K).
const S_SUTH: f32 = 110.4;

//
// Pure ISA functions. The module `atmosphere` is `pub(crate)`, so they are
// crate-internal. `atmosphere_at` takes the geometric altitude. The other
// altitude functions take the geopotential altitude.
//

/// Converts the geometric altitude `h` (m) to the geopotential altitude `H` (m).
pub(crate) fn geopotential_altitude(h: f32) -> f32 {
    R0_ISA * h / (R0_ISA + h)
}

/// Calculates the ISA temperature (K) at the geopotential altitude `h_gp` (m).
pub(crate) fn isa_temperature(h_gp: f32) -> f32 {
    if h_gp <= H_TROP {
        T0 - L * h_gp
    } else {
        T_TROP
    }
}

/// Calculates the ISA static pressure (Pa) at the geopotential altitude `h_gp` (m).
pub(crate) fn isa_pressure(h_gp: f32) -> f32 {
    if h_gp <= H_TROP {
        let t = isa_temperature(h_gp);
        P0 * (t / T0).powf(TROP_EXPONENT)
    } else {
        // Pressure at the tropopause boundary.
        let p_trop = P0 * (T_TROP / T0).powf(TROP_EXPONENT);
        p_trop * ((-G * (h_gp - H_TROP)) / (R_AIR * T_TROP)).exp()
    }
}

/// Calculates the ISA air density (kg/m³) at the geopotential altitude `h_gp` (m).
#[cfg(test)]
pub(crate) fn isa_density(h_gp: f32) -> f32 {
    let t = isa_temperature(h_gp);
    let p = isa_pressure(h_gp);
    p / (R_AIR * t)
}

/// Calculates the speed of sound (m/s) at the temperature `t` (K).
pub(crate) fn speed_of_sound(t: f32) -> f32 {
    (GAMMA * R_AIR * t).sqrt()
}

/// Calculates the dynamic viscosity (kg/(m·s)) with Sutherland's law at the temperature `t` (K).
pub(crate) fn sutherland_viscosity(t: f32) -> f32 {
    MU_REF_SUTH * (t / T_REF_SUTH).powf(1.5) * (T_REF_SUTH + S_SUTH) / (t + S_SUTH)
}

/// Returns an [`AtmosphereState`] for the geometric altitude `h` (m).
///
/// It converts `h` to the geopotential altitude with
/// [`geopotential_altitude`], then applies the ISA formulas.
pub(crate) fn atmosphere_at(h: f32) -> AtmosphereState {
    let h_gp = geopotential_altitude(h);
    let temperature_k = isa_temperature(h_gp);
    let pressure_pa = isa_pressure(h_gp);
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
// Unit tests
//

#[cfg(test)]
mod tests {
    use super::*;

    /// Tolerance for ISA validation: ±0.1% on density, ±0.5% on pressure.
    /// Reference values from ICAO Doc 7488 / standard ISA tables.
    const DENS_TOL: f32 = 0.001;
    const PRES_TOL: f32 = 0.005;
    const TEMP_TOL: f32 = 0.001;

    fn pct_err(got: f32, expected: f32) -> f32 {
        (got - expected).abs() / expected
    }

    /// The ICAO tables list geopotential altitudes. `atmosphere_at` takes
    /// geometric altitudes. This is the inverse of `geopotential_altitude`.
    fn geometric_altitude(h_gp: f32) -> f32 {
        R0_ISA * h_gp / (R0_ISA - h_gp)
    }

    #[test]
    fn geopotential_altitude_matches_icao_table() {
        // Geometric altitudes of the ISA layer boundaries: 11 019 m and 20 063 m.
        assert_eq!(geopotential_altitude(0.0), 0.0);
        assert!((geopotential_altitude(11_019.07) - 11_000.0).abs() < 0.01);
        assert!((geopotential_altitude(20_063.1) - 20_000.0).abs() < 0.05);
        assert!((geopotential_altitude(1_000.0) - 999.84).abs() < 0.01);
    }

    #[test]
    fn geopotential_altitude_round_trip() {
        for h in [-500.0, 0.5, 1_000.3, 3_000.0, 10_000.0, 20_000.0] {
            let back = geometric_altitude(geopotential_altitude(h));
            assert!((back - h).abs() < 2e-3, "h {h}, round trip {back}");
        }
    }

    /// The tropopause is at the geometric altitude 11 019 m, not 11 000 m.
    #[test]
    fn atmosphere_at_converts_to_geopotential() {
        let at_tropopause = atmosphere_at(geometric_altitude(H_TROP));
        assert!((at_tropopause.temperature_k - T_TROP).abs() < 0.01);

        // Without the conversion, this is T_TROP.
        let below = atmosphere_at(H_TROP);
        let expected = T0 - L * geopotential_altitude(H_TROP);
        assert!((below.temperature_k - expected).abs() < 0.01);
        assert!(below.temperature_k > T_TROP + 0.1);
    }

    #[test]
    fn isa_sea_level() {
        // ICAO: T = 288.15 K, p = 101325 Pa, rho = 1.2250 kg/m3
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
        // ICAO: T = 281.65 K, p = 89874 Pa, rho = 1.1117 kg/m3
        let atm = atmosphere_at(geometric_altitude(1_000.0));
        assert!(pct_err(atm.temperature_k, 281.65) < TEMP_TOL);
        assert!(pct_err(atm.pressure_pa, 89_874.0) < PRES_TOL);
        assert!(pct_err(atm.density_kgm3, 1.1117) < DENS_TOL);
    }

    #[test]
    fn isa_5000m() {
        // ICAO: T = 255.65 K, p = 54048 Pa, rho = 0.7364 kg/m3
        let atm = atmosphere_at(geometric_altitude(5_000.0));
        assert!(pct_err(atm.temperature_k, 255.65) < TEMP_TOL);
        assert!(pct_err(atm.pressure_pa, 54_048.0) < PRES_TOL);
        assert!(pct_err(atm.density_kgm3, 0.7364) < DENS_TOL);
    }

    #[test]
    fn isa_11000m_tropopause() {
        // ICAO: T = 216.65 K, p = 22632 Pa, rho = 0.3639 kg/m3
        let atm = atmosphere_at(geometric_altitude(11_000.0));
        assert!(pct_err(atm.temperature_k, 216.65) < TEMP_TOL);
        assert!(pct_err(atm.pressure_pa, 22_632.0) < PRES_TOL);
        assert!(pct_err(atm.density_kgm3, 0.3639) < DENS_TOL);
    }

    #[test]
    fn isa_stratosphere_is_isothermal() {
        // Above 11 km, the temperature must stay at 216.65 K.
        assert!((isa_temperature(15_000.0) - 216.65).abs() < 0.01);
        assert!((isa_temperature(20_000.0) - 216.65).abs() < 0.01);
    }

    #[test]
    fn speed_of_sound_sea_level() {
        // ICAO: a_0 = 340.294 m/s at sea level
        let a = speed_of_sound(288.15);
        assert!((a - 340.294).abs() < 0.01, "a₀ = {a}");
    }

    #[test]
    fn sutherland_sea_level() {
        // Standard value: mu = approximately 1.789e-5 kg/(m*s) at 288.15 K
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
