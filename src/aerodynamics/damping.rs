//! Step 4 (LOD fallback): whole-aircraft angular-rate damping torque.

use bevy_math::{DQuat, DVec3};

use crate::components::{AircraftGeometry, FlightState, LodDamping};

/// Compute whole-aircraft angular-rate damping torque in world coordinates.
///
/// This is a **LOD (Level of Detail) fallback** for aircraft with too few
/// zones to produce realistic physical damping from geometry alone. At full
/// fidelity, use [`zone_local_angles`][super::local_angles::zone_local_angles]
/// per zone instead; the differential forces from wing and tail zones naturally
/// oppose body rates without any explicit derivatives.
///
/// # When to use
///
/// Supply a [`LodDamping`] value only for Single-zone bodies (missiles, pylons)
/// or low-fidelity AI aircraft with minimal zone layouts.
///
/// For any aircraft with realistic wing, h-stab, and v-tail zones, leave
/// `lod_damping = None` and let zone physics do the work.
///
/// # How it works
///
/// - **Roll damping (Cl_p):** Descending wing sees increased AoA → more lift;
///   rising wing sees less.  Differential lift opposes the roll.
/// - **Pitch damping (Cm_q):** Nose-up pitch drives the tail downward,
///   increasing its AoA and generating a restoring nose-down moment.
/// - **Yaw damping (Cn_r):** Right yaw sweeps the vertical tail into a
///   sideslip that generates a leftward (restoring) force.
///
/// # Non-dimensional form
///
/// **Damping moment = derivative × normalised rate × q̄ · S · length.**
/// All derivatives are negative (oppose motion):
///
/// ```text
/// ΔL = Cl_p · (p · b / 2V) · q̄ · S · b
/// ΔM = Cm_q · (q · c̄ / 2V) · q̄ · S · c̄
/// ΔN = Cn_r · (r · b / 2V) · q̄ · S · b
/// ```
///
/// Typical values (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.
pub fn damping_torque(
    flight: &FlightState,
    lod: &LodDamping,
    geo: &AircraftGeometry,
    body_to_world: DQuat,
) -> DVec3 {
    let v = flight.airspeed_ms;
    let qbar = flight.dynamic_pressure_pa;
    let s = geo.wing_area_m2;
    let b = geo.wing_span_m;
    let c = geo.chord_m;

    let damp_body = DVec3::new(
        lod.cl_p * (flight.p_rads * b / (2.0 * v)) * qbar * s * b,
        lod.cm_q * (flight.q_rads * c / (2.0 * v)) * qbar * s * c,
        lod.cn_r * (flight.r_rads * b / (2.0 * v)) * qbar * s * b,
    );

    body_to_world * damp_body
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::LodDamping;
    use bevy_math::DQuat;

    fn geo() -> AircraftGeometry {
        AircraftGeometry {
            wing_span_m: 10.0,
            chord_m: 1.6,
            wing_area_m2: 16.0,
        }
    }

    fn lod_full() -> LodDamping {
        LodDamping {
            cl_p: -0.45,
            cm_q: -12.0,
            cn_r: -0.12,
        }
    }

    fn flight_rates(p: f64, q: f64, r: f64) -> FlightState {
        FlightState {
            p_rads: p,
            q_rads: q,
            r_rads: r,
            airspeed_ms: 50.0,
            dynamic_pressure_pa: 1531.0,
            ..Default::default()
        }
    }

    #[test]
    fn roll_damping_opposes_roll_rate() {
        let damp = damping_torque(
            &flight_rates(1.0, 0.0, 0.0),
            &LodDamping {
                cl_p: -0.45,
                cm_q: 0.0,
                cn_r: 0.0,
            },
            &geo(),
            DQuat::IDENTITY,
        );
        assert!(
            damp.x < 0.0,
            "roll damping should oppose positive p, got {}",
            damp.x
        );
    }

    #[test]
    fn zero_rates_produce_zero_damping() {
        let damp = damping_torque(
            &flight_rates(0.0, 0.0, 0.0),
            &lod_full(),
            &geo(),
            DQuat::IDENTITY,
        );
        assert!(
            damp.length() < 1e-10,
            "zero rates should produce zero damping"
        );
    }

    #[test]
    fn all_axes_damping_combine_independently() {
        let damp = damping_torque(
            &flight_rates(1.0, 1.0, 1.0),
            &lod_full(),
            &geo(),
            DQuat::IDENTITY,
        );
        assert!(
            damp.x < 0.0,
            "roll damping should oppose p>0, got x={}",
            damp.x
        );
        assert!(
            damp.y < 0.0,
            "pitch damping should oppose q>0, got y={}",
            damp.y
        );
        assert!(
            damp.z < 0.0,
            "yaw damping should oppose r>0, got z={}",
            damp.z
        );
        assert!(
            damp.z.abs() < damp.x.abs(),
            "yaw damp weaker than roll (|cn_r| < |cl_p|), z={}, x={}",
            damp.z,
            damp.x
        );
    }

    /// Rotation should change damping direction but not magnitude.
    #[test]
    fn damping_torque_rotates_with_body() {
        let flight = flight_rates(1.0, 0.0, 0.0);
        let lod = LodDamping {
            cl_p: -0.45,
            cm_q: 0.0,
            cn_r: 0.0,
        };
        let identity = damping_torque(&flight, &lod, &geo(), DQuat::IDENTITY);
        let rotated = damping_torque(
            &flight,
            &lod,
            &geo(),
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        assert!(
            (rotated.length() - identity.length()).abs() < 1e-5,
            "rotation should not change damping magnitude"
        );
    }
}
