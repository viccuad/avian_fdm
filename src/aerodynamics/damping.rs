//! Step 5 (LOD fallback): angular-rate damping torque of the full aircraft.

use crate::components::{AircraftGeometry, FlightState, LodDamping};
use bevy_math::{Quat, Vec3};

/// Calculates the angular-rate damping torque of the full aircraft, in world
/// coordinates.
///
/// This is a **LOD (level of detail) fallback**. It is for aircraft with too
/// few zones to make realistic physical damping from geometry. For full
/// detail, use [`zone_local_angles`][super::local_angles::zone_local_angles]
/// for each zone. The differences between the forces of the wing and tail
/// zones then oppose the body rates without explicit derivatives.
///
/// # When to use it
///
/// Add a [`LodDamping`] component only for single-zone bodies (missiles,
/// pylons) or for low-detail AI aircraft with few zones.
///
/// For aircraft with realistic wing, horizontal stabilizer, and vertical tail
/// zones, do not add `LodDamping`. The zone physics then make the damping.
///
/// # How it works
///
/// - **Roll damping (Cl_p):** The descending wing gets a higher AoA and more
///   lift. The rising wing gets less. The difference in lift opposes the roll.
/// - **Pitch damping (Cm_q):** A nose-up pitch moves the tail down. Its AoA
///   increases, and it makes a restoring nose-down moment.
/// - **Yaw damping (Cn_r):** A right yaw moves the vertical tail into a
///   sideslip. The sideslip makes a restoring force to the left.
///
/// # Non-dimensional form
///
/// **Damping moment = derivative × normalized rate × qbar × S × length.**
/// All derivatives are negative (they oppose the motion):
///
/// ```text
/// delta_L = Cl_p * (p * b / 2V)     * qbar * S * b
/// delta_M = Cm_q * (q * c_bar / 2V) * qbar * S * c_bar
/// delta_N = Cn_r * (r * b / 2V)     * qbar * S * b
/// ```
///
/// Typical values (Nelson 1998, Table B1):
/// `Cl_p ≈ −0.45`, `Cm_q ≈ −12.0`, `Cn_r ≈ −0.12`.
pub fn damping_torque(
    flight: &FlightState,
    lod: &LodDamping,
    geo: &AircraftGeometry,
    body_to_world: Quat,
) -> Vec3 {
    let v = flight.airspeed_ms;
    let qbar = flight.dynamic_pressure_pa;
    let s = geo.wing_area_m2;
    let b = geo.wing_span_m;
    let c = geo.chord_m;

    let damp_body = Vec3::new(
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

    fn flight_rates(p: f32, q: f32, r: f32) -> FlightState {
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
            Quat::IDENTITY,
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
            Quat::IDENTITY,
        );
        assert!(
            damp.length() < 1e-5,
            "zero rates should produce zero damping"
        );
    }

    #[test]
    fn all_axes_damping_combine_independently() {
        let damp = damping_torque(
            &flight_rates(1.0, 1.0, 1.0),
            &lod_full(),
            &geo(),
            Quat::IDENTITY,
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
        let identity = damping_torque(&flight, &lod, &geo(), Quat::IDENTITY);
        let rotated = damping_torque(
            &flight,
            &lod,
            &geo(),
            Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
        );
        assert!(
            (rotated.length() - identity.length()).abs() < 1e-3,
            "rotation should not change damping magnitude"
        );
    }
}
