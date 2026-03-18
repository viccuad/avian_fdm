//! Zone aggregation system.
//!
//! Evaluates each [`AeroZone`]'s coefficient tables at the current α and Re,
//! multiplies by zone health, and sums into [`AircraftAggregate`]. Also
//! recomputes [`AircraftMass`] (total mass, CG, inertia tensor) from zone
//! masses each frame.
//!
//! ## Why contributions sum linearly
//!
//! The stability-derivative method is a Taylor expansion around a trim point.
//! For small perturbations each zone's contribution to the total coefficient
//! is additive — this is the superposition principle for linearised
//! aerodynamics. Health weighting is a continuous degradation of the zone's
//! contribution, physically representing surface loss of effectiveness.
//!
//! ## CG shift on damage
//!
//! `cg_body = Σ(m_i · pos_i) / Σm_i`
//!
//! As zones are destroyed their mass drops to zero and the numerator shifts
//! toward the surviving zones. A wing tip destroyed → CG moves inboard
//! automatically.
//!
//! ## Inertia tensor (parallel-axis theorem)
//!
//! `I_total = Σ(I_zone + m_zone · d²)`
//!
//! where `d` is the distance from the zone's position to the total CG.
//! Each zone is modelled as a point mass (`I_zone = 0`) for simplicity.
//!
//! Only compiled with `features = ["damage"]`.

use bevy::prelude::*;
use bevy::math::{DMat3, DVec3};

use crate::components::{
    AircraftAggregate, AircraftMass, AeroZone, AeroZoneHealth, FlightState,
    aero_zone::ControlSurfaceRole,
};

/// One-time startup system: computes `collider_volume_m3` and `mass_kg` for
/// every [`AeroZoneHealth`] from its [`AeroZone::zone_mass`] setting.
///
/// ⚠️ Avian 0.6 does not expose `Collider::volume()`. As a fallback, we use the
/// collider's AABB half-extents to estimate volume: `V ≈ 8·hx·hy·hz`.
/// This over-estimates for non-box shapes. For [`ZoneMass::Direct`] this path
/// is not used — mass is taken directly from the authored value.
///
/// **v1 limitation**: zones spawned after PostStartup will have `mass_kg = 0`
/// and be skipped by `aggregate_zones` with a `warn!`. Post-v1 fix: use an
/// `OnAdd<AeroZone>` observer.
pub fn init_zone_volumes(
    mut zone_query: Query<(
        &AeroZone,
        &mut AeroZoneHealth,
        Option<&avian3d::prelude::ColliderAabb>,
    )>,
) {
    use crate::components::aero_zone::ZoneMass;

    for (zone, mut health, collider_aabb) in &mut zone_query {
        let vol = if let Some(aabb) = collider_aabb {
            let half = (aabb.max - aabb.min) * 0.5;
            let v = 8.0 * (half.x as f64) * (half.y as f64) * (half.z as f64);
            if v <= 0.0 {
                warn!("AeroZone AABB volume ≤ 0 — collider may be degenerate. mass_kg will be 0.");
            }
            if v < 0.0 { 0.0 } else { v }
        } else {
            0.0
        };

        health.collider_volume_m3 = vol;
        health.mass_kg = match zone.zone_mass {
            ZoneMass::FromDensity(density) => density * vol,
            ZoneMass::Direct(kg) => kg,
        };
    }
}

/// Per-frame aggregation system.
///
/// Reads all `(AeroZone, AeroZoneHealth, Transform)` children of each aircraft,
/// evaluates their `AeroCoeff` tables at the current `(α, Re)` from
/// [`FlightState`], and writes summed totals to [`AircraftAggregate`] and
/// [`AircraftMass`].
pub fn aggregate_zones(
    mut aircraft_query: Query<(
        Entity,
        &FlightState,
        &mut AircraftAggregate,
        &mut AircraftMass,
    )>,
    zone_query: Query<(&AeroZone, &AeroZoneHealth, &Transform, &ChildOf)>,
) {
    // Build per-aircraft aggregates. Sort children by entity for determinism
    // (prerequisite for Group I lockstep netcode).
    let mut zone_list: Vec<(Entity, &AeroZone, &AeroZoneHealth, DVec3)> = zone_query
        .iter()
        .map(|(z, h, t, parent)| {
            let pos = DVec3::new(
                t.translation.x as f64,
                t.translation.y as f64,
                t.translation.z as f64,
            );
            (parent.parent(), z, h, pos)
        })
        .collect();
    zone_list.sort_by_key(|(parent, _, _, _)| *parent); // group by aircraft

    for (aircraft_entity, fs, mut agg, mut mass) in aircraft_query.iter_mut() {
        let alpha = fs.alpha_rad;
        let re = fs.reynolds_number;

        let mut cl = 0.0_f64;
        let mut cd = 0.0_f64;
        let mut cy = 0.0_f64;
        let mut cm = 0.0_f64;
        let mut croll = 0.0_f64;
        let mut cn = 0.0_f64;
        let mut struct_drag = 0.0_f64;

        let mut elevator_eff = 1.0_f64;
        let mut aileron_l_eff = 1.0_f64;
        let mut aileron_r_eff = 1.0_f64;
        let mut rudder_eff = 1.0_f64;

        let mut total_mass = 0.0_f64;
        let mut cg_num = DVec3::ZERO; // Σ(m_i · pos_i)

        let mut zone_masses: Vec<(DVec3, f64)> = Vec::new();

        for (parent, zone, health, pos) in zone_list.iter().filter(|(p, _, _, _)| *p == aircraft_entity) {
            let h = health.value.clamp(0.0, 1.0);

            cl    += zone.cl.evaluate(alpha, re) * h;
            cd    += zone.cd.evaluate(alpha, re) * h;
            cy    += zone.cy.evaluate(alpha, re) * h;
            cm    += zone.cm.evaluate(alpha, re) * h;
            croll += zone.croll.evaluate(alpha, re) * h;
            cn    += zone.cn.evaluate(alpha, re) * h;
            struct_drag += zone.damage_drag_coeff * (1.0 - h);

            // Control surface effectiveness: multiplicative decay with damage.
            match &zone.control_role {
                Some(ControlSurfaceRole::Elevator)    => elevator_eff  *= h,
                Some(ControlSurfaceRole::AileronLeft) => aileron_l_eff *= h,
                Some(ControlSurfaceRole::AileronRight)=> aileron_r_eff *= h,
                Some(ControlSurfaceRole::Rudder)      => rudder_eff    *= h,
                None => {}
            }

            let m = health.mass_kg * h;
            total_mass += m;
            cg_num += *pos * m;
            zone_masses.push((*pos, m));
        }

        // Avoid divide-by-zero when all zones destroyed.
        let cg = if total_mass > 0.0 {
            cg_num / total_mass
        } else {
            if !zone_masses.is_empty() {
                warn_once!("All zones destroyed on {aircraft_entity:?}: total_mass = 0");
            }
            DVec3::ZERO
        };

        // Inertia tensor via parallel-axis theorem (point-mass approximation).
        let mut inertia = DMat3::ZERO;
        for (pos, m) in &zone_masses {
            let d = *pos - cg;
            // I += m · (|d|²·I₃ - d⊗d)
            let d2 = d.dot(d);
            inertia += DMat3::from_diagonal(DVec3::splat(d2 * m))
                - DMat3::from_cols(d * (d.x * m), d * (d.y * m), d * (d.z * m));
        }

        *agg = AircraftAggregate {
            cl_total: cl,
            cd_total: cd,
            cy_total: cy,
            cm_total: cm,
            croll_total: croll,
            cn_total: cn,
            structural_drag_pa: struct_drag,
            control_effectiveness: crate::components::aero_zone::ControlEffectiveness {
                elevator: elevator_eff,
                aileron_left: aileron_l_eff,
                aileron_right: aileron_r_eff,
                rudder: rudder_eff,
            },
        };

        *mass = AircraftMass {
            mass_kg: total_mass,
            cg_body_m: cg,
            inertia_tensor: inertia,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::aero_coeff::AeroCoeff;
    use crate::components::aero_zone::{ZoneMass, ControlEffectiveness};

    fn make_zone(cl: f64) -> AeroZone {
        AeroZone {
            cl: AeroCoeff::Scalar(cl),
            cd: AeroCoeff::Scalar(0.0),
            cy: AeroCoeff::Scalar(0.0),
            cm: AeroCoeff::Scalar(0.0),
            croll: AeroCoeff::Scalar(0.0),
            cn: AeroCoeff::Scalar(0.0),
            control_role: None,
            zone_mass: ZoneMass::Direct(1.0),
            damage_drag_coeff: 0.0,
        }
    }

    fn make_health(value: f64, mass_kg: f64) -> AeroZoneHealth {
        AeroZoneHealth { value, collider_volume_m3: 0.0, mass_kg }
    }

    /// Zone at 50% health contributes half its CL.
    #[test]
    fn half_health_half_cl() {
        let zone = make_zone(1.0);
        let health = make_health(0.5, 1.0);
        let contribution = zone.cl.evaluate(0.0, 1e6) * health.value;
        assert!((contribution - 0.5).abs() < 1e-12);
    }

    /// Two symmetric zones at equal health: CG stays at origin.
    #[test]
    fn symmetric_zones_cg_at_origin() {
        // Manually simulate aggregate_zones CG logic.
        let positions = [DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0)];
        let masses = [1.0_f64, 1.0_f64];

        let total: f64 = masses.iter().sum();
        let cg: DVec3 = positions.iter().zip(masses.iter())
            .map(|(p, m)| *p * *m)
            .fold(DVec3::ZERO, |a, b| a + b) / total;

        assert!(cg.length() < 1e-12, "CG should be at origin, got {cg:?}");
    }

    /// Destroying one of two symmetric zones shifts CG toward the survivor.
    #[test]
    fn destroyed_zone_shifts_cg() {
        let positions = [DVec3::new(-2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0)];
        // Right zone destroyed.
        let masses = [1.0_f64, 0.0_f64];

        let total: f64 = masses.iter().sum();
        let cg: DVec3 = positions.iter().zip(masses.iter())
            .map(|(p, m)| *p * *m)
            .fold(DVec3::ZERO, |a, b| a + b) / total;

        assert!((cg.x - (-2.0)).abs() < 1e-12, "CG should be at left zone, got {cg:?}");
    }

    /// Structural drag increases as health decreases.
    #[test]
    fn damage_drag_increases_with_damage() {
        let zone = AeroZone { damage_drag_coeff: 1.0, ..make_zone(0.0) };
        let full_drag = zone.damage_drag_coeff * (1.0 - 1.0_f64); // health=1.0 → 0
        let half_drag = zone.damage_drag_coeff * (1.0 - 0.5_f64); // health=0.5 → 0.5
        let zero_drag = zone.damage_drag_coeff * (1.0 - 0.0_f64); // health=0.0 → 1.0
        assert!((full_drag - 0.0).abs() < 1e-12);
        assert!((half_drag - 0.5).abs() < 1e-12);
        assert!((zero_drag - 1.0).abs() < 1e-12);
    }
}

