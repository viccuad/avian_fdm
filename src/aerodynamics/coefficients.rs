//! Step 1: coefficient evaluation. Table evaluation, control-surface scaling, and damage.

use crate::components::{AeroZone, ControlInputs, ControlSurfaceRole};
use avian3d::math::Scalar;

/// Six non-dimensional aerodynamic coefficients with all scaling applied. To
/// get the forces, multiply them by the dynamic pressure and the reference
/// area.
pub(crate) struct ZoneCoefficients {
    /// Lift coefficient (positive = up in the stability frame).
    pub cl: Scalar,
    /// Drag coefficient (positive = opposite to the motion).
    pub cd: Scalar,
    /// Side-force coefficient (positive = starboard).
    pub cy: Scalar,
    /// Pitching-moment coefficient (positive = nose up).
    pub cm: Scalar,
    /// Rolling-moment coefficient (positive = starboard wing down).
    pub croll: Scalar,
    /// Yawing-moment coefficient (positive = nose right).
    pub cn: Scalar,
}

/// Evaluates the coefficient tables of an [`AeroZone`]. Then applies the
/// control-surface scaling and the performance loss from damage.
///
/// # How it works
///
/// 1. **Table evaluation**. The function evaluates CL, CD, CM, Croll, and Cn
///    at `alpha_local`. This is the local angle of attack of the zone, with
///    the body-rate corrections from
///    [`zone_local_angles`][super::local_angles::zone_local_angles]. It
///    evaluates CY (side force) at `beta_local`. A table can be a constant
///    (`Scalar`), a 1-D function of the primary angle (`Table1D`), or a 2-D
///    function of (angle, Re) (`Table2D`).
///
/// 2. **Control-surface scaling**. If the zone has a [`ControlSurfaceRole`],
///    the function multiplies its lift, side-force, and moment coefficients
///    by the matching pilot input in [−1, 1]. The table coefficients give the
///    authority at *full deflection*. The scale by the input gives the
///    contribution at partial deflection.
///
///    Drag always *increases* with deflection, because the flow separates
///    from the deflected surface. Thus, the function scales CD by `|input|`,
///    not by the signed input. `AileronRight` inverts the aileron sign. Thus,
///    a left stick input moves the trailing edge of the left aileron down
///    (more lift) and the trailing edge of the right aileron up (less lift).
///
/// 3. **Performance loss from damage**. The function multiplies all
///    coefficients by `remaining`, in (0, 1]. A wing with 0.5 remaining makes
///    half the lift. Structural deformation also adds parasitic drag. This
///    drag is largest at intermediate damage:
///
///    ```text
///    CD_effective = (CD_base * |input| + damage_drag * (1 − remaining) / qbar) * remaining
///    ```
///
///    At `remaining = 0`, the zone is fully detached and makes no force. The
///    caller handles this case, not this function.
pub(crate) fn evaluate_zone_coefficients(
    zone: &AeroZone,
    ctrl: &ControlInputs,
    alpha_local: Scalar,
    beta_local: Scalar,
    re: Scalar,
    qbar: Scalar,
    remaining: Scalar,
) -> ZoneCoefficients {
    let cl_base = zone.cl.evaluate(alpha_local, re);
    let cd_base = zone.cd.evaluate(alpha_local, re);
    let cy_base = zone.cy.evaluate(beta_local, re);
    let cm_base = zone.cm.evaluate(alpha_local, re);
    let croll_base = zone.croll.evaluate(alpha_local, re);
    let cn_base = zone.cn.evaluate(alpha_local, re);

    let (scale, cd_scale) = match &zone.control_role {
        Some(ControlSurfaceRole::Elevator) => (ctrl.elevator, ctrl.elevator.abs()),
        Some(ControlSurfaceRole::AileronLeft) => (ctrl.aileron, ctrl.aileron.abs()),
        Some(ControlSurfaceRole::AileronRight) => (-ctrl.aileron, ctrl.aileron.abs()),
        Some(ControlSurfaceRole::Rudder) => (ctrl.rudder, ctrl.rudder.abs()),
        None => (1.0, 1.0),
    };

    let extra_cd = zone
        .damage_drag_coeff
        .map(|coeff| coeff * (1.0 - remaining) / qbar.max(1e-4))
        .unwrap_or(0.0);

    ZoneCoefficients {
        cl: cl_base * scale * remaining,
        cd: (cd_base * cd_scale + extra_cd) * remaining,
        cy: cy_base * scale * remaining,
        cm: cm_base * scale * remaining,
        croll: croll_base * scale * remaining,
        cn: cn_base * scale * remaining,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::aero_coeff::AeroCoeff;
    use crate::components::{AeroZone, ControlInputs, ControlSurfaceRole};

    fn neutral() -> ControlInputs {
        ControlInputs {
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
            throttle: 0.0,
        }
    }

    fn simple_zone(cl: Scalar, cd: Scalar) -> AeroZone {
        AeroZone {
            cl: AeroCoeff::Scalar(cl),
            cd: AeroCoeff::Scalar(cd),
            ..Default::default()
        }
    }

    #[test]
    fn zero_remaining_produces_zero_coefficients() {
        let zone = simple_zone(0.5, 0.03);
        let c = evaluate_zone_coefficients(&zone, &neutral(), 0.1, 0.0, 1e6, 1000.0, 0.0);
        assert_eq!(c.cl, 0.0);
        assert_eq!(c.cd, 0.0);
    }

    #[test]
    fn half_remaining_halves_lift() {
        let zone = simple_zone(1.0, 0.0);
        let full = evaluate_zone_coefficients(&zone, &neutral(), 0.1, 0.0, 1e6, 1000.0, 1.0);
        let half = evaluate_zone_coefficients(&zone, &neutral(), 0.1, 0.0, 1e6, 1000.0, 0.5);
        assert!((half.cl - full.cl * 0.5).abs() < 1e-12);
    }

    #[test]
    fn damage_drag_peaks_at_intermediate_remaining() {
        let mut zone = simple_zone(0.0, 0.03);
        zone.damage_drag_coeff = Some(500.0);
        let full = evaluate_zone_coefficients(&zone, &neutral(), 0.0, 0.0, 1e6, 1000.0, 1.0);
        let half = evaluate_zone_coefficients(&zone, &neutral(), 0.0, 0.0, 1e6, 1000.0, 0.5);
        assert!(
            half.cd > full.cd * 0.5,
            "damage drag should add extra at intermediate remaining"
        );
    }

    #[test]
    fn elevator_scales_lift_by_input() {
        let mut zone = simple_zone(1.0, 0.0);
        zone.control_role = Some(ControlSurfaceRole::Elevator);
        let half = evaluate_zone_coefficients(
            &zone,
            &ControlInputs {
                elevator: 0.5,
                ..neutral()
            },
            0.1,
            0.0,
            1e6,
            1000.0,
            1.0,
        );
        let full = evaluate_zone_coefficients(
            &zone,
            &ControlInputs {
                elevator: 1.0,
                ..neutral()
            },
            0.1,
            0.0,
            1e6,
            1000.0,
            1.0,
        );
        assert!((half.cl - full.cl * 0.5).abs() < 1e-12);
    }

    #[test]
    fn aileron_right_mirrors_left() {
        let mut zone_l = simple_zone(0.8, 0.0);
        zone_l.control_role = Some(ControlSurfaceRole::AileronLeft);
        let mut zone_r = simple_zone(0.8, 0.0);
        zone_r.control_role = Some(ControlSurfaceRole::AileronRight);
        let ctrl = ControlInputs {
            aileron: 0.6,
            ..neutral()
        };
        let cl_l = evaluate_zone_coefficients(&zone_l, &ctrl, 0.1, 0.0, 1e6, 1000.0, 1.0).cl;
        let cl_r = evaluate_zone_coefficients(&zone_r, &ctrl, 0.1, 0.0, 1e6, 1000.0, 1.0).cl;
        assert!(
            (cl_l + cl_r).abs() < 1e-12,
            "ailerons should produce opposite CL"
        );
    }

    #[test]
    fn control_deflection_always_increases_drag() {
        let mut zone = simple_zone(0.0, 0.05);
        zone.control_role = Some(ControlSurfaceRole::Elevator);
        let n = evaluate_zone_coefficients(&zone, &neutral(), 0.0, 0.0, 1e6, 1000.0, 1.0);
        let d = evaluate_zone_coefficients(
            &zone,
            &ControlInputs {
                elevator: 0.8,
                ..neutral()
            },
            0.0,
            0.0,
            1e6,
            1000.0,
            1.0,
        );
        assert!(
            d.cd >= n.cd * 0.8 - 1e-12,
            "deflection should not reduce drag"
        );
    }

    #[test]
    fn rudder_scales_side_force() {
        let mut zone = AeroZone {
            cy: AeroCoeff::Scalar(1.0),
            ..Default::default()
        };
        zone.control_role = Some(ControlSurfaceRole::Rudder);
        let full = evaluate_zone_coefficients(
            &zone,
            &ControlInputs {
                rudder: 1.0,
                ..neutral()
            },
            0.0,
            0.0,
            1e6,
            1000.0,
            1.0,
        );
        let half = evaluate_zone_coefficients(
            &zone,
            &ControlInputs {
                rudder: 0.5,
                ..neutral()
            },
            0.0,
            0.0,
            1e6,
            1000.0,
            1.0,
        );
        assert!((full.cy - 1.0).abs() < 1e-12, "full rudder -> CY=1");
        assert!((half.cy - 0.5).abs() < 1e-12, "half rudder -> CY=0.5");
        assert_eq!(full.cl, 0.0, "rudder must not affect CL");
    }

    #[test]
    fn combined_damage_and_control_deflection() {
        let mut zone = simple_zone(1.0, 0.1);
        zone.control_role = Some(ControlSurfaceRole::Elevator);
        let ctrl = ControlInputs {
            elevator: 0.5,
            ..neutral()
        };
        let intact = evaluate_zone_coefficients(&zone, &ctrl, 0.0, 0.0, 1e6, 1000.0, 1.0);
        let damaged = evaluate_zone_coefficients(&zone, &ctrl, 0.0, 0.0, 1e6, 1000.0, 0.5);
        assert!((intact.cl - 0.5).abs() < 1e-12, "intact CL");
        assert!(
            (damaged.cl - 0.25).abs() < 1e-12,
            "half-remaining halves CL further"
        );
        assert!(damaged.cl < intact.cl);
    }

    #[test]
    fn absent_secondary_fields_produce_no_moment() {
        let zone = AeroZone {
            cl: AeroCoeff::Scalar(1.0),
            cd: AeroCoeff::Scalar(0.1),
            ..Default::default() // cy/cm/croll/cn = Absent
        };
        let c = evaluate_zone_coefficients(&zone, &neutral(), 0.2, 0.0, 1e6, 1000.0, 1.0);
        assert_eq!(c.cy, 0.0, "Absent cy -> 0");
        assert_eq!(c.cm, 0.0, "Absent cm -> 0");
        assert_eq!(c.croll, 0.0, "Absent croll -> 0");
        assert_eq!(c.cn, 0.0, "Absent cn -> 0");
    }
}
