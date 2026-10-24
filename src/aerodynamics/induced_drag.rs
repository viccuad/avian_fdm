//! Induced drag of the full aircraft: CD_i = CL^2 / (pi * e * AR).
//!
//! This is a correction for the full aircraft. The crate applies it once per
//! physics step, after it adds up the aerodynamic forces of all zones. It
//! models the extra drag that lift causes on a wing with a finite span. The
//! pressure gradient along the span causes trailing vortices. These vortices
//! tilt the local flow and turn the lift vector aft. The result is a force
//! component to the rear.
//!
//! Reference: Anderson, "Introduction to Flight", Chapter 5.

use avian3d::math::{Scalar, Vector, PI};

use crate::components::InducedDrag;

/// Calculates the induced drag force for the total force of the aircraft.
///
/// `total_cl_x_area` is the area-weighted sum of the CL values of the zones
/// (sum of CL_i * S_i). The function divides it by `s_ref` to get the CL of
/// the full aircraft.
///
/// The induced drag coefficient is:
///
/// ```text
/// CD_i = CL^2 / (pi * e * AR)
/// ```
///
/// Here, AR = b^2 / S_ref, and `e` is the Oswald span efficiency factor.
/// The drag force acts opposite to the velocity, in world space.
pub(super) fn apply_induced_drag(
    id: &InducedDrag,
    total_cl_x_area: Scalar,
    s_ref: Scalar,
    wing_span: Scalar,
    qbar: Scalar,
    vel_body_unit_global: Vector,
    body_to_world: avian3d::prelude::Rotation,
) -> Vector {
    let ar = wing_span * wing_span / s_ref;
    let cl_aircraft = total_cl_x_area / s_ref;
    let cd_i = cl_aircraft * cl_aircraft / (PI * id.oswald_factor * ar);
    body_to_world * (vel_body_unit_global * (-cd_i * qbar * s_ref))
}

#[cfg(test)]
mod tests {
    use avian3d::prelude::Rotation;

    use super::*;

    fn identity_rot() -> Rotation {
        Rotation::default()
    }

    #[test]
    fn zero_cl_produces_zero_induced_drag() {
        let id = InducedDrag { oswald_factor: 0.8 };
        let drag = apply_induced_drag(&id, 0.0, 10.0, 10.0, 100.0, Vector::X, identity_rot());
        assert!(
            drag.length() < 1e-10,
            "zero CL → zero induced drag, got {drag:?}"
        );
    }

    #[test]
    fn induced_drag_opposes_velocity() {
        let id = InducedDrag { oswald_factor: 0.8 };
        // Velocity in +X; induced drag should be in -X.
        let drag = apply_induced_drag(&id, 10.0, 10.0, 10.0, 100.0, Vector::X, identity_rot());
        assert!(
            drag.x < 0.0,
            "induced drag should oppose velocity, got {drag:?}"
        );
        assert!(drag.y.abs() < 1e-10);
        assert!(drag.z.abs() < 1e-10);
    }

    #[test]
    fn induced_drag_scales_with_cl_squared() {
        let id = InducedDrag { oswald_factor: 0.8 };
        let drag1 = apply_induced_drag(&id, 10.0, 10.0, 10.0, 100.0, Vector::X, identity_rot());
        let drag2 = apply_induced_drag(&id, 20.0, 10.0, 10.0, 100.0, Vector::X, identity_rot());
        // Doubling CL → 4x induced drag.
        let ratio = drag2.x / drag1.x;
        assert!((ratio - 4.0).abs() < 1e-5, "CD_i ∝ CL², ratio={ratio}");
    }
}
