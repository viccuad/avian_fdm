//! Induced drag: whole-aircraft CD_i = CL² / (π · e · AR).
//!
//! This is a whole-aircraft correction applied once per frame after all
//! per-zone aerodynamic forces have been accumulated. It models the drag
//! penalty that arises from generating lift on a finite-span wing: the
//! spanwise pressure gradient causes trailing vortices which tilt the
//! local flow and increase the effective angle of attack, producing a
//! rearward force component.
//!
//! Reference: Anderson, "Introduction to Flight", Chapter 5.

use avian3d::math::{Scalar, Vector, PI};

use crate::components::InducedDrag;

/// Apply induced drag to the aircraft's total force accumulator.
///
/// `total_cl_x_area` is the area-weighted sum of per-zone CLs (∑ CL_i · S_i).
/// Divide by `s_ref` to get the whole-aircraft CL.
///
/// The induced drag coefficient is:
///
/// ```text
/// CD_i = CL² / (π · e · AR)
/// ```
///
/// where AR = b² / S_ref and `e` is the Oswald span efficiency factor.
/// The resulting drag force is applied along the negative velocity direction
/// in world space.
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
        assert!(drag.length() < 1e-10, "zero CL → zero induced drag, got {drag:?}");
    }

    #[test]
    fn induced_drag_opposes_velocity() {
        let id = InducedDrag { oswald_factor: 0.8 };
        // Velocity in +X; induced drag should be in -X.
        let drag = apply_induced_drag(&id, 10.0, 10.0, 10.0, 100.0, Vector::X, identity_rot());
        assert!(drag.x < 0.0, "induced drag should oppose velocity, got {drag:?}");
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
