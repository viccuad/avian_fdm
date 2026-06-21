//! Viterna-Corrigan post-stall extension for [`AeroCoeff`] tables.
//!
//! Reference: Viterna, L.A. & Corrigan, R.D. (1982), "Fixed Pitch Rotor
//! Performance of Large Horizontal Axis Wind Turbines", NASA CP-2230.
//!
//! The model extends aerodynamic coefficient tables from their last data
//! point out to +/-180 degrees using flat-plate theory. This prevents
//! table clamping during post-stall flight, tumbling, or any orientation
//! where the local angle of attack exceeds the wind-tunnel data range.

use avian3d::math::Scalar as S;

use super::types::AeroCoeff;

#[allow(clippy::unnecessary_cast)]
const PI: S = std::f64::consts::PI as S;
const HALF_PI: S = PI / 2.0;

/// Viterna CD_max for a finite-aspect-ratio surface.
fn viterna_cd_max(ar: S) -> S {
    1.11 + 0.018 * ar
}

/// Angles (in radians) at which to generate extension points.
/// Covers 25 deg to 180 deg in 5-deg steps from 25 to 50, then 10-deg steps.
fn extension_angles() -> Vec<S> {
    let mut angles = Vec::new();
    // Fine steps near stall transition (25-50 deg)
    for deg in (25..=50).step_by(5) {
        angles.push((deg as S).to_radians());
    }
    // Coarser steps for the rest (60-180 deg)
    for deg in (60..=180).step_by(10) {
        angles.push((deg as S).to_radians());
    }
    angles
}

/// Viterna lift coefficient at angle `a` (radians, positive).
///
/// For |a| <= pi/2: CL = A1 * sin(2a) + A2 * cos^2(a) / sin(a)
/// For |a| > pi/2:  CL = A1 * sin(2a)   (flat plate only)
///
/// The A2 * cos^2/sin term provides a smooth blend from the stall-angle
/// value into flat-plate behavior. Beyond 90 deg, only the sin(2a) term
/// remains because the A2 term would diverge as sin(a) approaches zero
/// near 180 deg.
fn viterna_cl(a: S, a1: S, a2: S) -> S {
    let sa = a.sin();
    let flat = a1 * (2.0 * a).sin();
    if a.abs() <= HALF_PI && sa.abs() > 1e-10 {
        flat + a2 * a.cos().powi(2) / sa
    } else {
        flat
    }
}

/// Viterna drag coefficient at angle `a`.
///
///   CD = CD_0_eff + (CD_max - CD_0_eff) * sin^2(a)
///
/// This gives CD_0_eff at 0 and 180 deg, and CD_max at 90 deg.
fn viterna_cd(a: S, cd0_eff: S, cd_max: S) -> S {
    cd0_eff + (cd_max - cd0_eff) * a.sin().powi(2)
}

/// Compute the Viterna A2 coefficient for continuity at the stall angle.
///
///   A2 = (CL_s - A1 * sin(2*alpha_s)) * sin(alpha_s) / cos^2(alpha_s)
///
/// If alpha_s is close to 90 deg (cos^2 near zero), returns 0 (pure flat plate).
fn viterna_a2(alpha_s: S, cl_s: S, a1: S) -> S {
    let cos2 = alpha_s.cos().powi(2);
    if cos2 < 1e-6 {
        return 0.0;
    }
    (cl_s - a1 * (2.0 * alpha_s).sin()) * alpha_s.sin() / cos2
}

/// Compute effective CD_0 for continuity at the stall angle.
///
///   CD_0_eff = (CD_s - CD_max * sin^2(alpha_s)) / cos^2(alpha_s)
///
/// If alpha_s is near 90 deg, falls back to CD_s (accept minor discontinuity).
fn effective_cd0(alpha_s: S, cd_s: S, cd_max: S) -> S {
    let cos2 = alpha_s.cos().powi(2);
    if cos2 < 1e-6 {
        return cd_s;
    }
    (cd_s - cd_max * alpha_s.sin().powi(2)) / cos2
}

/// Extend a 1-D lift table to +/-pi.
fn extend_1d_lift(bp: &[S], vals: &[S], ar: S) -> (Vec<S>, Vec<S>) {
    if bp.is_empty() {
        return (bp.to_vec(), vals.to_vec());
    }
    let cd_max = viterna_cd_max(ar);
    let a1 = cd_max / 2.0;

    let alpha_max = *bp.last().unwrap();
    let alpha_min = bp[0];
    let cl_at_max = *vals.last().unwrap();
    let cl_at_min = vals[0];

    if alpha_max >= PI - 0.01 && alpha_min <= -(PI - 0.01) {
        return (bp.to_vec(), vals.to_vec());
    }

    let ext_angles = extension_angles();

    let mut new_bp = Vec::with_capacity(bp.len() + 40);
    let mut new_vals = Vec::with_capacity(bp.len() + 40);

    // Negative side extension (from -pi up to alpha_min).
    if alpha_min > -(PI - 0.01) {
        let a2_neg = viterna_a2(alpha_min.abs(), -cl_at_min, a1);
        for &a in ext_angles.iter().rev() {
            let neg_a = -a;
            if neg_a < alpha_min - 0.001 {
                new_bp.push(neg_a);
                new_vals.push(-viterna_cl(a, a1, a2_neg));
            }
        }
    }

    new_bp.extend_from_slice(bp);
    new_vals.extend_from_slice(vals);

    // Positive side extension (from alpha_max up to +pi).
    if alpha_max < PI - 0.01 {
        let a2_pos = viterna_a2(alpha_max, cl_at_max, a1);
        for &a in &ext_angles {
            if a > alpha_max + 0.001 {
                new_bp.push(a);
                new_vals.push(viterna_cl(a, a1, a2_pos));
            }
        }
    }

    (new_bp, new_vals)
}

/// Extend a 1-D drag table to +/-pi.
fn extend_1d_drag(bp: &[S], vals: &[S], ar: S) -> (Vec<S>, Vec<S>) {
    if bp.is_empty() {
        return (bp.to_vec(), vals.to_vec());
    }
    let cd_max = viterna_cd_max(ar);

    let alpha_max = *bp.last().unwrap();
    let alpha_min = bp[0];
    let cd_at_max = *vals.last().unwrap();
    let cd_at_min = vals[0];

    if alpha_max >= PI - 0.01 && alpha_min <= -(PI - 0.01) {
        return (bp.to_vec(), vals.to_vec());
    }

    let ext_angles = extension_angles();

    let mut new_bp = Vec::with_capacity(bp.len() + 40);
    let mut new_vals = Vec::with_capacity(bp.len() + 40);

    // Negative side.
    if alpha_min > -(PI - 0.01) {
        let cd0_neg = effective_cd0(alpha_min.abs(), cd_at_min, cd_max);
        for &a in ext_angles.iter().rev() {
            let neg_a = -a;
            if neg_a < alpha_min - 0.001 {
                new_bp.push(neg_a);
                new_vals.push(viterna_cd(a, cd0_neg, cd_max));
            }
        }
    }

    new_bp.extend_from_slice(bp);
    new_vals.extend_from_slice(vals);

    // Positive side.
    if alpha_max < PI - 0.01 {
        let cd0_pos = effective_cd0(alpha_max, cd_at_max, cd_max);
        for &a in &ext_angles {
            if a > alpha_max + 0.001 {
                new_bp.push(a);
                new_vals.push(viterna_cd(a, cd0_pos, cd_max));
            }
        }
    }

    (new_bp, new_vals)
}

/// Convert a scalar CD to a Table1D with flat-plate drag progression.
fn scalar_to_drag_table(cd0: S, ar: S) -> (Vec<S>, Vec<S>) {
    let cd_max = viterna_cd_max(ar);
    let all_angles = extension_angles();

    let mut bp = Vec::with_capacity(all_angles.len() * 2 + 1);
    let mut vals = Vec::with_capacity(all_angles.len() * 2 + 1);

    for &a in all_angles.iter().rev() {
        bp.push(-a);
        vals.push(viterna_cd(a, cd0, cd_max));
    }
    bp.push(0.0);
    vals.push(cd0);
    for &a in &all_angles {
        bp.push(a);
        vals.push(viterna_cd(a, cd0, cd_max));
    }

    (bp, vals)
}

/// Extend a 2-D lift table to +/-pi. Each Reynolds-number column is extended independently.
fn extend_2d_lift(rows: &[S], cols: &[S], data: &[S], ar: S) -> (Vec<S>, Vec<S>) {
    let nc = cols.len();
    if rows.is_empty() || nc == 0 {
        return (rows.to_vec(), data.to_vec());
    }
    let mut col_results: Vec<(Vec<S>, Vec<S>)> = Vec::with_capacity(nc);
    for j in 0..nc {
        let col_vals: Vec<S> = (0..rows.len()).map(|i| data[i * nc + j]).collect();
        col_results.push(extend_1d_lift(rows, &col_vals, ar));
    }
    let new_rows = col_results[0].0.clone();
    let nr = new_rows.len();
    let mut new_data = vec![0.0; nr * nc];
    for j in 0..nc {
        for i in 0..nr {
            new_data[i * nc + j] = col_results[j].1[i];
        }
    }
    (new_rows, new_data)
}

/// Extend a 2-D drag table to +/-pi.
fn extend_2d_drag(rows: &[S], cols: &[S], data: &[S], ar: S) -> (Vec<S>, Vec<S>) {
    let nc = cols.len();
    if rows.is_empty() || nc == 0 {
        return (rows.to_vec(), data.to_vec());
    }
    let mut col_results: Vec<(Vec<S>, Vec<S>)> = Vec::with_capacity(nc);
    for j in 0..nc {
        let col_vals: Vec<S> = (0..rows.len()).map(|i| data[i * nc + j]).collect();
        col_results.push(extend_1d_drag(rows, &col_vals, ar));
    }
    let new_rows = col_results[0].0.clone();
    let nr = new_rows.len();
    let mut new_data = vec![0.0; nr * nc];
    for j in 0..nc {
        for i in 0..nr {
            new_data[i * nc + j] = col_results[j].1[i];
        }
    }
    (new_rows, new_data)
}

impl AeroCoeff {
    /// Extend a lift-type coefficient (CL or CY) to +/-180 deg using the
    /// Viterna-Corrigan post-stall model.
    ///
    /// Takes the **aspect ratio** (AR) of the aerodynamic surface, defined as
    /// span divided by chord: AR = b / c. For a rectangular wing with span
    /// 10 m and chord 1.5 m, AR = 6.67. This parameter controls CD_max,
    /// the flat-plate drag limit at 90 deg angle of attack:
    ///
    ///   CD_max = 1.11 + 0.018 * AR    (Viterna & Corrigan 1982)
    ///
    /// The extension blends smoothly from the last user-provided breakpoint
    /// into flat-plate aerodynamics, then continues to +/-180 deg (fully
    /// reversed flight). Within the original breakpoint range, values are
    /// unchanged.
    ///
    /// From the stall angle alpha_s to 90 deg, the Viterna lift formula is:
    ///
    ///   CL = A1 * sin(2*alpha) + A2 * cos(alpha)^2 / sin(alpha)
    ///
    /// where A1 = CD_max / 2, and A2 is chosen for continuity at alpha_s.
    /// From 90 to 180 deg (reversed flight), only the flat-plate term
    /// remains: CL = A1 * sin(2*alpha).
    ///
    /// When a table covers the full +/-180 deg range, no clamping occurs
    /// during coefficient evaluation regardless of how extreme the local
    /// angle of attack becomes during post-stall flight or tumbling.
    ///
    /// Has no effect on `Absent`, `Placeholder`, or `Scalar` variants.
    ///
    /// # Example
    ///
    /// ```
    /// # use avian_fdm::components::aero_coeff::AeroCoeff;
    /// let cl = AeroCoeff::Table1D {
    ///     breakpoints: vec![-0.35, 0.0, 0.35],
    ///     values: vec![-2.5, 0.0, 2.5],
    /// };
    /// let cl_full = cl.with_post_stall_lift(3.0);
    /// // Original data is preserved; extension points added to +/-180 deg.
    /// // At 90 deg AoA, CL is near zero (flat plate).
    /// let cl_90 = cl_full.evaluate(std::f32::consts::FRAC_PI_2 as _, 0.0);
    /// assert!(cl_90.abs() < 0.01);
    /// ```
    pub fn with_post_stall_lift(self, aspect_ratio: S) -> Self {
        match self {
            AeroCoeff::Table1D {
                breakpoints,
                values,
            } => {
                let (bp, vals) = extend_1d_lift(&breakpoints, &values, aspect_ratio);
                AeroCoeff::Table1D {
                    breakpoints: bp,
                    values: vals,
                }
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                let (new_rows, new_data) = extend_2d_lift(&rows, &cols, &data, aspect_ratio);
                AeroCoeff::Table2D {
                    rows: new_rows,
                    cols,
                    data: new_data,
                }
            }
            other => other,
        }
    }

    /// Extend a drag coefficient (CD) to +/-180 deg using a flat-plate
    /// drag model derived from Viterna-Corrigan.
    ///
    /// Takes the **aspect ratio** (AR = span / chord) of the aerodynamic
    /// surface. CD_max at 90 deg AoA is:
    ///
    ///   CD_max = 1.11 + 0.018 * AR
    ///
    /// The model interpolates between the user's last breakpoint value and
    /// CD_max using:
    ///
    ///   CD = CD_0_eff + (CD_max - CD_0_eff) * sin(alpha)^2
    ///
    /// where CD_0_eff is solved for continuity at the boundary.
    ///
    /// For `Scalar(cd0)`, this converts to a `Table1D` that transitions
    /// from cd0 at zero AoA to CD_max at 90 deg and back to cd0 at 180.
    /// This is important because a constant CD is unphysical at high AoA:
    /// a surface broadside to the wind has much higher drag.
    ///
    /// Has no effect on `Absent` or `Placeholder` variants.
    ///
    /// # Example
    ///
    /// ```
    /// # use avian_fdm::components::aero_coeff::AeroCoeff;
    /// let cd = AeroCoeff::Scalar(0.01);
    /// let cd_full = cd.with_post_stall_drag(3.0);
    /// // At 90 deg AoA, CD is near CD_max (flat plate):
    /// let cd_90 = cd_full.evaluate(std::f32::consts::FRAC_PI_2 as _, 0.0);
    /// assert!((cd_90 - 1.164).abs() < 0.01);
    /// ```
    pub fn with_post_stall_drag(self, aspect_ratio: S) -> Self {
        match self {
            AeroCoeff::Scalar(cd0) => {
                let (bp, vals) = scalar_to_drag_table(cd0, aspect_ratio);
                AeroCoeff::Table1D {
                    breakpoints: bp,
                    values: vals,
                }
            }
            AeroCoeff::Table1D {
                breakpoints,
                values,
            } => {
                let (bp, vals) = extend_1d_drag(&breakpoints, &values, aspect_ratio);
                AeroCoeff::Table1D {
                    breakpoints: bp,
                    values: vals,
                }
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                let (new_rows, new_data) = extend_2d_drag(&rows, &cols, &data, aspect_ratio);
                AeroCoeff::Table2D {
                    rows: new_rows,
                    cols,
                    data: new_data,
                }
            }
            other => other,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn post_stall_lift_preserves_original_data() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        assert!((extended.evaluate(0.0, 0.0) - 0.0).abs() < 1e-10);
        assert!((extended.evaluate(0.35, 0.0) - 2.5).abs() < 1e-10);
        assert!((extended.evaluate(-0.35, 0.0) - (-2.5)).abs() < 1e-10);
    }

    #[test]
    fn post_stall_lift_covers_full_range() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        if let AeroCoeff::Table1D {
            ref breakpoints, ..
        } = extended
        {
            assert!(
                *breakpoints.first().unwrap() <= -PI + 0.01,
                "table should extend to -pi, got {}",
                breakpoints.first().unwrap()
            );
            assert!(
                *breakpoints.last().unwrap() >= PI - 0.01,
                "table should extend to +pi, got {}",
                breakpoints.last().unwrap()
            );
        } else {
            panic!("expected Table1D");
        }
    }

    #[test]
    fn post_stall_lift_zero_at_90_deg() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        let cl_90 = extended.evaluate(HALF_PI, 0.0);
        assert!(
            cl_90.abs() < 0.05,
            "CL at 90 deg should be near zero, got {cl_90}"
        );
    }

    #[test]
    fn post_stall_lift_zero_at_180_deg() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        let cl_180 = extended.evaluate(PI, 0.0);
        assert!(
            cl_180.abs() < 0.05,
            "CL at 180 deg should be near zero, got {cl_180}"
        );
    }

    #[test]
    fn post_stall_lift_antisymmetric() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        for deg in [45.0 as S, 60.0, 90.0, 120.0, 150.0] {
            let a = deg.to_radians();
            let pos = extended.evaluate(a, 0.0);
            let neg = extended.evaluate(-a, 0.0);
            assert!(
                (pos + neg).abs() < 0.1,
                "CL should be antisymmetric at {deg} deg: CL(+)={pos:.3}, CL(-)={neg:.3}"
            );
        }
    }

    #[test]
    fn post_stall_lift_continuous_at_boundary() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        let inside = extended.evaluate(0.34, 0.0);
        let boundary = extended.evaluate(0.35, 0.0);
        let outside = extended.evaluate(0.44, 0.0);
        assert!(
            (boundary - inside).abs() < 0.5,
            "discontinuity at boundary: {inside:.3} vs {boundary:.3}"
        );
        assert!(
            (outside - boundary).abs() < 1.0,
            "large jump past boundary: {boundary:.3} vs {outside:.3}"
        );
    }

    #[test]
    fn post_stall_drag_scalar_to_table() {
        let cd = AeroCoeff::Scalar(0.01);
        let extended = cd.with_post_stall_drag(3.0);
        assert!(matches!(extended, AeroCoeff::Table1D { .. }));
        assert!((extended.evaluate(0.0, 0.0) - 0.01).abs() < 1e-6);
        let cd_max = viterna_cd_max(3.0);
        assert!(
            (extended.evaluate(HALF_PI, 0.0) - cd_max).abs() < 0.02,
            "CD at 90 should be {cd_max:.3}, got {:.3}",
            extended.evaluate(HALF_PI, 0.0)
        );
    }

    #[test]
    fn post_stall_drag_table_covers_full_range() {
        let cd = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![0.05, 0.01, 0.05],
        };
        let extended = cd.with_post_stall_drag(6.0);
        if let AeroCoeff::Table1D {
            ref breakpoints, ..
        } = extended
        {
            assert!(*breakpoints.first().unwrap() <= -PI + 0.01);
            assert!(*breakpoints.last().unwrap() >= PI - 0.01);
        }
        let cd_90 = extended.evaluate(HALF_PI, 0.0);
        assert!((cd_90 - 1.218).abs() < 0.1, "CD at 90 = {cd_90:.3}");
    }

    #[test]
    fn post_stall_drag_symmetric() {
        let cd = AeroCoeff::Scalar(0.01).with_post_stall_drag(3.0);
        for deg in [30.0 as S, 60.0, 90.0, 120.0, 150.0] {
            let a = deg.to_radians();
            let pos = cd.evaluate(a, 0.0);
            let neg = cd.evaluate(-a, 0.0);
            assert!(
                (pos - neg).abs() < 0.01,
                "CD should be symmetric at {deg} deg: CD(+)={pos:.3}, CD(-)={neg:.3}"
            );
        }
    }

    #[test]
    fn post_stall_absent_unchanged() {
        let c = AeroCoeff::Absent;
        let extended = c.with_post_stall_lift(3.0);
        assert!(matches!(extended, AeroCoeff::Absent));
        let c = AeroCoeff::Absent;
        let extended = c.with_post_stall_drag(3.0);
        assert!(matches!(extended, AeroCoeff::Absent));
    }

    #[test]
    fn post_stall_already_full_range_unchanged() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-PI, 0.0, PI],
            values: vec![-1.0, 0.0, 1.0],
        };
        let extended = cl.with_post_stall_lift(3.0);
        if let AeroCoeff::Table1D { breakpoints, .. } = extended {
            assert_eq!(
                breakpoints.len(),
                3,
                "should not add points to full-range table"
            );
        }
    }

    #[test]
    fn post_stall_lift_2d_preserves_columns() {
        let cl = AeroCoeff::Table2D {
            rows: vec![-0.35, 0.0, 0.35],
            cols: vec![1e6, 3e6],
            data: vec![-2.0, -2.5, 0.0, 0.0, 2.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        if let AeroCoeff::Table2D {
            ref rows, ref cols, ..
        } = extended
        {
            assert_eq!(cols.len(), 2, "Re columns unchanged");
            assert!(*rows.first().unwrap() <= -PI + 0.01);
            assert!(*rows.last().unwrap() >= PI - 0.01);
        }
        assert!((extended.evaluate(0.0, 1e6) - 0.0).abs() < 1e-10);
        assert!((extended.evaluate(0.35, 3e6) - 2.5).abs() < 1e-10);
    }
}
