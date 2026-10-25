//! [`AeroCoeff::evaluate`], bilinear interpolation, and clamp helper functions.

use avian3d::math::Scalar as S;
use bevy_log::{warn, warn_once};

use crate::math::lerp_1d;

use super::types::AeroCoeff;

/// Clamps `v` to `[lo, hi]`. If the clamp changes the value, emits a `warn_once!`.
pub(super) fn clamp_with_warn(v: S, lo: S, hi: S, label: &'static str) -> S {
    if v < lo {
        warn_once!("{label} = {v:.4} is below table minimum {lo:.4}; clamping");
        lo
    } else if v > hi {
        warn_once!("{label} = {v:.4} is above table maximum {hi:.4}; clamping");
        hi
    } else {
        v
    }
}

/// Bilinear interpolation in a flat, row-major 2-D table.
/// The caller must clamp `angle_rad` and `re` to their ranges first.
pub(super) fn bilerp(angle_rad: S, re: S, rows: &[S], cols: &[S], data: &[S]) -> S {
    let nc = cols.len();

    // saturating_sub(2) handles the degenerate case of a single row or a single column.
    let ri = rows
        .partition_point(|&r| r <= angle_rad)
        .saturating_sub(1)
        .min(rows.len().saturating_sub(2));
    let ci = cols
        .partition_point(|&c| c <= re)
        .saturating_sub(1)
        .min(cols.len().saturating_sub(2));

    // If there is only one row or one column, the "next" index is the same, and t = 0.
    let ri1 = (ri + 1).min(rows.len() - 1);
    let ci1 = (ci + 1).min(cols.len() - 1);

    let ta = if rows[ri1] != rows[ri] {
        (angle_rad - rows[ri]) / (rows[ri1] - rows[ri])
    } else {
        0.0
    };
    let tr = if cols[ci1] != cols[ci] {
        (re - cols[ci]) / (cols[ci1] - cols[ci])
    } else {
        0.0
    };

    let v00 = data[ri * nc + ci];
    let v01 = data[ri * nc + ci1];
    let v10 = data[ri1 * nc + ci];
    let v11 = data[ri1 * nc + ci1];

    let v0 = v00 + tr * (v01 - v00); // interpolate along Re at lower angle row
    let v1 = v10 + tr * (v11 - v10); // interpolate along Re at upper angle row
    v0 + ta * (v1 - v0) // interpolate along angle
}

impl AeroCoeff {
    /// Evaluates the coefficient at the given primary angle (rad) and Reynolds number.
    ///
    /// The primary angle is the first axis of the table:
    /// - For CL, CD, CM, Croll, and Cn: give the local angle of attack `alpha_local`.
    /// - For CY (side force): give the local sideslip angle `beta_local`.
    ///
    /// The result for each variant:
    /// - [`AeroCoeff::Absent`]: returns `0.0` with no warning (does not apply, by design).
    /// - [`AeroCoeff::Placeholder`]: emits `warn_once!` and returns `0.0`.
    /// - [`AeroCoeff::Scalar`]: returns the constant. It ignores both inputs.
    /// - [`AeroCoeff::Table1D`]: linear interpolation on `angle_rad`. It ignores `re`.
    ///   If the angle is out of range, it clamps to the first or last
    ///   breakpoint and emits a `warn_once!`.
    /// - [`AeroCoeff::Table2D`]: bilinear interpolation on `(angle_rad, re)`.
    ///   If an input is out of range, it clamps each axis independently and
    ///   emits a `warn_once!`.
    ///
    /// It never panics in release builds. For a degenerate table (no
    /// breakpoints), it emits a `warn!` and returns `0.0`.
    pub fn evaluate(&self, angle_rad: S, re: S) -> S {
        match self {
            AeroCoeff::Absent => 0.0,
            AeroCoeff::Placeholder => {
                warn_once!(
                    "AeroCoeff::Placeholder evaluated: this coefficient has no data yet. \
                     Replace with Scalar, Table1D, or Table2D."
                );
                0.0
            }
            AeroCoeff::Scalar(v) => *v,
            AeroCoeff::Table1D {
                breakpoints,
                values,
            } => {
                if breakpoints.is_empty() {
                    warn!("AeroCoeff::Table1D has empty breakpoints; returning 0.0");
                    return 0.0;
                }
                let angle_rad = clamp_with_warn(
                    angle_rad,
                    breakpoints[0],
                    *breakpoints.last().unwrap(),
                    "Table1D angle_rad",
                );
                lerp_1d(angle_rad, breakpoints, values)
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                if rows.is_empty() || cols.is_empty() {
                    warn!("AeroCoeff::Table2D has empty rows or cols; returning 0.0");
                    return 0.0;
                }
                let angle_rad = clamp_with_warn(
                    angle_rad,
                    rows[0],
                    *rows.last().unwrap(),
                    "Table2D angle_rad",
                );
                let re = clamp_with_warn(re, cols[0], *cols.last().unwrap(), "Table2D re");
                bilerp(angle_rad, re, rows, cols, data)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_returns_value() {
        let c = AeroCoeff::Scalar(0.5);
        assert_eq!(c.evaluate(99.0, 99.0), 0.5);
        assert_eq!(c.evaluate(-99.0, 0.0), 0.5);
    }

    #[test]
    fn table1d_exact_breakpoint() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.0, 0.1, 0.2],
            values: vec![0.0, 1.0, 2.0],
        };
        assert!((c.evaluate(0.1, 0.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn table1d_midpoint() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.0, 0.2],
            values: vec![0.0, 2.0],
        };
        assert!((c.evaluate(0.1, 0.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn table1d_clamp_below() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.1, 0.2],
            values: vec![10.0, 20.0],
        };
        assert!((c.evaluate(-1.0, 0.0) - 10.0).abs() < 1e-12);
    }

    #[test]
    fn table1d_clamp_above() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.1, 0.2],
            values: vec![10.0, 20.0],
        };
        assert!((c.evaluate(99.0, 0.0) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn table2d_exact_corner() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![0.0, 1.0],
            data: vec![1.0, 2.0, 3.0, 4.0],
        };
        assert!((c.evaluate(0.0, 0.0) - 1.0).abs() < 1e-12);
        assert!((c.evaluate(0.0, 1.0) - 2.0).abs() < 1e-12);
        assert!((c.evaluate(1.0, 0.0) - 3.0).abs() < 1e-12);
        assert!((c.evaluate(1.0, 1.0) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn table2d_midpoint_alpha() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![1e6],
            data: vec![0.0, 1.0],
        };
        assert!((c.evaluate(0.5, 1e6) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn table2d_midpoint_both() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![0.0, 1.0],
            data: vec![0.0, 0.0, 0.0, 4.0],
        };
        assert!((c.evaluate(0.5, 0.5) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn table2d_clamp_both_axes() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![0.0, 1.0],
            data: vec![1.0, 2.0, 3.0, 4.0],
        };
        let v = c.evaluate(-99.0, 99.0);
        assert!((v - 2.0).abs() < 1e-12);
    }

    #[test]
    fn table2d_single_row() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![1e6],
            data: vec![0.5, 0.5],
        };
        assert!((c.evaluate(0.5, 1e6) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn table1d_empty_returns_zero() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![],
            values: vec![],
        };
        assert_eq!(c.evaluate(0.0, 0.0), 0.0);
    }

    /// A single-breakpoint Table1D must return that value for any input.
    #[test]
    fn table1d_single_breakpoint_returns_value() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.3],
            values: vec![7.5],
        };
        assert!((c.evaluate(0.3, 0.0) - 7.5).abs() < 1e-12, "exact hit");
        assert!((c.evaluate(-5.0, 0.0) - 7.5).abs() < 1e-12, "clamped below");
        assert!((c.evaluate(99.0, 0.0) - 7.5).abs() < 1e-12, "clamped above");
    }

    /// Table2D with a single Re column must handle the degenerate
    /// `cols[ci1] == cols[ci]` case without dividing by zero.
    #[test]
    fn table2d_single_re_column_no_panic() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![1e6],
            data: vec![0.0, 2.0],
        };
        assert!((c.evaluate(0.0, 1e6) - 0.0).abs() < 1e-12);
        assert!((c.evaluate(1.0, 1e6) - 2.0).abs() < 1e-12);
        assert!(
            (c.evaluate(0.5, 1e6) - 1.0).abs() < 1e-12,
            "midpoint on alpha"
        );
        assert!(
            (c.evaluate(0.5, 999.0) - 1.0).abs() < 1e-12,
            "Re clamped to only column"
        );
    }

    #[test]
    fn placeholder_evaluates_to_zero() {
        assert_eq!(AeroCoeff::Placeholder.evaluate(0.3, 1e6), 0.0);
        assert_eq!(AeroCoeff::Placeholder.evaluate(-1.0, 2e6), 0.0);
    }

    #[test]
    fn absent_evaluates_to_zero_silently() {
        assert_eq!(AeroCoeff::Absent.evaluate(0.3, 1e6), 0.0);
        assert_eq!(AeroCoeff::Absent.evaluate(-1.0, 2e6), 0.0);
    }
}
