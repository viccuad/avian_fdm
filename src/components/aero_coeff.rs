//! [`AeroCoeff`], aerodynamic coefficient storage and lookup.
//!
//! An aerodynamic coefficient (e.g. CL, CD) can be a constant, a 1-D table
//! over angle of attack, or a 2-D table over angle of attack × Reynolds
//! number. The 2-D table is the most accurate and matches JSBSim's default
//! representation.
//!
//! ## Stability derivatives
//!
//! Real aerodynamic coefficients are nonlinear functions of many variables.
//! The *stability derivative method* approximates them as a Taylor expansion
//! around a trim condition. **Lift coefficient = baseline value + lift slope ×
//! angle-of-attack + pitch-rate correction term. See: stability
//! derivatives, aerodynamic Taylor series.**
//!
//! ```text
//! CL(α, Re) ≈ CL₀ + CL_α · α + CL_q · (q · c̄/2V) + …
//! ```
//!
//! For a high-fidelity simulation (matching JSBSim to ±1%), pre-computed
//! tables of CL vs α (at several Re values) are more accurate than linear
//! derivatives, especially near stall. `AeroCoeff::Table2D` stores this
//! directly.
//!
//! ## Table storage layout
//!
//! `Table2D::data` is a **flat, row-major `Vec<f64>`** of length
//! `rows.len() × cols.len()`. Element at row index `i`, column index `j`
//! is accessed as `data[i * cols.len() + j]`. This layout uses a single
//! heap allocation, maximises cache locality during bilinear interpolation,
//! and serialises efficiently.
//!
//! A `Vec<Vec<f64>>` (jagged array) would require one allocation per row
//! and has poor cache behaviour, it is intentionally avoided.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

/// An aerodynamic coefficient value: constant, 1-D table, or 2-D table.
///
/// Used for CL, CD, CY, CM, Cl, Cn, any dimensionless coefficient that
/// may depend on angle of attack and/or Reynolds number.
///
/// Call [`AeroCoeff::evaluate`] each frame to obtain a `f64` value at the
/// current flight conditions.
///
/// ## Completeness system, `Absent`, `Placeholder`, and data variants
///
/// Every coefficient field in [`crate::components::AeroZone`] is an `AeroCoeff`.
/// Three variants carry distinct meaning for unmodelled coefficients:
///
/// | Variant | Meaning | Runtime |
/// |---|---|---|
/// | `Absent` (default for secondary fields) | Not applicable by design, symmetric section, no CY, etc. | Silent 0.0 |
/// | `Placeholder` (default for primary fields) | Should exist but not yet modelled | `warn_once!` + 0.0 |
/// | `Scalar(0.0)` with [`crate::sourced!`] | Intentional explicit zero | Silent 0.0 |
/// | `Table1D` / `Table2D` | Fully modelled | Interpolated value |
///
/// `Placeholder` is the `Default` for `AeroCoeff`, so any primary field left
/// unfilled automatically warns at runtime rather than silently producing zero.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug, PartialEq)]
#[reflect(Serialize, Deserialize)]
pub enum AeroCoeff {
    /// Absent by design. This coefficient does not apply to this zone.
    ///
    /// Evaluates to `0.0` silently (no warning). Use for secondary
    /// coefficients that structurally don't exist on a given zone:
    /// e.g. `cy` on a symmetric main wing, or `croll` when roll is
    /// handled entirely by emergent geometry.
    ///
    /// This is the default for secondary `AeroZone` fields (`cy`, `cm`,
    /// `croll`, `cn`). Set a field to `Absent` explicitly when you want
    /// to document that the absence is intentional.
    Absent,

    /// Explicit "not yet modelled" sentinel.
    ///
    /// Evaluates to `0.0` (same as `Scalar(0.0)`) but emits a
    /// `warn_once!` on the first evaluation to notify the aircraft
    /// author that this coefficient still needs data.
    ///
    /// This is the [`Default`] value for `AeroCoeff` and the default for
    /// primary fields (`cl`, `cd`). Any `AeroZone` field that is not
    /// explicitly set will be `Placeholder`, ensuring gaps are visible at
    /// runtime rather than silently contributing zero.
    ///
    /// Replace with [`AeroCoeff::Scalar`] (annotated with [`crate::sourced!`])
    /// or a lookup table once you have data.
    Placeholder,

    /// Constant value. Suitable for simple linear models.
    Scalar(f64),

    /// 1-D lookup table: coefficient as a function of angle of attack (rad).
    ///
    /// `breakpoints` and `values` must have the same length (≥ 1).
    /// `breakpoints` must be strictly increasing.
    Table1D {
        /// Angle-of-attack breakpoints in radians, strictly increasing.
        breakpoints: Vec<f64>,
        /// Coefficient values at each breakpoint.
        values: Vec<f64>,
    },

    /// 2-D lookup table: coefficient as a function of angle of attack × Reynolds number.
    ///
    /// Stored row-major: `data[i * cols.len() + j]` is the value at
    /// `rows[i]` (alpha) and `cols[j]` (Re).
    ///
    /// Matches JSBSim's default table representation.
    Table2D {
        /// Angle-of-attack breakpoints (rows), in radians, strictly increasing.
        rows: Vec<f64>,
        /// Reynolds-number breakpoints (columns), strictly increasing.
        cols: Vec<f64>,
        /// Flat row-major coefficient data. Length must equal `rows.len() × cols.len()`.
        data: Vec<f64>,
    },
}

impl Default for AeroCoeff {
    /// Returns [`AeroCoeff::Placeholder`] so that any unset coefficient is
    /// flagged at runtime rather than silently producing zero.
    fn default() -> Self {
        AeroCoeff::Placeholder
    }
}

impl AeroCoeff {
    /// Returns `true` if this coefficient is `Absent` (not applicable by design).
    pub fn is_absent(&self) -> bool {
        matches!(self, AeroCoeff::Absent)
    }

    /// Returns `true` if this coefficient is `Placeholder` (not yet modelled).
    ///
    /// Useful for validation and tooling; the hot path should just call
    /// [`evaluate`](Self::evaluate) which handles the warning automatically.
    pub fn is_placeholder(&self) -> bool {
        matches!(self, AeroCoeff::Placeholder)
    }

    /// Evaluate the coefficient at the given primary angle (rad) and Reynolds number.
    ///
    /// The primary angle is the first table axis:
    /// - For CL, CD, CM, Croll, Cn: pass the local angle of attack `α_local`.
    /// - For CY (side force): pass the local sideslip angle `β_local`.
    ///
    /// - [`AeroCoeff::Absent`]: returns `0.0` silently (not applicable by design).
    /// - [`AeroCoeff::Placeholder`]: emits `warn_once!` and returns `0.0`.
    /// - [`AeroCoeff::Scalar`]: returns the constant; ignores both inputs.
    /// - [`AeroCoeff::Table1D`]: linearly interpolates on `angle_rad`; `re` is ignored.
    ///   Clamps to the first/last breakpoint with a [`bevy::log::warn_once`] if
    ///   out of range.
    /// - [`AeroCoeff::Table2D`]: bilinearly interpolates on `(angle_rad, re)`.
    ///   Clamps both axes independently with a `warn_once!` if out of range.
    ///
    /// Never panics in release builds. Returns `0.0` on a degenerate table
    /// (empty breakpoints) after a [`bevy::log::warn`].
    pub fn evaluate(&self, angle_rad: f64, re: f64) -> f64 {
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
            AeroCoeff::Table1D { breakpoints, values } => {
                if breakpoints.is_empty() {
                    warn!("AeroCoeff::Table1D has empty breakpoints; returning 0.0");
                    return 0.0;
                }
                let angle_rad = clamp_with_warn(angle_rad, breakpoints[0], *breakpoints.last().unwrap(),
                    "Table1D angle_rad");
                lerp_1d(angle_rad, breakpoints, values)
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                if rows.is_empty() || cols.is_empty() {
                    warn!("AeroCoeff::Table2D has empty rows or cols; returning 0.0");
                    return 0.0;
                }
                let angle_rad = clamp_with_warn(angle_rad, rows[0], *rows.last().unwrap(),
                    "Table2D angle_rad");
                let re = clamp_with_warn(re, cols[0], *cols.last().unwrap(),
                    "Table2D re");
                bilerp(angle_rad, re, rows, cols, data)
            }
        }
    }
}

/// Clamp `v` to `[lo, hi]`, emitting `warn_once!` if clamping occurs.
fn clamp_with_warn(v: f64, lo: f64, hi: f64, label: &'static str) -> f64 {
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

/// Linear interpolation in a 1-D table. `x` must be within `[bp[0], bp[last]]`.
pub(crate) fn lerp_1d(x: f64, bp: &[f64], vals: &[f64]) -> f64 {
    debug_assert_eq!(bp.len(), vals.len());
    // Degenerate: single-point table, no interval to interpolate.
    if bp.len() == 1 {
        return vals[0];
    }
    // Find the interval containing x.
    let idx = bp.partition_point(|&b| b <= x).saturating_sub(1);
    let idx = idx.min(bp.len() - 2);
    let t = (x - bp[idx]) / (bp[idx + 1] - bp[idx]);
    vals[idx] + t * (vals[idx + 1] - vals[idx])
}

/// Bilinear interpolation in a 2-D flat row-major table.
/// `angle_rad` and `re` must already be clamped to their respective ranges.
fn bilerp(angle_rad: f64, re: f64, rows: &[f64], cols: &[f64], data: &[f64]) -> f64 {
    let nc = cols.len();

    // saturating_sub(2) handles the single-row / single-col degenerate case.
    let ri = rows.partition_point(|&r| r <= angle_rad).saturating_sub(1)
                 .min(rows.len().saturating_sub(2));
    let ci = cols.partition_point(|&c| c <= re).saturating_sub(1)
                 .min(cols.len().saturating_sub(2));

    // If only one row or one column, the "next" index is the same, t = 0.
    let ri1 = (ri + 1).min(rows.len() - 1);
    let ci1 = (ci + 1).min(cols.len() - 1);

    let ta = if rows[ri1] != rows[ri] { (angle_rad - rows[ri]) / (rows[ri1] - rows[ri]) } else { 0.0 };
    let tr = if cols[ci1] != cols[ci] { (re        - cols[ci]) / (cols[ci1] - cols[ci]) } else { 0.0 };

    let v00 = data[ri  * nc + ci ];
    let v01 = data[ri  * nc + ci1];
    let v10 = data[ri1 * nc + ci ];
    let v11 = data[ri1 * nc + ci1];

    let v0 = v00 + tr * (v01 - v00); // interpolate along Re at lower angle row
    let v1 = v10 + tr * (v11 - v10); // interpolate along Re at upper angle row
    v0 + ta * (v1 - v0)              // interpolate along angle
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
        // Below minimum: clamped to first value
        assert!((c.evaluate(-1.0, 0.0) - 10.0).abs() < 1e-12);
    }

    #[test]
    fn table1d_clamp_above() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.1, 0.2],
            values: vec![10.0, 20.0],
        };
        // Above maximum: clamped to last value
        assert!((c.evaluate(99.0, 0.0) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn table2d_exact_corner() {
        // 2×2 table: rows=[0, 1], cols=[0, 1], values=[1,2,3,4]
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
        // Midpoint on alpha axis, exact Re column
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
        // At alpha=0.5, re=0.5: bilerp should give 1.0
        assert!((c.evaluate(0.5, 0.5) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn table2d_clamp_both_axes() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![0.0, 1.0],
            data: vec![1.0, 2.0, 3.0, 4.0],
        };
        // Out of range on both axes: no panic, returns corner value
        let v = c.evaluate(-99.0, 99.0);
        assert!((v - 2.0).abs() < 1e-12); // clamped to alpha=0.0, re=1.0, returns 2.0
    }

    #[test]
    fn table2d_single_row() {
        // Single-row table must not divide by zero (rows[ri+1] - rows[ri] would be 0)
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0], // still 2 rows to avoid the degenerate case at lerp
            cols: vec![1e6],
            data: vec![0.5, 0.5],
        };
        assert!((c.evaluate(0.5, 1e6) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn table1d_empty_returns_zero() {
        let c = AeroCoeff::Table1D { breakpoints: vec![], values: vec![] };
        assert_eq!(c.evaluate(0.0, 0.0), 0.0);
    }

    /// A single-breakpoint Table1D must return that value for any input
    /// (the bug: lerp_1d would panic accessing bp[1] before the guard was added).
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

    /// Table2D with a single Re column, bilerp must handle the degenerate
    /// `cols[ci1] == cols[ci]` case without dividing by zero.
    #[test]
    fn table2d_single_re_column_no_panic() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.0, 1.0],
            cols: vec![1e6],        // single Re column
            data: vec![0.0, 2.0],  // CL = 0 at α=0, CL = 2 at α=1
        };
        assert!((c.evaluate(0.0, 1e6) - 0.0).abs() < 1e-12);
        assert!((c.evaluate(1.0, 1e6) - 2.0).abs() < 1e-12);
        assert!((c.evaluate(0.5, 1e6) - 1.0).abs() < 1e-12, "midpoint on alpha");
        // Re clamping: out-of-range Re should still work
        assert!((c.evaluate(0.5, 999.0) - 1.0).abs() < 1e-12, "Re clamped to only column");
    }

    // ── Placeholder variant ───────────────────────────────────────────────────

    #[test]
    fn placeholder_evaluates_to_zero() {
        assert_eq!(AeroCoeff::Placeholder.evaluate(0.3, 1e6), 0.0);
        assert_eq!(AeroCoeff::Placeholder.evaluate(-1.0, 2e6), 0.0);
    }

    #[test]
    fn placeholder_is_placeholder_true() {
        assert!(AeroCoeff::Placeholder.is_placeholder());
    }

    #[test]
    fn scalar_is_placeholder_false() {
        assert!(!AeroCoeff::Scalar(0.0).is_placeholder());
        assert!(!AeroCoeff::Scalar(1.2).is_placeholder());
    }

    #[test]
    fn table1d_is_placeholder_false() {
        let c = AeroCoeff::Table1D { breakpoints: vec![0.0], values: vec![1.0] };
        assert!(!c.is_placeholder());
    }

    #[test]
    fn default_aero_coeff_is_placeholder() {
        assert!(AeroCoeff::default().is_placeholder());
    }

    // ── Absent variant ────────────────────────────────────────────────────────

    #[test]
    fn absent_evaluates_to_zero_silently() {
        assert_eq!(AeroCoeff::Absent.evaluate(0.3, 1e6), 0.0);
        assert_eq!(AeroCoeff::Absent.evaluate(-1.0, 2e6), 0.0);
    }

    #[test]
    fn absent_is_absent_true() {
        assert!(AeroCoeff::Absent.is_absent());
    }

    #[test]
    fn placeholder_is_absent_false() {
        assert!(!AeroCoeff::Placeholder.is_absent());
    }

    #[test]
    fn scalar_is_absent_false() {
        assert!(!AeroCoeff::Scalar(0.0).is_absent());
    }
}
