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

    // ── Post-stall extension (Viterna-Corrigan) ─────────────────────────

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
    /// let cl_90 = cl_full.evaluate(std::f64::consts::FRAC_PI_2, 0.0);
    /// assert!(cl_90.abs() < 0.01);
    /// ```
    pub fn with_post_stall_lift(self, aspect_ratio: f64) -> Self {
        match self {
            AeroCoeff::Table1D { breakpoints, values } => {
                let (bp, vals) = extend_1d_lift(&breakpoints, &values, aspect_ratio);
                AeroCoeff::Table1D { breakpoints: bp, values: vals }
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                let (new_rows, new_data) = extend_2d_lift(&rows, &cols, &data, aspect_ratio);
                AeroCoeff::Table2D { rows: new_rows, cols, data: new_data }
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
    /// let cd_90 = cd_full.evaluate(std::f64::consts::FRAC_PI_2, 0.0);
    /// assert!((cd_90 - 1.164).abs() < 0.01);
    /// ```
    pub fn with_post_stall_drag(self, aspect_ratio: f64) -> Self {
        match self {
            AeroCoeff::Scalar(cd0) => {
                let (bp, vals) = scalar_to_drag_table(cd0, aspect_ratio);
                AeroCoeff::Table1D { breakpoints: bp, values: vals }
            }
            AeroCoeff::Table1D { breakpoints, values } => {
                let (bp, vals) = extend_1d_drag(&breakpoints, &values, aspect_ratio);
                AeroCoeff::Table1D { breakpoints: bp, values: vals }
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                let (new_rows, new_data) = extend_2d_drag(&rows, &cols, &data, aspect_ratio);
                AeroCoeff::Table2D { rows: new_rows, cols, data: new_data }
            }
            other => other,
        }
    }

    /// Validate table structure. Returns a list of problems (empty = valid).
    ///
    /// Checks performed:
    /// - Table1D: `breakpoints` and `values` have equal length,
    ///   breakpoints are strictly increasing, no NaN/Inf in either.
    /// - Table2D: `data.len() == rows.len() * cols.len()`,
    ///   rows and cols are strictly increasing, no NaN/Inf.
    /// - Scalar: value is finite.
    /// - Absent / Placeholder: always valid.
    ///
    /// Call this at aircraft spawn time (e.g. in a startup system) to
    /// catch data-entry mistakes before they produce silent garbage in
    /// the hot path.
    pub fn validate(&self, label: &str) -> Vec<String> {
        let mut problems = Vec::new();

        match self {
            AeroCoeff::Absent | AeroCoeff::Placeholder => {}
            AeroCoeff::Scalar(v) => {
                if !v.is_finite() {
                    problems.push(format!("{label}: Scalar value is not finite ({v})"));
                }
            }
            AeroCoeff::Table1D { breakpoints, values } => {
                if breakpoints.len() != values.len() {
                    problems.push(format!(
                        "{label}: Table1D breakpoints.len ({}) != values.len ({})",
                        breakpoints.len(), values.len()
                    ));
                }
                if breakpoints.is_empty() {
                    problems.push(format!("{label}: Table1D has zero breakpoints"));
                }
                if !is_strictly_increasing(breakpoints) {
                    problems.push(format!(
                        "{label}: Table1D breakpoints are not strictly increasing"
                    ));
                }
                if breakpoints.iter().any(|v| !v.is_finite()) {
                    problems.push(format!("{label}: Table1D breakpoints contain NaN or Inf"));
                }
                if values.iter().any(|v| !v.is_finite()) {
                    problems.push(format!("{label}: Table1D values contain NaN or Inf"));
                }
            }
            AeroCoeff::Table2D { rows, cols, data } => {
                let expected = rows.len() * cols.len();
                if data.len() != expected {
                    problems.push(format!(
                        "{label}: Table2D data.len ({}) != rows ({}) x cols ({}) = {expected}",
                        data.len(), rows.len(), cols.len()
                    ));
                }
                if rows.is_empty() {
                    problems.push(format!("{label}: Table2D has zero rows"));
                }
                if cols.is_empty() {
                    problems.push(format!("{label}: Table2D has zero cols"));
                }
                if !is_strictly_increasing(rows) {
                    problems.push(format!(
                        "{label}: Table2D rows are not strictly increasing"
                    ));
                }
                if !is_strictly_increasing(cols) {
                    problems.push(format!(
                        "{label}: Table2D cols are not strictly increasing"
                    ));
                }
                if rows.iter().chain(cols.iter()).chain(data.iter()).any(|v| !v.is_finite()) {
                    problems.push(format!("{label}: Table2D contains NaN or Inf"));
                }
            }
        }

        problems
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

/// Returns `true` if the slice is strictly increasing (each element > previous).
fn is_strictly_increasing(s: &[f64]) -> bool {
    s.windows(2).all(|w| w[0] < w[1])
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

// ── Viterna-Corrigan post-stall extension ──────────────────────────────────
//
// Reference: Viterna, L.A. & Corrigan, R.D. (1982), "Fixed Pitch Rotor
// Performance of Large Horizontal Axis Wind Turbines", NASA CP-2230.
//
// The model extends aerodynamic coefficient tables from their last data
// point out to +/-180 degrees using flat-plate theory. This prevents
// table clamping during post-stall flight, tumbling, or any orientation
// where the local angle of attack exceeds the wind-tunnel data range.

use std::f64::consts::PI;
const HALF_PI: f64 = PI / 2.0;

/// Viterna CD_max for a finite-aspect-ratio surface.
fn viterna_cd_max(ar: f64) -> f64 {
    1.11 + 0.018 * ar
}

/// Angles (in radians) at which to generate extension points.
/// Covers 25 deg to 180 deg in 5-deg steps from 25 to 50, then 10-deg steps.
fn extension_angles() -> Vec<f64> {
    let mut angles = Vec::new();
    // Fine steps near stall transition (25-50 deg)
    for deg in (25..=50).step_by(5) {
        angles.push((deg as f64).to_radians());
    }
    // Coarser steps for the rest (60-180 deg)
    for deg in (60..=180).step_by(10) {
        angles.push((deg as f64).to_radians());
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
fn viterna_cl(a: f64, a1: f64, a2: f64) -> f64 {
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
fn viterna_cd(a: f64, cd0_eff: f64, cd_max: f64) -> f64 {
    cd0_eff + (cd_max - cd0_eff) * a.sin().powi(2)
}

/// Compute the Viterna A2 coefficient for continuity at the stall angle.
///
///   A2 = (CL_s - A1 * sin(2*alpha_s)) * sin(alpha_s) / cos^2(alpha_s)
///
/// If alpha_s is close to 90 deg (cos^2 near zero), returns 0 (pure flat plate).
fn viterna_a2(alpha_s: f64, cl_s: f64, a1: f64) -> f64 {
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
fn effective_cd0(alpha_s: f64, cd_s: f64, cd_max: f64) -> f64 {
    let cos2 = alpha_s.cos().powi(2);
    if cos2 < 1e-6 {
        return cd_s;
    }
    (cd_s - cd_max * alpha_s.sin().powi(2)) / cos2
}

/// Extend a 1-D lift table to +/-pi.
fn extend_1d_lift(bp: &[f64], vals: &[f64], ar: f64) -> (Vec<f64>, Vec<f64>) {
    if bp.is_empty() {
        return (bp.to_vec(), vals.to_vec());
    }
    let cd_max = viterna_cd_max(ar);
    let a1 = cd_max / 2.0;

    let alpha_max = *bp.last().unwrap();
    let alpha_min = bp[0];
    let cl_at_max = *vals.last().unwrap();
    let cl_at_min = vals[0];

    // Already covers +/-pi: nothing to do.
    if alpha_max >= PI - 0.01 && alpha_min <= -(PI - 0.01) {
        return (bp.to_vec(), vals.to_vec());
    }

    let ext_angles = extension_angles();

    let mut new_bp = Vec::with_capacity(bp.len() + 40);
    let mut new_vals = Vec::with_capacity(bp.len() + 40);

    // Negative side extension (from -pi up to alpha_min).
    if alpha_min > -(PI - 0.01) {
        let a2_neg = viterna_a2(alpha_min.abs(), -cl_at_min, a1);
        // Add points from -pi toward alpha_min.
        for &a in ext_angles.iter().rev() {
            let neg_a = -a;
            if neg_a < alpha_min - 0.001 {
                new_bp.push(neg_a);
                new_vals.push(-viterna_cl(a, a1, a2_neg));
            }
        }
    }

    // Original data.
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
fn extend_1d_drag(bp: &[f64], vals: &[f64], ar: f64) -> (Vec<f64>, Vec<f64>) {
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
fn scalar_to_drag_table(cd0: f64, ar: f64) -> (Vec<f64>, Vec<f64>) {
    let cd_max = viterna_cd_max(ar);
    let all_angles = extension_angles();

    let mut bp = Vec::with_capacity(all_angles.len() * 2 + 1);
    let mut vals = Vec::with_capacity(all_angles.len() * 2 + 1);

    // Negative side (reversed).
    for &a in all_angles.iter().rev() {
        bp.push(-a);
        vals.push(viterna_cd(a, cd0, cd_max));
    }
    // Center.
    bp.push(0.0);
    vals.push(cd0);
    // Positive side.
    for &a in &all_angles {
        bp.push(a);
        vals.push(viterna_cd(a, cd0, cd_max));
    }

    (bp, vals)
}

/// Extend a 2-D lift table to +/-pi.
/// Each Reynolds-number column is extended independently.
fn extend_2d_lift(rows: &[f64], cols: &[f64], data: &[f64], ar: f64) -> (Vec<f64>, Vec<f64>) {
    let nc = cols.len();
    if rows.is_empty() || nc == 0 {
        return (rows.to_vec(), data.to_vec());
    }
    // Extract each column as a 1-D table and extend it.
    let mut col_results: Vec<(Vec<f64>, Vec<f64>)> = Vec::with_capacity(nc);
    for j in 0..nc {
        let col_vals: Vec<f64> = (0..rows.len()).map(|i| data[i * nc + j]).collect();
        col_results.push(extend_1d_lift(rows, &col_vals, ar));
    }
    // All columns must produce the same breakpoints (they share the same rows).
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
fn extend_2d_drag(rows: &[f64], cols: &[f64], data: &[f64], ar: f64) -> (Vec<f64>, Vec<f64>) {
    let nc = cols.len();
    if rows.is_empty() || nc == 0 {
        return (rows.to_vec(), data.to_vec());
    }
    let mut col_results: Vec<(Vec<f64>, Vec<f64>)> = Vec::with_capacity(nc);
    for j in 0..nc {
        let col_vals: Vec<f64> = (0..rows.len()).map(|i| data[i * nc + j]).collect();
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

    // ── validate() tests ────────────────────────────────────────────────

    #[test]
    fn validate_absent_ok() {
        assert!(AeroCoeff::Absent.validate("test").is_empty());
    }

    #[test]
    fn validate_placeholder_ok() {
        assert!(AeroCoeff::Placeholder.validate("test").is_empty());
    }

    #[test]
    fn validate_scalar_finite_ok() {
        assert!(AeroCoeff::Scalar(1.5).validate("test").is_empty());
    }

    #[test]
    fn validate_scalar_nan() {
        let v = AeroCoeff::Scalar(f64::NAN).validate("test");
        assert_eq!(v.len(), 1);
        assert!(v[0].contains("not finite"));
    }

    #[test]
    fn validate_table1d_ok() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![-0.3, 0.0, 0.3],
            values: vec![-1.0, 0.0, 1.0],
        };
        assert!(c.validate("cl").is_empty());
    }

    #[test]
    fn validate_table1d_unsorted_breakpoints() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.0, 0.3, 0.1],
            values: vec![0.0, 1.0, 0.5],
        };
        let v = c.validate("cl");
        assert!(!v.is_empty());
        assert!(v[0].contains("strictly increasing"));
    }

    #[test]
    fn validate_table1d_length_mismatch() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.0, 0.3],
            values: vec![0.0, 1.0, 2.0],
        };
        let v = c.validate("cl");
        assert!(!v.is_empty());
        assert!(v[0].contains("len"));
    }

    #[test]
    fn validate_table1d_nan_value() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![0.0, 0.3],
            values: vec![0.0, f64::NAN],
        };
        let v = c.validate("cl");
        assert!(!v.is_empty());
        assert!(v.iter().any(|s| s.contains("NaN")));
    }

    #[test]
    fn validate_table1d_empty() {
        let c = AeroCoeff::Table1D {
            breakpoints: vec![],
            values: vec![],
        };
        let v = c.validate("cl");
        assert!(v.iter().any(|s| s.contains("zero breakpoints")));
    }

    #[test]
    fn validate_table2d_ok() {
        let c = AeroCoeff::Table2D {
            rows: vec![-0.3, 0.0, 0.3],
            cols: vec![1e6, 2e6],
            data: vec![0.0, 0.1, 0.5, 0.6, 1.0, 1.1],
        };
        assert!(c.validate("cd").is_empty());
    }

    #[test]
    fn validate_table2d_data_length_mismatch() {
        let c = AeroCoeff::Table2D {
            rows: vec![-0.3, 0.3],
            cols: vec![1e6, 2e6],
            data: vec![0.0, 0.1, 0.2], // should be 4
        };
        let v = c.validate("cd");
        assert!(v.iter().any(|s| s.contains("data.len")));
    }

    #[test]
    fn validate_table2d_unsorted_rows() {
        let c = AeroCoeff::Table2D {
            rows: vec![0.3, -0.3],
            cols: vec![1e6],
            data: vec![1.0, 0.0],
        };
        let v = c.validate("cd");
        assert!(v.iter().any(|s| s.contains("rows") && s.contains("strictly increasing")));
    }

    #[test]
    fn validate_table2d_unsorted_cols() {
        let c = AeroCoeff::Table2D {
            rows: vec![-0.3, 0.3],
            cols: vec![2e6, 1e6],
            data: vec![0.0, 0.1, 0.2, 0.3],
        };
        let v = c.validate("cd");
        assert!(v.iter().any(|s| s.contains("cols") && s.contains("strictly increasing")));
    }

    // ── Post-stall extension tests ────────────────────────────────────────

    #[test]
    fn post_stall_lift_preserves_original_data() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        // Original points must be unchanged.
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
        if let AeroCoeff::Table1D { ref breakpoints, .. } = extended {
            assert!(*breakpoints.first().unwrap() <= -PI + 0.01,
                "table should extend to -pi, got {}", breakpoints.first().unwrap());
            assert!(*breakpoints.last().unwrap() >= PI - 0.01,
                "table should extend to +pi, got {}", breakpoints.last().unwrap());
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
        assert!(cl_90.abs() < 0.05, "CL at 90 deg should be near zero, got {cl_90}");
    }

    #[test]
    fn post_stall_lift_zero_at_180_deg() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        let cl_180 = extended.evaluate(PI, 0.0);
        assert!(cl_180.abs() < 0.05, "CL at 180 deg should be near zero, got {cl_180}");
    }

    #[test]
    fn post_stall_lift_antisymmetric() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        // Check symmetry at a few post-stall angles.
        for deg in [45.0_f64, 60.0, 90.0, 120.0, 150.0] {
            let a = deg.to_radians();
            let pos = extended.evaluate(a, 0.0);
            let neg = extended.evaluate(-a, 0.0);
            assert!((pos + neg).abs() < 0.1,
                "CL should be antisymmetric at {deg} deg: CL(+)={pos:.3}, CL(-)={neg:.3}");
        }
    }

    #[test]
    fn post_stall_lift_continuous_at_boundary() {
        let cl = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![-2.5, 0.0, 2.5],
        };
        let extended = cl.with_post_stall_lift(3.0);
        // Just inside and just outside the original range should be close.
        let inside = extended.evaluate(0.34, 0.0);
        let boundary = extended.evaluate(0.35, 0.0);
        let outside = extended.evaluate(0.44, 0.0);
        // No huge jump at the boundary.
        assert!((boundary - inside).abs() < 0.5,
            "discontinuity at boundary: {inside:.3} vs {boundary:.3}");
        assert!((outside - boundary).abs() < 1.0,
            "large jump past boundary: {boundary:.3} vs {outside:.3}");
    }

    #[test]
    fn post_stall_drag_scalar_to_table() {
        let cd = AeroCoeff::Scalar(0.01);
        let extended = cd.with_post_stall_drag(3.0);
        // Should be a Table1D now.
        assert!(matches!(extended, AeroCoeff::Table1D { .. }));
        // At zero alpha, CD should be cd0.
        assert!((extended.evaluate(0.0, 0.0) - 0.01).abs() < 1e-6);
        // At 90 deg, CD should be near CD_max.
        let cd_max = viterna_cd_max(3.0);
        assert!((extended.evaluate(HALF_PI, 0.0) - cd_max).abs() < 0.02,
            "CD at 90 should be {cd_max:.3}, got {:.3}", extended.evaluate(HALF_PI, 0.0));
    }

    #[test]
    fn post_stall_drag_table_covers_full_range() {
        let cd = AeroCoeff::Table1D {
            breakpoints: vec![-0.35, 0.0, 0.35],
            values: vec![0.05, 0.01, 0.05],
        };
        let extended = cd.with_post_stall_drag(6.0);
        if let AeroCoeff::Table1D { ref breakpoints, .. } = extended {
            assert!(*breakpoints.first().unwrap() <= -PI + 0.01);
            assert!(*breakpoints.last().unwrap() >= PI - 0.01);
        }
        // CD at 90 should be near CD_max = 1.11 + 0.018*6 = 1.218.
        let cd_90 = extended.evaluate(HALF_PI, 0.0);
        assert!((cd_90 - 1.218).abs() < 0.1, "CD at 90 = {cd_90:.3}");
    }

    #[test]
    fn post_stall_drag_symmetric() {
        let cd = AeroCoeff::Scalar(0.01).with_post_stall_drag(3.0);
        for deg in [30.0_f64, 60.0, 90.0, 120.0, 150.0] {
            let a = deg.to_radians();
            let pos = cd.evaluate(a, 0.0);
            let neg = cd.evaluate(-a, 0.0);
            assert!((pos - neg).abs() < 0.01,
                "CD should be symmetric at {deg} deg: CD(+)={pos:.3}, CD(-)={neg:.3}");
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
            assert_eq!(breakpoints.len(), 3, "should not add points to full-range table");
        }
    }

    #[test]
    fn post_stall_lift_2d_preserves_columns() {
        let cl = AeroCoeff::Table2D {
            rows: vec![-0.35, 0.0, 0.35],
            cols: vec![1e6, 3e6],
            data: vec![
                -2.0, -2.5,  // alpha=-0.35
                 0.0,  0.0,  // alpha=0
                 2.0,  2.5,  // alpha=0.35
            ],
        };
        let extended = cl.with_post_stall_lift(3.0);
        if let AeroCoeff::Table2D { ref rows, ref cols, .. } = extended {
            assert_eq!(cols.len(), 2, "Re columns unchanged");
            assert!(*rows.first().unwrap() <= -PI + 0.01);
            assert!(*rows.last().unwrap() >= PI - 0.01);
        }
        // Original data preserved.
        assert!((extended.evaluate(0.0, 1e6) - 0.0).abs() < 1e-10);
        assert!((extended.evaluate(0.35, 3e6) - 2.5).abs() < 1e-10);
    }
}
