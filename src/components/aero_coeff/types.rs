//! [`AeroCoeff`] enum and its simple accessors.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

/// The value of one aerodynamic coefficient: a constant, a 1-D table, or a
/// 2-D table.
///
/// The crate uses it for CL, CD, CY, CM, Croll, and Cn. It fits each
/// dimensionless coefficient that can change with the angle of attack, the
/// Reynolds number, or both.
///
/// Call [`AeroCoeff::evaluate`] in each physics step to get an `f32` value
/// at the current flight conditions.
///
/// ## Completeness system: `Absent`, `Placeholder`, and the data variants
///
/// Each coefficient field in [`crate::components::AeroZone`] is an `AeroCoeff`.
/// The variants have different meanings for coefficients without a model:
///
/// | Variant | Meaning | At runtime |
/// |---|---|---|
/// | `Absent` (default for secondary fields) | Does not apply, by design (symmetric section, no CY) | 0.0, no warning |
/// | `Placeholder` (default for primary fields) | Must exist, but has no model yet | `warn_once!` and 0.0 |
/// | `Scalar(0.0)` with [`crate::sourced!`] | Intentional explicit zero | 0.0, no warning |
/// | `Table1D` / `Table2D` | Full model | Interpolated value |
///
/// `Placeholder` is the `Default` for `AeroCoeff`. Thus, a primary field
/// without data gives a warning at runtime. It does not give zero without a
/// warning.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug, PartialEq)]
#[reflect(Serialize, Deserialize)]
pub enum AeroCoeff {
    /// Absent by design. This coefficient does not apply to this zone.
    ///
    /// Evaluates to `0.0` with no warning. Use it for secondary coefficients
    /// that do not exist on a zone because of its structure. Examples: `cy` on
    /// a symmetric main wing, or `croll` when the roll comes fully from the
    /// zone geometry.
    ///
    /// This is the default for the secondary `AeroZone` fields (`cy`, `cm`,
    /// `croll`, `cn`). To show that the absence is intentional, set the field
    /// to `Absent` explicitly.
    Absent,

    /// Explicit sentinel for "no model yet".
    ///
    /// Evaluates to `0.0` (the same as `Scalar(0.0)`). On the first
    /// evaluation, it also emits a `warn_once!`. The warning tells the
    /// aircraft author that this coefficient still needs data.
    ///
    /// This is the [`Default`] value for `AeroCoeff`. It is also the default
    /// for the primary fields (`cl`, `cd`). Each `AeroZone` field that you do
    /// not set explicitly is `Placeholder`. Thus, the gaps are visible at
    /// runtime. They do not add a zero without a warning.
    ///
    /// When you have data, replace it with [`AeroCoeff::Scalar`] (with a
    /// [`crate::sourced!`] note) or with a table.
    Placeholder,

    /// Constant value. Good for simple models.
    Scalar(f32),

    /// 1-D table: the coefficient as a function of the angle of attack (rad).
    ///
    /// `breakpoints` and `values` must have the same length (>= 1).
    /// `breakpoints` must be strictly increasing.
    Table1D {
        /// Angle-of-attack breakpoints in radians, strictly increasing.
        breakpoints: Vec<f32>,
        /// Coefficient values at each breakpoint.
        values: Vec<f32>,
    },

    /// 2-D table: the coefficient as a function of the angle of attack and the Reynolds number.
    ///
    /// The storage is row-major: `data[i * cols.len() + j]` is the value at
    /// `rows[i]` (alpha) and `cols[j]` (Re).
    Table2D {
        /// Angle-of-attack breakpoints (rows), in radians, strictly increasing.
        rows: Vec<f32>,
        /// Reynolds-number breakpoints (columns), strictly increasing.
        cols: Vec<f32>,
        /// Flat row-major coefficient data. Length must equal `rows.len() × cols.len()`.
        data: Vec<f32>,
    },
}

impl Default for AeroCoeff {
    /// Returns [`AeroCoeff::Placeholder`]. Thus, each coefficient without data
    /// gives a warning at runtime, not a zero without a warning.
    fn default() -> Self {
        AeroCoeff::Placeholder
    }
}

impl AeroCoeff {
    /// Returns `true` if this coefficient is `Absent` (does not apply, by design).
    pub fn is_absent(&self) -> bool {
        matches!(self, AeroCoeff::Absent)
    }

    /// Returns `true` if this coefficient is `Placeholder` (no model yet).
    ///
    /// Use it for validation and tools. In the hot path, call
    /// [`evaluate`](Self::evaluate). It gives the warning automatically.
    pub fn is_placeholder(&self) -> bool {
        matches!(self, AeroCoeff::Placeholder)
    }
}
