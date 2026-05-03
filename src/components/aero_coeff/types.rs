//! [`AeroCoeff`] enum and its trivial accessors.

use avian3d::math::Scalar as S;
use serde::{Deserialize, Serialize};
use crate::_bevy::*;

/// An aerodynamic coefficient value: constant, 1-D table, or 2-D table.
///
/// Used for CL, CD, CY, CM, Cl, Cn, any dimensionless coefficient that
/// may depend on angle of attack and/or Reynolds number.
///
/// Call [`AeroCoeff::evaluate`] each frame to obtain a `Scalar` value at the
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
    Scalar(S),

    /// 1-D lookup table: coefficient as a function of angle of attack (rad).
    ///
    /// `breakpoints` and `values` must have the same length (>= 1).
    /// `breakpoints` must be strictly increasing.
    Table1D {
        /// Angle-of-attack breakpoints in radians, strictly increasing.
        breakpoints: Vec<S>,
        /// Coefficient values at each breakpoint.
        values: Vec<S>,
    },

    /// 2-D lookup table: coefficient as a function of angle of attack x Reynolds number.
    ///
    /// Stored row-major: `data[i * cols.len() + j]` is the value at
    /// `rows[i]` (alpha) and `cols[j]` (Re).
    Table2D {
        /// Angle-of-attack breakpoints (rows), in radians, strictly increasing.
        rows: Vec<S>,
        /// Reynolds-number breakpoints (columns), strictly increasing.
        cols: Vec<S>,
        /// Flat row-major coefficient data. Length must equal `rows.len() × cols.len()`.
        data: Vec<S>,
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
}
