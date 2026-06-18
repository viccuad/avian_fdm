//! Throttle-to-thrust-fraction interpolation.

use crate::math::lerp_1d;
use avian3d::math::Scalar;

/// Linear interpolation over a `[[throttle, fraction]; N]` lookup table.
///
/// Clamps to the boundary values when `x` is outside the table range.
pub(crate) fn interp_curve(curve: &[[Scalar; 2]], x: Scalar) -> Scalar {
    if curve.is_empty() {
        return 0.0;
    }
    if curve.len() == 1 {
        return curve[0][1];
    }
    let x = x.clamp(curve[0][0], curve[curve.len() - 1][0]);
    let bp: Vec<Scalar> = curve.iter().map(|p| p[0]).collect();
    let vals: Vec<Scalar> = curve.iter().map(|p| p[1]).collect();
    lerp_1d(x, &bp, &vals)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_throttle_curve_midpoint() {
        let curve = vec![[0.0, 0.0], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.5) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn throttle_curve_clamp_above() {
        let curve = vec![[0.0, 0.0], [1.0, 0.9]];
        assert!((interp_curve(&curve, 1.5) - 0.9).abs() < 1e-10);
    }

    #[test]
    fn throttle_curve_clamp_below() {
        let curve = vec![[0.2, 0.1], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.0) - 0.1).abs() < 1e-10);
    }

    #[test]
    fn interp_curve_empty_returns_zero() {
        assert_eq!(interp_curve(&[], 0.5), 0.0);
    }

    #[test]
    fn interp_curve_single_entry_returns_that_value() {
        let curve = vec![[0.5, 0.8]];
        assert!((interp_curve(&curve, 0.0) - 0.8).abs() < 1e-12, "below");
        assert!((interp_curve(&curve, 0.5) - 0.8).abs() < 1e-12, "exact");
        assert!((interp_curve(&curve, 1.0) - 0.8).abs() < 1e-12, "above");
    }

    #[test]
    fn interp_curve_three_breakpoints() {
        let curve = vec![[0.0, 0.0], [0.5, 0.6], [1.0, 1.0]];
        assert!(
            (interp_curve(&curve, 0.0) - 0.0).abs() < 1e-12,
            "lower clamp"
        );
        assert!(
            (interp_curve(&curve, 0.25) - 0.3).abs() < 1e-12,
            "lower segment mid"
        );
        assert!(
            (interp_curve(&curve, 0.5) - 0.6).abs() < 1e-12,
            "breakpoint"
        );
        assert!(
            (interp_curve(&curve, 0.75) - 0.8).abs() < 1e-12,
            "upper segment mid"
        );
        assert!(
            (interp_curve(&curve, 1.0) - 1.0).abs() < 1e-12,
            "upper clamp"
        );
    }

    #[test]
    fn interp_curve_clamps_outside_range() {
        let curve = vec![[0.2, 0.1], [0.8, 0.9]];
        assert!(
            (interp_curve(&curve, 0.0) - 0.1).abs() < 1e-12,
            "below range clamps to first value"
        );
        assert!(
            (interp_curve(&curve, 1.0) - 0.9).abs() < 1e-12,
            "above range clamps to last value"
        );
    }
}
