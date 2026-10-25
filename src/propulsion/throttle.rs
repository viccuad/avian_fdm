//! Interpolation from the throttle position to the thrust fraction.

use crate::math::lerp_1d;

/// Linear interpolation in a `[[throttle, fraction]; N]` table.
///
/// If `x` is outside the table range, it clamps to the boundary values.
pub(crate) fn interp_curve(curve: &[[f32; 2]], x: f32) -> f32 {
    if curve.is_empty() {
        return 0.0;
    }
    if curve.len() == 1 {
        return curve[0][1];
    }
    let x = x.clamp(curve[0][0], curve[curve.len() - 1][0]);
    let bp: Vec<f32> = curve.iter().map(|p| p[0]).collect();
    let vals: Vec<f32> = curve.iter().map(|p| p[1]).collect();
    lerp_1d(x, &bp, &vals)
}

#[cfg(test)]
mod tests {
    use super::*;

    // The interpolation runs in f32.
    const EPSILON_TOLERANCE: f32 = 1e-6;

    #[test]
    fn linear_throttle_curve_midpoint() {
        let curve = vec![[0.0, 0.0], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.5) - 0.5).abs() < EPSILON_TOLERANCE);
    }

    #[test]
    fn throttle_curve_clamp_above() {
        let curve = vec![[0.0, 0.0], [1.0, 0.9]];
        assert!((interp_curve(&curve, 1.5) - 0.9).abs() < EPSILON_TOLERANCE);
    }

    #[test]
    fn throttle_curve_clamp_below() {
        let curve = vec![[0.2, 0.1], [1.0, 1.0]];
        assert!((interp_curve(&curve, 0.0) - 0.1).abs() < EPSILON_TOLERANCE);
    }

    #[test]
    fn interp_curve_empty_returns_zero() {
        assert_eq!(interp_curve(&[], 0.5), 0.0);
    }

    #[test]
    fn interp_curve_single_entry_returns_that_value() {
        let curve = vec![[0.5, 0.8]];
        assert!(
            (interp_curve(&curve, 0.0) - 0.8).abs() < EPSILON_TOLERANCE,
            "below"
        );
        assert!(
            (interp_curve(&curve, 0.5) - 0.8).abs() < EPSILON_TOLERANCE,
            "exact"
        );
        assert!(
            (interp_curve(&curve, 1.0) - 0.8).abs() < EPSILON_TOLERANCE,
            "above"
        );
    }

    #[test]
    fn interp_curve_three_breakpoints() {
        let curve = vec![[0.0, 0.0], [0.5, 0.6], [1.0, 1.0]];
        assert!(
            (interp_curve(&curve, 0.0) - 0.0).abs() < EPSILON_TOLERANCE,
            "lower clamp"
        );
        assert!(
            (interp_curve(&curve, 0.25) - 0.3).abs() < EPSILON_TOLERANCE,
            "lower segment mid"
        );
        assert!(
            (interp_curve(&curve, 0.5) - 0.6).abs() < EPSILON_TOLERANCE,
            "breakpoint"
        );
        assert!(
            (interp_curve(&curve, 0.75) - 0.8).abs() < EPSILON_TOLERANCE,
            "upper segment mid"
        );
        assert!(
            (interp_curve(&curve, 1.0) - 1.0).abs() < EPSILON_TOLERANCE,
            "upper clamp"
        );
    }

    #[test]
    fn interp_curve_clamps_outside_range() {
        let curve = vec![[0.2, 0.1], [0.8, 0.9]];
        assert!(
            (interp_curve(&curve, 0.0) - 0.1).abs() < EPSILON_TOLERANCE,
            "below range clamps to first value"
        );
        assert!(
            (interp_curve(&curve, 1.0) - 0.9).abs() < EPSILON_TOLERANCE,
            "above range clamps to last value"
        );
    }
}
