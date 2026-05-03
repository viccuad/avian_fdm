//! [`AeroCoeff::validate`] and the `is_strictly_increasing` helper.

use avian3d::math::Scalar as S;

use super::types::AeroCoeff;

/// Returns `true` if the slice is strictly increasing (each element > previous).
pub(super) fn is_strictly_increasing(s: &[S]) -> bool {
    s.windows(2).all(|w| w[0] < w[1])
}

impl AeroCoeff {
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
            AeroCoeff::Table1D {
                breakpoints,
                values,
            } => {
                if breakpoints.len() != values.len() {
                    problems.push(format!(
                        "{label}: Table1D breakpoints.len ({}) != values.len ({})",
                        breakpoints.len(),
                        values.len()
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
                        data.len(),
                        rows.len(),
                        cols.len()
                    ));
                }
                if rows.is_empty() {
                    problems.push(format!("{label}: Table2D has zero rows"));
                }
                if cols.is_empty() {
                    problems.push(format!("{label}: Table2D has zero cols"));
                }
                if !is_strictly_increasing(rows) {
                    problems.push(format!("{label}: Table2D rows are not strictly increasing"));
                }
                if !is_strictly_increasing(cols) {
                    problems.push(format!("{label}: Table2D cols are not strictly increasing"));
                }
                if rows
                    .iter()
                    .chain(cols.iter())
                    .chain(data.iter())
                    .any(|v| !v.is_finite())
                {
                    problems.push(format!("{label}: Table2D contains NaN or Inf"));
                }
            }
        }

        problems
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let v = AeroCoeff::Scalar(S::NAN).validate("test");
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
            values: vec![0.0, S::NAN],
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
        assert!(v
            .iter()
            .any(|s| s.contains("rows") && s.contains("strictly increasing")));
    }

    #[test]
    fn validate_table2d_unsorted_cols() {
        let c = AeroCoeff::Table2D {
            rows: vec![-0.3, 0.3],
            cols: vec![2e6, 1e6],
            data: vec![0.0, 0.1, 0.2, 0.3],
        };
        let v = c.validate("cd");
        assert!(v
            .iter()
            .any(|s| s.contains("cols") && s.contains("strictly increasing")));
    }
}
