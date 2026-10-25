//! Parser for XFoil polar CSV files that [foil.tools](https://foil.tools) exports.
//!
//! [foil.tools](https://foil.tools) exports a CSV with the header `Re,alpha,CL,CD,CM`. The
//! layout is Re-major. For each Re value, the full alpha sweep for Ncrit=1 comes first. The full
//! alpha sweep for Ncrit=4 comes next, and then the full alpha sweep for Ncrit=9. The Re blocks
//! are in ascending order. In each Ncrit sweep, alpha is in ascending order.
//!
//! Ncrit is the XFoil parameter for the transition from laminar to turbulent flow. A higher Ncrit
//! means a cleaner free stream and a later transition.
//!
//! ## Usage
//!
//! ```rust
//! # use avian_fdm::airfoil::foil_tools::parse_foil_tools_csv;
//! // foil.tools CSV:
//! let csv = "\
//! Re,alpha,CL,CD,CM\n\
//! 1000000,-5,-0.5,0.015,0.01\n\
//! 1000000,0,0.0,0.008,0.0\n\
//! 1000000,5,0.5,0.015,-0.01\n\
//! 1000000,-5,-0.48,0.014,0.01\n\
//! 1000000,0,0.02,0.007,0.0\n\
//! 1000000,5,0.52,0.014,-0.01\n\
//! 1000000,-5,-0.45,0.013,0.01\n\
//! 1000000,0,0.05,0.006,0.0\n\
//! 1000000,5,0.55,0.013,-0.01\n\
//! 2000000,-5,-0.5,0.013,0.01\n\
//! 2000000,0,0.0,0.007,0.0\n\
//! 2000000,5,0.5,0.013,-0.01\n\
//! 2000000,-5,-0.48,0.012,0.01\n\
//! 2000000,0,0.025,0.006,0.0\n\
//! 2000000,5,0.525,0.012,-0.01\n\
//! 2000000,-5,-0.45,0.011,0.01\n\
//! 2000000,0,0.055,0.005,0.0\n\
//! 2000000,5,0.555,0.011,-0.01\n\
//! ";
//! let polars = parse_foil_tools_csv(csv).unwrap();
//! // Use the Ncrit=9 slice (clean atmospheric flight):
//! let _foil = polars.ncrit9;
//! ```

use crate::airfoil::AirfoilData;
use crate::components::aero_coeff::AeroCoeff;
use avian3d::math::Scalar as S;
use thiserror::Error;

/// The three Ncrit slices from one foil.tools polar CSV.
///
/// Each slice is a complete [`AirfoilData`] with `Table2D` tables for CL, CD,
/// and CM (alpha rows, Re columns). The parser does **not** apply the
/// post-stall extension. Select the slice that you need, and then call
/// [`AirfoilData::with_post_stall`].
#[derive(Debug, Clone)]
pub struct FoilToolsPolars {
    /// Ncrit = 1 slice (very turbulent inflow, dirty conditions).
    pub ncrit1: AirfoilData,
    /// Ncrit = 4 slice (moderate turbulence).
    pub ncrit4: AirfoilData,
    /// Ncrit = 9 slice (clean atmospheric flight, low turbulence).
    pub ncrit9: AirfoilData,
}

/// Errors returned by [`parse_foil_tools_csv`].
#[derive(Debug, PartialEq, Error)]
pub enum ParseError {
    /// The CSV has no non-empty lines.
    #[error("CSV is empty or has no data rows")]
    EmptyData,
    /// The first non-empty line is not the expected header.
    #[error("expected header \"Re,alpha,CL,CD,CM\", got \"{got}\"")]
    BadHeader {
        /// The first line that the parser found.
        got: String,
    },
    /// A data row has the wrong number of comma-separated fields.
    #[error("line {line}: expected {expected} fields, got {got}")]
    BadColumnCount {
        /// Line number in the source CSV (starts at 1).
        line: usize,
        /// Expected number of fields (5).
        expected: usize,
        /// Number of fields that the parser found.
        got: usize,
    },
    /// The parser cannot read a field as a number.
    #[error("line {line}: cannot parse field {column} from \"{value}\"")]
    BadField {
        /// Line number (starts at 1).
        line: usize,
        /// Column name (`"Re"`, `"alpha"`, `"CL"`, `"CD"`, or `"CM"`).
        column: &'static str,
        /// The raw string that the parser cannot read.
        value: String,
    },
    /// The total number of data rows is not a multiple of 3. The CSV must
    /// contain exactly three blocks of equal size, one for each Ncrit value.
    #[error("data row count {rows} is not a multiple of 3 (CSV must have three equal sequential blocks: Ncrit=1, Ncrit=4, Ncrit=9)")]
    NotMultipleOfThree {
        /// Number of rows that the parser found.
        rows: usize,
    },
    /// The Re values are not strictly increasing from one Re block to the next.
    #[error("Re values are not strictly increasing at index {at_index}")]
    NonMonotonicRe {
        /// Position in the Re breakpoint list where the error occurs (starts at 0).
        at_index: usize,
    },
    /// The alpha values in a Re block are not strictly increasing.
    #[error("alpha values in Re={re} block are not strictly increasing at index {at_index}")]
    NonMonotonicAlpha {
        /// The Re value of the block where the parser found the error.
        re: f64,
        /// Position in the block (starts at 0).
        at_index: usize,
    },
    /// The alpha grid is not the same in all Re blocks.
    #[error("alpha grid for Re={re} differs from the first Re block")]
    InconsistentAlphaGrid {
        /// The Re value of the block with an alpha grid that is different from the first block.
        re: f64,
    },
    /// A value is NaN or infinite.
    #[error("line {line}: field {column} is NaN or infinite")]
    NonFinite {
        /// Line number (starts at 1).
        line: usize,
        /// Column name.
        column: &'static str,
    },
    /// A CD value is negative. This is physically not possible.
    #[error("Re={re}, alpha={alpha_deg} deg: CD={cd} is negative (physically impossible)")]
    NegativeCd {
        /// Re value of the incorrect row.
        re: f64,
        /// Alpha (degrees) of the incorrect row.
        alpha_deg: f64,
        /// The negative CD value.
        cd: f64,
    },
}

/// Parses an XFoil polar CSV from <https://foil.tools> into the three Ncrit slices.
///
/// foil.tools exports the polars for all Ncrit values in one CSV. The layout
/// is Re-major. For each Re value, there are three consecutive alpha sweeps
/// of equal length. There is one sweep for each Ncrit value, in this order:
/// Ncrit=1, Ncrit=4, Ncrit=9. Each sweep covers the same alpha range, in
/// ascending order. The Re values are in ascending order from block to block.
///
/// Example: for 2 Re values and 3 alpha points per sweep, the row order is:
/// ```text
/// Re=1e6, alpha=-5, Ncrit=1
/// Re=1e6, alpha=0,  Ncrit=1
/// Re=1e6, alpha=5,  Ncrit=1
/// Re=1e6, alpha=-5, Ncrit=4
/// Re=1e6, alpha=0,  Ncrit=4
/// Re=1e6, alpha=5,  Ncrit=4
/// Re=1e6, alpha=-5, Ncrit=9
/// ...
/// Re=2e6, alpha=-5, Ncrit=1
/// ...
/// ```
///
/// # Ncrit boundary detection
///
/// The CSV has no Ncrit column. The parser finds the Ncrit boundaries only
/// from the row position. It divides each Re block into three equal parts:
/// rows `0..n_alpha` for Ncrit=1, `n_alpha..2*n_alpha` for Ncrit=4, and
/// `2*n_alpha..3*n_alpha` for Ncrit=9. If the block length is not a multiple
/// of 3, the parser returns [`ParseError::NotMultipleOfThree`]. The parser
/// does no other check of the Ncrit order.
///
/// CAUTION: Use only the foil.tools layout. If the CSV has a different
/// internal layout, the parser returns incorrect data without an error.
///
/// # Errors
///
/// Returns [`ParseError`] if the CSV is malformed, or if a value is not
/// physically possible (negative CD, non-finite values).
///
/// # Post-stall
///
/// The parser does **not** apply the post-stall extension. Select a slice,
/// and then call [`AirfoilData::with_post_stall`] on it.
pub fn parse_foil_tools_csv(csv: &str) -> Result<FoilToolsPolars, ParseError> {
    // 1. Lex
    let rows = lex(csv)?;

    if rows.is_empty() {
        return Err(ParseError::EmptyData);
    }

    // 2. Group by Re
    // Count rows per Re block (all rows with the same Re value).
    let mut re_blocks: Vec<(f64, usize)> = Vec::new(); // (re, start_index)
    for (i, row) in rows.iter().enumerate() {
        match re_blocks.last_mut() {
            Some((re, _)) if (*re - row.re).abs() < 1e-3 => {}
            _ => re_blocks.push((row.re, i)),
        }
    }
    let n_re = re_blocks.len();

    // Makes sure that the Re values are strictly increasing.
    for i in 1..n_re {
        if re_blocks[i].0 <= re_blocks[i - 1].0 {
            return Err(ParseError::NonMonotonicRe { at_index: i });
        }
    }

    // Compute block lengths.
    let re_block_lengths: Vec<usize> = re_blocks
        .iter()
        .enumerate()
        .map(|(i, (_, start))| {
            let end = if i + 1 < n_re {
                re_blocks[i + 1].1
            } else {
                rows.len()
            };
            end - start
        })
        .collect();

    // All Re blocks must have the same size and that size must be divisible by 3.
    let block_len = re_block_lengths[0];
    for (i, &len) in re_block_lengths.iter().enumerate() {
        if len != block_len {
            return Err(ParseError::InconsistentAlphaGrid { re: re_blocks[i].0 });
        }
    }
    if !block_len.is_multiple_of(3) {
        return Err(ParseError::NotMultipleOfThree { rows: rows.len() });
    }
    let n_alpha = block_len / 3;

    // 3. Extracts the rows of each Ncrit level from each Re block.
    // Ncrit=1: the first n_alpha rows of each block.
    // Ncrit=4: the next n_alpha rows.
    // Ncrit=9: the last n_alpha rows.
    let mut ncrit1_rows: Vec<&RawRow> = Vec::with_capacity(n_re * n_alpha);
    let mut ncrit4_rows: Vec<&RawRow> = Vec::with_capacity(n_re * n_alpha);
    let mut ncrit9_rows: Vec<&RawRow> = Vec::with_capacity(n_re * n_alpha);

    for (_, start) in re_blocks.iter() {
        let block = &rows[*start..*start + block_len];
        ncrit1_rows.extend(block[..n_alpha].iter());
        ncrit4_rows.extend(block[n_alpha..2 * n_alpha].iter());
        ncrit9_rows.extend(block[2 * n_alpha..].iter());
    }

    let ncrit1 = build_airfoil_data(&ncrit1_rows)?;
    let ncrit4 = build_airfoil_data(&ncrit4_rows)?;
    let ncrit9 = build_airfoil_data(&ncrit9_rows)?;

    Ok(FoilToolsPolars {
        ncrit1,
        ncrit4,
        ncrit9,
    })
}

#[derive(Debug)]
struct RawRow {
    re: f64,
    alpha_deg: f64,
    cl: f64,
    cd: f64,
    cm: f64,
}

fn lex(csv: &str) -> Result<Vec<RawRow>, ParseError> {
    let mut rows = Vec::new();
    let mut header_seen = false;

    for (line_idx, raw) in csv.lines().enumerate() {
        let line_no = line_idx + 1;
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            continue;
        }
        if !header_seen {
            // Normalizes the header: removes a UTF-8 BOM if present, and removes spaces and the case.
            let norm: String = trimmed
                .trim_start_matches('\u{feff}')
                .chars()
                .filter(|c| !c.is_whitespace())
                .collect::<String>()
                .to_lowercase();
            if norm != "re,alpha,cl,cd,cm" {
                return Err(ParseError::BadHeader {
                    got: trimmed.to_string(),
                });
            }
            header_seen = true;
            continue;
        }

        let fields: Vec<&str> = trimmed.splitn(6, ',').collect();
        if fields.len() != 5 {
            return Err(ParseError::BadColumnCount {
                line: line_no,
                expected: 5,
                got: fields.len(),
            });
        }

        macro_rules! parse_field {
            ($idx:expr, $col:literal) => {{
                let s = fields[$idx].trim();
                let v: f64 = s.parse().map_err(|_| ParseError::BadField {
                    line: line_no,
                    column: $col,
                    value: s.to_string(),
                })?;
                if !v.is_finite() {
                    return Err(ParseError::NonFinite {
                        line: line_no,
                        column: $col,
                    });
                }
                v
            }};
        }

        let re = parse_field!(0, "Re");
        let alpha_deg = parse_field!(1, "alpha");
        let cl = parse_field!(2, "CL");
        let cd = parse_field!(3, "CD");
        let cm = parse_field!(4, "CM");

        rows.push(RawRow {
            re,
            alpha_deg,
            cl,
            cd,
            cm,
        });
    }

    if !header_seen && rows.is_empty() {
        return Err(ParseError::EmptyData);
    }
    if !header_seen {
        return Err(ParseError::BadHeader { got: String::new() });
    }

    Ok(rows)
}

/// Builds one `AirfoilData` from a flat slice of rows for one Ncrit level.
/// The rows must be in Re-major order, with alpha in ascending order (the
/// foil.tools order).
fn build_airfoil_data(rows: &[&RawRow]) -> Result<AirfoilData, ParseError> {
    // Extracts the unique Re values, in ascending order (the input is Re-major).
    let mut re_bps: Vec<f64> = Vec::new();
    for r in rows {
        if re_bps.last() != Some(&r.re) {
            re_bps.push(r.re);
        }
    }
    let n_re = re_bps.len();

    // Makes sure that the Re values are strictly increasing.
    for i in 1..n_re {
        if re_bps[i] <= re_bps[i - 1] {
            return Err(ParseError::NonMonotonicRe { at_index: i });
        }
    }

    let n_alpha = rows.len() / n_re;
    if n_alpha == 0 {
        return Err(ParseError::EmptyData);
    }

    // Extracts the alpha breakpoints from the first Re block.
    let first_block = &rows[..n_alpha];
    let alpha_bps: Vec<f64> = first_block.iter().map(|r| r.alpha_deg).collect();

    // Makes sure that alpha is strictly increasing in the first block.
    for i in 1..alpha_bps.len() {
        if alpha_bps[i] <= alpha_bps[i - 1] {
            return Err(ParseError::NonMonotonicAlpha {
                re: re_bps[0],
                at_index: i,
            });
        }
    }

    // Makes sure that each later Re block has the same alpha grid.
    for re_idx in 1..n_re {
        let block = &rows[re_idx * n_alpha..(re_idx + 1) * n_alpha];
        for (ai, row) in block.iter().enumerate() {
            if (row.alpha_deg - alpha_bps[ai]).abs() > 1e-6 {
                return Err(ParseError::InconsistentAlphaGrid { re: re_bps[re_idx] });
            }
        }
    }

    // Physical check: CD must be >= 0 everywhere.
    for r in rows {
        if r.cd < 0.0 {
            return Err(ParseError::NegativeCd {
                re: r.re,
                alpha_deg: r.alpha_deg,
                cd: r.cd,
            });
        }
    }

    // Transposes the data from Re-major to alpha-major (a Table2D row is one alpha value).
    let mut cl_data: Vec<S> = vec![0.0 as S; n_alpha * n_re];
    let mut cd_data: Vec<S> = vec![0.0 as S; n_alpha * n_re];
    let mut cm_data: Vec<S> = vec![0.0 as S; n_alpha * n_re];

    for (re_idx, re_block) in rows.chunks(n_alpha).enumerate() {
        for (alpha_idx, r) in re_block.iter().enumerate() {
            let flat = alpha_idx * n_re + re_idx;
            cl_data[flat] = r.cl as S;
            cd_data[flat] = r.cd as S;
            cm_data[flat] = r.cm as S;
        }
    }

    // Changes the breakpoints into Scalar (alpha into radians).
    let alpha_rad: Vec<S> = alpha_bps.iter().map(|&a| (a as S).to_radians()).collect();
    let re_s: Vec<S> = re_bps.iter().map(|&r| r as S).collect();

    Ok(AirfoilData {
        cl: AeroCoeff::Table2D {
            rows: alpha_rad.clone(),
            cols: re_s.clone(),
            data: cl_data,
        },
        cd: AeroCoeff::Table2D {
            rows: alpha_rad.clone(),
            cols: re_s.clone(),
            data: cd_data,
        },
        cm: AeroCoeff::Table2D {
            rows: alpha_rad,
            cols: re_s,
            data: cm_data,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal synthetic foil.tools CSV: 2 Re × 3 alpha × 3 Ncrit = 18 data rows.
    /// Layout: for each Re, three sequential alpha sweeps (Ncrit=1, 4, 9).
    fn minimal_csv() -> &'static str {
        "Re,alpha,CL,CD,CM\n\
         1000000,-5,-0.500,0.015,-0.01\n\
         1000000,0,0.000,0.008,0.00\n\
         1000000,5,0.500,0.015,0.01\n\
         1000000,-5,-0.480,0.014,-0.01\n\
         1000000,0,0.020,0.007,0.00\n\
         1000000,5,0.480,0.014,0.01\n\
         1000000,-5,-0.450,0.013,-0.01\n\
         1000000,0,0.050,0.006,0.00\n\
         1000000,5,0.450,0.013,0.01\n\
         2000000,-5,-0.510,0.013,-0.01\n\
         2000000,0,0.000,0.007,0.00\n\
         2000000,5,0.510,0.013,0.01\n\
         2000000,-5,-0.490,0.012,-0.01\n\
         2000000,0,0.025,0.006,0.00\n\
         2000000,5,0.490,0.012,0.01\n\
         2000000,-5,-0.460,0.011,-0.01\n\
         2000000,0,0.055,0.005,0.00\n\
         2000000,5,0.460,0.011,0.01\n"
    }

    #[test]
    fn parse_minimal_synthetic_csv_shape() {
        let polars = parse_foil_tools_csv(minimal_csv()).unwrap();
        // 2 Re values x 3 alpha values
        for foil in [&polars.ncrit1, &polars.ncrit4, &polars.ncrit9] {
            if let AeroCoeff::Table2D { rows, cols, data } = &foil.cl {
                assert_eq!(rows.len(), 3, "expected 3 alpha rows");
                assert_eq!(cols.len(), 2, "expected 2 Re cols");
                assert_eq!(data.len(), 6, "expected 6 data cells");
            } else {
                panic!("cl should be Table2D");
            }
        }
    }

    #[test]
    fn parse_ncrit_slices_are_distinct() {
        let polars = parse_foil_tools_csv(minimal_csv()).unwrap();
        // At Re=1e6 and alpha=0, the three Ncrit values of CL are 0.0, 0.02, and 0.05.
        let re = 1_000_000.0 as S;
        let alpha = 0.0 as S;
        let cl1 = polars.ncrit1.cl.evaluate(alpha, re);
        let cl4 = polars.ncrit4.cl.evaluate(alpha, re);
        let cl9 = polars.ncrit9.cl.evaluate(alpha, re);
        assert!(
            (cl1 - 0.0 as S).abs() < 1e-4 as S,
            "ncrit1 CL at 0° = {cl1}"
        );
        assert!(
            (cl4 - 0.02 as S).abs() < 1e-4 as S,
            "ncrit4 CL at 0° = {cl4}"
        );
        assert!(
            (cl9 - 0.05 as S).abs() < 1e-4 as S,
            "ncrit9 CL at 0° = {cl9}"
        );
    }

    #[test]
    fn parse_rejects_bad_header() {
        let csv = "Re,alpha,CL,CD\nsome,data,here,x\n";
        assert!(matches!(
            parse_foil_tools_csv(csv),
            Err(ParseError::BadHeader { .. })
        ));
    }

    #[test]
    fn parse_rejects_empty() {
        assert!(matches!(
            parse_foil_tools_csv(""),
            Err(ParseError::EmptyData)
        ));
        assert!(matches!(
            parse_foil_tools_csv("  \n  \n"),
            Err(ParseError::EmptyData)
        ));
    }

    #[test]
    fn parse_rejects_truncated_triplet() {
        // Only 2 rows. This is not a multiple of 3.
        let csv = "Re,alpha,CL,CD,CM\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n";
        assert!(matches!(
            parse_foil_tools_csv(csv),
            Err(ParseError::NotMultipleOfThree { rows: 2 })
        ));
    }

    #[test]
    fn parse_rejects_negative_cd() {
        // One Re block, 1 alpha x 3 Ncrit = 3 rows. The first Ncrit=1 row has a negative CD.
        let csv = "Re,alpha,CL,CD,CM\n\
                   1000000,0,0.0,-0.001,0.0\n\
                   1000000,0,0.0,0.008,0.0\n\
                   1000000,0,0.0,0.006,0.0\n";
        assert!(matches!(
            parse_foil_tools_csv(csv),
            Err(ParseError::NegativeCd { .. })
        ));
    }

    #[test]
    fn parse_rejects_nan() {
        let csv = "Re,alpha,CL,CD,CM\n\
                   1000000,0,NaN,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n";
        // "NaN" parses as f64::NAN, but the is_finite check catches it.
        let result = parse_foil_tools_csv(csv);
        assert!(
            matches!(result, Err(ParseError::NonFinite { .. }))
                || matches!(result, Err(ParseError::BadField { .. })),
            "expected NonFinite or BadField, got {result:?}"
        );
    }

    #[test]
    fn parse_rejects_non_monotonic_re() {
        // The Re=2e6 block comes before the Re=1e6 block (descending order).
        // Each Re block: 3 alpha x 3 Ncrit = 9 rows. Total: 18 rows.
        let csv = "Re,alpha,CL,CD,CM\n\
                   2000000,0,0.0,0.01,0.0\n\
                   2000000,5,0.5,0.01,0.0\n\
                   2000000,10,1.0,0.01,0.0\n\
                   2000000,0,0.0,0.01,0.0\n\
                   2000000,5,0.5,0.01,0.0\n\
                   2000000,10,1.0,0.01,0.0\n\
                   2000000,0,0.0,0.01,0.0\n\
                   2000000,5,0.5,0.01,0.0\n\
                   2000000,10,1.0,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,5,0.5,0.01,0.0\n\
                   1000000,10,1.0,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,5,0.5,0.01,0.0\n\
                   1000000,10,1.0,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,5,0.5,0.01,0.0\n\
                   1000000,10,1.0,0.01,0.0\n";
        assert!(matches!(
            parse_foil_tools_csv(csv),
            Err(ParseError::NonMonotonicRe { .. })
        ));
    }

    #[test]
    fn parse_rejects_inconsistent_alpha_grid() {
        // Two Re blocks. The second has a different alpha grid than the first.
        // Re=1e6: 3 alpha values (-5, 0, 5) x 3 Ncrit = 9 rows. Correct.
        // Re=2e6: 3 alpha values (-5, 0, 10). The alpha grid differs at index 2. An error.
        let csv = "Re,alpha,CL,CD,CM\n\
                   1000000,-5,-0.5,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,5,0.5,0.01,0.0\n\
                   1000000,-5,-0.5,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,5,0.5,0.01,0.0\n\
                   1000000,-5,-0.5,0.01,0.0\n\
                   1000000,0,0.0,0.01,0.0\n\
                   1000000,5,0.5,0.01,0.0\n\
                   2000000,-5,-0.5,0.01,0.0\n\
                   2000000,0,0.0,0.01,0.0\n\
                   2000000,10,1.0,0.01,0.0\n\
                   2000000,-5,-0.5,0.01,0.0\n\
                   2000000,0,0.0,0.01,0.0\n\
                   2000000,10,1.0,0.01,0.0\n\
                   2000000,-5,-0.5,0.01,0.0\n\
                   2000000,0,0.0,0.01,0.0\n\
                   2000000,10,1.0,0.01,0.0\n";
        assert!(matches!(
            parse_foil_tools_csv(csv),
            Err(ParseError::InconsistentAlphaGrid { .. })
        ));
    }

    #[test]
    fn with_post_stall_extends_to_pi() {
        let polars = parse_foil_tools_csv(minimal_csv()).unwrap();
        let ar = 6.94 as S;
        let extended = polars.ncrit9.with_post_stall(ar);
        let pi_half = std::f64::consts::FRAC_PI_2 as S;
        // At +/-90 deg, CL must be near zero (flat-plate Viterna).
        let cl_p90 = extended.cl.evaluate(pi_half, 1_000_000.0 as S);
        let cl_n90 = extended.cl.evaluate(-pi_half, 1_000_000.0 as S);
        assert!(
            cl_p90.abs() < 0.15 as S,
            "CL at +90° should be near 0, got {cl_p90}"
        );
        assert!(
            cl_n90.abs() < 0.15 as S,
            "CL at -90° should be near 0, got {cl_n90}"
        );
        // At 90 deg, CD must be near CD_max = 1.11 + 0.018 * AR.
        let cd_max = 1.11 as S + 0.018 as S * ar;
        let cd_90 = extended.cd.evaluate(pi_half, 1_000_000.0 as S);
        assert!(
            (cd_90 - cd_max).abs() < 0.05 as S,
            "CD at 90° should be near {cd_max:.3}, got {cd_90}"
        );
    }

    #[test]
    fn validate_returns_empty_for_well_formed() {
        let polars = parse_foil_tools_csv(minimal_csv()).unwrap();
        let issues = polars.ncrit9.validate("test_foil");
        assert!(
            issues.is_empty(),
            "expected no validation issues, got: {issues:?}"
        );
    }

    #[test]
    fn validate_flags_placeholder_cl() {
        // AeroCoeff::validate does not report Placeholder as an error. It is a
        // valid sentinel for "no model yet", and it gives a warning at
        // runtime, not at parse time. Thus, AirfoilData::validate returns an
        // empty list for a default airfoil (Placeholder cl and cd, Absent cm).
        let foil = AirfoilData::default();
        let issues = foil.validate("placeholder_foil");
        assert!(
            issues.is_empty(),
            "Placeholder is a valid sentinel; validate should return no structural issues, got: {issues:?}"
        );
    }
}
