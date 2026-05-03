//! Parser for [foil.tools](https://foil.tools) XFoil polar CSV exports.
//!
//! foil.tools exports a CSV with header `Re,alpha,CL,CD,CM`. The layout is Re-major: for each Re
//! value, the full alpha sweep for Ncrit=1 appears first, then the full alpha sweep for Ncrit=4,
//! then the full alpha sweep for Ncrit=9. Re blocks are sorted ascending; alpha is sorted
//! ascending within each Ncrit sweep.
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
//! // Use Ncrit=9 (clean atmospheric flight) slice:
//! let _foil = polars.ncrit9;
//! ```

use crate::airfoil::AirfoilData;
use crate::components::aero_coeff::AeroCoeff;
use avian3d::math::Scalar as S;
use thiserror::Error;

/// All three Ncrit slices parsed from one foil.tools polar CSV.
///
/// Each slice is a complete [`AirfoilData`] with `Table2D` CL, CD, and CM
/// tables (alpha-major, Re columns). Post-stall extension is **not** applied;
/// call [`AirfoilData::with_post_stall`] after selecting the slice you need.
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
    /// CSV has no non-empty lines.
    #[error("CSV is empty or has no data rows")]
    EmptyData,
    /// First non-empty line is not the expected header.
    #[error("expected header \"Re,alpha,CL,CD,CM\", got \"{got}\"")]
    BadHeader {
        /// The actual first line found.
        got: String,
    },
    /// A data row has the wrong number of comma-separated fields.
    #[error("line {line}: expected {expected} fields, got {got}")]
    BadColumnCount {
        /// 1-indexed line number in the source CSV.
        line: usize,
        /// Expected field count (5).
        expected: usize,
        /// Actual field count found.
        got: usize,
    },
    /// A field could not be parsed as a number.
    #[error("line {line}: cannot parse field {column} from \"{value}\"")]
    BadField {
        /// 1-indexed line number.
        line: usize,
        /// Column name (`"Re"`, `"alpha"`, `"CL"`, `"CD"`, or `"CM"`).
        column: &'static str,
        /// The raw string value that failed to parse.
        value: String,
    },
    /// Total data row count is not a multiple of 3 (the CSV must contain
    /// exactly three equal-sized blocks: one per Ncrit value).
    #[error("data row count {rows} is not a multiple of 3 (CSV must have three equal sequential blocks: Ncrit=1, Ncrit=4, Ncrit=9)")]
    NotMultipleOfThree {
        /// Actual row count.
        rows: usize,
    },
    /// Re values are not strictly increasing across Re blocks.
    #[error("Re values are not strictly increasing at index {at_index}")]
    NonMonotonicRe {
        /// 0-indexed position in the Re breakpoint list where the violation occurs.
        at_index: usize,
    },
    /// Alpha values within a Re block are not strictly increasing.
    #[error("alpha values in Re={re} block are not strictly increasing at index {at_index}")]
    NonMonotonicAlpha {
        /// The Re value of the block where the violation was found.
        re: f64,
        /// 0-indexed position within the block.
        at_index: usize,
    },
    /// The alpha grid is not identical across all Re blocks.
    #[error("alpha grid for Re={re} differs from the first Re block")]
    InconsistentAlphaGrid {
        /// The Re value of the block whose alpha grid differs from the first block.
        re: f64,
    },
    /// A value is NaN or infinite.
    #[error("line {line}: field {column} is NaN or infinite")]
    NonFinite {
        /// 1-indexed line number.
        line: usize,
        /// Column name.
        column: &'static str,
    },
    /// A CD value is negative, which is physically impossible.
    #[error("Re={re}, alpha={alpha_deg} deg: CD={cd} is negative (physically impossible)")]
    NegativeCd {
        /// Re value of the offending row.
        re: f64,
        /// Alpha (degrees) of the offending row.
        alpha_deg: f64,
        /// The negative CD value.
        cd: f64,
    },
}

/// Parse a foil.tools XFoil polar CSV into all three Ncrit slices.
///
/// foil.tools exports polars for all Ncrit values in a single CSV. The layout
/// is Re-major: for each Re value, there are three consecutive alpha sweeps of
/// equal length (one per Ncrit value, in order: Ncrit=1, Ncrit=4, Ncrit=9).
/// Each sweep covers the same alpha range and is sorted alpha ascending. Re
/// values are sorted ascending across the blocks.
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
/// The CSV has no Ncrit column. The parser detects Ncrit boundaries purely by
/// position: each Re block is divided into three equal thirds (rows
/// `0..n_alpha` → Ncrit=1, `n_alpha..2*n_alpha` → Ncrit=4,
/// `2*n_alpha..3*n_alpha` → Ncrit=9). If the block length is not divisible by
/// 3, [`ParseError::NotMultipleOfThree`] is returned. No other validation of
/// the Ncrit ordering is performed — a CSV with a different internal layout
/// would be parsed silently into wrong data.
///
/// # Errors
///
/// Returns [`ParseError`] if the CSV is malformed or fails physical-sanity
/// checks (negative CD, non-finite values).
///
/// # Post-stall
///
/// Post-stall extension is **not** applied. Call [`AirfoilData::with_post_stall`]
/// on the returned slice after selecting it.
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

    // Validate Re monotonicity.
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

    // 3. Extract per-Ncrit rows from each Re block
    // Ncrit=1: first n_alpha rows of each block.
    // Ncrit=4: next n_alpha rows.
    // Ncrit=9: last n_alpha rows.
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
            // Normalise: strip UTF-8 BOM if present, lower-case, remove spaces.
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

/// Build one `AirfoilData` from a flat slice of rows that belong to a single
/// Ncrit level. Rows must be in Re-major, alpha-ascending order (as produced
/// by foil.tools).
fn build_airfoil_data(rows: &[&RawRow]) -> Result<AirfoilData, ParseError> {
    // extract sorted unique Re values (Re-major order in input)
    let mut re_bps: Vec<f64> = Vec::new();
    for r in rows {
        if re_bps.last() != Some(&r.re) {
            re_bps.push(r.re);
        }
    }
    let n_re = re_bps.len();

    // Validate Re monotonicity.
    for i in 1..n_re {
        if re_bps[i] <= re_bps[i - 1] {
            return Err(ParseError::NonMonotonicRe { at_index: i });
        }
    }

    let n_alpha = rows.len() / n_re;
    if n_alpha == 0 {
        return Err(ParseError::EmptyData);
    }

    // extract alpha breakpoints from the first Re block
    let first_block = &rows[..n_alpha];
    let alpha_bps: Vec<f64> = first_block.iter().map(|r| r.alpha_deg).collect();

    // Validate alpha monotonicity in the first block.
    for i in 1..alpha_bps.len() {
        if alpha_bps[i] <= alpha_bps[i - 1] {
            return Err(ParseError::NonMonotonicAlpha {
                re: re_bps[0],
                at_index: i,
            });
        }
    }

    // Validate that every subsequent Re block has the same alpha grid.
    for re_idx in 1..n_re {
        let block = &rows[re_idx * n_alpha..(re_idx + 1) * n_alpha];
        for (ai, row) in block.iter().enumerate() {
            if (row.alpha_deg - alpha_bps[ai]).abs() > 1e-6 {
                return Err(ParseError::InconsistentAlphaGrid { re: re_bps[re_idx] });
            }
        }
    }

    // physical sanity: CD >= 0 everywhere
    for r in rows {
        if r.cd < 0.0 {
            return Err(ParseError::NegativeCd {
                re: r.re,
                alpha_deg: r.alpha_deg,
                cd: r.cd,
            });
        }
    }

    // transpose Re-major -> alpha-major (Table2D row = alpha)
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

    // Convert breakpoints to Scalar (alpha to radians).
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
        // 2 Re × 3 alpha table
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
        // At Re=1e6, alpha=0, the three Ncrit CL values are 0.0, 0.02, 0.05 respectively.
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
        // Only 2 rows: not divisible by 3.
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
        // One Re block, 1 alpha × 3 Ncrit = 3 rows. First Ncrit=1 row has negative CD.
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
        // "NaN" parses as f64::NAN but is caught by the is_finite check.
        let result = parse_foil_tools_csv(csv);
        assert!(
            matches!(result, Err(ParseError::NonFinite { .. }))
                || matches!(result, Err(ParseError::BadField { .. })),
            "expected NonFinite or BadField, got {result:?}"
        );
    }

    #[test]
    fn parse_rejects_non_monotonic_re() {
        // Re=2e6 block comes before Re=1e6 block (descending).
        // Each Re block: 3 alpha × 3 Ncrit = 9 rows. Total: 18 rows.
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
        // Two Re blocks, but the second has a different alpha grid than the first.
        // Re=1e6: 3 alpha (-5, 0, 5) × 3 Ncrit = 9 rows. ✓
        // Re=2e6: 3 alpha (-5, 0, 10) — alpha grid differs at index 2. → error.
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
        // CL at ±90° should be near zero (flat-plate Viterna).
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
        // CD at 90° should be near CD_max = 1.11 + 0.018 * AR.
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
        // AeroCoeff::validate does not flag Placeholder as an error — it is a
        // valid "not yet modelled" sentinel that warns at runtime, not at
        // parse time. AirfoilData::validate therefore returns empty for a
        // default (Placeholder cl/cd, Absent cm) airfoil.
        let foil = AirfoilData::default();
        let issues = foil.validate("placeholder_foil");
        assert!(
            issues.is_empty(),
            "Placeholder is a valid sentinel; validate should return no structural issues, got: {issues:?}"
        );
    }
}
