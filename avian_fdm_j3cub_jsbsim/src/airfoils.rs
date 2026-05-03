//! Airfoil data for the J3 Cub preset.
//!
//! The USA-35B polar CSV was obtained from <https://foil.tools/foil/usa35b>.
//! It is embedded at compile time and parsed at startup; no runtime I/O occurs.

use avian_fdm::airfoil::foil_tools::parse_foil_tools_csv;
use avian_fdm::airfoil::AirfoilData;

/// USA-35B airfoil at Ncrit = 9 (clean atmospheric flight).
///
/// Parses the embedded `assets/usa35b/usa35b_polars.csv` (foil.tools XFoil
/// export) and returns the Ncrit=9 slice as raw polar tables.
///
/// Post-stall extension is **not** applied here; [`wing_zone`] calls
/// [`AeroZone::with_post_stall_extension`] at zone-construction time using the
/// per-panel aspect ratio.
///
/// # Panics
///
/// Panics if the embedded CSV fails to parse. This is intentional: the CSV is
/// fixed at compile time and any corruption is a build-time defect, not a
/// recoverable runtime error.
///
/// [`wing_zone`]: crate::presets::j3cub::wing_zone
/// [`AeroZone::with_post_stall_extension`]: avian_fdm::components::AeroZone::with_post_stall_extension
pub fn usa35b() -> AirfoilData {
    const CSV: &str = include_str!("../assets/usa35b/usa35b_polars.csv");
    parse_foil_tools_csv(CSV)
        .expect("embedded USA-35B CSV must parse cleanly")
        .ncrit9
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use avian_fdm::components::aero_coeff::AeroCoeff;
    use std::f32::consts::PI;

    #[test]
    fn usa35b_parses_ok() {
        let foil = usa35b();
        // Both CL and CD must be Table2D after parsing the full CSV.
        assert!(
            matches!(foil.cl, AeroCoeff::Table2D { .. }),
            "USA-35B CL should be Table2D"
        );
        assert!(
            matches!(foil.cd, AeroCoeff::Table2D { .. }),
            "USA-35B CD should be Table2D"
        );
    }

    #[test]
    fn usa35b_validate_clean() {
        let foil = usa35b();
        let issues = foil.validate("USA-35B");
        assert!(
            issues.is_empty(),
            "USA-35B should have no validation issues: {issues:?}"
        );
    }

    /// Smoke-test that post-stall extension works on the real data.
    #[test]
    fn usa35b_with_post_stall_extends_to_pi() {
        // Use a per-panel-ish AR; exact value doesn't matter for this smoke test.
        let ar = 1.1_f32;
        let foil = usa35b().with_post_stall(ar as avian3d::math::Scalar);
        let cl_90 = foil.cl.evaluate(PI / 2.0, 1_000_000.0);
        assert!(
            cl_90.abs() < 0.2,
            "CL at 90° after post-stall extension should be near 0, got {cl_90}"
        );
    }
}
