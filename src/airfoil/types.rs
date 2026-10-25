//! [`AirfoilData`]: aerodynamic coefficient data for a 2-D airfoil section.

use crate::_bevy::*;
use crate::components::aero_coeff::AeroCoeff;
use serde::{Deserialize, Serialize};

/// Aerodynamic coefficient data for a 2-D airfoil section.
///
/// It contains the three coefficients that have a meaning for a section:
/// lift (CL), drag (CD), and pitching moment about the quarter-chord (CM).
///
/// If the tail geometry gives the pitching moment, and not the airfoil data,
/// use [`AeroCoeff::Absent`] for `cm`. This is usual for symmetric sections.
#[derive(Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Serialize, Deserialize)]
pub struct AirfoilData {
    /// Lift coefficient as a function of the angle of attack.
    pub cl: AeroCoeff,
    /// Drag coefficient as a function of the angle of attack.
    pub cd: AeroCoeff,
    /// Pitching-moment coefficient (about the quarter-chord) as a function of
    /// the angle of attack.
    ///
    /// Default: [`AeroCoeff::Absent`]. For cambered sections, set a `Table1D`
    /// or a `Scalar` (for example, NACA 2412 has cm0 ≈ −0.05 at zero lift).
    pub cm: AeroCoeff,
}

impl Default for AirfoilData {
    fn default() -> Self {
        Self {
            cl: AeroCoeff::Placeholder,
            cd: AeroCoeff::Placeholder,
            cm: AeroCoeff::Absent,
        }
    }
}

impl AirfoilData {
    /// Applies the Viterna-Corrigan post-stall extension to CL and CD.
    ///
    /// The Viterna model uses `aspect_ratio` (AR = span / chord, for a
    /// rectangular surface). CM does not change, because no standard Viterna
    /// model exists for pitching-moment coefficients.
    ///
    /// Call this method once per airfoil, after the CSV import and before you
    /// give the data to the zone constructors. Refer to
    /// [`AeroCoeff::with_post_stall_lift`] and
    /// [`AeroCoeff::with_post_stall_drag`] for the algorithms.
    pub fn with_post_stall(self, aspect_ratio: f32) -> Self {
        Self {
            cl: self.cl.with_post_stall_lift(aspect_ratio),
            cd: self.cd.with_post_stall_drag(aspect_ratio),
            cm: self.cm,
        }
    }

    /// Validates the structure and the contents of the three coefficient tables.
    ///
    /// It calls [`AeroCoeff::validate`] for `cl`, `cd`, and `cm`. It returns a
    /// list of problem descriptions that a person can read. An empty list
    /// means that the structure of the airfoil data is correct.
    ///
    /// `label` is a descriptive name. Each message starts with it (for
    /// example, the airfoil name, or the zone that uses the airfoil).
    pub fn validate(&self, label: &str) -> Vec<String> {
        let mut problems = Vec::new();
        problems.extend(self.cl.validate(&format!("{label}.cl")));
        problems.extend(self.cd.validate(&format!("{label}.cd")));
        problems.extend(self.cm.validate(&format!("{label}.cm")));
        problems
    }
}
