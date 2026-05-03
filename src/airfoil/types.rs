//! [`AirfoilData`]: aerodynamic coefficient data for a 2-D airfoil section.

use crate::_bevy::*;
use crate::components::aero_coeff::AeroCoeff;
use avian3d::math::Scalar;
use serde::{Deserialize, Serialize};

/// Aerodynamic coefficient data for a 2-D airfoil section.
///
/// Contains the three coefficients that are meaningful at the section level:
/// lift (CL), drag (CD), and pitching moment about the quarter-chord (CM).
///
/// Use [`AeroCoeff::Absent`] for `cm` when pitching moment is handled by tail
/// geometry rather than airfoil data (the common case for symmetric sections).
#[derive(Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Serialize, Deserialize)]
pub struct AirfoilData {
    /// Lift coefficient vs angle of attack.
    pub cl: AeroCoeff,
    /// Drag coefficient vs angle of attack.
    pub cd: AeroCoeff,
    /// Pitching-moment coefficient vs angle of attack (about quarter-chord).
    ///
    /// Defaults to [`AeroCoeff::Absent`]. Set to a `Table1D` or `Scalar` for
    /// cambered sections (e.g. NACA 2412 has cm0 ≈ −0.05 at zero lift).
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
    /// Apply Viterna-Corrigan post-stall extension to CL and CD.
    ///
    /// Uses `aspect_ratio` (AR = span / chord) for the Viterna model.
    /// CM is left unchanged — there is no standard Viterna analogue for
    /// pitching-moment coefficients.
    ///
    /// Call this once per airfoil after importing from CSV, before passing
    /// the data to zone constructors. See [`AeroCoeff::with_post_stall_lift`]
    /// and [`AeroCoeff::with_post_stall_drag`] for the underlying algorithms.
    pub fn with_post_stall(self, aspect_ratio: Scalar) -> Self {
        Self {
            cl: self.cl.with_post_stall_lift(aspect_ratio),
            cd: self.cd.with_post_stall_drag(aspect_ratio),
            cm: self.cm,
        }
    }

    /// Validate the structure and contents of all three coefficient tables.
    ///
    /// Delegates to [`AeroCoeff::validate`] for each of `cl`, `cd`, and `cm`.
    /// Returns a list of human-readable problem descriptions; an empty list
    /// means the airfoil data is structurally sound.
    ///
    /// `label` is a descriptive name used as a prefix in each message (e.g.
    /// the airfoil name or the zone it was registered under).
    pub fn validate(&self, label: &str) -> Vec<String> {
        let mut problems = Vec::new();
        problems.extend(self.cl.validate(&format!("{label}.cl")));
        problems.extend(self.cd.validate(&format!("{label}.cd")));
        problems.extend(self.cm.validate(&format!("{label}.cm")));
        problems
    }
}
