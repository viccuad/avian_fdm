//! [`AirfoilLibrary`] resource.
//!
//! An airfoil section is used in [`crate::components::AeroZone`]. It provides three dimensionless
//! coefficients as functions of angle of attack: lift (CL), drag (CD), and pitching moment (CM).
//! The other three aircraft-level coefficients (CY, Croll, Cn) are geometry-emergent and are not
//! meaningful at the 2-D section level.
//!
//! ## Usage
//!
//! Register named airfoils at startup via [`RegisterAirfoil`], then set
//! [`crate::components::AeroZone::airfoil_name`] to the registered name. A system running before
//! the FDM resolves each zone's `Placeholder` cl/cd/cm fields from the library.
//!
//! ```rust,no_run
//! # use avian_fdm::prelude::*;
//! # use avian_fdm::airfoil::{AirfoilData, RegisterAirfoil};
//! # use avian_fdm::components::aero_coeff::AeroCoeff;
//! # use bevy::prelude::*;
//! App::new()
//!     .add_plugins(AircraftFdmPlugin::default())
//!     .register_airfoil("MyFoil", AirfoilData {
//!         cl: AeroCoeff::Scalar(5.0),
//!         cd: AeroCoeff::Scalar(0.02),
//!         cm: AeroCoeff::Absent,
//!     });
//! ```

use crate::_bevy::*;
use crate::components::aero_coeff::AeroCoeff;
use avian3d::math::Scalar;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub mod foil_tools;

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

/// Registry of named airfoil definitions.
///
/// Populated at startup via [`RegisterAirfoil::register_airfoil`].
/// The library starts empty; aircraft crates are responsible for parsing
/// and registering their own airfoil data (e.g. via
/// [`foil_tools::parse_foil_tools_csv`]).
///
/// Looked up by the airfoil resolution system to populate
/// [`crate::components::AeroZone`] fields at spawn time.
#[derive(Resource, Default)]
pub struct AirfoilLibrary {
    map: HashMap<String, AirfoilData>,
}

impl AirfoilLibrary {
    /// Register an airfoil under `name`. Overwrites any existing entry.
    pub fn register(&mut self, name: impl Into<String>, data: AirfoilData) {
        self.map.insert(name.into(), data);
    }

    /// Look up an airfoil by name. Returns `None` if not registered.
    pub fn lookup(&self, name: &str) -> Option<&AirfoilData> {
        self.map.get(name)
    }

    /// Number of registered airfoils.
    pub fn len(&self) -> usize {
        self.map.len()
    }

    /// Returns `true` if no airfoils are registered.
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }
}

/// Extension trait for [`App`] — register a named airfoil into [`AirfoilLibrary`].
///
/// # Example
///
/// ```rust,no_run
/// # use avian_fdm::prelude::*;
/// # use avian_fdm::airfoil::{AirfoilData, RegisterAirfoil};
/// # use avian_fdm::components::aero_coeff::AeroCoeff;
/// # use bevy::prelude::*;
/// App::new()
///     .add_plugins(AircraftFdmPlugin::default())
///     .register_airfoil("MyFoil", AirfoilData {
///         cl: AeroCoeff::Scalar(5.0),
///         cd: AeroCoeff::Scalar(0.02),
///         cm: AeroCoeff::Absent,
///     });
/// ```
pub trait RegisterAirfoil {
    /// Register `data` under `name` in the [`AirfoilLibrary`] resource.
    ///
    /// Call after [`AircraftFdmPlugin`](crate::plugin::AircraftFdmPlugin) has been added
    /// (the plugin initialises the resource). Overwrites any existing entry.
    fn register_airfoil(&mut self, name: impl Into<String>, data: AirfoilData) -> &mut Self;
}

impl RegisterAirfoil for App {
    fn register_airfoil(&mut self, name: impl Into<String>, data: AirfoilData) -> &mut Self {
        self.world_mut()
            .resource_mut::<AirfoilLibrary>()
            .register(name, data);
        self
    }
}

/// System that resolves `AeroZone::airfoil_name` to coefficient tables.
///
/// Runs in `PreUpdate` on every newly added `AeroZone`. For each zone with
/// a non-empty `airfoil_name`, looks up the name in [`AirfoilLibrary`] and
/// overwrites any [`AeroCoeff::Placeholder`] `cl`/`cd`/`cm` field with the
/// corresponding data from the library. Explicit (non-`Placeholder`) fields
/// are never overwritten.
///
/// Emits a `warn_once!` if the name is not found in the library.
pub fn resolve_airfoil_names(
    library: Res<AirfoilLibrary>,
    mut zones: Query<&mut crate::components::AeroZone, Added<crate::components::AeroZone>>,
) {
    for mut zone in &mut zones {
        if zone.airfoil_name.is_empty() {
            continue;
        }
        let name = zone.airfoil_name.clone();
        let Some(data) = library.lookup(&name) else {
            warn_once!(
                "AeroZone references unknown airfoil \"{name}\". \
                 Register it via App::register_airfoil() before spawning zones."
            );
            continue;
        };
        if matches!(zone.cl, AeroCoeff::Placeholder) {
            zone.cl = data.cl.clone();
        }
        if matches!(zone.cd, AeroCoeff::Placeholder) {
            zone.cd = data.cd.clone();
        }
        if matches!(zone.cm, AeroCoeff::Placeholder) {
            zone.cm = data.cm.clone();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::aero_coeff::AeroCoeff;

    #[test]
    fn register_and_lookup_round_trip() {
        let mut lib = AirfoilLibrary::default();
        let data = AirfoilData {
            cl: AeroCoeff::Scalar(1.0),
            cd: AeroCoeff::Scalar(0.02),
            cm: AeroCoeff::Absent,
        };
        lib.register("TestFoil", data.clone());
        let found = lib.lookup("TestFoil").expect("should be registered");
        assert_eq!(found.cl, data.cl);
        assert_eq!(found.cd, data.cd);
    }
}
