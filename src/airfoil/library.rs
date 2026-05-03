//! [`AirfoilLibrary`]: registry of named airfoil definitions.

use std::collections::HashMap;

use crate::_bevy::Resource;

use super::types::AirfoilData;

/// Registry of named airfoil definitions.
///
/// Populated at startup via [`super::plugin::RegisterAirfoil::register_airfoil`].
/// The library starts empty; aircraft crates are responsible for parsing
/// and registering their own airfoil data (e.g. via
/// [`super::foil_tools::parse_foil_tools_csv`]).
///
/// Looked up by the airfoil resolution system to populate
/// [`crate::components::AeroZone`] fields at spawn time.
#[derive(Resource, Default)]
pub struct AirfoilLibrary {
    pub(super) map: HashMap<String, AirfoilData>,
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

#[cfg(test)]
mod tests {
    use crate::components::aero_coeff::AeroCoeff;

    use super::*;

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
