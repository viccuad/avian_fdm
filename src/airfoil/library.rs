//! [`AirfoilLibrary`]: registry of named airfoil definitions.

use std::collections::HashMap;

use crate::_bevy::Resource;

use super::types::AirfoilData;

/// Registry of named airfoil definitions.
///
/// At startup, [`super::plugin::RegisterAirfoil::register_airfoil`] fills it.
/// The library starts empty. Each aircraft crate must parse and register its
/// own airfoil data (for example, with
/// [`super::foil_tools::parse_foil_tools_csv`]).
///
/// At spawn time, the airfoil resolution system reads the library to fill the
/// fields of each [`crate::components::AeroZone`].
#[derive(Resource, Default)]
pub struct AirfoilLibrary {
    pub(super) map: HashMap<String, AirfoilData>,
}

impl AirfoilLibrary {
    /// Registers an airfoil under `name`. Replaces an entry with the same name.
    pub fn register(&mut self, name: impl Into<String>, data: AirfoilData) {
        self.map.insert(name.into(), data);
    }

    /// Finds an airfoil by name. Returns `None` if the name is not registered.
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
