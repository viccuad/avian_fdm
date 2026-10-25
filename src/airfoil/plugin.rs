//! [`RegisterAirfoil`] extension trait and the `resolve_airfoil_names` system.

use crate::_bevy::*;
use crate::components::aero_coeff::AeroCoeff;

use super::library::AirfoilLibrary;
use super::types::AirfoilData;

/// Extension trait for [`App`]: registers a named airfoil in [`AirfoilLibrary`].
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
    /// Registers `data` under `name` in the [`AirfoilLibrary`] resource.
    ///
    /// Call it after you add [`AircraftFdmPlugin`](crate::plugin::AircraftFdmPlugin).
    /// The plugin creates the resource. This method replaces an entry with
    /// the same name.
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

/// System that changes `AeroZone::airfoil_name` into coefficient tables.
///
/// It runs in `PreUpdate` on each new `AeroZone`. For each zone with a
/// non-empty `airfoil_name`, it finds the name in [`AirfoilLibrary`]. Then it
/// replaces each [`AeroCoeff::Placeholder`] field in `cl`, `cd`, and `cm` with
/// the matching library data. It never replaces an explicit
/// (non-`Placeholder`) field.
///
/// If the library does not contain the name, it emits a `warn_once!`.
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
