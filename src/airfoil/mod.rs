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

pub mod foil_tools;
mod library;
mod plugin;
mod types;

pub use library::AirfoilLibrary;
pub use plugin::{resolve_airfoil_names, RegisterAirfoil};
pub use types::AirfoilData;
