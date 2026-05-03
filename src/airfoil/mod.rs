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
//! ### With real polar data from [foil.tools](https://foil.tools)
//!
//! The recommended workflow for real aircraft is to export a polar CSV from
//! [foil.tools](https://foil.tools), embed it at compile time with `include_str!`, parse it with
//! [`foil_tools::parse_foil_tools_csv`], then optionally extend into the post-stall regime with
//! [`AirfoilData::with_post_stall`] before registering.
//!
//! ```rust,no_run
//! # use avian_fdm::prelude::*;
//! # use avian_fdm::airfoil::{AirfoilData, RegisterAirfoil, foil_tools::parse_foil_tools_csv};
//! # use bevy::prelude::*;
//! const MY_FOIL_CSV: &str = include_str!("../assets/my_airfoil/polars.csv");
//!
//! App::new()
//!     .add_plugins(AircraftFdmPlugin::default())
//!     .register_airfoil(
//!         "MyFoil",
//!         parse_foil_tools_csv(MY_FOIL_CSV)
//!             .expect("embedded polar CSV must parse cleanly")
//!             .ncrit9                         // pick the desired Ncrit slice
//!             .with_post_stall(7.0)           // aspect ratio of the lifting surface
//!     );
//! ```
//!
//! ### With hand-coded coefficients (prototyping)
//!
//! For quick iteration or simple flat-plate approximations, coefficients can be
//! specified directly. Any field left as [`crate::components::aero_coeff::AeroCoeff::Absent`] is
//! silently zero; [`crate::components::aero_coeff::AeroCoeff::Placeholder`] emits a runtime
//! warning so you notice it is still missing.
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
