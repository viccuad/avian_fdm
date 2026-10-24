//! [`AirfoilLibrary`] resource and airfoil registration.
//!
//! An [`crate::components::AeroZone`] can use an airfoil section. The section gives three
//! dimensionless coefficients as functions of the angle of attack: lift (CL), drag (CD), and
//! pitching moment (CM). The other three aircraft coefficients (CY, Croll, Cn) come from the
//! geometry. They have no meaning for a 2-D section.
//!
//! ## Usage
//!
//! At startup, register named airfoils with [`RegisterAirfoil`]. Then set
//! [`crate::components::AeroZone::airfoil_name`] to the registered name. A system that runs before
//! the FDM replaces the `Placeholder` fields `cl`, `cd`, and `cm` of each zone with the library
//! data.
//!
//! ### With real polar data from [foil.tools](https://foil.tools)
//!
//! This is the recommended workflow for real aircraft:
//!
//! 1. Export a polar CSV from [foil.tools](https://foil.tools).
//! 2. Embed the CSV at compile time with `include_str!`.
//! 3. Parse it with [`foil_tools::parse_foil_tools_csv`].
//! 4. Optionally, extend it into the post-stall range with [`AirfoilData::with_post_stall`].
//! 5. Register the result.
//!
//! ```rust,no_run
//! # use avian_fdm::prelude::*;
//! # use avian_fdm::airfoil::{AirfoilData, RegisterAirfoil, foil_tools::parse_foil_tools_csv};
//! # use bevy::prelude::*;
//! // In real code: const MY_FOIL_CSV: &str = include_str!("../assets/my_airfoil/polars.csv");
//! # let my_foil_csv = "";
//!
//! App::new()
//!     .add_plugins(AircraftFdmPlugin::default())
//!     .register_airfoil(
//!         "MyFoil",
//!         parse_foil_tools_csv(my_foil_csv)
//!             .expect("embedded polar CSV must parse cleanly")
//!             .ncrit9                         // select the Ncrit slice
//!             .with_post_stall(7.0)           // aspect ratio of the lifting surface
//!     );
//! ```
//!
//! ### With coefficients that you write in the code (prototypes)
//!
//! For fast iteration or for simple flat-plate approximations, you can write the coefficients
//! directly. A field that is [`crate::components::aero_coeff::AeroCoeff::Absent`] gives zero with
//! no warning. A field that is [`crate::components::aero_coeff::AeroCoeff::Placeholder`] emits a
//! warning at runtime, so that you see that it has no data yet.
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
