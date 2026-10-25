//! Piston engine thrust model.
//!
//! # Overview
//!
//! The model uses the Gagg-Ferrar altitude correction. **Thrust = maximum
//! thrust × health fraction × throttle fraction × (air density ratio)^0.7.**
//! The 0.7 exponent is empirical, for naturally aspirated piston engines. At
//! altitude, the air is thinner, and the thrust decreases.
//!
//! ```text
//! T = T_max * remaining * throttle_fraction * (rho / rho_0)^0.7
//! ```
//!
//! An optional factor models the loss of efficiency of a fixed-pitch
//! propeller as the airspeed increases. It scales the thrust by
//! max(0, 1 - (V / V_zero)^2). The thrust is zero at the windmilling speed.

pub mod engine_zone;
pub(crate) mod throttle;
pub(crate) mod thrust;

pub use engine_zone::EngineZone;
pub use thrust::compute_engine_zone_forces;
