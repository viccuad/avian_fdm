//! Piston engine thrust model.
//!
//! # Overview
//!
//! Gagg-Ferrar altitude correction: thrust = max thrust x health fraction x
//! throttle position x air density ratio raised to the 0.7 power. The 0.7
//! exponent is empirical for naturally-aspirated piston engines: thrust falls
//! with altitude as air thins.
//!
//! ```text
//! T = T_max * remaining * throttle_fraction * (rho / rho_0)^0.7
//! ```
//!
//! An optional speed-dependent factor models fixed-pitch propeller efficiency
//! drop: thrust scales by max(0, 1 - (V / V_zero)^2), reaching zero at the
//! windmilling speed.

pub mod engine_zone;
pub(crate) mod throttle;
pub(crate) mod thrust;

pub use engine_zone::EngineZone;
pub use thrust::compute_engine_zone_forces;
