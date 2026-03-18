//! Piston engine thrust model.
//!
//! Computes thrust via the Gagg–Ferrar altitude correction, stores propeller
//! induced velocity in [`crate::components::PropwashState`], and accumulates
//! thrust into Avian's `ExternalForce`.
//!
//! Only compiled with `features = ["propulsion"]`.

// TODO(propulsion): implement compute_propulsion system
