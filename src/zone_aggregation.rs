//! Zone aggregation system.
//!
//! Evaluates each [`crate::components::AeroZone`]'s coefficient tables at the
//! current angle of attack and Reynolds number, multiplies by zone health, and
//! sums into [`crate::components::AircraftAggregate`]. Also recomputes
//! [`crate::components::AircraftMass`] (total mass, CG, inertia tensor) from
//! zone masses each frame.
//!
//! Only compiled with `features = ["damage"]`.

// TODO(zone-aggregation): implement init_zone_volumes + aggregate_zones systems
