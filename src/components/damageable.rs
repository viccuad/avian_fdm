//! Universal health component. Written by the game's hit/damage system;
//! read by domain components (`AeroZone`, `EngineZone`, etc.) independently.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// Universal health component. Attach to any entity that can be damaged.
///
/// This component is **cross-cutting**: it is written by one system (your
/// projectile / collision damage handler) and read independently by multiple
/// domain systems:
///
/// - `compute_aerodynamics` — scales coefficients and adds structural drag.
/// - `compute_propulsion` — scales engine thrust.
/// - `DetachPlugin` (optional) — detaches the entity from the Bevy hierarchy
///   when `health` reaches `0.0`.
///
/// # Semantics
/// - `1.0` — fully intact.
/// - `0.0` — completely destroyed / detached from the airframe.
///   Domain systems must treat `0.0` as **absent**: zero aerodynamic
///   contribution, zero thrust — not maximum drag.
/// - Values between `0.0` and `1.0` — partially damaged.
///
/// # Example
/// ```rust
/// use avian_fdm::components::Damageable;
///
/// // Spawn a zone at full health.
/// let d = Damageable::default();
/// assert_eq!(d.health, 1.0);
///
/// // A zone at zero health contributes nothing to the airframe.
/// let destroyed = Damageable { health: 0.0 };
/// assert_eq!(destroyed.health, 0.0);
/// ```
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct Damageable {
    /// Zone health: `1.0` = intact, `0.0` = destroyed.
    /// Write this from your projectile / collision system.
    pub health: f64,
}

impl Default for Damageable {
    fn default() -> Self {
        Self { health: 1.0 }
    }
}
