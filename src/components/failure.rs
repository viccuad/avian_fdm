//! [`Failure`] component, fraction of a zone's capability that remains after damage.
//! Written by the game's hit/damage system; read by domain systems independently.

use crate::_bevy::*;
use avian3d::math::Scalar;
use serde::{Deserialize, Serialize};

/// Fraction of a zone's nominal capability that remains after damage or failure.
///
/// This component is **cross-cutting**: it is written by one system (your
/// projectile / collision damage handler) and read independently by multiple
/// domain systems:
///
/// - `compute_aerodynamics`, scales coefficients and adds structural drag.
/// - `compute_propulsion`, scales engine thrust.
/// - `DetachPlugin` (optional), detaches the entity from the Bevy hierarchy
///   when `remaining` reaches `0.0`.
///
/// The name `Failure` describes the *state* of the zone, not the *cause*.
/// Future typed failure modes (`SurfaceBuckle`, `CylinderLoss`, ...) will sit
/// alongside this component; a resolver system will combine them into domain
/// state structs. For now this scalar covers the common case.
///
/// # Semantics
/// - `1.0`, fully intact; no performance loss.
/// - `0.0`, completely failed / detached from the airframe.
///   Domain systems must treat `0.0` as **absent**: zero aerodynamic
///   contribution, zero thrust, not maximum drag.
/// - `(0.0, 1.0)`, partial failure; outputs are scaled by `remaining`.
///   - An `AeroZone` at `0.4` produces 40 % of its nominal lift/drag.
///   - An `EngineZone` at `0.4` produces 40 % of its nominal thrust.
///
/// # Example
/// ```rust
/// use avian_fdm::components::Failure;
///
/// // Zone at full capability, the default state.
/// let f = Failure::default();
/// assert_eq!(f.remaining, 1.0);
///
/// // Zone at zero remaining capability contributes nothing.
/// let failed = Failure { remaining: 0.0 };
/// assert_eq!(failed.remaining, 0.0);
/// ```
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct Failure {
    /// Fraction of nominal capability remaining: `1.0` = intact, `0.0` = failed.
    ///
    /// Multiply any output (force, torque, thrust) by this value before applying
    /// it to the simulation. Write this from your projectile / collision system.
    pub remaining: Scalar,
}

impl Default for Failure {
    fn default() -> Self {
        Self { remaining: 1.0 }
    }
}

/// Extract `Failure::remaining` from an optional component, defaulting to `1.0`
/// (fully intact) when the component is absent.
///
/// This is the standard pattern used by all FDM domain systems: a zone without
/// a `Failure` component is treated as undamaged.
///
/// ```rust
/// use avian_fdm::components::{Failure, get_remaining};
/// assert_eq!(get_remaining(None), 1.0);
/// let f = Failure { remaining: 0.4 };
/// assert_eq!(get_remaining(Some(&f)), 0.4);
/// ```
#[inline]
pub fn get_remaining(failure: Option<&Failure>) -> Scalar {
    failure.map(|f| f.remaining).unwrap_or(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_means_fully_intact() {
        assert_eq!(get_remaining(None), 1.0);
    }

    #[test]
    fn some_failure_returns_remaining() {
        let f = Failure { remaining: 0.4 };
        assert!((get_remaining(Some(&f)) - 0.4).abs() < 1e-12);
    }

    #[test]
    fn zero_remaining_is_fully_failed() {
        let f = Failure { remaining: 0.0 };
        assert_eq!(get_remaining(Some(&f)), 0.0);
    }

    #[test]
    fn default_is_fully_intact() {
        assert_eq!(Failure::default().remaining, 1.0);
    }
}
