//! [`Failure`] component: the fraction of the capability of a zone that remains after damage.
//! The hit or damage system of the game writes it. The FDM systems read it.

use crate::_bevy::*;
use serde::{Deserialize, Serialize};

/// Fraction of the nominal capability of a zone that remains after damage or
/// a failure.
///
/// This component is **cross-cutting**. One system writes it (your damage
/// handler for projectiles or collisions). Many FDM systems read it
/// independently:
///
/// - `compute_aero_forces` scales the coefficients and adds structural drag.
/// - `compute_engine_zone_forces` scales the engine thrust.
///
/// The crate does not detach a zone when `remaining` is `0.0`. Detachment is
/// the responsibility of the game.
///
/// The name `Failure` describes the *state* of the zone, not the *cause*. In
/// the future, the crate will add typed failure modes (`SurfaceBuckle`,
/// `CylinderLoss`, and more) next to this component. A resolver system will
/// then combine them into state structs for each domain. At this time, this
/// one scalar covers the usual case.
///
/// # Meaning of the values
/// - `1.0`: fully intact. No performance loss.
/// - `0.0`: fully failed, or detached from the airframe.
///   FDM systems must treat `0.0` as **absent**: zero aerodynamic
///   contribution and zero thrust, not maximum drag.
/// - `(0.0, 1.0)`: partial failure. `remaining` scales the outputs.
///   - An `AeroZone` at `0.4` makes 40% of its nominal lift and drag.
///   - An `EngineZone` at `0.4` makes 40% of its nominal thrust.
///
/// # Example
/// ```rust
/// use avian_fdm::components::Failure;
///
/// // A zone at full capability. This is the default state.
/// let f = Failure::default();
/// assert_eq!(f.remaining, 1.0);
///
/// // A zone with zero remaining capability contributes nothing.
/// let failed = Failure { remaining: 0.0 };
/// assert_eq!(failed.remaining, 0.0);
/// ```
#[derive(Component, Reflect, Serialize, Deserialize, Clone, Debug)]
#[reflect(Component, Serialize, Deserialize)]
pub struct Failure {
    /// Fraction of the nominal capability that remains: `1.0` = intact, `0.0` = failed.
    ///
    /// The FDM systems multiply their outputs (force, torque, thrust) by this
    /// value before they apply them. Write it from your projectile or
    /// collision system.
    pub remaining: f32,
}

impl Default for Failure {
    fn default() -> Self {
        Self { remaining: 1.0 }
    }
}

/// Returns `Failure::remaining` from an optional component. If the component
/// is absent, it returns `1.0` (fully intact).
///
/// All FDM systems use this standard pattern. A zone without a `Failure`
/// component is an intact zone.
///
/// ```rust
/// use avian_fdm::components::{Failure, get_remaining};
/// assert_eq!(get_remaining(None), 1.0);
/// let f = Failure { remaining: 0.4 };
/// assert_eq!(get_remaining(Some(&f)), 0.4);
/// ```
#[inline]
pub fn get_remaining(failure: Option<&Failure>) -> f32 {
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
