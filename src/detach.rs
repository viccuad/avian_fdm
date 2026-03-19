//! `DetachPlugin` — optional plugin that turns destroyed zones into free
//! flying rigid bodies.
//!
//! Each frame, the `detach_destroyed_zones` system checks every entity whose
//! [`Damageable::health`] just changed. When health reaches `0.0` the system:
//!
//! 1. Removes `ChildOf` (detaches from the aircraft hierarchy).
//! 2. Inserts `RigidBody::Dynamic` + copies the parent's `LinearVelocity` and
//!    `AngularVelocity` so the piece inherits the aircraft's velocity.
//! 3. Avian automatically recomputes `ComputedMass` / `ComputedCenterOfMass` /
//!    `ComputedAngularInertia` on the parent. No library code required.
//!
//! `DetachPlugin` is **opt-in**. Games that prefer zero-contribution zones
//! without spawning debris (which happens automatically via `ZoneForce::default()`
//! when health = 0) can omit this plugin.
//!
//! Only compiled with `features = ["damage"]`.

use bevy::prelude::*;
use avian3d::prelude::{AngularVelocity, ColliderOf, LinearVelocity, RigidBody};

use crate::components::Damageable;

/// Optional plugin. Adds a system that detaches zero-health zones from the
/// hierarchy and spawns them as free rigid bodies.
pub struct DetachPlugin;

impl Plugin for DetachPlugin {
    fn build(&self, app: &mut App) {
        use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};
        // Run after force accumulation so the detached entity's ZoneForce is
        // already zeroed before Avian processes forces.
        app.add_systems(
            PhysicsSchedule,
            detach_destroyed_zones.in_set(PhysicsStepSystems::First),
        );
    }
}

/// Detaches zones whose `Damageable.health` just dropped to 0.0.
fn detach_destroyed_zones(
    mut commands: Commands,
    changed: Query<(Entity, &Damageable, &ColliderOf), Changed<Damageable>>,
    parent_vel: Query<(&LinearVelocity, &AngularVelocity)>,
) {
    for (entity, dmg, col_of) in changed.iter() {
        if dmg.health > 0.0 {
            continue;
        }
        let (lin_vel, ang_vel) = parent_vel
            .get(col_of.body)
            .map(|(l, a)| (*l, *a))
            .unwrap_or_default();

        commands
            .entity(entity)
            .remove::<ChildOf>()
            .insert((RigidBody::Dynamic, lin_vel, ang_vel));
    }
}
