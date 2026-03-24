//! Collider wireframe overlay for physics verification.
//!
//! When [`ShowColliders`] is true, draws the raw parry collider shapes
//! (cuboids, spheres, cylinders, capsules) on top of the normal zone
//! visuals so you can verify that a `GizmoShape` override matches the
//! underlying physics collider.
//!
//! Toggle from game code or a debug system:
//!
//! ```rust,no_run
//! # use bevy::prelude::*;
//! # use avian_fdm::debug_render::ShowColliders;
//! fn toggle_on_key(keys: Res<ButtonInput<KeyCode>>, mut show: ResMut<ShowColliders>) {
//!     if keys.just_pressed(KeyCode::KeyC) {
//!         show.0 = !show.0;
//!     }
//! }
//! ```

use avian3d::prelude::Collider;
use bevy_transform::TransformSystems;
use crate::_bevy::*;
use crate::components::{AeroZone, AircraftGeometry};

#[cfg(feature = "propulsion")]
use crate::components::EngineZone;

use super::configuration::FdmGizmos;

/// Controls the collider wireframe overlay.
///
/// When `true`, [`AircraftFdmDebugPlugin`] draws the actual parry collider
/// shape for every zone entity (orange wireframes). Use this to verify that
/// `GizmoShape` visuals match the physics colliders.
///
/// Toggle from any system that has `ResMut<ShowColliders>`. The resource is
/// registered by [`AircraftFdmDebugPlugin`] and defaults to `false`.
///
/// [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin
#[derive(Resource, Default, Debug)]
pub struct ShowColliders(pub bool);

/// Orange tint used for collider wireframe overlays.
const COLLIDER_COLOR: bevy_color::Color = bevy_color::Color::srgba(0.8, 0.5, 0.2, 0.8);

pub(super) fn plugin(app: &mut App) {
    app.init_resource::<ShowColliders>();
    app.add_systems(
        PostUpdate,
        debug_render_colliders
            .after(TransformSystems::Propagate)
            .run_if(|show: Res<ShowColliders>, store: Res<GizmoConfigStore>| {
                show.0 && store.config::<FdmGizmos>().0.enabled
            }),
    );
}

/// Draws each zone's raw collider shape as an orange wireframe.
///
/// Runs in `PostUpdate` after transform propagation so `GlobalTransform`
/// reflects the current physics state. Uses `avian3d::parry` (re-exported
/// by avian3d) so no direct parry dependency is needed in calling code.
pub(super) fn debug_render_colliders(
    mut gizmos: Gizmos<FdmGizmos>,
    root_query: Query<&Transform, With<AircraftGeometry>>,
    #[cfg(not(feature = "propulsion"))]
    zone_query: Query<(&Transform, Option<&Collider>), With<AeroZone>>,
    #[cfg(feature = "propulsion")]
    zone_query: Query<(&Transform, Option<&Collider>), Or<(With<AeroZone>, With<EngineZone>)>>,
) {
    use avian3d::parry::shape::TypedShape;

    let Ok(root_tf) = root_query.single() else { return };
    let t = root_tf.translation;
    let r = root_tf.rotation;

    let iso_at = |zone_tf: &Transform| {
        Isometry3d::new(
            t + r * zone_tf.translation,
            Quat::from_array(r.to_array())
                * Quat::from_array(zone_tf.rotation.to_array()),
        )
    };

    for (zone_tf, collider) in &zone_query {
        let Some(col) = collider else { continue };
        let iso = iso_at(zone_tf);
        match col.shape_scaled().as_typed_shape() {
            TypedShape::Cuboid(c) => {
                let he = c.half_extents;
                gizmos.primitive_3d(
                    &Cuboid::new(he.x as f32 * 2.0, he.y as f32 * 2.0, he.z as f32 * 2.0),
                    iso,
                    COLLIDER_COLOR,
                );
            }
            TypedShape::Ball(b) => {
                gizmos
                    .primitive_3d(
                        &Sphere::new(b.radius as f32),
                        iso,
                        COLLIDER_COLOR,
                    )
                    .resolution(32);
            }
            TypedShape::Cylinder(c) => {
                gizmos
                    .primitive_3d(
                        &Cylinder::new(c.radius as f32, c.half_height as f32 * 2.0),
                        iso,
                        COLLIDER_COLOR,
                    )
                    .resolution(32);
            }
            TypedShape::Capsule(c) => {
                gizmos.primitive_3d(
                    &Capsule3d::new(c.radius as f32, c.half_height() as f32 * 2.0),
                    iso,
                    COLLIDER_COLOR,
                );
            }
            _ => {}
        }
    }
}
