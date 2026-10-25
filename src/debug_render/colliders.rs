//! Collider wireframe overlay, to examine the physics colliders.
//!
//! When [`ShowColliders`] is true, the overlay draws the raw parry collider
//! shapes (cuboids, spheres, cylinders, capsules) on top of the zone
//! wireframes. Use it to make sure that a `GizmoShape` agrees with the physics
//! collider.
//!
//! To turn it on or off, change the resource from game code or from a debug
//! system:
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

use crate::_bevy::*;
use crate::components::{AeroZone, AircraftGeometry, EngineZone};
use avian3d::prelude::Collider;
use bevy_shape::{Capsule3d, Cuboid, Cylinder, Sphere};
use bevy_transform::TransformSystems;

use super::configuration::FdmGizmos;

/// Controls the collider wireframe overlay.
///
/// When `true`, [`AircraftFdmDebugPlugin`] draws the real parry collider
/// shape of each zone entity (orange wireframes). Use it to make sure that
/// the `GizmoShape` visuals agree with the physics colliders.
///
/// Each system with `ResMut<ShowColliders>` can change it.
/// [`AircraftFdmDebugPlugin`] registers the resource. The default is `false`.
///
/// [`AircraftFdmDebugPlugin`]: super::AircraftFdmDebugPlugin
#[derive(Resource, Default, Debug)]
pub struct ShowColliders(pub bool);

/// Orange color for the collider wireframe overlay.
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

/// Draws the raw collider shape of each zone as an orange wireframe.
///
/// It runs in `PostUpdate`, after transform propagation. Thus,
/// `GlobalTransform` has the current physics state. It uses `avian3d::parry`
/// (a re-export from avian3d). Thus, the calling code does not need a direct
/// parry dependency.
#[allow(clippy::unnecessary_cast)]
#[allow(clippy::type_complexity)]
pub(super) fn debug_render_colliders(
    mut gizmos: Gizmos<FdmGizmos>,
    root_query: Query<&Transform, With<AircraftGeometry>>,
    zone_query: Query<(&Transform, Option<&Collider>), Or<(With<AeroZone>, With<EngineZone>)>>,
) {
    use avian3d::parry::shape::TypedShape;

    let Ok(root_tf) = root_query.single() else {
        return;
    };
    let t = root_tf.translation;
    let r = root_tf.rotation;

    let iso_at = |zone_tf: &Transform| {
        Isometry3d::new(
            t + r * zone_tf.translation,
            Quat::from_array(r.to_array()) * Quat::from_array(zone_tf.rotation.to_array()),
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
                    .primitive_3d(&Sphere::new(b.radius as f32), iso, COLLIDER_COLOR)
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
