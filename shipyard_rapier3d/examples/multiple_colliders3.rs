use macroquad::prelude::*;
use rapier3d::{
    dynamics::{RigidBodyBuilder, RigidBodySet},
    geometry::{ColliderBuilder, ColliderSet},
    pipeline::PhysicsPipeline,
};
use shipyard::{
    AllStoragesViewMut, EntitiesView, EntityId, Get, IntoIter, IntoWithId, UniqueViewMut, View,
    ViewMut, World,
};
use shipyard_rapier3d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system, ColliderHandleComponent, RigidBodyHandleComponent,
    },
    render::{render_colliders, render_physics_stats},
};

#[macroquad::main("Multiple Colliders 3D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

    let camera = Camera3D {
        position: vec3(-40., 50., -50.),
        up: vec3(0., 1., 0.),
        target: vec3(0., 0., 0.),
        ..Default::default()
    };

    world.run(enable_physics_profiling).unwrap();

    loop {
        clear_background(WHITE);
        set_camera(camera);

        // Systems to update physics world
        world.run(create_body_and_collider_system).unwrap();
        world.run(create_joints_system).unwrap();
        world.run(create_child_collider_system).unwrap();
        world
            .run_with_data(step_world_system, get_frame_time())
            .unwrap();
        world.run(destroy_body_and_collider_system).unwrap();

        world.run(render_colliders).unwrap();

        set_default_camera();
        world.run(render_physics_stats).unwrap();

        next_frame().await
    }
}

fn enable_physics_profiling(mut pipeline: UniqueViewMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

pub fn setup_physics_world(mut all_storages: AllStoragesViewMut) {
    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, -ground_height, 0.0);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    all_storages.add_entity((rigid_body, collider));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.2;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift * 5.0 - centerx + offset;
                let y = j as f32 * (shift * 5.0) + centery + 3.0;
                let z = k as f32 * shift * 2.0 - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z);

                // Attach multiple colliders to this rigid-body using Bevy hierarchy.
                let collider1 = ColliderBuilder::cuboid(rad * 10.0, rad, rad);
                let collider2 = ColliderBuilder::cuboid(rad, rad * 10.0, rad).translation(
                    rad * 10.0,
                    rad * 10.0,
                    0.0,
                );
                let collider3 = ColliderBuilder::cuboid(rad, rad * 10.0, rad).translation(
                    -rad * 10.0,
                    rad * 10.0,
                    0.0,
                );

                // NOTE: we need the Transform and GlobalTransform
                // so that the transform of the entity with a rigid-body
                // is properly propagated to its children with collider meshes.
                let parent = all_storages.add_entity((rigid_body,));

                all_storages.add_entity((collider1, Child { parent }));
                all_storages.add_entity((collider2, Child { parent }));
                all_storages.add_entity((collider3, Child { parent }));
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

#[derive(Debug, Clone)]
pub struct Child {
    parent: EntityId,
}

pub fn create_child_collider_system(
    entities: EntitiesView,
    mut bodies: UniqueViewMut<RigidBodySet>,
    mut colliders: UniqueViewMut<ColliderSet>,
    mut collider_builders: ViewMut<ColliderBuilder>,
    mut collider_handles: ViewMut<ColliderHandleComponent>,
    body_handles: View<RigidBodyHandleComponent>,
    childs: View<Child>,
) {
    for (entity_id, (child, collider_builder)) in (&childs, &collider_builders).iter().with_id() {
        if let Ok(body_handle) = body_handles.get(child.parent) {
            let handle =
                colliders.insert(collider_builder.build(), body_handle.handle(), &mut bodies);
            entities.add_component(
                entity_id,
                &mut collider_handles,
                ColliderHandleComponent::from(handle),
            );
        }
    }

    collider_builders.clear();
}
