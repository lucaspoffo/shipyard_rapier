extern crate rapier2d as rapier; // For the debug UI.

use macroquad::prelude::*;
use rapier::{
    dynamics::{RigidBodyBuilder, RigidBodySet},
    geometry::{ColliderBuilder, ColliderSet},
    pipeline::PhysicsPipeline,
};
use shipyard::{
    AllStoragesViewMut, EntitiesView, EntityId, IntoIter, IntoWithId, UniqueViewMut, View, ViewMut,
    World,
};
use shipyard_rapier2d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system, ColliderHandleComponent, EntityMaps,
    },
    render::{render_colliders, render_physics_stats},
};

#[macroquad::main("Multiple Colliders 2D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

    let viewport_height = 120.0;
    let aspect = screen_width() / screen_height();
    let viewport_width = viewport_height * aspect;

    let camera = Camera2D {
        zoom: vec2(
            1.0 / viewport_width as f32 * 2.,
            -1.0 / viewport_height as f32 * 2.,
        ),
        target: vec2(0.0, -50.0),
        ..Default::default()
    };

    world.run(enable_physics_profiling).unwrap();

    loop {
        clear_background(WHITE);
        set_camera(camera);

        // Systems to update physics world
        world.run(create_body_and_collider_system).unwrap();
        world.run(create_joints_system).unwrap();
        world
            .run_with_data(step_world_system, get_frame_time())
            .unwrap();
        world.run(destroy_body_and_collider_system).unwrap();

        // Custom system to create colliders for entities with parents
        world.run(create_child_collider_system).unwrap();

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

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, -ground_height);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    all_storages.add_entity((rigid_body, collider));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.2;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            let x = i as f32 * shift * 5.0 - centerx + offset;
            let y = j as f32 * (shift * 5.0) + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y);

            // Attach multiple colliders to this rigid-body using Bevy hierarchy.
            let collider1 = ColliderBuilder::cuboid(rad * 10.0, rad);
            let collider2 =
                ColliderBuilder::cuboid(rad, rad * 10.0).translation(rad * 10.0, rad * 10.0);
            let collider3 =
                ColliderBuilder::cuboid(rad, rad * 10.0).translation(-rad * 10.0, rad * 10.0);

            let parent = all_storages.add_entity((rigid_body,));

            all_storages.add_entity((collider1, Child { parent }));
            all_storages.add_entity((collider2, Child { parent }));
            all_storages.add_entity((collider3, Child { parent }));
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
    mut entity_maps: UniqueViewMut<EntityMaps>,
    mut collider_builders: ViewMut<ColliderBuilder>,
    mut collider_handles: ViewMut<ColliderHandleComponent>,
    childs: View<Child>,
) {
    let mut colliders_builder_deleted = vec![];

    for (entity_id, (child, collider_builder)) in (&childs, &collider_builders).iter().with_id() {
        if let Some(body_handle) = entity_maps.bodies.get(&child.parent) {
            let handle = colliders.insert(collider_builder.build(), *body_handle, &mut bodies);
            entities.add_component(
                entity_id,
                &mut collider_handles,
                ColliderHandleComponent::from(handle),
            );
            colliders_builder_deleted.push(entity_id);
            entity_maps.colliders.insert(entity_id, handle);
        }
    }

    for entity_id in &colliders_builder_deleted {
        collider_builders.delete(*entity_id);
    }
}
