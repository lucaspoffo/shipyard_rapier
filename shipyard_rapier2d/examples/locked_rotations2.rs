extern crate rapier2d as rapier; // For the debug UI.

use macroquad::prelude::*;
use rapier::geometry::ColliderBuilder;
use rapier2d::dynamics::RigidBodyBuilder;
use rapier2d::pipeline::PhysicsPipeline;
use shipyard::{AllStoragesViewMut, UniqueViewMut, World};
use shipyard_rapier2d::{
    physics::systems::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system,
    },
    render::{render_colliders, render_physics_stats},
};

#[macroquad::main("Locked Rotations 2D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

    let viewport_height = 15.0;
    let aspect = screen_width() / screen_height();
    let viewport_width = viewport_height * aspect;

    let camera = Camera2D {
        zoom: vec2(
            1.0 / viewport_width as f32 * 2.,
            -1.0 / viewport_height as f32 * 2.,
        ),
        target: vec2(0.0, -2.5),
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
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, -ground_height);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    all_storages.add_entity((rigid_body, collider));

    /*
     * A rectangle that only rotate.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 3.0)
        .lock_translations();
    let collider = ColliderBuilder::cuboid(2.0, 0.6);
    all_storages.add_entity((rigid_body, collider));

    /*
     * A tilted cuboid that cannot rotate.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.3, 5.0)
        .rotation(1.0)
        .lock_rotations();
    let collider = ColliderBuilder::cuboid(0.6, 0.4);
    all_storages.add_entity((rigid_body, collider));
}
