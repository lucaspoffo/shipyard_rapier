use macroquad::prelude::*;
use rapier3d::{
    na::Vector3,
    dynamics::RigidBodyBuilder,
    geometry::ColliderBuilder,
    pipeline::PhysicsPipeline
};
use shipyard::{AllStoragesViewMut, UniqueViewMut, World};
use shipyard_rapier3d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system,
    },
    render::{render_colliders, render_physics_stats},
};

#[macroquad::main("Locked Rotation 3D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

    let camera = Camera3D {
        position: vec3(10., 5., 0.),
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

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, -ground_height, 0.0);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    all_storages.add_entity((rigid_body, collider));

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 3.0, 0.0)
        .lock_translations()
        .restrict_rotations(true, false, false);
    let collider = ColliderBuilder::cuboid(0.2, 0.6, 2.0);
    all_storages.add_entity((rigid_body, collider));

    /*
     * A tilted cuboid that cannot rotate.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 5.0, 0.0)
        .rotation(Vector3::x() * 1.0)
        .lock_rotations();
    let collider = ColliderBuilder::cuboid(0.6, 0.4, 0.4);
    all_storages.add_entity((rigid_body, collider));
}
