use macroquad::prelude::*;
use rapier3d::{dynamics::RigidBodyBuilder, geometry::ColliderBuilder, pipeline::PhysicsPipeline};
use shipyard::{AllStoragesViewMut, UniqueViewMut, World};
use shipyard_rapier3d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system, EventQueue,
    },
    render::{render_colliders, render_physics_stats},
};

#[macroquad::main("Events 3D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

    let camera = Camera3D {
        position: vec3(10., 12., 10.),
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

        world.run(display_events).unwrap();
        world.run(render_colliders).unwrap();

        set_default_camera();
        world.run(render_physics_stats).unwrap();

        next_frame().await
    }
}

fn enable_physics_profiling(mut pipeline: UniqueViewMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn display_events(events: UniqueViewMut<EventQueue>) {
    while let Ok(intersection_event) = events.intersection_events.pop() {
        println!("Received intersection event: {:?}", intersection_event);
    }

    while let Ok(contact_event) = events.contact_events.pop() {
        println!("Received contact event: {:?}", contact_event);
    }
}

pub fn setup_physics_world(mut all_storages: AllStoragesViewMut) {
    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::new_static();
    let collider = ColliderBuilder::cuboid(4.0, 1.2, 1.2);
    all_storages.add_entity((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, 5.0, 0.0);
    let collider = ColliderBuilder::cuboid(4.0, 1.2, 1.0).sensor(true);
    all_storages.add_entity((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_dynamic().translation(0.0, 13.0, 0.0);
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    all_storages.add_entity((rigid_body, collider));
}
