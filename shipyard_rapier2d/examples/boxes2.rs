extern crate rapier2d as rapier; // For the debug UI.

use macroquad::prelude::*;
use shipyard_rapier2d::{
    physics::systems::{create_body_and_collider_system, step_world_system,setup},
    render::{render_colliders, render_physics_stats}
};
use rapier2d::dynamics::RigidBodyBuilder;
use rapier::geometry::ColliderBuilder;
use rapier2d::pipeline::PhysicsPipeline;
use shipyard::{World, UniqueViewMut};

#[macroquad::main("Boxes2")]
async fn main() {
    let mut world = World::new();
    setup(&mut world);
    setup_physics(&mut world);
    
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
        set_camera(camera);
        world.run(create_body_and_collider_system).unwrap();
        world.run_with_data(step_world_system, get_frame_time()).unwrap();
        world.run(render_colliders).unwrap();
        
        set_default_camera();
        world.run(render_physics_stats).unwrap();

        next_frame().await
    }
}

fn enable_physics_profiling(mut pipeline: UniqueViewMut<PhysicsPipeline>) {
   pipeline.counters.enable()
}

pub fn setup_physics(world: &mut World) {
    /*
     * Ground
     */
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::new_static();
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    world.add_entity((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(ground_size, ground_size * 2.0);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    world.add_entity((rigid_body, collider));

    let body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(-ground_size, ground_size * 2.0);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    world.add_entity((body, collider));

    /*
     * Create the cubes
     */
    let num = 20;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            // Build the rigid body.
            let body = RigidBodyBuilder::new_dynamic().translation(x, y);
            let collider = ColliderBuilder::cuboid(rad, rad).density(1.0);
            world.add_entity((body, collider));
        }
    }
}

