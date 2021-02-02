extern crate rapier2d as rapier; // For the debug UI.

use macroquad::prelude::*;
use shipyard_rapier2d::{
    physics::{
        systems::{create_body_and_collider_system, step_world_system,setup, create_joints_system}, 
        components::JointBuilderComponent
    },
    render::{render_colliders, render_physics_stats}
};
use nalgebra::Point2;
use rapier::dynamics::{BallJoint, BodyStatus,RigidBodyBuilder};
use rapier::geometry::ColliderBuilder;
use rapier::pipeline::PhysicsPipeline;
use shipyard::{World, UniqueViewMut};

#[macroquad::main("Joints 2D")]
async fn main() {
    let mut world = World::new();
    setup(&mut world);
    setup_physics(&mut world);
    
    let viewport_height = 60.0;
    let aspect = screen_width() / screen_height();
    let viewport_width = viewport_height * aspect;

    let camera = Camera2D {
        zoom: vec2(
            1.0 / viewport_width as f32 * 2.,
            -1.0 / viewport_height as f32 * 2.,
        ),
        target: vec2(20.0, 20.0),
        ..Default::default()
    };

    world.run(enable_physics_profiling).unwrap();

    loop {
        set_camera(camera);
        world.run(create_body_and_collider_system).unwrap();
        world.run(create_joints_system).unwrap();
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
     * Create the balls
     */
    // Build the rigid body.
    // NOTE: a smaller radius (e.g. 0.1) breaks Box2D so
    // in order to be able to compare rapier with Box2D,
    // we set it to 0.4.
    let rad = 0.4;
    let numi = 40; // Num vertical nodes.
    let numk = 40; // Num horizontal nodes.
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == numk - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(fk * shift, -fi * shift);
            let collider = ColliderBuilder::cuboid(rad, rad).density(1.0);
            let child_entity = world.add_entity((rigid_body, collider));

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = BallJoint::new(Point2::origin(), Point2::new(0.0, shift));
                world.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - numi;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point2::origin(), Point2::new(-shift, 0.0));
                world.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            body_entities.push(child_entity);
        }
    }
}

