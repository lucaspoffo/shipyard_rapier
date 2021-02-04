use macroquad::prelude::*;
use nalgebra::Point2;
use rapier2d::{
    dynamics::{BallJoint, BodyStatus, RigidBodyBuilder},
    geometry::ColliderBuilder,
    pipeline::PhysicsPipeline,
};
use shipyard::{AllStoragesViewMut, EntityId, UniqueViewMut, World};
use shipyard_rapier2d::{
    physics::{
        components::JointBuilderComponent,
        systems::{
            create_body_and_collider_system, create_joints_system,
            destroy_body_and_collider_system, setup_physics, step_world_system,
        },
    },
    render::{render_colliders, render_physics_stats},
};

pub struct DespawnResource {
    pub entities: Vec<EntityId>,
}

#[macroquad::main("Joints Despawn 2D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

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
        clear_background(WHITE);
        set_camera(camera);

        // Systems to update physics world
        world.run(create_body_and_collider_system).unwrap();
        world.run(create_joints_system).unwrap();
        world
            .run_with_data(step_world_system, get_frame_time())
            .unwrap();
        world.run(destroy_body_and_collider_system).unwrap();

        world.run_with_data(despawn, get_time()).unwrap();
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
    let mut despawn_entities = vec![];

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
            let child_entity = all_storages.add_entity((rigid_body, collider));

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = BallJoint::new(Point2::origin(), Point2::new(0.0, shift));
                let entity = all_storages.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
                if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                    despawn_entities.push(entity);
                }
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - numi;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point2::origin(), Point2::new(-shift, 0.0));
                let entity = all_storages.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
                if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                    despawn_entities.push(entity);
                }
            }

            body_entities.push(child_entity);
        }
    }

    let despawn = DespawnResource {
        entities: despawn_entities,
    };
    all_storages.add_unique(despawn);
}

pub fn despawn(time: f64, mut all_storages: AllStoragesViewMut) {
    if time > 5.0 {
        let despawn_entities = {
            let mut despawn = all_storages
                .borrow::<UniqueViewMut<DespawnResource>>()
                .unwrap();
            let entities = despawn.entities.clone();
            despawn.entities.clear();
            entities
        };
        for entity in &despawn_entities {
            println!("Despawning joint entity");
            all_storages.delete_entity(*entity);
        }
    }
}
