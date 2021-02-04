use macroquad::prelude::*;
use rapier2d::{dynamics::RigidBodyBuilder, geometry::ColliderBuilder, pipeline::PhysicsPipeline};
use shipyard::{AllStoragesViewMut, EntityId, UniqueViewMut, World};
use shipyard_rapier2d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system,
    },
    render::{render_colliders, render_physics_stats},
};

pub struct DespawnResource {
    pub entities: Vec<EntityId>,
}

#[macroquad::main("Despawn 2D")]
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
     * Ground
     */
    let ground_size = 25.0;
    let mut despawn_entities = vec![];

    let rigid_body = RigidBodyBuilder::new_static();
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    let entity = all_storages.add_entity((rigid_body, collider));
    despawn_entities.push(entity);

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(ground_size, ground_size * 2.0);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    let entity = all_storages.add_entity((rigid_body, collider));
    despawn_entities.push(entity);

    let body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(-ground_size, ground_size * 2.0);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    let entity = all_storages.add_entity((body, collider));
    despawn_entities.push(entity);

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
            all_storages.add_entity((body, collider));
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
            println!("Despawning ground entity");
            all_storages.delete_entity(*entity);
        }
    }
}
