use macroquad::prelude::*;
use rapier3d::{dynamics::RigidBodyBuilder, geometry::ColliderBuilder, pipeline::PhysicsPipeline};
use shipyard::{AllStoragesViewMut, EntityId, UniqueViewMut, World};
use shipyard_rapier3d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system,
    },
    render::{render_colliders, render_physics_stats},
};

pub struct DespawnResource {
    pub entity: Option<EntityId>,
}

#[macroquad::main("Despawn 3D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.run(setup_physics_world).unwrap();

    let camera = Camera3D {
        position: vec3(-80., 30., -80.),
        up: vec3(0., 1., 0.),
        target: vec3(-20., 0., -20.),
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

        // Run despawn system
        world.run_with_data(despawn, get_time()).unwrap();

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
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, -ground_height, 0.0);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let ground_entity = all_storages.add_entity((rigid_body, collider));
    let despawn = DespawnResource {
        entity: Some(ground_entity),
    };
    all_storages.add_unique(despawn);
    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z);
                let collider = ColliderBuilder::cuboid(rad, rad, rad).density(1.0);
                all_storages.add_entity((rigid_body, collider));
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

pub fn despawn(time: f64, mut all_storages: AllStoragesViewMut) {
    if time > 5.0 {
        let despawn_entity = {
            let mut despawn = all_storages
                .borrow::<UniqueViewMut<DespawnResource>>()
                .unwrap();
            despawn.entity.take()
        };
        if let Some(entity) = despawn_entity {
            println!("Despawning ground entity");
            all_storages.delete_entity(entity);
        }
    }
}
