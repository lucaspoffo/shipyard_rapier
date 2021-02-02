use macroquad::prelude::*;
use rapier2d::{
    dynamics::RigidBodyBuilder,
    geometry::{ColliderBuilder, ContactPairFilter, PairFilterContext, SolverFlags},
    pipeline::PhysicsPipeline,
};
use shipyard::{AllStoragesViewMut, UniqueViewMut, World};
use shipyard_rapier2d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system, InteractionPairFilters,
    },
    render::{render_colliders, render_physics_stats},
};

// A custom filter that allows contacts only between rigid-bodies with the
// same user_data value.
// Note that using collision groups would be a more efficient way of doing
// this, but we use custom filters instead for demonstration purpose.
struct SameUserDataFilter;
impl ContactPairFilter for SameUserDataFilter {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        if context.rigid_body1.user_data == context.rigid_body2.user_data {
            Some(SolverFlags::COMPUTE_IMPULSES)
        } else {
            None
        }
    }
}

#[macroquad::main("Boxes 2D")]
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
        target: vec2(0.0, 0.0),
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
     * Ground
     */
    {
        let mut filters = all_storages
            .borrow::<UniqueViewMut<InteractionPairFilters>>()
            .unwrap();
        filters.contact_filter(SameUserDataFilter);
    }

    all_storages.add_unique(InteractionPairFilters::new().contact_filter(SameUserDataFilter));

    let ground_size = 10.0;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -10.0)
        .user_data(0);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    all_storages.add_entity((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static().user_data(1);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    all_storages.add_entity((rigid_body, collider));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = (i as f32 + j as f32 * 0.2) * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            // Build the rigid body.
            let body = RigidBodyBuilder::new_dynamic()
                .user_data(j as u128 % 2)
                .translation(x, y);
            let collider = ColliderBuilder::cuboid(rad, rad).density(1.0);
            all_storages.add_entity((body, collider));
        }
    }
}
