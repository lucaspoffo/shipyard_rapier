use macroquad::prelude::*;
use rapier2d::{
    na::Vector2,
    dynamics::{RigidBodyBuilder, RigidBodySet},
    geometry::ColliderBuilder,
};
use shipyard::{AllStoragesViewMut, IntoIter, UniqueView, UniqueViewMut, View, World};
use shipyard_rapier2d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system, RapierConfiguration, RigidBodyHandleComponent,
    },
    render::{render_colliders, render_physics_stats, RapierRenderColor},
};

#[derive(Debug, Default)]
struct Player {
    speed: f32,
}

#[macroquad::main("Player movement 2D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();

    let viewport_height = 1200.0;
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

    world.run(spawn_player).unwrap();
    world.run(setup_physics_world).unwrap();

    loop {
        clear_background(WHITE);
        set_camera(camera);

        // Systems to update physics world
        world.run(create_body_and_collider_system).unwrap();
        world.run(create_joints_system).unwrap();

        world.run(player_movement).unwrap();

        world
            .run_with_data(step_world_system, get_frame_time())
            .unwrap();
        world.run(destroy_body_and_collider_system).unwrap();

        world.run(render_colliders).unwrap();

        set_default_camera();
        world.run(render_physics_stats).unwrap();
        world.run(render_player_position).unwrap();

        next_frame().await
    }
}

pub fn setup_physics_world(mut all_storages: AllStoragesViewMut) {
    let ground_size = 20.0;

    for i in 0..4 {
        let translation = match i {
            1 => (-ground_size, 0.0),
            3 => (ground_size, 0.0),
            2 => (0.0, -ground_size),
            0 => (0.0, ground_size),
            _ => unreachable!(),
        };
        let rigid_body = RigidBodyBuilder::new_static()
            .rotation(std::f32::consts::FRAC_PI_2 * i as f32)
            .translation(translation.0, translation.1);
        let collider = ColliderBuilder::cuboid(ground_size, 1.2);
        all_storages.add_entity((rigid_body, collider));
    }
}

fn spawn_player(mut all_storages: AllStoragesViewMut) {
    let scale = 20.0;
    {
        let mut rapier_config = all_storages
            .borrow::<UniqueViewMut<RapierConfiguration>>()
            .unwrap();
        rapier_config.gravity = Vector2::zeros();
        // While we want our sprite to look ~40 px square, we want to keep the physics units smaller
        // to prevent float rounding problems. To do this, we set the scale factor in RapierConfiguration
        // and divide our sprite_size by the scale.
        rapier_config.scale = scale;
    }
    let sprite_size_x = 40.0;
    let sprite_size_y = 40.0;

    let collider_size_x = sprite_size_x / scale;
    let collider_size_y = sprite_size_y / scale;

    // Spawn entity with `Player` struct as a component for access in movement query.
    let rigid_body = RigidBodyBuilder::new_dynamic();
    let collider = ColliderBuilder::cuboid(collider_size_x / 2.0, collider_size_y / 2.0);
    let color = RapierRenderColor(1.0, 0.0, 0.0);

    let player = Player { speed: 300.0 };

    all_storages.add_entity((rigid_body, collider, player, color));
}

fn player_movement(
    rapier_parameters: UniqueView<RapierConfiguration>,
    player: View<Player>,
    body_handles: View<RigidBodyHandleComponent>,
    mut rigid_bodies: UniqueViewMut<RigidBodySet>,
) {
    for (player, rigid_body_component) in (&player, &body_handles).iter() {
        let x_axis = is_key_down(KeyCode::D) as i8 - is_key_down(KeyCode::A) as i8;
        let y_axis = is_key_down(KeyCode::W) as i8 - is_key_down(KeyCode::S) as i8;

        let mut move_delta = Vector2::new(x_axis as f32, y_axis as f32);
        if move_delta != Vector2::zeros() {
            // Note that the RapierConfiguration::Scale factor is also used here to transform
            // the move_delta from: 'pixels/second' to 'physics_units/second'
            move_delta /= move_delta.magnitude() * rapier_parameters.scale;
        }

        // Update the velocity on the rigid_body_component,
        if let Some(rb) = rigid_bodies.get_mut(rigid_body_component.handle()) {
            rb.set_linvel(move_delta * player.speed, true);
        }
    }
}

fn render_player_position(
    player: View<Player>,
    body_handles: View<RigidBodyHandleComponent>,
    rigid_bodies: UniqueView<RigidBodySet>,
) {
    for (_, rigid_body_component) in (&player, &body_handles).iter() {
        if let Some(rb) = rigid_bodies.get(rigid_body_component.handle()) {
            let pos = rb.position();
            let text = format!("Player: ({}, {})", pos.translation.x, pos.translation.y);
            draw_text(&text, 10.0, 70.0, 30.0, BLACK);
        }
    }
}
