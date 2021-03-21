<div align="center">

# Shipyard Rapier

Physics plugin with [rapier](https://github.com/dimforge/rapier) for the [shipyard](https://github.com/leudz/shipyard) ECS.

2D:  ![Crates.io](https://img.shields.io/crates/v/shipyard_rapier2d)
[![Documentation](https://docs.rs/shipyard_rapier2d/badge.svg)](https://docs.rs/shipyard_rapier2d)

3D:  ![Crates.io](https://img.shields.io/crates/v/shipyard_rapier3d)
[![Documentation](https://docs.rs/shipyard_rapier3d/badge.svg)](https://docs.rs/shipyard_rapier3d)

</div>

## How to use

Setup the physics in the shipyard world:
```rust
let world = World::new();
world.run(setup_physics).unwrap();
```

Create an body and a collider component, add those to an existent entity, or create a new one:
```rust
let body = RigidBodyBuilder::new_dynamic().translation(x, y);
let collider = ColliderBuilder::cuboid(rad, rad).density(1.0);
all_storages.add_entity((body, collider));
```

In your gameplay loop, execute the physics systems to simulate the world:
```rust
// Create rapier colliders, bodies and joints based on the shipyard components.
world.run(create_body_and_collider_system).unwrap();
world.run(create_joints_system).unwrap();

// Step the world based on a frame rate.
let frame_time = 60.0 / 1000.0; // 60 fps simulation
world.run_with_data(step_world_system, frame_time).unwrap();

// Remove any physics components from deleted entities.
world.run(destroy_body_and_collider_system).unwrap();
```

This plugin is based of [bevy_rapier](https://github.com/dimforge/bevy_rapier/) plugin.
