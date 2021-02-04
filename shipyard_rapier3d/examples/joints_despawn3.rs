use macroquad::prelude::*;
use nalgebra::{Isometry3, Point3, Unit, Vector3};
use rapier3d::{
    dynamics::{
        BallJoint, BodyStatus, FixedJoint, PrismaticJoint, RevoluteJoint, RigidBodyBuilder,
    },
    geometry::ColliderBuilder,
    pipeline::PhysicsPipeline,
};
use shipyard::{AllStoragesViewMut, EntityId, UniqueViewMut, World};
use shipyard_rapier3d::{
    physics::{
        create_body_and_collider_system, create_joints_system, destroy_body_and_collider_system,
        setup_physics, step_world_system, JointBuilderComponent,
    },
    render::{render_colliders, render_physics_stats},
};

#[derive(Default)]
pub struct DespawnResource {
    pub entities: Vec<EntityId>,
}

#[macroquad::main("Joints Despawn 3D")]
async fn main() {
    let world = World::new();
    world.run(setup_physics).unwrap();
    world.add_unique(DespawnResource::default()).unwrap();

    world
        .run_with_data(create_prismatic_joints, (Point3::new(20.0, 10.0, 0.0), 5))
        .unwrap();
    world
        .run_with_data(create_revolute_joints, (Point3::new(20.0, 0.0, 0.0), 3))
        .unwrap();
    world
        .run_with_data(create_fixed_joints, (Point3::new(0.0, 10.0, 0.0), 5))
        .unwrap();
    world.run_with_data(create_ball_joints, 15).unwrap();

    let camera = Camera3D {
        position: vec3(-20., 20., -20.),
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

fn create_prismatic_joints(
    (origin, num): (Point3<f32>, usize),
    mut all_storages: AllStoragesViewMut,
) {
    let mut despawn_entities = vec![];
    let rad = 0.4;
    let shift = 1.0;

    let ground = RigidBodyBuilder::new_static().translation(origin.x, origin.y, origin.z);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    let mut curr_parent = all_storages.add_entity((ground, collider));

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let density = 1.0;
        let rigid_body = RigidBodyBuilder::new_dynamic().translation(origin.x, origin.y, z);
        let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
        let curr_child = all_storages.add_entity((rigid_body, collider));

        let axis = if i % 2 == 0 {
            Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
        } else {
            Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
        };

        let z = Vector3::z();
        let mut prism = PrismaticJoint::new(
            Point3::origin(),
            axis,
            z,
            Point3::new(0.0, 0.0, -shift),
            axis,
            z,
        );
        prism.limits_enabled = true;
        prism.limits[0] = -2.0;
        prism.limits[1] = 2.0;

        let entity =
            all_storages.add_entity((JointBuilderComponent::new(prism, curr_parent, curr_child),));
        if i == 2 {
            despawn_entities.push(entity);
        }

        curr_parent = curr_child;
    }
    let mut despawn = all_storages
        .borrow::<UniqueViewMut<DespawnResource>>()
        .unwrap();
    despawn.entities.append(&mut despawn_entities);
}

fn create_revolute_joints(
    (origin, num): (Point3<f32>, usize),
    mut all_storages: AllStoragesViewMut,
) {
    let mut despawn_entities = vec![];
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::new_static().translation(origin.x, origin.y, 0.0);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    let mut curr_parent = all_storages.add_entity((ground, collider));

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;
        let positions = [
            Isometry3::translation(origin.x, origin.y, z),
            Isometry3::translation(origin.x + shift, origin.y, z),
            Isometry3::translation(origin.x + shift, origin.y, z + shift),
            Isometry3::translation(origin.x, origin.y, z + shift),
        ];

        let mut handles = [curr_parent; 4];
        for k in 0..4 {
            let density = 1.0;
            let rigid_body = RigidBodyBuilder::new_dynamic().position(positions[k]);
            let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
            handles[k] = all_storages.add_entity((rigid_body, collider));
        }

        // Setup four joints.
        let o = Point3::origin();
        let x = Vector3::x_axis();
        let z = Vector3::z_axis();

        let revs = [
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(-shift, 0.0, 0.0), x),
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(shift, 0.0, 0.0), x),
        ];

        let e1 =
            all_storages
                .add_entity((JointBuilderComponent::new(revs[0], curr_parent, handles[0]),));
        let e2 =
            all_storages.add_entity((JointBuilderComponent::new(revs[1], handles[0], handles[1]),));
        let e3 =
            all_storages.add_entity((JointBuilderComponent::new(revs[2], handles[1], handles[2]),));
        let e4 =
            all_storages.add_entity((JointBuilderComponent::new(revs[3], handles[2], handles[3]),));

        if i == 0 {
            despawn_entities.push(e1);
            despawn_entities.push(e2);
            despawn_entities.push(e3);
            despawn_entities.push(e4);
        }

        curr_parent = handles[3];
    }
    let mut despawn = all_storages
        .borrow::<UniqueViewMut<DespawnResource>>()
        .unwrap();
    despawn.entities.append(&mut despawn_entities);
}

fn create_fixed_joints((origin, num): (Point3<f32>, usize), mut all_storages: AllStoragesViewMut) {
    let mut despawn_entities = vec![];
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let status = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(
                origin.x + fk * shift,
                origin.y,
                origin.z + fi * shift,
            );
            let collider = ColliderBuilder::ball(rad).density(1.0);
            let child_entity = all_storages.add_entity((rigid_body, collider));

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(0.0, 0.0, -shift),
                );
                all_storages.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(-shift, 0.0, 0.0),
                );
                let entity = all_storages.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
                if k == 2 {
                    despawn_entities.push(entity);
                }
            }

            body_entities.push(child_entity);
        }
    }
    let mut despawn = all_storages
        .borrow::<UniqueViewMut<DespawnResource>>()
        .unwrap();
    despawn.entities.append(&mut despawn_entities);
}

fn create_ball_joints(num: usize, mut all_storages: AllStoragesViewMut) {
    let mut despawn_entities = vec![];
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(fk * shift, 0.0, fi * shift);
            let collider = ColliderBuilder::ball(rad).density(1.0);
            let child_entity = all_storages.add_entity((collider, rigid_body));

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = BallJoint::new(Point3::origin(), Point3::new(0.0, 0.0, -shift));
                let entity = all_storages.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));

                if i == 2 {
                    despawn_entities.push(entity);
                }
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point3::origin(), Point3::new(-shift, 0.0, 0.0));
                all_storages.add_entity((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            body_entities.push(child_entity);
        }
    }
    let mut despawn = all_storages
        .borrow::<UniqueViewMut<DespawnResource>>()
        .unwrap();
    despawn.entities.append(&mut despawn_entities);
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
