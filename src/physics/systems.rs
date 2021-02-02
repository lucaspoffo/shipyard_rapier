use crate::physics::{
    ColliderHandleComponent, EntityMaps, EventQueue, InteractionPairFilters, JointBuilderComponent,
    JointHandleComponent, PhysicsInterpolationComponent, RapierConfiguration,
    RigidBodyHandleComponent, SimulationToRenderTime,
};

use crate::rapier::pipeline::QueryPipeline;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
use rapier::pipeline::PhysicsPipeline;

use shipyard::{
    EntitiesView, Get, IntoIter, IntoWithId, UniqueView, UniqueViewMut, View, ViewMut, World,
};

pub fn setup(world: &mut World) {
    world.add_unique(PhysicsPipeline::new()).unwrap();
    world.add_unique(QueryPipeline::new()).unwrap();
    world.add_unique(RapierConfiguration::default()).unwrap();
    world.add_unique(IntegrationParameters::default()).unwrap();
    world.add_unique(BroadPhase::new()).unwrap();
    world.add_unique(NarrowPhase::new()).unwrap();
    world.add_unique(RigidBodySet::new()).unwrap();
    world.add_unique(ColliderSet::new()).unwrap();
    world.add_unique(JointSet::new()).unwrap();
    world.add_unique(InteractionPairFilters::new()).unwrap();
    world.add_unique(EventQueue::new(true)).unwrap();
    world.add_unique(SimulationToRenderTime::default()).unwrap();
    world.add_unique(EntityMaps::default()).unwrap();
}

/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn create_body_and_collider_system(
    entities: EntitiesView,
    mut bodies: UniqueViewMut<RigidBodySet>,
    mut colliders: UniqueViewMut<ColliderSet>,
    mut entity_maps: UniqueViewMut<EntityMaps>,
    mut rigid_body_builders: ViewMut<RigidBodyBuilder>,
    mut rigid_body_handles: ViewMut<RigidBodyHandleComponent>,
    mut collider_builders: ViewMut<ColliderBuilder>,
    mut collider_handles: ViewMut<ColliderHandleComponent>,
) {
    let mut rigid_bodies_builder_deleted = vec![];
    let mut colliders_builder_deleted = vec![];

    for (entity_id, body_builder) in rigid_body_builders.iter().with_id() {
        let handle = bodies.insert(body_builder.build());
        entities.add_component(
            entity_id,
            &mut rigid_body_handles,
            RigidBodyHandleComponent::from(handle),
        );
        rigid_bodies_builder_deleted.push(entity_id);
        entity_maps.bodies.insert(entity_id, handle);

        if let Ok(collider_builder) = collider_builders.get(entity_id) {
            let collider = collider_builder.build();
            let handle = colliders.insert(collider, handle, &mut bodies);
            entities.add_component(
                entity_id,
                &mut collider_handles,
                ColliderHandleComponent::from(handle),
            );
            colliders_builder_deleted.push(entity_id);
            entity_maps.colliders.insert(entity_id, handle);
        }
    }

    for entity_id in rigid_bodies_builder_deleted.iter() {
        rigid_body_builders.delete(*entity_id);
    }

    for entity_id in colliders_builder_deleted.iter() {
        collider_builders.delete(*entity_id);
    }

    /* No parents in shipyard
    for (entity, parent, collider_builder) in parented_collider_query.iter() {
        if let Some(body_handle) = entity_maps.bodies.get(&parent.0) {
            let handle = colliders.insert(collider_builder.build(), *body_handle, &mut bodies);
            commands.insert_one(entity, ColliderHandleComponent::from(handle));
            commands.remove_one::<ColliderBuilder>(entity);
            entity_maps.colliders.insert(entity, handle);
        }
    } */
}

#[test]
fn test_create_body_and_collider_system() {
    use shipyard::*;

    let mut world = World::new();

    world.add_unique(RigidBodySet::new()).unwrap();
    world.add_unique(ColliderSet::new()).unwrap();
    world.add_unique(EntityMaps::default()).unwrap();

    let body_and_collider_entity =
        world.add_entity((RigidBodyBuilder::new_dynamic(), ColliderBuilder::ball(1.0)));

    let body_only_entity = world.add_entity((RigidBodyBuilder::new_static(),));

    world.run(create_body_and_collider_system).unwrap();

    let body_set = world.borrow::<UniqueView<RigidBodySet>>().unwrap();
    let collider_set = world.borrow::<UniqueView<ColliderSet>>().unwrap();
    let entity_maps = world.borrow::<UniqueView<EntityMaps>>().unwrap();

    let (rigid_bodies_handles, colliders_handles) = world
        .borrow::<(
            ViewMut<RigidBodyHandleComponent>,
            ViewMut<ColliderHandleComponent>,
        )>()
        .unwrap();

    // body attached alongside collider
    let attached_body_handle = rigid_bodies_handles
        .get(body_and_collider_entity)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.bodies[&body_and_collider_entity],
        attached_body_handle
    );
    assert!(body_set.get(attached_body_handle).unwrap().is_dynamic());

    // collider attached from same entity
    let collider_handle = colliders_handles
        .get(body_and_collider_entity)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.colliders[&body_and_collider_entity],
        collider_handle
    );
    let collider = collider_set.get(collider_handle).unwrap();
    assert_eq!(attached_body_handle, collider.parent());
    assert_eq!(collider.shape().as_ball().unwrap().radius, 1.0);

    // standalone body with no collider, jointed to the attached body
    let standalone_body_handle = rigid_bodies_handles.get(body_only_entity).unwrap().handle();
    assert_eq!(
        entity_maps.bodies[&body_only_entity],
        standalone_body_handle
    );
    assert!(body_set.get(standalone_body_handle).unwrap().is_static());
}

/// System responsible for creating Rapier joints from their builder resources.
pub fn create_joints_system(
    entities: EntitiesView,
    mut bodies: UniqueViewMut<RigidBodySet>,
    mut joints: UniqueViewMut<JointSet>,
    mut entity_maps: UniqueViewMut<EntityMaps>,
    mut joint_builders: ViewMut<JointBuilderComponent>,
    mut joint_handles: ViewMut<JointHandleComponent>,
    bodies_handles: View<RigidBodyHandleComponent>,
) {
    let mut joint_builders_deleted = vec![];
    for (entity_id, joint_builder) in joint_builders.iter().with_id() {
        let body1 = bodies_handles.get(joint_builder.entity1);
        let body2 = bodies_handles.get(joint_builder.entity2);
        if let (Ok(body1), Ok(body2)) = (body1, body2) {
            let handle = joints.insert(
                &mut bodies,
                body1.handle(),
                body2.handle(),
                joint_builder.params,
            );
            entities.add_component(
                entity_id,
                &mut joint_handles,
                JointHandleComponent::new(handle, joint_builder.entity1, joint_builder.entity2),
            );

            joint_builders_deleted.push(entity_id);
            entity_maps.joints.insert(entity_id, handle);
        }
    }

    for entity_id in joint_builders_deleted.iter() {
        joint_builders.delete(*entity_id);
    }
}

/// System responsible for performing one timestep of the physics world.
pub fn step_world_system(
    delta_seconds: f32,
    mut sim_to_render_time: UniqueViewMut<SimulationToRenderTime>,
    (configuration, integration_parameters): (
        UniqueView<RapierConfiguration>,
        UniqueView<IntegrationParameters>,
    ),
    filter: UniqueView<InteractionPairFilters>,
    (mut pipeline, mut query_pipeline): (
        UniqueViewMut<PhysicsPipeline>,
        UniqueViewMut<QueryPipeline>,
    ),
    (mut broad_phase, mut narrow_phase): (UniqueViewMut<BroadPhase>, UniqueViewMut<NarrowPhase>),
    (mut bodies, mut colliders): (UniqueViewMut<RigidBodySet>, UniqueViewMut<ColliderSet>),
    mut joints: UniqueViewMut<JointSet>,
    events: UniqueViewMut<EventQueue>,
    (rigid_bodies_handles, mut physics_interpolation): (
        View<RigidBodyHandleComponent>,
        ViewMut<PhysicsInterpolationComponent>,
    ),
) {
    if events.auto_clear {
        events.clear();
    }

    if configuration.time_dependent_number_of_timesteps {
        sim_to_render_time.diff += delta_seconds;

        let sim_dt = integration_parameters.dt;
        while sim_to_render_time.diff >= sim_dt {
            if configuration.physics_pipeline_active {
                // NOTE: in this comparison we do the same computations we
                // will do for the next `while` iteration test, to make sure we
                // don't get bit by potential float inaccuracy.
                if sim_to_render_time.diff - sim_dt < sim_dt {
                    // This is the last simulation step to be executed in the loop
                    // Update the previous state transforms
                    // for (body_handle, mut previous_state) in query.iter_mut() {
                    for (body_handle, mut previous_state) in
                        (&rigid_bodies_handles, &mut physics_interpolation).iter()
                    {
                        if let Some(body) = bodies.get(body_handle.handle()) {
                            previous_state.0 = Some(*body.position());
                        }
                    }
                }
                pipeline.step(
                    &configuration.gravity,
                    &integration_parameters,
                    &mut broad_phase,
                    &mut narrow_phase,
                    &mut bodies,
                    &mut colliders,
                    &mut joints,
                    filter.contact_filter.as_deref(),
                    filter.intersection_filter.as_deref(),
                    &*events,
                );
            }
            sim_to_render_time.diff -= sim_dt;
        }
    } else if configuration.physics_pipeline_active {
        pipeline.step(
            &configuration.gravity,
            &integration_parameters,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut joints,
            filter.contact_filter.as_deref(),
            filter.intersection_filter.as_deref(),
            &*events,
        );
    }

    if configuration.query_pipeline_active {
        query_pipeline.update(&mut bodies, &colliders);
    }
}

/*
/// System responsible for removing joints, colliders, and bodies that have
/// been removed from the scene
pub fn destroy_body_and_collider_system(
    mut bodies: UniqueViewMut<RigidBodySet>,
    mut colliders: UniqueViewMut<ColliderSet>,
    mut joints: UniqueViewMut<JointSet>,
    mut entity_maps: UniqueViewMut<EntityMaps>,
    collider_handles: ViewMut<ColliderHandleComponent>,
    joint_handles: ViewMut<JointHandleComponent>,
    body_handles: ViewMut<RigidBodyHandleComponent>,
) {
    // Components removed before this system
    let bodies_removed = body_query.removed::<RigidBodyHandleComponent>();
    let colliders_removed = collider_query.removed::<ColliderHandleComponent>();
    let joints_removed = joint_query.removed::<JointHandleComponent>();

    for entity in bodies_removed {
        if let Some(body_handle) = entity_maps.bodies.get(entity) {
            bodies.remove(*body_handle, &mut colliders, &mut joints);
            entity_maps.bodies.remove(entity);

            // Removing a body also removes its colliders and joints. If they were
            // not also removed then we must remove them here.
            commands.remove_one::<ColliderHandleComponent>(*entity);
            entity_maps.colliders.remove(entity);
            commands.remove_one::<JointHandleComponent>(*entity);
            entity_maps.joints.remove(entity);
        }
    }
    for entity in colliders_removed {
        if let Some(collider_handle) = entity_maps.colliders.get(entity) {
            colliders.remove(*collider_handle, &mut bodies, true);
            entity_maps.colliders.remove(entity);
        }
    }
    for entity in joints_removed {
        if let Some(joint_handle) = entity_maps.joints.get(entity) {
            joints.remove(*joint_handle, &mut bodies, true);
            entity_maps.joints.remove(entity);
        }
    }
}
*/
