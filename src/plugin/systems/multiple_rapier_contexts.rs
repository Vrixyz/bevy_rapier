//! systems to support multiple worlds, and changes between them.

use crate::dynamics::{
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle,
};
use crate::geometry::RapierColliderHandle;
use crate::plugin::{RapierContext, RapierContextEntityLink};
use bevy::prelude::*;

/// If an entity is turned into the child of something with a physics context link,
/// the child should become a part of that physics context
///
/// If this fails to happen, weirdness will ensue.
pub fn on_add_entity_with_parent(
    q_add_entity_without_parent: Query<
        (Entity, &Parent),
        (
            With<RapierContextEntityLink>,
            Or<(Changed<RapierContextEntityLink>, Changed<Parent>)>,
        ),
    >,
    q_parent: Query<&Parent>,
    q_physics_world: Query<&RapierContextEntityLink>,
    mut commands: Commands,
) {
    for (ent, parent) in &q_add_entity_without_parent {
        let mut parent = Some(parent.get());
        while let Some(parent_entity) = parent {
            if let Ok(pw) = q_physics_world.get(parent_entity) {
                if q_physics_world.get(ent).map(|x| x != pw).unwrap_or(true) {
                    info!("Removing old physx for {ent:?}!");
                    remove_old_physics(ent, &mut commands);
                    commands.entity(ent).insert(*pw);
                }
                break;
            }
            parent = q_parent.get(parent_entity).ok().map(|x| x.get());
        }
    }
}

/// Flags the entity to have its old physics removed
fn remove_old_physics(entity: Entity, commands: &mut Commands) {
    commands
        .entity(entity)
        .remove::<RapierColliderHandle>()
        .remove::<RapierRigidBodyHandle>()
        .remove::<RapierMultibodyJointHandle>()
        .remove::<RapierImpulseJointHandle>();
}

/// Flags the entity to have its physics updated to reflect new world
///
/// Also recursively bubbles down world changes to children & flags them to apply any needed physics changes
pub fn on_change_world(
    q_changed_worlds: Query<
        (Entity, Ref<RapierContextEntityLink>),
        Changed<RapierContextEntityLink>,
    >,
    q_children: Query<&Children>,
    q_physics_world: Query<&RapierContextEntityLink>,
    q_context: Query<&RapierContext>,
    mut commands: Commands,
) {
    for (entity, new_physics_world) in &q_changed_worlds {
        let context = q_context.get(new_physics_world.0);
        // Ensure the world actually changed before removing them from the world
        if !context
            .map(|x| {
                // They are already apart of this world if any of these are true
                x.entity2collider.contains_key(&entity)
                    || x.entity2body.contains_key(&entity)
                    || x.entity2impulse_joint.contains_key(&entity)
                    || x.entity2multibody_joint.contains_key(&entity)
            })
            .unwrap_or(false)
        {
            remove_old_physics(entity, &mut commands);
            bubble_down_world_change(
                &mut commands,
                entity,
                &q_children,
                *new_physics_world,
                &q_physics_world,
            );
        }
    }
}

fn bubble_down_world_change(
    commands: &mut Commands,
    entity: Entity,
    q_children: &Query<&Children>,
    new_physics_world: RapierContextEntityLink,
    q_physics_world: &Query<&RapierContextEntityLink>,
) {
    let Ok(children) = q_children.get(entity) else {
        return;
    };

    children.iter().for_each(|&child| {
        if q_physics_world
            .get(child)
            .map(|x| *x == new_physics_world)
            .unwrap_or(false)
        {
            return;
        }
        println!("bubbling down for {child}");

        remove_old_physics(child, commands);
        commands.entity(child).insert(new_physics_world);

        bubble_down_world_change(
            commands,
            child,
            q_children,
            new_physics_world,
            q_physics_world,
        );
    });
}

#[cfg(test)]
mod test {
    use crate::plugin::systems::tests::HeadlessRenderPlugin;
    use crate::plugin::{
        systems, DefaultRapierContext, NoUserData, PhysicsSet, RapierContext,
        RapierContextEntityLink, RapierContextInitialization, RapierPhysicsPlugin, TimestepMode,
    };
    use crate::prelude::{ActiveEvents, Collider, ContactForceEventThreshold, RigidBody, Sensor};
    use bevy::ecs::schedule::Stepping;
    use bevy::prelude::*;
    use bevy::tasks::{IoTaskPool, TaskPoolBuilder};
    use bevy::time::{TimePlugin, TimeUpdateStrategy};
    use rapier::math::Real;

    #[test]
    pub fn multi_world_hierarchy_update() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .add_systems(
            PostUpdate,
            setup_physics
                .run_if(run_once())
                .before(PhysicsSet::SyncBackend),
        );
        // Simulates 60 updates per seconds
        app.insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));
        app.update();
        // Verify all rapier entities have a `RapierContextEntityLink`.
        let world = app.world_mut();
        let mut query = world.query_filtered::<Entity, With<Marker<'R'>>>();
        for entity in query.iter(&world) {
            world
                .get::<RapierContextEntityLink>(entity)
                .unwrap_or_else(|| panic!("no link to rapier context entity from {entity}."));
        }
        // Verify link is correctly updated for children.
        let new_rapier_context = world.spawn(RapierContext::default()).id();
        // FIXME: We need to wait 1 frame when creating a world.
        // Ideally we should be able to order the systems so that we don't have to wait.
        app.update();
        let mut world = app.world_mut();
        let mut query = world.query_filtered::<&mut RapierContextEntityLink, With<Marker<'P'>>>();
        let mut link_parent = query.get_single_mut(&mut world).unwrap();
        link_parent.0 = new_rapier_context;
        app.update();
        let mut world = app.world_mut();
        let mut query = world.query_filtered::<&RapierContextEntityLink, With<Marker<'C'>>>();
        let link_child = query.get_single_mut(&mut world).unwrap();
        assert_eq!(link_child.0, new_rapier_context);
        return;

        #[derive(Component)]
        pub struct Marker<const MARKER: char>;

        #[cfg(feature = "dim3")]
        fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
            Collider::cuboid(hx, hy, hz)
        }
        #[cfg(feature = "dim2")]
        fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
            Collider::cuboid(hx, hy)
        }
        pub fn setup_physics(mut commands: Commands) {
            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, -1.2, 0.0)),
                cuboid(4.0, 1.0, 1.0),
                Marker::<'R'>,
            ));

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, 5.0, 0.0)),
                cuboid(4.0, 1.5, 1.0),
                Sensor,
                Marker::<'R'>,
            ));

            commands
                .spawn((
                    TransformBundle::from(Transform::from_xyz(0.0, 13.0, 0.0)),
                    RigidBody::Dynamic,
                    cuboid(0.5, 0.5, 0.5),
                    ActiveEvents::COLLISION_EVENTS,
                    ContactForceEventThreshold(30.0),
                    Marker::<'P'>,
                    Marker::<'R'>,
                ))
                .with_children(|child_builder| {
                    child_builder.spawn((
                        TransformBundle::from(Transform::from_xyz(0.0, -1.2, 0.0)),
                        cuboid(4.0, 1.0, 1.0),
                        Marker::<'C'>,
                        Marker::<'R'>,
                    ));
                });
        }
    }

    #[test]
    pub fn spam_change_context_interpolated() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            MinimalPlugins,
            TransformPlugin,
            RapierPhysicsPlugin::<NoUserData>::default()
                .with_custom_initialization(RapierContextInitialization::NoAutomaticRapierContext),
        ))
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 2,
        })
        .add_systems(Startup, (create_worlds, setup_physics).chain())
        .add_systems(Update, change_world);
        // Simulates 60 updates per seconds
        //app.insert_resource(TimeUpdateStrategy::ManualDuration(
        //    std::time::Duration::from_secs_f32(1f32 / 60f32),
        //));
        // First update for setups, + creating default rapier context.

        //app.run();
        //return;

        app.update();
        let world = app.world_mut();

        let mut query = world.query_filtered::<Entity, With<DefaultRapierContext>>();
        let default_rapier_context = query.single(world);
        let mut query =
            world.query_filtered::<Entity, (With<RapierContext>, Without<DefaultRapierContext>)>();
        let new_rapier_context = query.single(world);

        let mut stepping = Stepping::new();
        stepping
            .add_schedule(PostUpdate)
            .enable()
            .set_breakpoint(PostUpdate, systems::sync_removals)
            .set_breakpoint(PostUpdate, systems::init_rigid_bodies);
        app.insert_resource(stepping);

        let (mut context_with_entities, mut empty_context) =
            (new_rapier_context, default_rapier_context);
        for i in 0..1000 {
            // swap contexts
            (context_with_entities, empty_context) = (empty_context, context_with_entities);

            let mut stepping = app.world_mut().resource_mut::<Stepping>();
            stepping.continue_frame();
            // Breaking before sync_removals.
            app.update();
            // TODO: check that there are no deletect colliders, step once or twice (apply deferred), check there are

            let world = app.world_mut();
            let mut query = world.query::<&RapierContext>();
            let context_with_entities_data = query.get(world, context_with_entities).unwrap();
            assert_eq!(context_with_entities_data.deleted_colliders.len(), 0);

            let mut stepping = app.world_mut().resource_mut::<Stepping>();
            stepping.continue_frame();
            // Stepping after sync_removals.
            app.update();
            let world = app.world_mut();
            let mut query = world.query::<&RapierContext>();
            let context_with_entities_data = query.get(world, context_with_entities).unwrap();
            if context_with_entities_data.deleted_colliders.len() != 2 {
                println!("This will crash! (rapier context: {context_with_entities})")
            }
            if context_with_entities_data.deleted_colliders.len() != 2 {
                println!("This will crash! (rapier context: {context_with_entities})")
            }
            println!("{:?}", context_with_entities_data.colliders);
            let mut stepping = app.world_mut().resource_mut::<Stepping>();
            stepping.continue_frame();

            // Finishing the frame.
            app.update();

            let mut world = app.world_mut();
            let mut query =
                world.query_filtered::<&RapierContextEntityLink, With<MarkerChangeWorld>>();
            for link in query.iter(&mut world) {
                assert_eq!(link.0, context_with_entities);
                // TODO: stepping + verify deleted colliders
            }
        }
        return;

        #[derive(Component)]
        pub struct MarkerChangeWorld;

        #[cfg(feature = "dim3")]
        fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
            Collider::cuboid(hx, hy, hz)
        }
        #[cfg(feature = "dim2")]
        fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
            Collider::cuboid(hx, hy)
        }
        fn create_worlds(mut commands: Commands) {
            for i in 0..2 {
                let mut world = commands.spawn(RapierContext::default());
                if i == 0 {
                    world.insert(DefaultRapierContext);
                }
            }
        }
        pub fn setup_physics(mut commands: Commands) {
            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                ActiveEvents::all(),
            ));
            /*
             * Ground
             */
            let ground_size = 5.1;
            let ground_height = 0.1;
            let starting_y = -0.5 - ground_height;

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, starting_y, 0.0)),
                cuboid(ground_size, ground_height, ground_size),
            ));
        }
        fn change_world(
            query_context: Query<Entity, With<DefaultRapierContext>>,
            q_other_ctx: Query<Entity, (With<RapierContext>, Without<DefaultRapierContext>)>,
            mut query_links: Query<&mut RapierContextEntityLink>,
        ) {
            let default_context = query_context.single();
            for mut link in query_links.iter_mut() {
                if link.0 == default_context {
                    link.0 = q_other_ctx.single();
                    continue;
                }
                link.0 = default_context;
            }
        }
    }
}
