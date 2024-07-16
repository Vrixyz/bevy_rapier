use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::dynamics::RapierRigidBodyHandle;
use crate::dynamics::RigidBody;
use crate::geometry::Collider;
use crate::geometry::ColliderDisabled;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::systemparams::RapierContextAccessMut;
use crate::plugin::RapierContext;
use crate::prelude::MassModifiedEvent;
use crate::prelude::RigidBodyDisabled;
use crate::prelude::Sensor;
use bevy::ecs::query::QueryData;
use bevy::prelude::*;

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    //mut context_accessor: RapierContextAccessMut,
    mut context_accessor: RapierContextAccessMutEntity,
    mut removed_bodies: RemovedComponents<RapierRigidBodyHandle>,
    mut removed_colliders: RemovedComponents<RapierColliderHandle>,
    mut removed_impulse_joints: RemovedComponents<RapierImpulseJointHandle>,
    mut removed_multibody_joints: RemovedComponents<RapierMultibodyJointHandle>,
    orphan_bodies: Query<Entity, (With<RapierRigidBodyHandle>, Without<RigidBody>)>,
    orphan_colliders: Query<Entity, (With<RapierColliderHandle>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<RapierImpulseJointHandle>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<
        Entity,
        (With<RapierMultibodyJointHandle>, Without<MultibodyJoint>),
    >,

    mut removed_sensors: RemovedComponents<Sensor>,
    mut removed_rigid_body_disabled: RemovedComponents<RigidBodyDisabled>,
    mut removed_colliders_disabled: RemovedComponents<ColliderDisabled>,

    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    /*
     * Rigid-bodies removal detection.
     */
    for entity in removed_bodies.read() {
        let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2body.remove(&entity)
        }) else {
            dbg!("no context1!");
            continue;
        };
        let context = &mut *context.1;

        let _ = context.last_body_transform_set.remove(&handle);
        context.bodies.remove(
            handle,
            &mut context.islands,
            &mut context.colliders,
            &mut context.impulse_joints,
            &mut context.multibody_joints,
            false,
        );
    }

    for entity in orphan_bodies.iter() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2body.remove(&entity)
        }) {
            let context = &mut *context.1;
            let _ = context.last_body_transform_set.remove(&handle);
            context.bodies.remove(
                handle,
                &mut context.islands,
                &mut context.colliders,
                &mut context.impulse_joints,
                &mut context.multibody_joints,
                false,
            );
        }
        commands.entity(entity).remove::<RapierRigidBodyHandle>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders.read() {
        let Some((mut context_accessed, handle)) =
            find_context_entity(&mut context_accessor, |context| {
                context.entity2collider.remove(&entity)
            })
        else {
            dbg!("no context2!");
            continue;
        };
        let context = &mut *context_accessed.1;
        if let Some(parent) = context.collider_parent(entity) {
            mass_modified.send(parent.into());
        }

        context
            .colliders
            .remove(handle, &mut context.islands, &mut context.bodies, true);

        println!("removed {handle:?} from {}", context_accessed.0);
        context.deleted_colliders.insert(handle, entity);
    }

    for entity in orphan_colliders.iter() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2collider.remove(&entity)
        }) {
            let context = &mut *context.1;
            if let Some(parent) = context.collider_parent(entity) {
                mass_modified.send(parent.into());
            }

            context
                .colliders
                .remove(handle, &mut context.islands, &mut context.bodies, true);
            context.deleted_colliders.insert(handle, entity);
        }
        commands.entity(entity).remove::<RapierColliderHandle>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints.read() {
        let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2impulse_joint.remove(&entity)
        }) else {
            continue;
        };
        let context = &mut *context.1;
        context.impulse_joints.remove(handle, true);
    }

    for entity in orphan_impulse_joints.iter() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2impulse_joint.remove(&entity)
        }) {
            let context = &mut *context.1;
            context.impulse_joints.remove(handle, true);
        }
        commands.entity(entity).remove::<RapierImpulseJointHandle>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints.read() {
        let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2multibody_joint.remove(&entity)
        }) else {
            continue;
        };
        let context = &mut *context.1;
        context.multibody_joints.remove(handle, true);
    }

    for entity in orphan_multibody_joints.iter() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2multibody_joint.remove(&entity)
        }) {
            let context = &mut *context.1;
            context.multibody_joints.remove(handle, true);
        }
        commands
            .entity(entity)
            .remove::<RapierMultibodyJointHandle>();
    }

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors.read() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_sensor(false);
            }
        }
    }

    for entity in removed_colliders_disabled.read() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_enabled(true);
            }
        }
    }

    for entity in removed_rigid_body_disabled.read() {
        if let Some((mut context, handle)) = find_context_entity(&mut context_accessor, |context| {
            context.entity2body.get(&entity).copied()
        }) {
            if let Some(rb) = context.1.bodies.get_mut(handle) {
                rb.set_enabled(true);
            }
        }
    }

    // TODO: what about removing forces?
}

fn find_context<'a, T>(
    context_accessor: &'a mut RapierContextAccessMut,
    item_finder: impl Fn(&mut RapierContext) -> Option<T>,
) -> Option<(Mut<'a, RapierContext>, T)> {
    context_accessor
        .rapier_context
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context).map(|handle| (context, handle)))
}

/// Utility [`SystemParam`] to easily access any [`RapierContext`] mutably
#[derive(bevy::ecs::system::SystemParam)]
pub struct RapierContextAccessMutEntity<'w, 's> {
    /// Query used to retrieve a [`RapierContext`].
    /// It's helpful to iterate over every rapier contexts,
    /// or get a handle over a specific context, for example through:
    /// - a marker component such as [`DefaultRapierContext`]
    /// - a [`RapierContextEntityLink`]. See [context](RapierContextAccess::context)
    pub rapier_context: Query<'w, 's, (Entity, &'static mut RapierContext)>,
}

fn find_context_entity<'a, T>(
    context_accessor: &'a mut RapierContextAccessMutEntity,
    item_finder: impl Fn(&mut RapierContext) -> Option<T>,
) -> Option<((Entity, Mut<'a, RapierContext>), T)> {
    context_accessor
        .rapier_context
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context.1).map(|handle| (context, handle)))
}

fn find_context_generic<'a, 'w, 's, P: QueryData, T>(
    context_accessor: &'a mut Query<'w, 's, P>,
    item_finder: impl Fn(&mut <P as bevy::ecs::query::WorldQuery>::Item<'a>) -> Option<T>,
) -> Option<(<P as bevy::ecs::query::WorldQuery>::Item<'a>, T)> {
    context_accessor
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context).map(|handle| (context, handle)))
}
