//! ```plantuml
//!
//! flowchart TD
//!
//! A[Set to allow user to change physics data] --> COPY(copy physics into background)
//! COPY --> FORK@{ shape: fork, label: "Start bevy task" }
//! FORK --> Loop{elapsed time}
//! FORK --> SIM["rapier physics simulation (50ms ; 20fps)"]
//! Loop -->|"<50ms"| BU["bevy update (16.6ms ; 60FPS)"]
//! BU --> Loop
//! Loop -->|"\>=50ms"| J(join)
//! SIM --> J@{ shape: fork, label: "Join bevy task" }
//! J --> WRITE(Write physics from background task into bevy)
//! WRITE --> A
//!
//! ```

use std::{mem, time};

use bevy::{
    ecs::system::{StaticSystemParam, SystemParamItem},
    prelude::*,
    tasks::{block_on, futures_lite::future, AsyncComputeTaskPool, Task},
};
use rapier::prelude::*;
use std::time::Duration;

use crate::{
    pipeline::{CollisionEvent, ContactForceEvent},
    plugin::context,
    prelude::{
        RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierQueryPipeline,
        RapierRigidBodySet,
    },
};
use crate::{
    plugin::{RapierConfiguration, RapierContext, SimulationToRenderTime, TimestepMode},
    prelude::{BevyPhysicsHooks, RapierRigidBodyHandle, TransformInterpolation},
};
use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError};

use super::BevyPhysicsHooksAdapter;

pub struct SimulationTaskResult {
    pub context: RapierContextSimulation,
    pub bodies: RapierRigidBodySet,
    pub colliders: RapierContextColliders,
    pub joints: RapierContextJoints,
    pub query_pipeline: RapierQueryPipeline,
    /// The duration in seconds **simulated** by the simulation.
    ///
    /// This is different from the real time it took to simulate the physics.
    ///
    /// It is needed to synchronize the simulation with the render time.
    pub simulated_time: f32,
}

/// A component that holds a Rapier simulation task.
///
/// The task inside this component is polled by the system [`handle_tasks`].
///
/// It is unsafe to access the [`RapierContext`] from the same entity.
///
/// This component is removed when it's safe to access the [`RapierContext`] again.
#[derive(Component)]
pub struct SimulationTask {
    /// The time at which we started the simulation, as reported by the used render time [`Time::elapsed`].
    pub started_at_render_time: Duration,
    /// Amount of frames elapsed since the simulation started.
    pub render_frames_elapsed: u64,
    /// The channel end to receive the simulation result.
    pub recv: Receiver<SimulationTaskResult>,
}

/// This system queries for [`RapierContext`] that have our `Task<RapierSimulation>` component. It polls the
/// tasks to see if they're complete. If the task is complete it sends rapier'sbevy events and
/// removes the [`SimulationTask`] component from the entity.
pub(crate) fn handle_tasks(
    mut commands: Commands,
    mut time: Res<Time>,
    mut q_context: Query<(
        &mut RapierContextSimulation,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierQueryPipeline,
        &mut SimulationToRenderTime,
    )>,
    mut simulation_tasks: Query<(Entity, &mut SimulationTask)>,
    mut collision_events: EventWriter<CollisionEvent>,
    mut contact_force_events: EventWriter<ContactForceEvent>,
) {
    for (entity, mut task) in &mut simulation_tasks {
        let mut handle_result = |result: SimulationTaskResult| {
            let (
                mut context,
                mut bodies,
                mut colliders,
                mut query_pipeline,
                mut sim_to_render_time,
            ) = q_context.get_mut(entity).unwrap();
            // TODO: mem swap with a buffer
            *context = result.context;
            *bodies = result.bodies;
            *colliders = result.colliders;
            *query_pipeline = result.query_pipeline;
            sim_to_render_time.diff -= result.simulated_time;
            sim_to_render_time.diff += (time.elapsed() - task.started_at_render_time).as_secs_f32();
            if (sim_to_render_time.diff < 0f32) {
                // The simulation is behind the render time. The simulation slows down,
                // the strategy to handle this could be to :
                // - run more steps per simulation frame ( avoiding bevy overhead, synchronization, etc)
                // - reduce the quality of the simulation (reduce substeps, ccd...)
                // - allow a time drift: accept the simulation has slowed down, and don't attempt to catch up.
                // For now, we just reset the diff to 0, effectively allowing a time drift,
                // but ideally, we should report that to the user.
                sim_to_render_time.diff = 0f32;
            }
            context.send_bevy_events(&mut collision_events, &mut contact_force_events);
            commands.entity(entity).remove::<SimulationTask>();
        };
        // TODO: configure this somehow.
        if task.render_frames_elapsed > 20 {
            // Do not tolerate more delay over the rendering: block on the result of the simulation.
            if let Some(result) = task.recv.recv().ok() {
                handle_result(result);
            }
        } else {
            //if let Some(mut result) = block_on(future::poll_once(&mut task.0)) {
            if let Some(result) = task.recv.try_recv().ok() {
                handle_result(result);
            }
        }
        task.render_frames_elapsed += 1;
    }
}

/// This system generates tasks simulating computationally intensive
/// work that potentially spans multiple frames/ticks. A separate
/// system, [`handle_tasks`], will poll the spawned tasks on subsequent
/// frames/ticks, and use the results to spawn cubes
pub(crate) fn spawn_simulation_task<Hooks>(
    mut commands: Commands,
    mut q_context: Query<
        (
            Entity,
            &mut RapierContextSimulation,
            &RapierContextColliders,
            &RapierRigidBodySet,
            &RapierContextJoints,
            &RapierQueryPipeline,
            &RapierConfiguration,
            &mut SimulationToRenderTime,
        ),
        Without<SimulationTask>,
    >,
    timestep_mode: Res<TimestepMode>,
    hooks: StaticSystemParam<Hooks>,
    time: Res<Time>,
) where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, Hooks>: BevyPhysicsHooks,
{
    //let hooks_adapter = BevyPhysicsHooksAdapter::new(hooks.into_inner());
    //let hooks_adapter = BevyPhysicsHooksAdapter::new(());

    for (
        entity,
        mut context_ecs,
        colliders,
        bodies,
        joints,
        query_pipeline,
        config,
        sim_to_render_time,
    ) in q_context.iter_mut()
    {
        // FIXME: Clone this properly?
        let mut context = RapierContextSimulation::default();
        mem::swap(&mut context, &mut *context_ecs);
        // TODO: use a double buffering system to avoid this more expensive (to verify) cloning.
        let mut colliders = colliders.clone();
        let mut bodies = bodies.clone();
        let mut joints = joints.clone();
        let mut query_pipeline = query_pipeline.clone();
        // let mut context: RapierContext =
        //    unsafe { mem::transmute_copy::<RapierContext, RapierContext>(&*context_ecs) };
        let config = config.clone();
        let timestep_mode = timestep_mode.clone();
        let time = time.clone();
        let mut sim_to_render_time = sim_to_render_time.clone();

        let (sender, recv) = crossbeam_channel::unbounded();

        let thread_pool = AsyncComputeTaskPool::get();
        let task = thread_pool
            .spawn(async move {
                async_std::task::sleep(Duration::from_millis(1)).await;
                let time_to_catch_up = sim_to_render_time.diff;
                let mut simulated_time = 0f32;
                if config.physics_pipeline_active {
                    profiling::scope!("Rapier physics simulation");
                    let max_tries_to_catch_up = 3;
                    for i in 0..max_tries_to_catch_up {
                        if time_to_catch_up < simulated_time {
                            break;
                        }
                        simulated_time += context.step_simulation(
                            &mut colliders,
                            &mut joints,
                            &mut bodies,
                            config.gravity,
                            timestep_mode,
                            true,
                            &(), // FIXME: &hooks_adapter,
                            &time,
                            &mut sim_to_render_time,
                            None,
                        );
                    }
                } else {
                    bodies.propagate_modified_body_positions_to_colliders(&mut colliders);
                }

                if config.query_pipeline_active {
                    query_pipeline.update_query_pipeline(&colliders);
                }
                let result = SimulationTaskResult {
                    context,
                    bodies,
                    joints,
                    colliders,
                    query_pipeline,
                    simulated_time: simulated_time,
                };
                sender.send(result);
            })
            .detach();

        commands.entity(entity).insert(SimulationTask {
            recv,
            started_at_render_time: time.elapsed(),
            render_frames_elapsed: 0,
        });
    }
}
