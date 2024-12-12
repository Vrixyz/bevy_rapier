//! This example should be run with features bevy/multi_threaded and bevy_rapier2d/background_simulation

use std::{fs::File, io::Write};

use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::{color::palettes, prelude::*};
use bevy_mod_debugdump::{schedule_graph, schedule_graph_dot};
use bevy_rapier2d::plugin::systems::task::SimulationTask;
use bevy_rapier2d::prelude::*;
use bevy_transform_interpolation::prelude::{
    RotationInterpolation, TransformInterpolationPlugin, TranslationInterpolation,
};
use configuration::FixedSubsteps;

fn main() {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )))
    .insert_resource(TimestepMode::FixedSubsteps(FixedSubsteps {
        // target 1 steps per frame if 60 rendering frames per second.
        // For determinism, substep_dt should never be changed.
        substep_dt: (1.0 / 60.0) / 1.0,
        substeps: 1,
    }))
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .add_plugins((
        DefaultPlugins,
        FrameTimeDiagnosticsPlugin,
        TransformInterpolationPlugin::default(),
        RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0).in_fixed_schedule(),
        RapierDebugRenderPlugin::default(),
    ))
    .add_systems(Startup, (setup_graphics, setup_physics))
    .add_systems(Update, fps_text_update_system);
    app.add_systems(
        PostUpdate,
        debug_with_transform_info.after(TransformSystem::TransformPropagate),
    );
    app.add_systems(
        FixedUpdate,
        (
            sim_to_render_text_update_system,
            react_before_starting_simulation,
        )
            .chain()
            .after(PhysicsSet::StepSimulation)
            .before(PhysicsSet::StartBackgroundSimulation),
    );

    let mut debugdump_settings = schedule_graph::Settings::default();
    // Filter out some less relevant systems.
    debugdump_settings.include_system =
        Some(Box::new(|system: &(dyn System<In = (), Out = ()>)| {
            if system.name().starts_with("bevy_pbr")
                || system.name().starts_with("bevy_render")
                || system.name().starts_with("bevy_gizmos")
                || system.name().starts_with("bevy_winit")
                || system.name().starts_with("bevy_sprite")
            {
                return false;
            }
            true
        }));
    let dot = schedule_graph_dot(&mut app, PostUpdate, &debugdump_settings);

    let mut file = File::create("interpolation2.dot").expect("could not create file.");
    file.set_len(0).unwrap();
    file.write_all(&dot.as_bytes())
        .expect("Could not write to file");

    app.run();
}

#[derive(Component, Clone)]
pub struct VisualBallDebug;

#[derive(Component)]
struct SimToRenderTimeText;
#[derive(Component)]
struct FPSText;

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
        OrthographicProjection {
            scale: 15.0,
            ..OrthographicProjection::default_2d()
        },
        Transform::from_xyz(0.0, 50.0, 0.0),
    ));
    commands
        .spawn(Node {
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            justify_content: JustifyContent::SpaceBetween,
            ..default()
        })
        .insert(PickingBehavior::IGNORE)
        .with_children(|parent| {
            parent.spawn((
                Text::new(""),
                TextFont {
                    font_size: 30.0,
                    ..default()
                },
                TextColor(GOLD.into()),
                SimToRenderTimeText,
            ));
            parent.spawn((
                // Create a Text with multiple child spans.
                Text::new("FPS: "),
                TextFont {
                    font_size: 42.0,
                    ..default()
                },
                FPSText,
            ));
        });
}

fn sim_to_render_text_update_system(
    mut time_step_mode: ResMut<TimestepMode>,
    sim_to_render: Query<(&SimulationToRenderTime, &RapierContextColliders)>,
    mut query: Query<(&mut TextColor, &mut Text), With<SimToRenderTimeText>>,
) {
    let TimestepMode::FixedSubsteps(update_strat) = *time_step_mode else {
        return;
    };
    let (sim_to_render_time, colliders) = sim_to_render.get_single().unwrap();
    for (mut color, mut text) in &mut query {
        text.0 = format!(
            "real lag to render: {:.2}
lag taken into account: {:.2}
nb of frames to simulate physics: {}
nb of substeps for last physics simulation: {}
substep deltatime: {:.2}
amount of entities: {}",
            sim_to_render_time.diff,
            sim_to_render_time.accumulated_diff,
            sim_to_render_time.last_simulation_frame_count,
            update_strat.substeps,
            update_strat.substep_dt,
            colliders.colliders.len()
        );
        *color = if sim_to_render_time.diff < 0.0 {
            // simulation is ahead!
            palettes::basic::GREEN.into()
        } else {
            // simulation is behind!
            palettes::basic::BLUE.into()
        };
    }
}
fn fps_text_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<(&mut TextColor, &mut Text), With<FPSText>>,
) {
    for (mut color, mut text) in &mut query {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                text.0 = format!("FPS: {value:.0}");
                *color = if value > 50.0 {
                    palettes::basic::GREEN.into()
                } else {
                    palettes::basic::RED.into()
                };
            }
        }
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 5000.0;
    let ground_height = 100.0;

    commands.spawn((
        Transform::from_xyz(0.0, 0.0 * -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
    ));

    let ball = (
        Transform::from_xyz(0.0, 200.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(20.0),
        Restitution {
            coefficient: 0.99,
            combine_rule: CoefficientCombineRule::Max,
        },
        VisualBallDebug,
    );
    let ball_column_height = 1000;
    for i in 0..ball_column_height {
        let y_offset = i as f32 * 41f32;
        let x_noise_offset = i as f32 / ball_column_height as f32 - 0.5f32;
        commands.spawn(ball.clone()).insert(Transform::from_xyz(
            80.0 + x_noise_offset,
            200.0 + y_offset,
            0.0,
        ));
        commands.spawn(ball.clone()).insert((
            Transform::from_xyz(0.0 + x_noise_offset, 200.0 + y_offset, 0.0),
            //TranslationInterpolation,
        ));
        commands.spawn(ball.clone()).insert((
            Transform::from_xyz(-80.0 + x_noise_offset, 200.0 + y_offset, 0.0),
            //TranslationInterpolation,
            ColliderDebug::NeverRender,
        ));

        for i in 0..4 {
            let x_offset = 80.0 * i as f32;
            commands.spawn(ball.clone()).insert((
                Transform::from_xyz(-x_offset + x_noise_offset, 200.0 + y_offset, 0.0),
                //TranslationInterpolation,
                ColliderDebug::NeverRender,
            ));
            commands.spawn(ball.clone()).insert((
                Transform::from_xyz(x_offset + x_noise_offset, 200.0 + y_offset, 0.0),
                //TranslationInterpolation,
                ColliderDebug::NeverRender,
            ));
        }
    }
}

pub fn debug_with_transform_info(
    mut gizmos: Gizmos,
    entities: Query<(&Transform, &Collider), With<VisualBallDebug>>,
) {
    for (transform, collider) in entities.iter() {
        gizmos.circle(
            transform.translation,
            collider.as_ball().unwrap().radius(),
            palettes::basic::RED,
        );
    }
}

pub fn react_before_starting_simulation(
    mut time_step_mode: ResMut<TimestepMode>,
    mut q_context: Query<&mut SimulationToRenderTime, Without<SimulationTask>>,
) {
    for mut sim_to_render_time in q_context.iter_mut() {
        profiling::scope!("react_before_starting_simulation");
        dbg!("before starting simulation");
        if let TimestepMode::FixedSubsteps(sync) = &mut *time_step_mode {
            if sim_to_render_time.diff > 0.5f32 {
                // The simulation is behind the render time. The simulation slows down,
                // the strategy to handle this could be to :
                // - run more steps per simulation frame ( avoiding bevy overhead, synchronization, etc)
                // - reduce the quality of the simulation (reduce substeps, ccd...)
                // - allow a time drift: accept the simulation has slowed down, and don't attempt to catch up.
                // For now, we just reset the diff to 0, effectively allowing a time drift,
                // but ideally, we should report that to the user.
                //dbg!(sim_to_render_time.diff = 0f32);

                let half_time_to_catch_back = sim_to_render_time.diff / 2f32;
                if sim_to_render_time.diff > 1f32 {
                    // Allow a time drift, the simulation is too far behind.
                    sim_to_render_time.diff = 0f32;
                }
                let clamped_substeps_runs =
                    ((half_time_to_catch_back / sync.substep_dt as f32) as usize).clamp(10, 40);
                sync.substeps = clamped_substeps_runs;
                dbg!(sync.substeps);
            } else {
                // the physics simulation is ahead of the real time, it should either:
                // - do nothing: wait for the render to catch up
                // - run again, then wait for the render to catch up before reading.
                // - allow user to customize ?
                // This could be a target_sim_to_render ; where we should starting new when sim_to_render.diff > target_overtime

                // TODO: estimate how many frames it will take to catch up.
                // If we're not behind, we can expect it to take 1 frame.
                let time_to_advance = (1.0 / 60.0 + sim_to_render_time.diff).max(0f32);
                sync.substeps = (time_to_advance / sync.substep_dt) as usize;
            }
        }
    }
}
