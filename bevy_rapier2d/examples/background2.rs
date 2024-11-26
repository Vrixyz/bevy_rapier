//! This example should be run with features bevy/multi_threaded and bevy_rapier2d/background_simulation

use std::{fs::File, io::Write};

use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::{color::palettes, prelude::*};
use bevy_mod_debugdump::{schedule_graph, schedule_graph_dot};
use bevy_rapier2d::prelude::*;
use bevy_transform_interpolation::prelude::{
    RotationInterpolation, TransformInterpolationPlugin, TranslationInterpolation,
};

fn main() {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )))
    .insert_resource(TimestepMode::Variable {
        max_dt: 100f32,
        time_scale: 1f32,
        substeps: 10,
    })
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .add_plugins((
        DefaultPlugins,
        FrameTimeDiagnosticsPlugin,
        LogDiagnosticsPlugin::default(),
        TransformInterpolationPlugin::default(),
        RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0).in_fixed_schedule(),
        RapierDebugRenderPlugin::default(),
    ))
    .add_systems(Startup, (setup_graphics, setup_physics));
    app.add_systems(
        PostUpdate,
        debug_with_transform_info.after(TransformSystem::TransformPropagate),
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

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
        OrthographicProjection {
            scale: 15.0,
            ..OrthographicProjection::default_2d()
        },
        Transform::from_xyz(0.0, 50.0, 0.0),
    ));
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
            TranslationInterpolation,
        ));
        commands.spawn(ball.clone()).insert((
            Transform::from_xyz(-80.0 + x_noise_offset, 200.0 + y_offset, 0.0),
            TranslationInterpolation,
            ColliderDebug::NeverRender,
        ));

        for i in 0..4 {
            let x_offset = 80.0 * i as f32;
            commands.spawn(ball.clone()).insert((
                Transform::from_xyz(-x_offset + x_noise_offset, 200.0 + y_offset, 0.0),
                TranslationInterpolation,
                RotationInterpolation,
                ColliderDebug::NeverRender,
            ));
            commands.spawn(ball.clone()).insert((
                Transform::from_xyz(x_offset + x_noise_offset, 200.0 + y_offset, 0.0),
                TranslationInterpolation,
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
