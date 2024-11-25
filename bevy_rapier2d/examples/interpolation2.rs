use std::{fs::File, io::Write};

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
    .insert_resource(Time::<Fixed>::from_hz(5.0))
    .add_plugins((
        DefaultPlugins,
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
    commands.spawn((Camera2d::default(), Transform::from_xyz(0.0, 20.0, 0.0)));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 500.0;
    let ground_height = 10.0;

    commands.spawn((
        Transform::from_xyz(0.0, 0.0 * -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
    ));

    let interpolated_ball = (
        Transform::from_xyz(-0.0, 200.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(20.0),
        Restitution {
            coefficient: 0.99,
            combine_rule: CoefficientCombineRule::Max,
        },
        TranslationInterpolation,
        RotationInterpolation,
        VisualBallDebug,
    );
    commands.spawn(interpolated_ball.clone());
    commands.spawn(interpolated_ball).insert((
        ColliderDebug::NeverRender,
        Transform::from_xyz(-80.0, 200.0, 0.0),
    ));
    commands.spawn((
        Transform::from_xyz(80.0, 200.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(20.0),
        Restitution {
            coefficient: 0.99,
            combine_rule: CoefficientCombineRule::Max,
        },
        VisualBallDebug,
    ));
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
