use std::{fs::File, io::Write};

use bevy::prelude::*;
use bevy_mod_debugdump::{schedule_graph, schedule_graph_dot};
use bevy_rapier2d::prelude::*;
use bevy_transform_interpolation::{
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
    .insert_resource(Time::<Fixed>::from_seconds(0.4))
    .add_plugins((
        DefaultPlugins,
        TransformInterpolationPlugin::default(),
        RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0).in_fixed_schedule(),
        RapierDebugRenderPlugin::default(),
    ))
    .add_systems(Startup, (setup_graphics, setup_physics));
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

    commands.spawn((
        Transform::from_xyz(-40.0, 200.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(20.0),
        Restitution {
            coefficient: 1.0,
            combine_rule: CoefficientCombineRule::Max,
        },
        TranslationInterpolation,
        RotationInterpolation,
    ));
    commands.spawn((
        Transform::from_xyz(40.0, 200.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(20.0),
        Restitution {
            coefficient: 1.0,
            combine_rule: CoefficientCombineRule::Max,
        },
    ));
}
