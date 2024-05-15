use std::f32::consts::TAU;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default().with_length_unit(10f32),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, remove_sleeping_bodies)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-30.0, 30.0, 100.0)
            .looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 20.1;
    let ground_height = 0.1;

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * Walls
     */
    let wall_size = 2.;
    let wall_height = 10.0;

    for rotation in [0.0, 0.25, 0.5, 0.75] {
        let mut position = Transform::from_xyz(0.0, wall_height, ground_size + wall_size);
        position.rotate_around(Vec3::Y, Quat::from_rotation_y(TAU * rotation));
        commands.spawn((
            TransformBundle::from(position),
            Collider::cuboid(ground_size, wall_height, wall_size),
        ));
    }

    /*
     * Create the balls
     */
    let num = 1;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
    let mut color = 0;
    let colors = [
        Color::hsl(220.0, 1.0, 0.3),
        Color::hsl(180.0, 1.0, 0.3),
        Color::hsl(260.0, 1.0, 0.7),
    ];

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                commands
                    .spawn(TransformBundle::from(Transform::from_rotation(
                        Quat::from_rotation_x(0.2),
                    )))
                    .with_children(|child| {
                        child.spawn((
                            TransformBundle::from(Transform::from_xyz(x, y, z)),
                            RigidBody::Dynamic,
                            Collider::ball(rad),
                            ColliderDebugColor(colors[color % 3]),
                            Sleeping::default(),
                        ));
                    });
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

pub fn remove_sleeping_bodies(mut commands: Commands, q_sleep_status: Query<(Entity, &Sleeping)>) {
    for (entity, sleeping) in q_sleep_status.iter() {
        if sleeping.sleeping {
            commands.entity(entity).despawn_recursive();
        }
    }
}
