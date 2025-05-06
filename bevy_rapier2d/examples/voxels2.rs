use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use nalgebra::{point, Vector2};
use rapier2d::prelude::{SharedShape, VoxelPrimitiveGeometry};

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((Camera2d, Transform::from_xyz(0.0, 20.0, 0.0)));
}

pub fn setup_physics(mut commands: Commands) {
    let scale = 15f32;
    /*
     * Create dynamic objects to fall on voxels.
     */
    let nx = 50;
    for i in 0..nx {
        for j in 0..10 {
            let falling_objects = (j + i) % 3;

            let ball_radius = 0.5 * scale;
            let co = match falling_objects {
                0 => Collider::ball(ball_radius),
                1 => Collider::cuboid(ball_radius, ball_radius),
                2 => Collider::capsule_y(ball_radius, ball_radius),
                _ => unreachable!(),
            };
            commands.spawn((
                Transform::from_xyz(
                    (i as f32 * 2.0 - nx as f32 / 2.0) * scale,
                    (20.0 + j as f32 * 2.0) * scale,
                    0.0,
                ),
                RigidBody::Dynamic,
                co,
            ));
        }
    }

    /*
     * Voxelization.
     */
    let polyline = [
        point![0.0, 0.0],
        point![0.0, 10.0],
        point![7.0, 4.0],
        point![14.0, 10.0],
        point![14.0, 0.0],
        point![13.0, 7.0],
        point![7.0, 2.0],
        point![1.0, 7.0],
    ]
    .iter()
    .map(|p| p * scale)
    .collect::<Vec<_>>();
    let indices: Vec<_> = (0..polyline.len() as u32)
        .map(|i| [i, (i + 1) % polyline.len() as u32])
        .collect();

    let shape = SharedShape::voxelized_mesh(
        VoxelPrimitiveGeometry::PseudoCube,
        &polyline,
        &indices,
        0.2 * scale,
        FillMode::default(),
    );
    commands.spawn((
        Transform::from_xyz(-20.0 * scale, -10.0 * scale, 0.0),
        Collider::from(shape),
    ));

    /*
     * A voxel wavy floor.
     */
    let voxel_size = Vector2::new(scale, scale);
    let voxels: Vec<_> = (0..300)
        .map(|i| {
            let y = (i as f32 / 20.0).sin().clamp(-0.5, 0.5) * 20.0;
            point![(i as f32 - 125.0) * voxel_size.x / 2.0, y * voxel_size.y]
        })
        .collect();
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        Collider::from(rapier2d::prelude::SharedShape::voxels_from_points(
            VoxelPrimitiveGeometry::PseudoCube,
            voxel_size,
            &voxels,
        )),
    ));
}
