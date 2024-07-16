use bevy::{input::common_conditions::input_just_pressed, prelude::*};
use bevy_rapier3d::prelude::*;

const N_WORLDS: usize = 2;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default()
                .with_custom_initialization(RapierContextInitialization::NoAutomaticRapierContext),
            RapierDebugRenderPlugin::default(),
        ))
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 2,
        })
        .add_systems(
            Startup,
            ((create_worlds, setup_physics).chain(), setup_graphics),
        )
        .add_systems(
            Update,
            change_world, //.run_if(input_just_pressed(KeyCode::KeyC)),
        )
        .run();
}

fn create_worlds(mut commands: Commands) {
    for i in 0..N_WORLDS {
        let mut world = commands.spawn(RapierContext::default());
        if i == 0 {
            world.insert(DefaultRapierContext);
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 3.0, -10.0)
            .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        ..Default::default()
    });
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

pub fn setup_physics(mut commands: Commands) {
    /*
     * Create the cube
     */

    let color = [
        Hsla::hsl(220.0, 1.0, 0.3),
        Hsla::hsl(180.0, 1.0, 0.3),
        Hsla::hsl(260.0, 1.0, 0.7),
    ][0 % 3];

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 0.5),
        ColliderDebugColor(color),
        ActiveEvents::all(),
    ));

    /*
     * Ground
     */
    let color = Hsla::hsl(260.0, 1.0, 0.7);
    let ground_size = 5.1;
    let ground_height = 0.1;
    let starting_y = -0.5 - ground_height;

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, starting_y, 0.0)),
        Collider::cuboid(ground_size, ground_height, ground_size),
        ColliderDebugColor(color),
    ));
}
