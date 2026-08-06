use std::sync::{Arc, Mutex};

use minkowski_space::{
    CollisionGroup, CollisionObject, MVector, MotionMode, ObjectConfig, ProcessTimeCallback,
    StartPosition, Vector2D, WorldConfig, MWorld,
};

fn object(position: f64, velocity: f64, radius: f64, group: CollisionGroup) -> ObjectConfig {
    ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(position, 0.0))),
        velocity: Vector2D::new(velocity, 0.0),
        radius,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        collision_group: group,
    }
}

fn isolated_world() -> (MWorld, CollisionGroup) {
    let mut config = WorldConfig::default();
    let object_group = config.define_collision_group();
    let frame_group = config.define_collision_group();
    config.frame_collision_group = CollisionGroup::CollisionGroup(frame_group);
    config.collision_pairs.insert(minkowski_space::CollisionGroupPair(object_group, object_group));
    (MWorld::with_config(config).unwrap(), CollisionGroup::CollisionGroup(object_group))
}

fn collisions(callbacks: &[ProcessTimeCallback]) -> Vec<&minkowski_space::Collision> {
    callbacks
        .iter()
        .filter_map(|callback| match callback {
            ProcessTimeCallback::Collision(collision) => Some(collision),
            _ => None,
        })
        .collect()
}

#[test]
fn fast_objects_crossing_report_earliest_contact_and_centers() {
    let (mut world, group) = isolated_world();
    let first = world.register_object(object(-1.0, 0.8, 0.1, group));
    let second = world.register_object(object(1.0, -0.8, 0.1, group));

    let callbacks = world.advance_by_proper_time(2.0);
    let detected = collisions(&callbacks);
    assert_eq!(detected.len(), 1);
    let collision = detected[0];
    assert_eq!(collision.object_a, CollisionObject::Object(first));
    assert_eq!(collision.object_b, CollisionObject::Object(second));
    assert!((collision.time - (25.0 / 24.0)).abs() < 1e-9);
    assert!((collision.contact_point_object_a.x + (1.0 / 6.0)).abs() < 1e-9);
    assert!((collision.contact_point_object_b.x - (1.0 / 6.0)).abs() < 1e-9);
}

#[test]
fn stationary_overlap_emits_once_until_separation_and_recontact() {
    let (mut world, group) = isolated_world();
    let first = world.register_object(object(0.0, 0.0, 0.5, group));
    let mut second_config = object(0.5, 0.0, 0.5, group);
    second_config.motion_mode = MotionMode::Dynamic;
    let second = world.register_object(second_config);

    assert_eq!(collisions(&world.advance_by_proper_time(0.1)).len(), 1);
    assert_eq!(collisions(&world.advance_by_proper_time(0.1)).len(), 0);

    world.set_velocity(&second, Vector2D::new(0.5, 0.0)).unwrap();
    assert_eq!(collisions(&world.advance_by_proper_time(2.0)).len(), 0);

    world.set_velocity(&second, Vector2D::new(-0.5, 0.0)).unwrap();
    assert_eq!(collisions(&world.advance_by_proper_time(2.0)).len(), 1);
    assert_eq!(first, 0);
}

#[test]
fn zero_radius_participants_do_not_collide_with_each_other() {
    let (mut world, group) = isolated_world();
    world.register_object(object(-1.0, 0.5, 0.0, group));
    world.register_object(object(1.0, -0.5, 0.0, group));

    assert!(collisions(&world.advance_by_proper_time(2.0)).is_empty());
}

#[test]
fn zero_radius_participant_collides_with_positive_radius() {
    let (mut world, group) = isolated_world();
    world.register_object(object(-1.0, 0.5, 0.0, group));
    world.register_object(object(0.0, 0.0, 0.2, group));

    assert_eq!(collisions(&world.advance_by_proper_time(2.0)).len(), 1);
}

#[test]
fn frame_participates_in_collisions() {
    let mut world = MWorld::new();
    let object_id = world.register_object(object(1.0, -0.5, 0.1, CollisionGroup::All));

    let callbacks = world.advance_by_proper_time(2.0);
    let detected = collisions(&callbacks);
    assert_eq!(detected.len(), 1);
    assert_eq!(detected[0].object_a, CollisionObject::Object(object_id));
    assert_eq!(detected[0].object_b, CollisionObject::Frame);
}

#[test]
fn configured_same_and_cross_group_pairs_are_symmetric() {
    let mut config = WorldConfig::default();
    let group_a = config.define_collision_group();
    let group_b = config.define_collision_group();
    config.collision_pairs.insert(minkowski_space::CollisionGroupPair(group_a, group_a));
    config.collision_pairs.insert(minkowski_space::CollisionGroupPair(group_a, group_b));
    let mut world = MWorld::with_config(config).unwrap();
    world.register_object(object(0.0, 0.0, 0.5, CollisionGroup::CollisionGroup(group_a)));
    world.register_object(object(0.5, 0.0, 0.5, CollisionGroup::CollisionGroup(group_a)));
    world.register_object(object(1.0, 0.0, 0.5, CollisionGroup::CollisionGroup(group_b)));

    assert_eq!(collisions(&world.advance_by_proper_time(0.1)).len(), 3);
}

#[test]
fn collision_callbacks_follow_registration_order_and_snapshot_matching() {
    let (mut world, group) = isolated_world();
    world.register_object(object(0.0, 0.0, 0.5, group));
    world.register_object(object(0.5, 0.0, 0.5, group));
    let calls = Arc::new(Mutex::new(Vec::new()));

    let first_calls = Arc::clone(&calls);
    world.register_collision_callback(move |world, _collision| {
        first_calls.lock().unwrap().push(1);
        world.register_collision_callback(|_, _| {});
    });
    let second_calls = Arc::clone(&calls);
    world.register_collision_callback(move |_, _| {
        second_calls.lock().unwrap().push(2);
    });

    assert_eq!(collisions(&world.advance_by_proper_time(0.1)).len(), 1);
    assert_eq!(*calls.lock().unwrap(), vec![1, 2]);
}

#[test]
fn global_callback_receives_frame_collision() {
    let mut world = MWorld::new();
    world.register_object(object(1.0, -0.5, 0.1, CollisionGroup::All));
    let called = Arc::new(Mutex::new(0));
    let called_by_callback = Arc::clone(&called);
    world.register_collision_callback(move |_, collision| {
        assert!(matches!(collision.object_b, CollisionObject::Frame));
        *called_by_callback.lock().unwrap() += 1;
    });

    world.advance_by_proper_time(2.0);
    assert_eq!(*called.lock().unwrap(), 1);
}
