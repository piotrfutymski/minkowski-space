use vector2d::Vector2D;
use minkowski_space::{MVector, MWorld, MotionMode, ObjectConfig, ProcessTimeCallback, StartPosition, WorldConfig};

#[test]
fn test_collision() {
    let mut m_world = MWorld::with_config(
        WorldConfig::with_collisions(vec![(0,1)])
    ).unwrap();

    let distance = 2.0;
    let radius = 0.2;
    let v: f64 = 0.9;
    let collision_time = (distance - 2.0 * radius * (1.0 - v.powi(2)).sqrt()) / (2.0 * v);

    let left = m_world.register_object(ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(-distance/2.0, 0.3))),
        velocity: Vector2D::new(v, 0.0),
        radius,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        collision_group: 0.into(),
    });

    let right = m_world.register_object(ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(distance/2.0, 0.3))),
        velocity: Vector2D::new(-v, 0.0),
        radius,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        collision_group: 1.into(),
    });

    let mut all_callbacks = vec![];

    let interval = 0.01;
    for _i in 0..(collision_time/interval)as u32 + 1 {
        all_callbacks.append( &mut m_world.advance_by_proper_time(interval));
    }

    let collision = all_callbacks.into_iter().filter_map(|e|match e {
        ProcessTimeCallback::Collision(collision) => Some(collision),
        ProcessTimeCallback::Event(_) => None
    }).next().unwrap();
    let contact_point = collision.time;
    assert!(contact_point > collision_time);
}