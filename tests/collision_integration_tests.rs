use minkowski_space::{
    MVector, MWorld, MotionMode, ObjectConfig, ProcessTimeCallback, StartPosition, WorldConfig,
};
use vector2d::Vector2D;

fn test_collision_with_v(v: f64) {
    let mut m_world = MWorld::with_config(WorldConfig::with_collisions(vec![(0, 1)])).unwrap();

    let distance = 2.0;
    let radius = 0.2;
    let collision_time = (distance - 2.0 * radius * (1.0 - v.powi(2)).sqrt()) / (2.0 * v);

    let _left = m_world.register_object(ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(-distance / 2.0, 0.3))),
        velocity: Vector2D::new(v, 0.0),
        radius,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        collision_group: 0.into(),
    });

    let _right = m_world.register_object(ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(distance / 2.0, 0.3))),
        velocity: Vector2D::new(-v, 0.0),
        radius,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        collision_group: 1.into(),
    });

    let mut all_callbacks = vec![];

    let interval = 0.01;
    for _i in 0..(2.0 * collision_time / interval) as u32 {
        all_callbacks.append(&mut m_world.advance_by_proper_time(interval));
    }

    let collisions = all_callbacks
        .into_iter()
        .filter_map(|e| match e {
            ProcessTimeCallback::Collision(collision) => Some(collision.position),
            ProcessTimeCallback::Event(_) => None,
        })
        .collect::<Vec<_>>();
    let contact_point = collisions[0];
    assert_eq!(collisions.len(), 1);
    assert!(contact_point.time > collision_time);
}

#[test]
fn test_two_collisions_after_objects_turn_back() {
    let mut m_world = MWorld::with_config(WorldConfig::with_collisions(vec![(0, 1)])).unwrap();

    let left = m_world.register_object(ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(-1.0, 0.0))),
        velocity: Vector2D::new(0.5, 0.0),
        radius: 0.2,
        motion_mode: MotionMode::Dynamic,
        collision_group: 0.into(),
    });
    let right = m_world.register_object(ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, Vector2D::new(1.0, 0.0))),
        velocity: Vector2D::new(-0.5, 0.0),
        radius: 0.2,
        motion_mode: MotionMode::Dynamic,
        collision_group: 1.into(),
    });

    let mut collision_positions = Vec::new();

    for _ in 0..300 {
        let callbacks = m_world.advance_by_proper_time(0.01);
        let collision = callbacks.into_iter().find_map(|callback| {
            if let ProcessTimeCallback::Collision(collision) = callback {
                Some(collision.position)
            } else {
                None
            }
        });
        if let Some(collision) = collision {
            collision_positions.push(collision);
            m_world
                .set_velocity(left, Vector2D::new(-0.5, 0.0))
                .unwrap();
            m_world
                .set_velocity(right, Vector2D::new(0.5, 0.0))
                .unwrap();
            break;
        }
    }
    assert_eq!(collision_positions.len(), 1);

    for _ in 0..100 {
        m_world.advance_by_proper_time(0.01);
    }
    m_world.set_velocity(left, Vector2D::new(0.5, 0.0)).unwrap();
    m_world
        .set_velocity(right, Vector2D::new(-0.5, 0.0))
        .unwrap();

    for _ in 0..300 {
        let second_collision =
            m_world
                .advance_by_proper_time(0.01)
                .into_iter()
                .find_map(|callback| {
                    if let ProcessTimeCallback::Collision(collision) = callback {
                        Some(collision.position)
                    } else {
                        None
                    }
                });
        if let Some(collision) = second_collision {
            collision_positions.push(collision);
            break;
        }
    }

    assert_eq!(collision_positions.len(), 2);
    assert!(collision_positions[1].time > collision_positions[0].time);
}

#[test]
fn test_collision_slow() {
    test_collision_with_v(0.5)
}

#[test]
fn test_collision_fast() {
    test_collision_with_v(0.95)
}
