use vector2d::Vector2D;
use minkowski_space::config::{MotionMode, ObjectConfig};
use minkowski_space::m_world::MWorld;
use minkowski_space::m_vector::MVector;
use minkowski_space::observation::ObjectObservation;

fn test_minkowski_space(motion_mode: MotionMode){
    let mut m_world = MWorld::new();
    let transform = 0.5f64.sqrt();
    m_world.get_frame_object_mut().set_velocity(Vector2D::new(0.8, 0.8) * transform);
    let id = m_world.register_object(ObjectConfig{
        position: MVector::new(0.0, Vector2D::new(2.0, 2.0) * transform),
        velocity: Vector2D::new(-0.6, -0.6) * transform,
        radius: 0.1,
        motion_mode,
        collision_group: None,
    });
    assert_eq!(m_world.observe_object(&id).unwrap(), ObjectObservation::NotVisible);

    for _i in 0..60 {
        m_world.process_time(0.01);
    }

    assert_eq!(m_world.observe_object(&id).unwrap(), ObjectObservation::NotVisible);

    let obj_pos = *m_world.get_frame_object_mut().get_m_pos();
    assert!((obj_pos.time - 1.0).abs() < 1e-6);
    assert!((obj_pos.pos.x / transform - 0.8).abs() < 1e-6);
    assert!((m_world.get_frame_object_mut().get_tau() - 0.6).abs() < 1e-6);


    for _i in 0..15 {
        m_world.process_time(0.01);
    }

    assert!(matches!(m_world.observe_object(&id).unwrap(), ObjectObservation::Visible(_)));

    let obj_pos = *m_world.get_frame_object_mut().get_m_pos();
    assert!((obj_pos.time - 1.25).abs() < 1e-6);
    assert!((obj_pos.pos.x / transform - 1.0).abs() < 1e-6);
    assert!((m_world.get_frame_object_mut().get_tau() - 0.75).abs() < 1e-6);

    let visible_object = m_world.observe_visible_object(&id).unwrap();
    let tracked_obj_pos = visible_object.visible_position;
    assert!((tracked_obj_pos.time - 5.0/8.0).abs() < 1e-6);
    assert!((visible_object.relative_frequency - 6.0).abs() < 1e-6)
}

#[test]
fn test_minkowski_space_const_v(){
    test_minkowski_space(MotionMode::AlwaysConstantVelocity)
}

#[test]
fn test_minkowski_space_non_const_v(){
    test_minkowski_space(MotionMode::Dynamic)
}