use vector2d::Vector2D;
use minkowski_space::m_frame::MFrame;
use minkowski_space::m_vector::MVector;

fn test_minkowski_space(const_speed: bool){
    let mut m_frame = MFrame::new();
    let transform = 0.5f64.sqrt();
    m_frame.get_frame_object_mut().set_velocity(Vector2D::new(0.8, 0.8) * transform);
    let id = m_frame.register_object(
        MVector::new(0.0, Vector2D::new(2.0, 2.0) * transform),
        Vector2D::new(-0.6, -0.6) * transform,
        const_speed,
        0.1
    );
    assert!(!m_frame.get_object_with_properties(&id).unwrap().1.get_object_was_seen());

    for _i in 0..60 {
        m_frame.process_time(0.01);
    }

    assert!(!m_frame.get_object_with_properties(&id).unwrap().1.get_object_was_seen());

    let obj_pos = *m_frame.get_frame_object_mut().get_m_pos();
    assert!((obj_pos.time - 1.0).abs() < 1e-6);
    assert!((obj_pos.pos.x / transform - 0.8).abs() < 1e-6);
    assert!((m_frame.get_frame_object_mut().get_tau() - 0.6).abs() < 1e-6);


    for _i in 0..15 {
        m_frame.process_time(0.01);
    }

    assert!(m_frame.get_object_with_properties(&id).unwrap().1.get_object_was_seen());

    let obj_pos = *m_frame.get_frame_object_mut().get_m_pos();
    assert!((obj_pos.time - 1.25).abs() < 1e-6);
    assert!((obj_pos.pos.x / transform - 1.0).abs() < 1e-6);
    assert!((m_frame.get_frame_object_mut().get_tau() - 0.75).abs() < 1e-6);

    let tracked_obj = m_frame.get_object_with_properties(&id).unwrap();
    let tracked_obj_pos = tracked_obj.1.get_visible_m_vector();
    assert!((tracked_obj_pos.time - 5.0/8.0).abs() < 1e-6);
    assert!((tracked_obj.1.get_relative_frequency() - 6.0).abs() < 1e-6)
}

#[test]
fn test_minkowski_space_const_v(){
    test_minkowski_space(true)
}

#[test]
fn test_minkowski_space_non_const_v(){
    test_minkowski_space(false)
}