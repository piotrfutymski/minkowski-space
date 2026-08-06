use std::sync::{Arc, Mutex};

use vector2d::Vector2D;
use minkowski_space::{CollisionGroup, DetectionObject, EventObservation, MVector, MWorld, MotionMode, ObjectConfig, ProcessTimeCallback, StartPosition};

fn stationary_object(position: Vector2D<f64>) -> ObjectConfig {
    ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, position)),
        velocity: Vector2D::new(0.0, 0.0),
        radius: 0.1,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        collision_group: CollisionGroup::All,
    }
}

#[test]
fn event_becomes_visible_to_frame_after_light_travel_time() {
    let mut world = MWorld::new();
    let event_id = world.create_event(Vector2D::new(2.0, 0.0));

    assert_eq!(world.observe_event(&event_id), Some(EventObservation::NotVisible));
    let callbacks = world.advance_by_proper_time(1.9);
    assert!(!callbacks.iter().any(|callback| matches!(callback, ProcessTimeCallback::Event(_))));
    assert_eq!(world.observe_event(&event_id), Some(EventObservation::NotVisible));

    let callbacks = world.advance_by_proper_time(0.2);
    assert!(callbacks.iter().any(|callback| matches!(callback, ProcessTimeCallback::Event(event)
        if event.event_id == event_id && matches!(event.detection_object, DetectionObject::FrameObject))));
    assert!(matches!(world.observe_event(&event_id), Some(EventObservation::Visible(_))));
}

#[test]
fn callback_reports_frame_and_object_detection_without_duplicates() {
    let mut world = MWorld::new();
    let object_id = world.register_object(stationary_object(Vector2D::new(4.0, 0.0)));
    let event_id = world.create_event(Vector2D::new(0.0, 0.0));

    let first = world.advance_by_proper_time(4.0);
    let events: Vec<_> = first.iter().filter_map(|callback| match callback {
        ProcessTimeCallback::Event(event) => Some(event),
        _ => None,
    }).collect();
    assert_eq!(events.len(), 2);
    assert!(events.iter().any(|event| event.event_id == event_id && matches!(event.detection_object, DetectionObject::FrameObject)));
    assert!(events.iter().any(|event| event.event_id == event_id && matches!(event.detection_object, DetectionObject::MObject(id) if id == object_id)));

    let second = world.advance_by_proper_time(1.0);
    assert!(!second.iter().any(|callback| matches!(callback, ProcessTimeCallback::Event(event) if event.event_id == event_id)));

}

#[test]
fn multiple_objects_detect_same_event_independently() {
    let mut world = MWorld::new();
    let first_object = world.register_object(stationary_object(Vector2D::new(1.0, 0.0)));
    let second_object = world.register_object(stationary_object(Vector2D::new(-3.0, 0.0)));
    let event_id = world.create_event(Vector2D::new(0.0, 0.0));

    let callbacks = world.advance_by_proper_time(3.0);
    let object_detections: Vec<_> = callbacks.iter().filter_map(|callback| match callback {
        ProcessTimeCallback::Event(event) => match event.detection_object {
            DetectionObject::MObject(id) => Some(id),
            DetectionObject::FrameObject => None,
        },
        _ => None,
    }).collect();
    assert_eq!(object_detections.len(), 2);
    assert!(object_detections.contains(&first_object));
    assert!(object_detections.contains(&second_object));
    assert!(world.event(&event_id).is_some());
}
