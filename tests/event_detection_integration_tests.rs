use std::sync::{Arc, Mutex};

use minkowski_space::collision::CollisionGroup;
use minkowski_space::config::{MotionMode, ObjectConfig, StartPosition};
use minkowski_space::m_event::DetectionObject;
use minkowski_space::m_vector::MVector;
use minkowski_space::m_world::{MWorld, ProcessTimeCallback};
use minkowski_space::observation::EventObservation;
use vector2d::Vector2D;

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
    let callbacks = world.process_time(1.9);
    assert!(!callbacks.iter().any(|callback| matches!(callback, ProcessTimeCallback::Event(_))));
    assert_eq!(world.observe_event(&event_id), Some(EventObservation::NotVisible));

    let callbacks = world.process_time(0.2);
    assert!(callbacks.iter().any(|callback| matches!(callback, ProcessTimeCallback::Event(event)
        if event.event_id == event_id && matches!(event.detection_object, DetectionObject::FrameObject))));
    assert!(matches!(world.observe_event(&event_id), Some(EventObservation::Visible(_))));
}

#[test]
fn callback_reports_frame_and_object_detection_without_duplicates() {
    let detections = Arc::new(Mutex::new(Vec::<DetectionObject>::new()));
    let callback_detections = Arc::clone(&detections);
    let mut world = MWorld::new();
    let object_id = world.register_object(stationary_object(Vector2D::new(4.0, 0.0)));
    let event_id = world.create_event_with_callback(Vector2D::new(0.0, 0.0), move |_world, detection| {
        callback_detections.lock().unwrap().push(detection.detection_object);
    });

    let first = world.process_time(4.0);
    let events: Vec<_> = first.iter().filter_map(|callback| match callback {
        ProcessTimeCallback::Event(event) => Some(event),
        _ => None,
    }).collect();
    assert_eq!(events.len(), 2);
    assert!(events.iter().any(|event| event.event_id == event_id && matches!(event.detection_object, DetectionObject::FrameObject)));
    assert!(events.iter().any(|event| event.event_id == event_id && matches!(event.detection_object, DetectionObject::MObject(id) if id == object_id)));

    let callback_values = detections.lock().unwrap();
    assert_eq!(callback_values.len(), 2);
    assert!(callback_values.iter().any(|object| matches!(object, DetectionObject::FrameObject)));
    assert!(callback_values.iter().any(|object| matches!(object, DetectionObject::MObject(id) if *id == object_id)));
    drop(callback_values);

    let second = world.process_time(1.0);
    assert!(!second.iter().any(|callback| matches!(callback, ProcessTimeCallback::Event(event) if event.event_id == event_id)));
    assert_eq!(detections.lock().unwrap().len(), 2);
}

#[test]
fn multiple_objects_detect_same_event_independently() {
    let detections = Arc::new(Mutex::new(Vec::<usize>::new()));
    let callback_detections = Arc::clone(&detections);
    let mut world = MWorld::new();
    let first_object = world.register_object(stationary_object(Vector2D::new(1.0, 0.0)));
    let second_object = world.register_object(stationary_object(Vector2D::new(-3.0, 0.0)));
    let event_id = world.create_event_with_callback(Vector2D::new(0.0, 0.0), move |_world, detection| {
        if let DetectionObject::MObject(id) = detection.detection_object {
            callback_detections.lock().unwrap().push(id);
        }
    });

    let callbacks = world.process_time(3.0);
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
    assert_eq!(detections.lock().unwrap().len(), 2);
    assert!(world.event(&event_id).is_some());
}
