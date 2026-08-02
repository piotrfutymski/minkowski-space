//! Demonstrates the relativity of simultaneity with two light pulses and two mirrors.
//!
//! At `t = 0`, the pulses leave the center of a vehicle moving at `v = 0.6`.
//! In the laboratory frame, the mirrors detect them at different times; in the
//! vehicle's frame, the detections are simultaneous.

use std::cell::RefCell;
use std::rc::Rc;

use minkowski_space::config::ObjectConfig;
use minkowski_space::m_event::DetectionObject;
use minkowski_space::m_vector::MVector;
use minkowski_space::m_world::MWorld;
use vector2d::Vector2D;
use minkowski_space::observation::EventObservation;

fn main() {
    let vehicle_velocity = Vector2D::new(0.6, 0.0);
    let mut world = MWorld::new();
    // Observe the experiment from the vehicle's frame.
    world.set_frame_velocity(vehicle_velocity);

    // The mirrors are 1 unit from the origin in the laboratory frame.
    // Their proper separation is γ · 2 = 2.5 units.
    let left_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(
        Vector2D::new(-1.0, 0.0),
        vehicle_velocity,
    ));
    let right_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(
        Vector2D::new(1.0, 0.0),
        vehicle_velocity,
    ));

    let left_detection_event = Rc::new(RefCell::new(0usize));
    let left_detection_event_clone = left_detection_event.clone();
    let right_detection_event = Rc::new(RefCell::new(0usize));
    let right_detection_event_clone = right_detection_event.clone();

    // Emit light pulses from the vehicle's center.
    world.create_event_with_callback(
        Vector2D::new(0.0, 0.0),
        // Record the events at which the pulses reach the mirrors.
        move |world, detection| {
            if let DetectionObject::MObject(id) = detection.detection_object {
                if id == left_mirror {
                    *left_detection_event_clone.borrow_mut() =
                        world.create_event_at(detection.event_detection_position)
                }
                if id == right_mirror {
                    *right_detection_event_clone.borrow_mut() =
                        world.create_event_at(detection.event_detection_position)
                }
            }
        },
    );

    // Allow both pulses to reach their mirrors, with a small time margin.
    world.process_time(3.0);
    world.process_time(0.1);

    let left_event = world
        .event(&left_detection_event.borrow())
        .expect("left pulse should reach the mirror");
    let right_event = world
        .event(&right_detection_event.borrow())
        .expect("right pulse should reach the mirror");

    let left_lab_time = left_event.time;
    let right_lab_time = right_event.time;

    println!(
        "Laboratory frame — mirror detection times: left = {left_lab_time}, right = {right_lab_time}"
    );
    assert_ne!(left_lab_time, right_lab_time);

    let left_observation = world
        .observe_event(&left_detection_event.borrow())
        .expect("left pulse should reach the mirror");
    let right_observation = world
        .observe_event(&right_detection_event.borrow())
        .expect("right pulse should reach the mirror");

    if let (
        EventObservation::Visible(MVector {
            time: left_vehicle_time,
            ..
        }),
        EventObservation::Visible(MVector {
            time: right_vehicle_time,
            ..
        }),
    ) = (left_observation, right_observation)
    {
        println!(
            "Vehicle frame — mirror detection times: left = {left_vehicle_time}, right = {right_vehicle_time}"
        );
        assert!((left_vehicle_time - right_vehicle_time).abs() < 10e-6);
    }


}
