//! Demonstrates the relativity of simultaneity with two light pulses and two mirrors.
//!
//! At `t = 0`, the pulses leave the center of a vehicle moving at `v = 0.6`.
//! In the laboratory frame, the mirrors detect them at different times; in the
//! vehicle's frame, the detections are simultaneous.
//!
//! Analytically, with mirrors at `x = ±1` in the laboratory frame:
//!   * the left  pulse meets its mirror at `t = 1 / (1 + v) = 0.625`
//!   * the right pulse meets its mirror at `t = 1 / (1 - v) = 2.5`
//!
//! Therefore, the laboratory time difference is `1.875`, while the vehicle's
//! frame sees both detections at the same time.

use minkowski_space::{
    EventObservation, MVector, MWorld, ObjectConfig, ObjectSelection, ProcessTimeCallback,
};
use vector2d::Vector2D;

/// Speed of the vehicle in units of `c`.
const VEHICLE_SPEED: f64 = 0.6;
/// Distance of each mirror from the origin, in the laboratory frame.
const MIRROR_DISTANCE: f64 = 1.0;
/// Enough proper time for both pulses to be detected (`1 / (1 - v) = 2.5`) plus a margin.
const SIMULATION_TIME: f64 = 3.0;
const TOLERANCE: f64 = 1e-6;

fn main() {
    let vehicle_velocity = Vector2D::new(VEHICLE_SPEED, 0.0);
    let gamma = 1.0 / (1.0 - VEHICLE_SPEED * VEHICLE_SPEED).sqrt();

    let mut world = MWorld::new();
    world.set_observer_velocity(vehicle_velocity);

    // The mirrors are `MIRROR_DISTANCE` from the origin in the laboratory frame,
    // so their proper separation is γ · 2 · MIRROR_DISTANCE.
    let left_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(
        Vector2D::new(-MIRROR_DISTANCE, 0.0),
        vehicle_velocity,
    ));
    let right_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(
        Vector2D::new(MIRROR_DISTANCE, 0.0),
        vehicle_velocity,
    ));

    println!("v = {VEHICLE_SPEED}, γ = {gamma:.6}");
    println!(
        "mirror separation: {} (laboratory) / {:.6} (proper)",
        2.0 * MIRROR_DISTANCE,
        2.0 * MIRROR_DISTANCE * gamma
    );

    // Emit light pulses from the vehicle's center.
    world.create_event(Vector2D::new(0.0, 0.0));

    // Let both pulses reach their mirrors and collect the detections reported
    // by the simulation step, instead of registering a callback.
    let mut left_detection_position = None;
    let mut right_detection_position = None;
    for callback in world.advance_by_proper_time(SIMULATION_TIME) {
        let ProcessTimeCallback::Event(detection) = callback else {
            continue;
        };
        let ObjectSelection::Object(id) = detection.detection_object else {
            continue;
        };
        if id == left_mirror {
            left_detection_position = Some(detection.event_detection_position);
        } else if id == right_mirror {
            right_detection_position = Some(detection.event_detection_position);
        }
    }

    let left_detection_position =
        left_detection_position.expect("left pulse should reach the mirror");
    let right_detection_position =
        right_detection_position.expect("right pulse should reach the mirror");

    // Record the detections as events, so they can be observed from any frame.
    let left_detection_event = world.create_event_at(left_detection_position);
    let right_detection_event = world.create_event_at(right_detection_position);

    let left_lab_time = world.event(&left_detection_event).unwrap().time;
    let right_lab_time = world.event(&right_detection_event).unwrap().time;

    let expected_left = MIRROR_DISTANCE / (1.0 + VEHICLE_SPEED);
    let expected_right = MIRROR_DISTANCE / (1.0 - VEHICLE_SPEED);
    println!(
        "Laboratory frame — detection times: left = {left_lab_time:.6} (expected {expected_left:.6}), \
        right = {right_lab_time:.6} (expected {expected_right:.6}), Δt = {:.6} (expected {:.6})",
        right_lab_time - left_lab_time,
        expected_right - expected_left
    );

    let left_observation = world.observe_event(&left_detection_event).unwrap();
    let right_observation = world.observe_event(&right_detection_event).unwrap();

    let EventObservation::Visible(MVector {
        time: left_vehicle_time,
        ..
    }) = left_observation
    else {
        panic!()
    };
    let EventObservation::Visible(MVector {
        time: right_vehicle_time,
        ..
    }) = right_observation
    else {
        panic!()
    };

    let vehicle_delta = right_vehicle_time - left_vehicle_time;

    println!(
        "Vehicle frame — detection times: left = {left_vehicle_time:.6}, \
        right = {right_vehicle_time:.6}, Δt = {vehicle_delta:.6} (expected 0, tolerance {TOLERANCE:e})"
    );
    println!(
        "=> the detections are {} in the vehicle's frame",
        if vehicle_delta.abs() < TOLERANCE {
            "simultaneous"
        } else {
            "NOT simultaneous"
        }
    );
}
