use std::cell::RefCell;
use std::rc::Rc;
use vector2d::Vector2D;
use minkowski_space::config::ObjectConfig;
use minkowski_space::m_event::DetectionObject;
use minkowski_space::m_world::MWorld;

fn main() {

    let mut world = MWorld::new();

    let velocity = Vector2D::new(0.8, 0.0);

    world.set_frame_velocity(velocity);

    let _left_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(Vector2D::new(-1.0, 0.0), velocity));
    let _right_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(Vector2D::new(1.0, 0.0), velocity));

    let mirror_detections = Rc::new(RefCell::new(Vec::new()));
    let md = mirror_detections.clone();

    let _light_event = world.create_event_with_callback(
        Vector2D::new(0.0, 0.0),
        move |world, detection| {
            if let DetectionObject::MObject(_object_id) = detection.detection_object{
                md.borrow_mut().push(world.create_event_at(detection.event_position));
            }
        },
    );

    let _events = world.process_time(5.0);

    println!("Mirror detection objects: {:?}", mirror_detections.borrow());

}