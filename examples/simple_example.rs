use minkowski_space::{MWorld, ObjectConfig, ObjectObservation};
use vector2d::Vector2D;

fn main() {
    let mut world = MWorld::new();
    world.set_observer_velocity(Vector2D::new(0.3, 0.0));

    let object_id = world.register_object(ObjectConfig::at_position_with_const_speed(
        Vector2D::new(1.0, 1.0),
        Vector2D::new(0.6, 0.0),
    ));

    world.advance_by_proper_time(2.0);

    let object_observation = world.observe_object(&object_id).unwrap();
    match object_observation {
        ObjectObservation::Visible(visible_object) => {
            println!(
                "Object is visible at relative position {:?}; redshift factor: {}",
                visible_object.visible_position_in_observer_frame,
                1.0 / visible_object.relative_frequency - 1.0
            );
        }
        ObjectObservation::NotVisible => {
            println!("Object is not visible yet; only the past light cone is observable.");
        }
    }
}
