use vector2d::Vector2D;
use minkowski_space::config::ObjectConfig;
use minkowski_space::m_world::MWorld;

fn main() {

    let mut world = MWorld::new();

    let velocity = Vector2D::new(0.8, 0.0);

    world.set_frame_velocity(velocity);

    let left_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(Vector2D::new(-1.0, 0.0), velocity));
    let right_mirror = world.register_object(ObjectConfig::at_position_with_const_speed(Vector2D::new(1.0, 0.0), velocity));

    let light_event = world.create_event(Vector2D::new(0.0, 0.0));

    world.process_time(2.0);

}