use minkowski_space::MWorld;
use vector2d::Vector2D;

fn main() {
    let mut world = MWorld::new();

    world.set_observer_velocity(Vector2D::new(0.8, 0.0));
    world.advance_by_proper_time(1.0);

    world.set_observer_velocity(Vector2D::new(-0.8, 0.0));
    world.advance_by_proper_time(1.0);

    println!("The observation frame was moving in reference to the lab frame");
    println!("Moving frame and lab frame has the same position: (0.0,0.0)");
    println!(
        "But in moving frame less! time has passed: {} < {}",
        world.observer_tau(),
        world.lab_time()
    );
    let gamma = 1.0 / (1.0 - 0.8_f64.powi(2)).sqrt();
    println!(
        "In fact it can be easily computed: {} == {}",
        world.observer_tau() * gamma,
        world.lab_time()
    );
    assert!(world.observer_tau() < world.lab_time())
}
