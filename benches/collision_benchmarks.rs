use criterion::{Criterion, criterion_group, criterion_main};
use minkowski_space::{
    CollisionMask, MVector, MWorld, MotionMode, ObjectConfig, StartPosition, Vector2D,
};
use std::hint::black_box;

fn config(
    position: Vector2D<f64>,
    velocity: Vector2D<f64>,
    monitoring: CollisionMask,
    monitorable: CollisionMask,
) -> ObjectConfig {
    ObjectConfig {
        position: StartPosition::Position(MVector::new(0.0, position)),
        velocity,
        radius: 0.05,
        motion_mode: MotionMode::AlwaysConstantVelocity,
        monitoring_collision_mask: monitoring,
        monitorable_collision_mask: monitorable,
    }
}

fn build_world(
    count: usize,
    grid_width: usize,
    velocity: Vector2D<f64>,
    dense: bool,
    filtered: bool,
) -> MWorld {
    let mut world = MWorld::new();
    // Layer 0 monitors layer 0 => collisions within group A.
    // Layer 1 monitors layer 1 => collisions within group B.
    // If filtered, objects monitor a layer that no other object is monitorable on.
    let (monitoring_group, monitorable_group) = if filtered {
        // group 0 monitors layer 0, group 1 monitors layer 1
        (
            CollisionMask::from_layers(&[0]),
            CollisionMask::from_layers(&[1]),
        )
    } else {
        // all objects monitor and are monitorable on layer 0
        (
            CollisionMask::from_layers(&[0]),
            CollisionMask::from_layers(&[0]),
        )
    };
    let spacing = if dense { 0.08 } else { 4.0 };
    for index in 0..count {
        let x = (index % grid_width) as f64 * spacing;
        let y = (index / grid_width) as f64 * spacing;
        world.register_object(config(
            Vector2D::new(x, y),
            velocity,
            monitoring_group,
            monitorable_group,
        ));
    }
    world
}

fn bench_sparse_stationary(c: &mut Criterion) {
    for &(name, count, width) in &[("1k", 1_000, 100), ("10k", 10_000, 1000)] {
        c.bench_function(&format!("sparse_stationary_{name}"), |bench| {
            bench.iter(|| {
                let mut world = build_world(count, width, Vector2D::new(0.0, 0.0), false, false);
                black_box(world.advance_by_proper_time(1.0 / 120.0));
            });
        });
    }
}

fn bench_sparse_moving(c: &mut Criterion) {
    c.bench_function("sparse_moving_1k", |bench| {
        bench.iter(|| {
            let mut world = build_world(1_000, 100, Vector2D::new(0.2, 0.1), false, false);
            black_box(world.advance_by_proper_time(1.0 / 120.0));
        });
    });
}

fn bench_dense_moving(c: &mut Criterion) {
    c.bench_function("dense_moving_1k", |bench| {
        bench.iter(|| {
            let mut world = build_world(1_000, 100, Vector2D::new(0.2, 0.1), true, false);
            black_box(world.advance_by_proper_time(1.0 / 120.0));
        });
    });
}

fn bench_fast_crossing(c: &mut Criterion) {
    c.bench_function("fast_crossing_1k", |bench| {
        bench.iter(|| {
            let mut world = build_world(1_000, 100, Vector2D::new(0.8, 0.0), false, false);
            black_box(world.advance_by_proper_time(1.0));
        });
    });
}

fn bench_group_filtered(c: &mut Criterion) {
    c.bench_function("group_filtered_1k", |bench| {
        bench.iter(|| {
            let mut world = build_world(1_000, 100, Vector2D::new(0.2, 0.1), true, true);
            black_box(world.advance_by_proper_time(1.0 / 120.0));
        });
    });
}

criterion_group!(
    collision_benches,
    bench_sparse_stationary,
    bench_sparse_moving,
    bench_dense_moving,
    bench_fast_crossing,
    bench_group_filtered,
);
criterion_main!(collision_benches);
