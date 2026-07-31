use std::collections::BTreeSet;
use crate::m_vector::MVector;
use vector2d::Vector2D;
use crate::collision::{CollisionGroup, CollisionGroupId, CollisionGroupPair};

/// The integration strategy used by an object.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MotionMode {
    /// The trajectory is fixed at creation and can be integrated analytically.
    AlwaysConstantVelocity,
    /// Velocity and proper acceleration may be changed through the world API.
    Dynamic,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum StartPosition {
    Position(MVector<f64>),
    PositionNow(Vector2D<f64>),
}

/// Immutable physical configuration used when spawning an object.
#[derive(Copy, Clone, Debug)]
pub struct ObjectConfig {
    pub position: StartPosition,
    pub velocity: Vector2D<f64>,
    pub radius: f64,
    pub motion_mode: MotionMode,
    pub collision_group: CollisionGroup,
}

impl ObjectConfig {

    pub fn at_position_with_const_speed(initial_pos: Vector2D<f64>, initial_velocity: Vector2D<f64>) -> ObjectConfig {
        ObjectConfig{
            position: StartPosition::PositionNow(initial_pos),
            velocity: initial_velocity,
            radius: 0.0,
            motion_mode: MotionMode::AlwaysConstantVelocity,
            collision_group: CollisionGroup::Empty,
        }
    }

    pub fn at_position(initial_pos: Vector2D<f64>) -> ObjectConfig {
        ObjectConfig{
            position: StartPosition::PositionNow(initial_pos),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: CollisionGroup::Empty,
        }
    }
    pub fn default() -> ObjectConfig {
        ObjectConfig{
            position: StartPosition::Position(Default::default()),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: CollisionGroup::Empty,
        }
    }

    pub fn default_with_group(collision_group: CollisionGroup) -> ObjectConfig {
        ObjectConfig{
            position: StartPosition::Position(Default::default()),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group,
        }
    }
}

/// Configuration of a simulation world.
#[derive(Clone, Debug)]
pub struct WorldConfig {
    pub proper_time_step: f64,
    pub spatial_hash_cell_size: f64,
    pub collision_groups: BTreeSet<CollisionGroupId>,
    pub collision_pairs: BTreeSet<CollisionGroupPair>,
    pub frame_collision_group: CollisionGroup,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            proper_time_step: 1.0 / 120.0,
            spatial_hash_cell_size: 1.0,
            collision_groups: BTreeSet::new(),
            collision_pairs: BTreeSet::new(),
            frame_collision_group: CollisionGroup::Empty,
        }
    }
}
