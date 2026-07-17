use std::collections::BTreeSet;
use crate::m_vector::MVector;
use vector2d::Vector2D;
use crate::collision::{CollisionGroupId, CollisionGroupPair};

/// The integration strategy used by an object.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MotionMode {
    /// The trajectory is fixed at creation and can be integrated analytically.
    AlwaysConstantVelocity,
    /// Velocity and proper acceleration may be changed through the world API.
    Dynamic,
}

/// Immutable physical configuration used when spawning an object.
#[derive(Copy, Clone, Debug)]
pub struct ObjectConfig {
    pub position: MVector<f64>,
    pub velocity: Vector2D<f64>,
    pub radius: f64,
    pub motion_mode: MotionMode,
    pub collision_group: Option<CollisionGroupId>,
}

impl ObjectConfig {
    pub(crate) fn default() -> ObjectConfig {
        ObjectConfig{
            position: Default::default(),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: None,
        }
    }

    pub(crate) fn default_with_group(collision_group_id: Option<CollisionGroupId>) -> ObjectConfig {
        ObjectConfig{
            position: Default::default(),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: collision_group_id,
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
    pub frame_collision_group: Option<CollisionGroupId>,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            proper_time_step: 1.0 / 120.0,
            spatial_hash_cell_size: 1.0,
            collision_groups: BTreeSet::new(),
            collision_pairs: BTreeSet::new(),
            frame_collision_group: None,
        }
    }
}
