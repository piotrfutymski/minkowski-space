use crate::collision::{CollisionGroup, CollisionGroupId, CollisionGroupPair};
use crate::m_vector::MVector;
use std::collections::BTreeSet;
use vector2d::Vector2D;

/// Errors returned when a physical or world configuration violates the
/// simulation invariants.
#[derive(Clone, Debug, PartialEq)]
pub enum ConfigError {
    NonFinite { field: &'static str },
    Negative { field: &'static str },
    NonPositive { field: &'static str },
    SuperluminalVelocity,
    UnsupportedOperation(&'static str),
    InvalidCollisionGroupPair,
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinite { field } => write!(f, "{field} must be finite"),
            Self::Negative { field } => write!(f, "{field} must not be negative"),
            Self::NonPositive { field } => write!(f, "{field} must be greater than zero"),
            Self::SuperluminalVelocity => write!(f, "velocity must be subluminal"),
            Self::UnsupportedOperation(op) => write!(f, "unsupported operation: {op}"),
            Self::InvalidCollisionGroupPair => {
                write!(f, "collision pair references an undefined group")
            }
        }
    }
}
impl std::error::Error for ConfigError {}

fn valid_number(value: f64, field: &'static str) -> Result<(), ConfigError> {
    if !value.is_finite() {
        return Err(ConfigError::NonFinite { field });
    }
    Ok(())
}
fn valid_velocity(v: Vector2D<f64>) -> Result<(), ConfigError> {
    valid_number(v.x, "velocity.x")?;
    valid_number(v.y, "velocity.y")?;
    if v.length_squared() >= crate::MAX_SAFE_SPEED_SQUARED {
        return Err(ConfigError::SuperluminalVelocity);
    }
    Ok(())
}

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
    /// Validates values also exposed by the public fields.
    pub fn validate(&self) -> Result<(), ConfigError> {
        match self.position {
            StartPosition::Position(p) => {
                valid_number(p.time, "position.time")?;
                valid_number(p.pos.x, "position.x")?;
                valid_number(p.pos.y, "position.y")?;
            }
            StartPosition::PositionNow(p) => {
                valid_number(p.x, "position.x")?;
                valid_number(p.y, "position.y")?;
            }
        }
        valid_velocity(self.velocity)?;
        valid_number(self.radius, "radius")?;
        if self.radius < 0.0 {
            return Err(ConfigError::Negative { field: "radius" });
        }
        Ok(())
    }

    pub fn try_at_position_with_const_speed(
        initial_pos: Vector2D<f64>,
        initial_velocity: Vector2D<f64>,
    ) -> Result<Self, ConfigError> {
        let config = Self::at_position_with_const_speed(initial_pos, initial_velocity);
        config.validate().map(|_| config)
    }

    pub fn at_position_with_const_speed(
        initial_pos: Vector2D<f64>,
        initial_velocity: Vector2D<f64>,
    ) -> ObjectConfig {
        ObjectConfig {
            position: StartPosition::PositionNow(initial_pos),
            velocity: initial_velocity,
            radius: 0.0,
            motion_mode: MotionMode::AlwaysConstantVelocity,
            collision_group: CollisionGroup::Empty,
        }
    }

    pub fn at_position(initial_pos: Vector2D<f64>) -> ObjectConfig {
        ObjectConfig {
            position: StartPosition::PositionNow(initial_pos),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: CollisionGroup::Empty,
        }
    }
    pub fn default_with_group(collision_group: CollisionGroup) -> ObjectConfig {
        ObjectConfig {
            position: StartPosition::Position(Default::default()),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group,
        }
    }
}

impl Default for ObjectConfig {
    fn default() -> Self {
        Self {
            position: StartPosition::Position(Default::default()),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: CollisionGroup::Empty,
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
    pub frame_collision_radius: f64,
}

impl WorldConfig {
    /// Allocates a group identifier owned by this configuration.
    pub fn define_collision_group(&mut self) -> CollisionGroupId {
        let id = CollisionGroupId(
            self.collision_groups
                .iter()
                .map(|group| group.0)
                .max()
                .map_or(0, |id| id.saturating_add(1)),
        );
        self.collision_groups.insert(id);
        id
    }

    pub fn validate(&self) -> Result<(), ConfigError> {
        valid_number(self.proper_time_step, "proper_time_step")?;
        if self.proper_time_step <= 0.0 {
            return Err(ConfigError::NonPositive {
                field: "proper_time_step",
            });
        }
        valid_number(self.spatial_hash_cell_size, "spatial_hash_cell_size")?;
        if self.spatial_hash_cell_size <= 0.0 {
            return Err(ConfigError::NonPositive {
                field: "spatial_hash_cell_size",
            });
        }
        valid_number(self.frame_collision_radius, "frame_collision_radius")?;
        if self.frame_collision_radius < 0.0 {
            return Err(ConfigError::Negative {
                field: "frame_collision_radius",
            });
        }
        if self.collision_pairs.iter().any(|pair| {
            !self.collision_groups.contains(&pair.0) || !self.collision_groups.contains(&pair.1)
        }) {
            return Err(ConfigError::InvalidCollisionGroupPair);
        }
        Ok(())
    }
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            proper_time_step: 1.0 / 120.0,
            spatial_hash_cell_size: 1.0,
            collision_groups: BTreeSet::new(),
            collision_pairs: BTreeSet::new(),
            frame_collision_group: CollisionGroup::Empty,
            frame_collision_radius: 0.0,
        }
    }
}
