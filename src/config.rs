use crate::collision::{CollisionGroup, CollisionGroupId, CollisionGroupPair};
use crate::m_vector::MVector;
use std::collections::BTreeSet;
use vector2d::Vector2D;

/// Errors returned when a physical or world configuration violates the
/// simulation invariants.
#[derive(Clone, Debug, PartialEq)]
pub enum ConfigError {
    /// A configuration value is not finite (`NaN`, positive infinity, or negative infinity).
    NonFinite { field: &'static str },
    /// A configuration value is negative although only non-negative values are valid.
    Negative { field: &'static str },
    /// A configuration value must be greater than zero.
    NonPositive { field: &'static str },
    /// A velocity is equal to or greater than the speed of light.
    SuperluminalVelocity,
    /// The selected operation is not supported by the object or world.
    UnsupportedOperation(&'static str),
    /// A collision pair refers to a group that is not defined in the configuration.
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

/// The initial position of a registered object.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum StartPosition {
    /// An explicit spacetime position in laboratory coordinates.
    Position(MVector<f64>),
    /// A spatial position at the world's current laboratory time.
    PositionNow(Vector2D<f64>),
}

/// Immutable physical configuration used when spawning an object.
#[derive(Copy, Clone, Debug)]
pub struct ObjectConfig {
    /// Initial position in laboratory coordinates.
    pub position: StartPosition,
    /// Initial velocity in units where the speed of light is `1`.
    pub velocity: Vector2D<f64>,
    /// Object radius in spatial units.
    pub radius: f64,
    /// Integration strategy used by the object.
    pub motion_mode: MotionMode,
    /// Collision group assigned to the object.
    pub collision_group: CollisionGroup,
}

impl ObjectConfig {
    /// Validates all values exposed by the public fields.
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

    /// Creates and validates a constant-velocity object configuration.
    ///
    /// Returns an error when the initial velocity or position is invalid.
    pub fn try_at_position_with_const_speed(
        initial_pos: Vector2D<f64>,
        initial_velocity: Vector2D<f64>,
    ) -> Result<Self, ConfigError> {
        let config = Self::at_position_with_const_speed(initial_pos, initial_velocity);
        config.validate().map(|_| config)
    }

    /// Creates a constant-velocity object configuration at a spatial position.
    ///
    /// The position is interpreted at the world's current laboratory time.
    /// Call [`Self::validate`] before using untrusted input.
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

    /// Creates a dynamic object configuration at a spatial position.
    ///
    /// The initial velocity and acceleration are zero.
    pub fn at_position(initial_pos: Vector2D<f64>) -> ObjectConfig {
        ObjectConfig {
            position: StartPosition::PositionNow(initial_pos),
            velocity: Default::default(),
            radius: 0.0,
            motion_mode: MotionMode::Dynamic,
            collision_group: CollisionGroup::Empty,
        }
    }
    /// Creates a dynamic object configuration at the spacetime origin.
    ///
    /// The object is assigned the supplied collision group.
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
    /// Observer proper-time step used by dynamic integration.
    pub proper_time_step: f64,
    /// Spatial size of a cell used by broad-phase collision detection.
    pub spatial_hash_cell_size: f64,
    /// Collision groups defined for this world.
    pub collision_groups: BTreeSet<CollisionGroupId>,
    /// Pairs of groups that are allowed to collide.
    pub collision_pairs: BTreeSet<CollisionGroupPair>,
    /// Collision group assigned to the observer.
    pub observer_collision_group: CollisionGroup,
    /// Collision radius assigned to the observer.
    pub observer_collision_radius: f64,
}

impl WorldConfig {
    /// Creates a default world configuration with the supplied collision pairs.
    ///
    /// Each tuple contains the numeric IDs of two collision groups. The groups
    /// are added to the configuration automatically.
    pub fn with_collisions(collisions: Vec<(u32, u32)>) -> Self {
        let mut res = Self::default();
        collisions.into_iter().for_each(|(l, r)| {
            let l = CollisionGroupId(l);
            let r = CollisionGroupId(r);
            res.collision_groups.insert(l);
            res.collision_groups.insert(r);
            res.collision_pairs.insert(CollisionGroupPair(l, r));
        });
        res
    }

    /// Allocates a group identifier owned by this configuration.
    ///
    /// The returned ID is inserted into [`Self::collision_groups`] and can be
    /// used to construct a [`CollisionGroup::CollisionGroup`].
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

    /// Validates the physical and numerical invariants of this configuration.
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
        valid_number(self.observer_collision_radius, "frame_collision_radius")?;
        if self.observer_collision_radius < 0.0 {
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
            observer_collision_group: CollisionGroup::Empty,
            observer_collision_radius: 0.0,
        }
    }
}
