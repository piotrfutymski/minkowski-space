#![allow(dead_code)]
#![allow(clippy::assign_op_pattern)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(clippy::let_and_return)]

pub static MAX_SAFE_SPEED: f64 = 1.0 - 1e-6;
pub(crate) static MAX_SAFE_SPEED_SQUARED: f64 = MAX_SAFE_SPEED * MAX_SAFE_SPEED;

mod collision;
mod config;
mod m_event;
mod m_object;
pub mod m_vector;
mod m_world;
mod object_tracker;
mod observation;
mod photon;

pub use collision::{
    Collision, CollisionGroup, CollisionGroupId, CollisionGroupPair, CollisionObject,
};
pub use config::{ConfigError, MotionMode, ObjectConfig, StartPosition, WorldConfig};
pub use m_event::{DetectionObject, EventDetection};
pub use m_object::ObjectState;
pub use m_vector::{Causality, MVector};
pub use m_world::{MWorld, ProcessTimeCallback};
pub use observation::{EventObservation, ObjectObservation, VisibleObjectObservation};
pub use vector2d::Vector2D;
