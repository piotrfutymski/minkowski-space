#![allow(dead_code)]
#![allow(clippy::assign_op_pattern)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(clippy::let_and_return)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub static MAX_SAFE_SPEED: f64 = 1.0 - 10e-6;

pub mod m_vector;
 mod m_object;
 mod object_tracker;
 mod photon;
 mod m_world;
mod config;
mod collision;
mod m_event;
mod observation;

pub use collision::{Collision, CollisionGroup, CollisionGroupId, CollisionGroupPair};
pub use config::{ConfigError, MotionMode, ObjectConfig, StartPosition, WorldConfig};
pub use m_event::{DetectionObject, EventDetection};
pub use m_object::ObjectState;
pub use m_vector::MVector;
pub use m_world::{EventDetectionCallback, MWorld, ProcessTimeCallback};
pub use observation::{EventObservation, ObjectObservation, VisibleObjectObservation};
pub use vector2d::Vector2D;