#![allow(dead_code)]
#![allow(clippy::assign_op_pattern)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(clippy::let_and_return)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub static MAX_SAFE_SPEED: f64 = 1.0 - 10e-6;

pub mod m_vector;
pub mod m_object;
pub mod object_tracker;
mod photon;
pub mod m_world;
pub mod config;
pub mod collision;
pub mod m_event;
pub mod observation;