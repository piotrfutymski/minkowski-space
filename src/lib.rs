#![allow(dead_code)]
#![allow(clippy::assign_op_pattern)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(clippy::let_and_return)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub static UPDATE_RATIO: f64 = 1.0/120.0;

pub mod m_vector;
pub mod m_object;
pub mod object_tracker;
mod photon;
pub mod m_frame;