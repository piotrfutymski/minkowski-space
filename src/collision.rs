//! Public collision facts and collision-group identifiers.

use crate::m_vector::MVector;
use std::collections::{BTreeMap, BTreeSet};
use vector2d::Vector2D;

/// A collision group configured before a world is created.
///
/// Group identifiers are opaque and can only be obtained from
/// [`crate::WorldConfig::define_collision_group`]. An object may belong to at
/// most one group.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionGroupId(pub(crate) u32);

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionGroupPair(pub CollisionGroupId, pub CollisionGroupId);

/// A global fact emitted when two configured objects first touch.
#[derive(Debug, PartialEq)]
pub struct Collision {
    /// The lower object identifier in the canonical pair.
    pub object_a: usize,
    /// The higher object identifier in the canonical pair.
    pub object_b: usize,
    /// Coordinate time in the world's base frame.
    pub time: f64,
    /// Contact point in the world's base frame.
    pub contact_point: Vector2D<f64>,
}

pub(crate) struct  CollisionCalculator{
}

impl CollisionCalculator {
    pub(crate) fn calculate_collisions(&self) -> Vec<Collision> {
        //TODO IMPLEMENT
        vec![]
    }
}