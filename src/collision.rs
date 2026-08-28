pub(crate) mod collision_calculator;
pub(crate) mod hashgrid;

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
pub enum CollisionGroup {
    Empty,
    CollisionGroup(CollisionGroupId),
    All,
}

impl From<u32> for CollisionGroup{
    fn from(value: u32) -> Self {
        Self::CollisionGroup(CollisionGroupId(value))
    }
}

/// A participant in collision detection.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum CollisionObject {
    Object(usize),
    Frame,
}

impl CollisionGroup {
    pub(crate) fn collision_group_matches(
        &self,
        other: &CollisionGroup,
        configured_pairs: &BTreeSet<CollisionGroupPair>,
    ) -> bool {
        match (self, other) {
            (CollisionGroup::All, _) | (_, CollisionGroup::All) => true,
            (CollisionGroup::Empty, _) | (_, CollisionGroup::Empty) => false,
            (CollisionGroup::CollisionGroup(id_a), CollisionGroup::CollisionGroup(id_b)) => {
                CollisionGroupPair(*id_a, *id_b).is_configured(configured_pairs)
            }
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionGroupPair(pub CollisionGroupId, pub CollisionGroupId);

impl CollisionGroupPair {
    pub(crate) fn canonical(self) -> Self {
        if self.0 <= self.1 {
            self
        } else {
            Self(self.1, self.0)
        }
    }

    pub(crate) fn is_configured(self, configured_pairs: &BTreeSet<Self>) -> bool {
        configured_pairs.contains(&self.canonical())
    }
}

/// A global fact emitted when two configured objects first touch.
#[derive(Clone, Debug, PartialEq)]
pub struct Collision {
    /// The first participant in the canonical pair.
    pub object_a: CollisionObject,
    /// The second participant in the canonical pair.
    pub object_b: CollisionObject,
    /// Coordinate time in the world's base frame.
    pub time: f64,
}
