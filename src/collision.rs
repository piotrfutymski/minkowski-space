pub(crate) mod collision_calculator;
pub(crate) mod hashgrid;

use crate::m_vector::MVector;
use crate::m_world::ObjectSelection;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use vector2d::Vector2D;

/// A collision group configured before a world is created.
///
/// Group identifiers are opaque and can only be obtained from
/// [`crate::WorldConfig::define_collision_group`]. An object may belong to at
/// most one group.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionGroupId(pub(crate) u32);

/// Defines how an object participates in collision filtering.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum CollisionGroup {
    /// The object does not participate in collisions.
    Empty,
    /// A user-defined collision group.
    CollisionGroup(CollisionGroupId),
    /// Matches every collision group.
    All,
}

impl From<u32> for CollisionGroup {
    fn from(value: u32) -> Self {
        Self::CollisionGroup(CollisionGroupId(value))
    }
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

/// A pair of collision groups that is allowed to interact.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionGroupPair(
    /// First group in the pair.
    pub CollisionGroupId,
    /// Second group in the pair.
    pub CollisionGroupId,
);

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

/// The canonical identity of two objects in contact.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionPair(
    /// First object in the pair.
    pub ObjectSelection,
    /// Second object in the pair.
    pub ObjectSelection,
);

impl CollisionPair {
    /// Creates a pair in canonical order, so `(a, b)` equals `(b, a)`.
    pub fn new(a: ObjectSelection, b: ObjectSelection) -> Self {
        if a <= b { Self(a, b) } else { Self(b, a) }
    }

    /// Returns `true` when this pair contains `object`.
    pub fn contains(&self, object: ObjectSelection) -> bool {
        self.0 == object || self.1 == object
    }
}

/// A collision detected during a simulation step.
#[derive(Clone, Debug, PartialEq)]
pub struct Collision {
    /// The first participant in the canonical pair.
    pub object_a: ObjectSelection,
    /// The second participant in the canonical pair.
    pub object_b: ObjectSelection,
    /// Spacetime position of the contact in laboratory coordinates.
    pub position: MVector<f64>,
}
