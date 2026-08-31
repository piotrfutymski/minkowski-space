pub(crate) mod collision_calculator;
pub(crate) mod hashgrid;

use crate::m_vector::MVector;
use crate::m_world::ObjectSelection;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use vector2d::Vector2D;

/// Defines how an object participates in collision filtering.
/// Monitoring and Monitorable mas must match to detect collision
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CollisionMask {
    mask: u32,
}

impl From<u32> for CollisionMask {
    fn from(value: u32) -> Self {
        Self { mask: value }
    }
}

impl CollisionMask {
    /// Mask that matches any other mask.
    pub const ALL: CollisionMask = CollisionMask { mask: u32::MAX };
    /// Mask that matches no other mask.
    pub const EMPTY: CollisionMask = CollisionMask { mask: 0 };

    pub fn new(mask: u32) -> Self {
        mask.into()
    }

    pub fn from_layers(layers: &[u32]) -> Self {
        let mut res = 0;
        for i in layers {
            if *i < 32 {
                res = res | (1 << i)
            }
        }
        res.into()
    }
}

impl CollisionMask {
    pub(crate) fn mask_matches(&self, other: &CollisionMask) -> bool {
        self.mask & other.mask > 0
    }
}

/// The canonical identity of two objects in contact.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub(crate) struct CollisionPair(
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
    /// Monitoring participant
    pub monitoring: ObjectSelection,
    /// Monitorable participant
    pub monitorable: ObjectSelection,
    /// Spacetime position of the contact in laboratory coordinates.
    pub position: MVector<f64>,
}

#[cfg(test)]
mod tests {
    use crate::CollisionMask;

    #[test]
    fn test_masks() {
        let monitoring_first = CollisionMask::from_layers(&vec![2, 3]);
        let monitoring_second = CollisionMask::from_layers(&vec![1, 4]);
        let monitorable = CollisionMask::from_layers(&vec![0, 1]);
        assert_eq!(monitoring_first.mask, 12);
        assert_eq!(monitoring_second.mask, 18);
        assert_eq!(monitorable.mask, 3);
        assert!(!monitoring_first.mask_matches(&monitorable));
        assert!(monitoring_second.mask_matches(&monitorable));
    }
}
