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
    All
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
    /// Position of lower object when collision happen
    pub contact_point_object_a: Vector2D<f64>,
    /// Position of higher object when collision happen
    pub contact_point_object_b: Vector2D<f64>,
}

/// The endpoint state used to detect one interval's collision geometry.
#[derive(Clone, Debug)]
pub(crate) struct CollisionSnapshot {
    pub(crate) participant: CollisionObject,
    pub(crate) old_position: MVector<f64>,
    pub(crate) new_position: MVector<f64>,
    pub(crate) old_velocity: Vector2D<f64>,
    pub(crate) new_velocity: Vector2D<f64>,
    pub(crate) radius: f64,
    pub(crate) collision_group: CollisionGroup,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct SpatialAabb {
    pub(crate) min: Vector2D<f64>,
    pub(crate) max: Vector2D<f64>,
}

impl CollisionSnapshot {
    fn interval_duration(&self) -> f64 {
        (self.new_position.time - self.old_position.time).abs()
    }

    fn lorentz_gamma(velocity: Vector2D<f64>) -> f64 {
        (1.0 - velocity.length_squared()).sqrt().recip()
    }

    /// A scalar bound enclosing the Lorentz-transformed ellipse and the
    /// endpoint-velocity approximation error over this interval.
    pub(crate) fn effective_radius(&self) -> f64 {
        let gamma = Self::lorentz_gamma(self.old_velocity)
            .max(Self::lorentz_gamma(self.new_velocity));
        let velocity_change = (self.new_velocity - self.old_velocity).length();
        self.radius * gamma + velocity_change * self.interval_duration()
    }

    /// Returns an AABB enclosing the bound throughout the center trajectory.
    pub(crate) fn swept_aabb(&self) -> SpatialAabb {
        let extent = self.effective_radius();
        SpatialAabb {
            min: Vector2D::new(
                self.old_position.pos.x.min(self.new_position.pos.x) - extent,
                self.old_position.pos.y.min(self.new_position.pos.y) - extent,
            ),
            max: Vector2D::new(
                self.old_position.pos.x.max(self.new_position.pos.x) + extent,
                self.old_position.pos.y.max(self.new_position.pos.y) + extent,
            ),
        }
    }

    pub(crate) fn position_at(&self, fraction: f64) -> MVector<f64> {
        self.old_position + (self.new_position - self.old_position) * fraction
    }

    pub(crate) fn separated_at_end(
        &self,
        other: &CollisionSnapshot,
        separation_tolerance: f64,
    ) -> bool {
        let distance = (self.new_position.pos - other.new_position.pos).length();
        distance > self.effective_radius() + other.effective_radius() + separation_tolerance
    }

    /// Finds the earliest contact fraction for two linear center trajectories.
    pub(crate) fn earliest_contact_fraction(
        &self,
        other: &CollisionSnapshot,
        detection_tolerance: f64,
    ) -> Option<f64> {
        if self.radius == 0.0 && other.radius == 0.0 {
            return None;
        }

        let relative_start = self.old_position.pos - other.old_position.pos;
        let relative_delta = (self.new_position.pos - self.old_position.pos)
            - (other.new_position.pos - other.old_position.pos);
        let bound = self.effective_radius() + other.effective_radius()
            + detection_tolerance;
        let bound_squared = bound * bound;
        let c = relative_start.length_squared() - bound_squared;

        if c <= 0.0 {
            return Some(0.0);
        }

        let a = relative_delta.length_squared();
        if a == 0.0 {
            return None;
        }
        let b = 2.0 * Vector2D::dot(relative_start, relative_delta);
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return None;
        }

        let sqrt_discriminant = discriminant.sqrt();
        let first = (-b - sqrt_discriminant) / (2.0 * a);
        let second = (-b + sqrt_discriminant) / (2.0 * a);
        [first, second]
            .into_iter()
            .filter(|fraction| (0.0..=1.0).contains(fraction))
            .min_by(|left, right| left.total_cmp(right))
    }
}

pub(crate) struct CollisionCalculator;

impl CollisionCalculator {
    pub(crate) fn calculate_collisions(
        &self,
        snapshots: &[CollisionSnapshot],
        cell_size: f64,
        detection_tolerance: f64,
        configured_pairs: &BTreeSet<CollisionGroupPair>,
        active_pairs: &BTreeSet<(CollisionObject, CollisionObject)>,
    ) -> Vec<Collision> {
        let mut spatial_hash: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
        for (index, snapshot) in snapshots.iter().enumerate() {
            let bounds = snapshot.swept_aabb();
            let min_x = (bounds.min.x / cell_size).floor() as i64;
            let max_x = (bounds.max.x / cell_size).floor() as i64;
            let min_y = (bounds.min.y / cell_size).floor() as i64;
            let max_y = (bounds.max.y / cell_size).floor() as i64;
            for x in min_x..=max_x {
                for y in min_y..=max_y {
                    spatial_hash.entry((x, y)).or_default().push(index);
                }
            }
        }

        let mut candidate_pairs = BTreeSet::new();
        for participants in spatial_hash.values() {
            for (position, &first) in participants.iter().enumerate() {
                for &second in &participants[position + 1..] {
                    candidate_pairs.insert(if first < second {
                        (first, second)
                    } else {
                        (second, first)
                    });
                }
            }
        }

        let mut collisions = candidate_pairs
            .into_iter()
            .filter_map(|(first, second)| {
                let left = &snapshots[first];
                let right = &snapshots[second];
                if !left.collision_group.collision_group_matches(
                    &right.collision_group,
                    configured_pairs,
                ) {
                    return None;
                }
                let pair = if left.participant <= right.participant {
                    (left.participant, right.participant)
                } else {
                    (right.participant, left.participant)
                };
                if active_pairs.contains(&pair) {
                    return None;
                }
                let fraction = left.earliest_contact_fraction(right, detection_tolerance)?;
                let (object_a, object_b, point_a, point_b) = if left.participant <= right.participant {
                    (
                        left.participant,
                        right.participant,
                        left.position_at(fraction).pos,
                        right.position_at(fraction).pos,
                    )
                } else {
                    (
                        right.participant,
                        left.participant,
                        right.position_at(fraction).pos,
                        left.position_at(fraction).pos,
                    )
                };
                Some(Collision {
                    object_a,
                    object_b,
                    time: left.position_at(fraction).time,
                    contact_point_object_a: point_a,
                    contact_point_object_b: point_b,
                })
            })
            .collect::<Vec<_>>();

        collisions.sort_by(|left, right| {
            left.time
                .total_cmp(&right.time)
                .then_with(|| left.object_a.cmp(&right.object_a))
                .then_with(|| left.object_b.cmp(&right.object_b))
        });
        collisions
    }

    pub(crate) fn active_pairs_to_remove(
        &self,
        active_pairs: &BTreeSet<(CollisionObject, CollisionObject)>,
        snapshots: &[CollisionSnapshot],
        separation_tolerance: f64,
    ) -> BTreeSet<(CollisionObject, CollisionObject)> {
        let by_participant: BTreeMap<CollisionObject, &CollisionSnapshot> = snapshots
            .iter()
            .map(|snapshot| (snapshot.participant, snapshot))
            .collect();
        active_pairs
            .iter()
            .filter(|(first, second)| {
                let Some(left) = by_participant.get(first) else {
                    return true;
                };
                let Some(right) = by_participant.get(second) else {
                    return true;
                };
                left.separated_at_end(right, separation_tolerance)
            })
            .copied()
            .collect()
    }
}