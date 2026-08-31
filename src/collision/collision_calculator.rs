use crate::collision::hashgrid::HashGrid;
use crate::m_object::MObject;
use crate::{Collision, CollisionMask, EPSILON, MVector, MWorld, ObjectSelection};
use rayon::prelude::{ParallelBridge, ParallelIterator};
use vector2d::Vector2D;

pub(crate) struct CollisionCalculator<'a> {
    pub(crate) world: &'a MWorld,
}

impl CollisionCalculator<'_> {
    pub(crate) fn detect_collisions(&self) -> Vec<Collision> {
        std::iter::once(ObjectSelection::Observer)
            .chain(
                self.world
                    .get_registered_objects()
                    .values()
                    .map(|(object, _)| object.selection()),
            )
            .par_bridge()
            .flat_map(|selection| self.detect_collisions_for_object(selection))
            .collect()
    }

    fn detect_collisions_for_object(&self, selection: ObjectSelection) -> Vec<Collision> {
        let Some(object) = self.world.get_object_with_selection(&selection) else {
            return Vec::new();
        };
        let collision_candidates = self.detect_collision_candidates(selection, object);
        let mut result = Vec::with_capacity(collision_candidates.len());

        for (candidate_selection, candidate) in collision_candidates {
            if let Some(position) = Self::collision_detected(object, candidate) {
                if object
                    .monitoring_collision_mask()
                    .mask_matches(candidate.monitorable_collision_mask())
                {
                    result.push(Collision {
                        monitoring: selection,
                        monitorable: candidate_selection,
                        position,
                    });
                }
                if object
                    .monitorable_collision_mask()
                    .mask_matches(candidate.monitoring_collision_mask())
                {
                    result.push(Collision {
                        monitoring: candidate_selection,
                        monitorable: selection,
                        position,
                    });
                }
            }
        }
        result
    }

    fn detect_collision_candidates(
        &self,
        selection: ObjectSelection,
        object: &MObject,
    ) -> Vec<(ObjectSelection, &MObject)> {
        if !object.is_collision_detection_enabled()
            || *object.monitoring_collision_mask() == CollisionMask::EMPTY
        {
            return vec![];
        }
        self.world
            .get_hash_grid()
            .get_candidates(object)
            .into_iter()
            .filter(|candidate_selection| *candidate_selection > selection)
            .filter_map(|candidate_selection| {
                self.world
                    .get_object_with_selection(&candidate_selection)
                    .map(|object| (candidate_selection, object))
            })
            .filter(|(_, candidate)| {
                candidate.is_collision_detection_enabled()
                    && (object
                        .monitoring_collision_mask()
                        .mask_matches(candidate.monitorable_collision_mask())
                        || object
                            .monitorable_collision_mask()
                            .mask_matches(candidate.monitoring_collision_mask()))
            })
            .collect()
    }

    fn collision_detected(a: &MObject, b: &MObject) -> Option<MVector<f64>> {
        if let Some((bigger, smaller)) = Self::compare_sizes(a, b) {
            let detection_lines = smaller.get_detection_lines();
            let lorentz_matrix = MVector::lorentz_transform_matrix_with_precalculated_gamma(
                *bigger.get_velocity(),
                bigger.gamma(),
            );
            return detection_lines
                .into_iter()
                .filter_map(|l| Self::detection_line_intersects(bigger, l, lorentz_matrix))
                .next();
        }
        None
    }

    fn compare_sizes<'a>(a: &'a MObject, b: &'a MObject) -> Option<(&'a MObject, &'a MObject)> {
        if a.get_radius() < EPSILON && b.get_radius() < EPSILON {
            return None;
        }
        match a.get_radius() > b.get_radius() {
            true => Some((a, b)),
            false => Some((b, a)),
        }
    }

    fn detection_line_intersects(
        reference_object: &MObject,
        points: (MVector<f64>, MVector<f64>),
        lorentz_matrix: MVector<MVector<f64>>,
    ) -> Option<MVector<f64>> {
        let radius_sq = reference_object.get_radius().powi(2);
        let reference_center = *reference_object.position();
        let end_point = (points.1 - reference_center).transform(lorentz_matrix);
        let end_pos = end_point.pos;
        if end_pos.length_squared() < radius_sq {
            return Some(points.1);
        }
        let start_point = (points.0 - reference_center).transform(lorentz_matrix);
        let start_pos = start_point.pos;

        let delta_x = end_pos.x - start_pos.x;
        let delta_y = end_pos.y - start_pos.y;
        if delta_x == 0.0 && delta_y == 0.0 {
            return None;
        }

        let nearest_point = match (delta_y, delta_x) {
            (0.0, _) => Vector2D {
                x: 0.0,
                y: end_pos.y,
            },
            (_, 0.0) => Vector2D {
                x: end_pos.x,
                y: 0.0,
            },
            _ => {
                let a = delta_y / delta_x;
                let b = end_pos.y - end_pos.x * a;
                let x = -b / (a + 1.0 / a);
                Vector2D { x, y: a * x + b }
            }
        };
        let max_x = end_pos.x.max(start_pos.x);
        let min_x = end_pos.x.min(start_pos.x);
        let max_y = end_pos.y.max(start_pos.y);
        let min_y = end_pos.y.min(start_pos.y);
        if nearest_point.x >= min_x
            && nearest_point.x <= max_x
            && nearest_point.y >= min_y
            && nearest_point.y <= max_y
            && nearest_point.length_squared() < radius_sq
        {
            return Some(points.1);
        }
        None
    }
}
