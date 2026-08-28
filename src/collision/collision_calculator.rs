use rayon::iter::ParallelIterator;
use std::collections::HashMap;
use rayon::prelude::ParallelBridge;
use vector2d::Vector2D;
use crate::collision::hashgrid::HashGrid;
use crate::{Collision, CollisionObject, MVector, MWorld, ProcessTimeCallback, EPSILON};
use crate::m_object::MObject;

pub(crate) struct CollisionCalculator<'a> {
    pub(crate) world: &'a MWorld,
}

impl CollisionCalculator<'_> {
    pub(crate) fn detect_collisions(&self) -> Vec<ProcessTimeCallback> {
        (0..1usize)
            .into_iter()
            .chain(self.world.get_registered_objects().keys().into_iter().map(|e|*e))
            .par_bridge()
            .flat_map(|id|self.detect_collisions_for_object(id))
            .collect()
    }

    fn detect_collisions_for_object(&self, id: usize) -> Vec<ProcessTimeCallback> {
        self.detect_collisions_for_object_optional(id).unwrap_or_default()

    }

    fn detect_collisions_for_object_optional(&self, id: usize) -> Option<Vec<ProcessTimeCallback>> {
        let object = self.get_object_from_id(id)?;
        let collision_candidates = self.detect_collision_candidates(id, object);
        let mut res = vec![];
        for collision_candidate in collision_candidates {
            if let Some(collision_delta) =  Self::collision_detected(object, collision_candidate.1){
                res.push(ProcessTimeCallback::Collision(Collision{
                    object_a: Self::id_to_collision_object(id),
                    object_b: Self::id_to_collision_object(collision_candidate.0),
                    time: self.world.lab_time() + collision_delta,
                }))
            }
        }
        Some(res)
    }

    fn detect_collision_candidates(&self, object_id: usize, object: &MObject) -> HashMap<usize, &MObject> {
        self.world.get_hash_grid().get_candidates(object)
            .into_iter()
            .filter(|id| *id > object_id)
            .filter_map(|id|Some((id, self.get_object_from_id(id)?)))
            .filter(|(id, obj)|object.collision_group().collision_group_matches(obj.collision_group(), self.world.configured_pairs()))
            .collect()
    }

    fn get_object_from_id(&self, id: usize) -> Option<&MObject>{
        if id == 0{
            return Some(self.world.get_frame_object())
        }
        Some(&self.world.get_registered_objects().get(&id)?.0)
    }

    fn collision_detected(a: &MObject, b: &MObject) -> Option<f64> {
        if let Some((bigger, smaller)) = Self::compare_sizes(a,b){
            let detection_lines = smaller.get_detection_lines();
            let lorentz_matrix = MVector::lorentz_transform_matrix_with_precalculated_gamma(*bigger.get_velocity(), bigger.gamma());
            return detection_lines
                .into_iter()
                .filter_map(|l|Self::detection_line_intersects(bigger, l, lorentz_matrix))
                .next()
        }
        None
    }

    fn id_to_collision_object(id: usize) -> CollisionObject {
        match id {
            0 => CollisionObject::Frame,
            _ => CollisionObject::Object(id)
        }
    }

    fn compare_sizes<'a>(a: &'a MObject, b: &'a MObject) -> Option<(&'a MObject, &'a MObject)>{
        if a.get_radius() < EPSILON && b.get_radius() < EPSILON{
            return None;
        }
        match a.get_radius() > b.get_radius() {
            true => Some((a, b)),
            false => Some((b, a)),
        }
    }

    fn detection_line_intersects(reference_object: &MObject, points: (MVector<f64>, MVector<f64>, f64), lorentz_matrix: MVector<MVector<f64>>) -> Option<f64> {
        let radius_sq = reference_object.get_radius().powi(2);
        let reference_center = *reference_object.position();
        let end_point = (points.1 - reference_center).transform(lorentz_matrix);
        let end_pos = end_point.pos;
        if end_pos.length_squared() < radius_sq{
            return Some(points.2);
        }
        let start_point = (points.0 - reference_center).transform(lorentz_matrix);
        let start_pos = start_point.pos;

        let delta_x = end_pos.x - start_pos.x;
        let delta_y = end_pos.y - start_pos.y;
        if delta_x == 0.0 && delta_y == 0.0{
            return None
        }

        let nearest_point = match (delta_y, delta_x) {
            (0.0, _) => {
                Vector2D{
                    x: 0.0,
                    y: end_pos.y,
                }
            }
            (_, 0.0) => {
                Vector2D{
                    x: end_pos.x,
                    y: 0.0,
                }
            }
            _ => {
                let a = delta_y / delta_x;
                let b = end_pos.y - end_pos.x * a;
                let x = -b/(a + 1.0/a);
                Vector2D{
                    x,
                    y: a*x+b,
                }
            }
        };
        let max_x = end_pos.x.max(start_pos.x);
        let min_x = end_pos.x.min(start_pos.x);
        let max_y = end_pos.y.max(start_pos.y);
        let min_y = end_pos.y.min(start_pos.y);
        if nearest_point.x >= min_x && nearest_point.x <= max_x && nearest_point.y >= min_y && nearest_point.y <= max_y && nearest_point.length_squared() < radius_sq{
            return Some(points.2)
        }
        None
    }
}
