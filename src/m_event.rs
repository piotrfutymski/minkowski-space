use vector2d::Vector2D;
use crate::collision::CollisionGroupId;
use crate::config::MotionMode;
use crate::m_vector::MVector;
#[derive(Debug)]
pub struct EventDetection {
    pub event: usize,
    pub object: usize,
}

#[derive(Debug)]
pub struct MEvent{

    event_pos: MVector<f64>,
    collision_group: Option<CollisionGroupId>,

}