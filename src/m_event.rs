use vector2d::Vector2D;
use crate::collision::{CollisionGroup, CollisionGroupId};
use crate::config::MotionMode;
use crate::m_vector::MVector;
#[derive(Debug)]
pub struct EventDetection {
    pub event_id: usize,
    pub object_id: usize,
    pub time: MVector<f64>
}

#[derive(Copy, Clone, Debug)]
pub struct EventToCheck {
    pub event_id: usize,
    pub event_pos: MVector<f64>,
}

#[derive(Debug, Clone)]
pub struct MEvent{

    event_pos: MVector<f64>,
    collision_group: CollisionGroup,

}

impl MEvent {
    pub(crate) fn new(event_pos: MVector<f64>, collision_group: CollisionGroup) -> Self {
        Self{
            event_pos, collision_group
        }
    }
}