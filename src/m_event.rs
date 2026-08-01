use vector2d::Vector2D;
use crate::collision::CollisionGroup;
use crate::m_object::MObject;
use crate::m_vector::MVector;

/// Information about a detected event, passed to the [`on_detection`] callback.
///
/// [`on_detection`]: crate::m_world::MWorld::create_event_with_callback
#[derive(Debug, Copy, Clone)]
pub struct EventDetection {
    pub event_id: usize,
    pub detection_object: DetectionObject,
    pub event_position: MVector<f64>,
    pub world_time: f64,
}

#[derive(Debug, Copy, Clone)]
pub enum DetectionObject {
    MObject(usize),
    FrameObject
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
    pub(crate) fn position(&self) -> MVector<f64> {
        self.event_pos
    }

    pub(crate) fn new(event_pos: MVector<f64>, collision_group: CollisionGroup) -> Self {
        Self{
            event_pos, collision_group
        }
    }
}