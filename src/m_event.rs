use crate::ObjectSelection;
use crate::collision::CollisionGroup;
use crate::m_object::MObject;
use crate::m_vector::MVector;
use vector2d::Vector2D;

/// Information about a detected event, passed to the [`on_detection`] callback.
///
/// [`on_detection`]: crate::m_world::MWorld::create_event_with_callback
/// Information about an event detected during a simulation step.
#[derive(Debug, Copy, Clone)]
pub struct EventDetection {
    /// ID of the detected event.
    pub event_id: usize,
    /// Object that detected the event.
    pub detection_object: ObjectSelection,
    /// Detection position in laboratory coordinates.
    pub event_detection_position: MVector<f64>,
}

/// An internal event candidate used during event detection.
#[derive(Copy, Clone, Debug)]
pub struct EventToCheck {
    /// ID of the event.
    pub event_id: usize,
    /// Event position in laboratory coordinates.
    pub event_pos: MVector<f64>,
}

#[derive(Debug, Clone)]
pub struct MEvent {
    event_pos: MVector<f64>,
    collision_group: CollisionGroup,
}

impl MEvent {
    pub(crate) fn position(&self) -> &MVector<f64> {
        &self.event_pos
    }

    pub(crate) fn collision_group(&self) -> &CollisionGroup {
        &self.collision_group
    }

    pub(crate) fn new(event_pos: MVector<f64>, collision_group: CollisionGroup) -> Self {
        Self {
            event_pos,
            collision_group,
        }
    }
}
