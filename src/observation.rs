use crate::m_vector::MVector;
use vector2d::Vector2D;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VisibleObjectObservation {
    pub relative_position: Vector2D<f64>,
    pub basis_x: Vector2D<f64>,
    pub basis_y: Vector2D<f64>,
    pub relative_frequency: f64,
    pub visible_position: MVector<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ObjectObservation {
    Visible(VisibleObjectObservation),
    NotVisible,
}

/// Observation of a spacetime event by the world's observer.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum EventObservation {
    /// The event is in the observer's past light cone. The position is in the
    /// observer's current frame.
    Visible(MVector<f64>),
    /// The event cannot yet be observed by the observer.
    NotVisible,
}
