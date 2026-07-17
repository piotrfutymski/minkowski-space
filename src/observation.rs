use vector2d::Vector2D;
use crate::m_vector::MVector;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VisibleObjectObservation {
    pub relative_position: Vector2D<f64>,
    pub basis_x: Vector2D<f64>,
    pub basis_y: Vector2D<f64>,
    pub relative_frequency: f64,
    pub visible_position: MVector<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ObjectObservation{
    Visible(VisibleObjectObservation),
    NotVisible
}