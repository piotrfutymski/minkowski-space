use crate::m_vector::MVector;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PhotonEmittingPosition{
    CENTER,
    BACK,
    FRONT,
    TOP,
    BOTTOM
}

#[derive(Clone)]
pub struct Photon{
    m_pos: MVector<f64>,
    photon_pos: PhotonEmittingPosition
}

impl Photon{
    pub fn new(m_pos: MVector<f64>, photon_pos: PhotonEmittingPosition) -> Self {
        Self{
            m_pos,
            photon_pos,
        }
    }
}

impl Photon{
    pub fn get_emmit_type(&self) -> PhotonEmittingPosition {
        self.photon_pos
    }

    pub fn get_emmit_pos(&self) -> MVector<f64> {
        self.m_pos
    }
}