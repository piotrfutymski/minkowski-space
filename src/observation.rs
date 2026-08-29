use crate::m_vector::MVector;
use vector2d::Vector2D;

/// The apparent state of an object reconstructed from light received by the observer.
///
/// The observation is based on photons that have already reached the observer,
/// rather than on the object's simultaneous state in the laboratory frame. As a
/// result, the reported position and orientation describe how the object is
/// seen, including light-travel-time effects and the observer's Lorentz frame.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VisibleObjectObservation {
    /// Apparent spatial position in the observer's instantaneous frame.
    ///
    /// Use this if you want to simulate where object is visible for observer
    /// In other words it is object position in light cone of observer and transformed to observer frame
    pub visible_position_in_observer_frame: Vector2D<f64>,
    /// x-basis vector of matrix applying effects visible by observer.
    ///
    /// It is required to use it to obtain effects like Penrose-Terrell rotation
    pub basis_x: Vector2D<f64>,
    /// y-basis vector of matrix applying effects visible by observer.
    ///
    /// It is required to use it to obtain effects like Penrose-Terrell rotation
    pub basis_y: Vector2D<f64>,
    /// Ratio of the observed photon frequency to the source frequency.
    ///
    /// A value greater than `1` represents a blueshift and a value less than
    /// `1` represents a redshift. The value is estimated from the interval
    /// between the last two received photons from the object's center. It can be used to demonstrate doppler effect
    pub relative_frequency: f64,
    /// Apparent spatial position in the lab frame.
    ///
    /// This vector has value of [`visible_position_in_observer_frame`] but before transformation to observer frame
    pub visible_position_in_lab_frame: MVector<f64>,
}

/// Result of observing a registered object.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ObjectObservation {
    /// A sufficiently complete light-based observation is available.
    Visible(VisibleObjectObservation),
    /// No observation is available yet.
    NotVisible,
}

/// Observation of a spacetime event by the world's observer.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum EventObservation {
    /// The event lies in the observer's past or current light cone.
    Visible(MVector<f64>),
    /// The event is not yet observable by the observer.
    NotVisible,
}
