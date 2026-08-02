use std::ops::{Add, Div, Mul, Sub};
use vector2d::Vector2D;
const IDENTITY_TRANSFORM: MVector<MVector<f64>> = MVector{
    pos: Vector2D {
        x: MVector{ pos: Vector2D { x: 1.0, y: 0.0 }, time: 0.0 },
        y: MVector{ pos: Vector2D { x: 0.0, y: 1.0 }, time: 0.0 },
    },
    time: MVector{
        pos: Vector2D{x: 0.0, y: 0.0},
        time: 1.0,
    }
};

/// A vector in a two-dimensional Minkowski spacetime.
///
/// The `time` component is stored separately from the two spatial components
/// in [`pos`]. For `MVector<f64>`, the interval convention used by this type
/// is `time² - x² - y²`; this is sometimes called the `(+--)` signature.
///
/// # Example
///
/// ```
/// use minkowski_space::{MVector, Vector2D};
///
/// let event = MVector::new(3.0, Vector2D::new(1.0, 2.0));
/// assert_eq!(event.time, 3.0);
/// assert_eq!(event.pos, Vector2D::new(1.0, 2.0));
/// ```
#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct MVector<T>{
    /// The two spatial components of the vector.
    pub pos: Vector2D<T>,
    /// The time component of the vector.
    pub time: T,
}

/// Adds the spatial and time components independently.
impl<T> Add for MVector<T> where T: Add<T, Output = T> + Copy{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self{
            pos: self.pos + rhs.pos,
            time: self.time + rhs.time,
        }
    }
}

/// Subtracts the spatial and time components independently.
impl<T> Sub for MVector<T>  where T: Sub<T, Output=T> + Copy{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self{
            pos: self.pos - rhs.pos,
            time: self.time - rhs.time,
        }
    }
}

/// Multiplies every component by a scalar.
impl<T> Mul<T> for MVector<T> where T: Mul<T, Output=T> + Copy{
    type Output = MVector<T>;
    fn mul(self, rhs: T) -> Self::Output {
        Self{
            pos: self.pos * rhs,
            time: self.time * rhs
        }
    }
}

/// Divides every component by a scalar.
impl<T> Div<T> for MVector<T> where T: Div<T, Output=T> + Copy{
    type Output = MVector<T>;
    fn div(self, rhs: T) -> Self::Output {
        Self{
            pos: self.pos / rhs,
            time: self.time / rhs
        }
    }
}

/// The causal character of a spacetime vector.
///
/// Determined by the sign of `time² - x² - y²`:
/// - `TimeLike`: the interval is positive (inside the light-cone)
/// - `LightLike`: the interval is zero (on the light-cone)
/// - `SpaceLike`: the interval is negative (outside the light-cone)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Causality {
    /// The interval is positive – inside the light-cone.
    TimeLike,
    /// The interval is zero – on the light-cone (within floating-point tolerance).
    LightLike,
    /// The interval is negative – outside the light-cone.
    SpaceLike,
}

impl MVector<f64> {

    /// Creates a spacetime vector from its time and spatial components.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MVector, Vector2D};
    ///
    /// let vector = MVector::new(2.0, Vector2D::new(3.0, 4.0));
    /// assert_eq!(vector.length_squared(), -21.0);
    /// ```
    pub fn new(time: f64, pos: Vector2D<f64>) -> Self{
        Self{
            pos,
            time,
        }
    }
    /// Returns the signed squared Minkowski length, `time² - x² - y²`.
    ///
    /// A positive value denotes a time-like vector, a negative value a
    /// space-like vector, and zero a light-like vector (up to floating-point
    /// precision). Unlike [`length`](Self::length), this method preserves the
    /// causal sign.
    pub fn length_squared(&self) -> f64{
        self.time.powi(2) - self.pos.length_squared()
    }

    /// Absolute interval magnitude. Use `length_squared` and the causal
    /// predicates when the time-like/space-like sign matters.
    pub fn length(&self) -> f64{
        self.length_squared().abs().sqrt()
    }

    /// Classifies the causal character of the vector.
    ///
    /// Uses a scale-aware epsilon for the light-like case so that very
    /// large vectors are not misclassified due to floating-point noise.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MVector, Vector2D, Causality};
    ///
    /// assert_eq!(MVector::new(5.0, Vector2D::new(3.0, 0.0)).causal_character(), Causality::TimeLike);
    /// assert_eq!(MVector::new(1.0, Vector2D::new(1.0, 0.0)).causal_character(), Causality::LightLike);
    /// assert_eq!(MVector::new(0.0, Vector2D::new(1.0, 0.0)).causal_character(), Causality::SpaceLike);
    /// ```
    pub fn causal_character(&self) -> Causality {
        let interval = self.length_squared();
        if !interval.is_finite() {
            return Causality::SpaceLike;
        }
        let eps = 1e-12 * (1.0 + self.time.powi(2).max(self.pos.length_squared()));
        if interval > eps {
            Causality::TimeLike
        } else if interval < -eps {
            Causality::SpaceLike
        } else {
            Causality::LightLike
        }
    }

    /// Returns `true` if the vector is time-like.
    ///
    /// This is equivalent to `causal_character() == Causality::TimeLike`.
    pub fn is_time_like(&self) -> bool{
        self.causal_character() == Causality::TimeLike
    }

    /// Returns `true` if the vector is time-like or light-like.
    pub fn is_time_or_light_like(&self) -> bool{
        matches!(self.causal_character(), Causality::TimeLike | Causality::LightLike)
    }

    /// Returns `true` if the vector is space-like.
    ///
    /// This is equivalent to `causal_character() == Causality::SpaceLike`.
    pub fn is_space_like(&self) -> bool{
        self.causal_character() == Causality::SpaceLike
    }

    /// Returns `true` if the vector has a zero Minkowski interval within a
    /// scale-aware floating-point tolerance.
    ///
    /// Non-finite intervals are never considered light-like.
    pub fn is_light_like(&self) -> bool{
        self.causal_character() == Causality::LightLike
    }

    /// Computes the Euclidean dot product `x₁*x₂ + y₁*y₂ + t₁*t₂`.
    ///
    /// This is **not** the Minkowski metric – use [`length_squared`](Self::length_squared)
    /// for the relativistic interval. This method is `pub(crate)` because it
    /// is only needed for the matrix-multiplication helpers inside this module.
    pub(crate) fn euclidean_dot(&self, rhs: &MVector<f64>) -> f64{
        self.pos.x * rhs.pos.x + self.pos.y * rhs.pos.y + self.time * rhs.time
    }

    /// Applies a 3×3 linear transformation represented by nested vectors.
    ///
    /// The outer vector contains the rows for `x`, `y`, and `time`; each row
    /// is dotted with `self` using [`euclidean_dot`](Self::euclidean_dot).
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MVector, Vector2D};
    ///
    /// let value = MVector::new(2.0, Vector2D::new(3.0, 4.0));
    /// let identity = MVector {
    ///     pos: Vector2D {
    ///         x: MVector::new(0.0, Vector2D::new(1.0, 0.0)),
    ///         y: MVector::new(0.0, Vector2D::new(0.0, 1.0)),
    ///     },
    ///     time: MVector::new(1.0, Vector2D::new(0.0, 0.0)),
    /// };
    /// let transformed = value.transform(identity);
    /// assert_eq!(transformed, value);
    /// ```
    pub fn transform(&self, matrix: MVector<MVector<f64>>) -> Self {
        MVector{
            pos: Vector2D {
                x: matrix.pos.x.euclidean_dot(self),
                y: matrix.pos.y.euclidean_dot(self)
            },
            time: matrix.time.euclidean_dot(self),
        }
    }

    /// Returns the zero spacetime vector.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MVector, Vector2D};
    ///
    /// assert_eq!(MVector::zero(), MVector::new(0.0, Vector2D::new(0.0, 0.0)));
    /// ```
    pub fn zero() -> Self{
        Self{
            pos: Vector2D::new(0.0, 0.0),
            time: 0.0,
        }
    }

    /// Transforms this vector to a frame moving with the given velocity.
    ///
    /// The velocity is expressed in units where the speed of light is `1`.
    /// Therefore, its squared magnitude must be, smaller than `1`; otherwise
    /// the Lorentz factor is not finite.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MVector, Vector2D};
    ///
    /// let event = MVector::new(3.0, Vector2D::new(1.0, 0.0));
    /// let at_some_moving_frame = event.lorentz_transform(Vector2D::new(0.6, 0.0));
    /// assert_ne!(at_some_moving_frame, event);
    /// ```
    pub fn lorentz_transform(&self, velocity: Vector2D<f64>) -> Self{
        self.transform(MVector::lorentz_transform_matrix(velocity))
    }

    /// Builds the Lorentz transformation matrix for `velocity`.
    ///
    /// The returned matrix can be passed to [`transform`](Self::transform).
    /// The velocity is measured in units where the speed of light is `1` and
    /// must satisfy `velocity.length_squared() < 1.0`.
    pub fn lorentz_transform_matrix(velocity: Vector2D<f64>) -> MVector<MVector<f64>>{
        let v_length_squared = velocity.length_squared();
        let gamma = 1.0/(1.0 - v_length_squared).sqrt();
        Self::lorentz_transform_matrix_with_precalculated_gamma(velocity, gamma)
    }
    pub(crate) fn lorentz_transform_matrix_with_precalculated_gamma(velocity: Vector2D<f64>, gamma: f64) -> MVector<MVector<f64>>{
        let vx_squared = velocity.x * velocity.x;
        let vy_squared = velocity.y * velocity.y;
        let v_length_squared = vx_squared + vy_squared;
        if v_length_squared == 0.0 {
            return IDENTITY_TRANSFORM
        }
        let min_vx_gamma = - velocity.x * gamma;
        let min_vy_gamma = - velocity.y * gamma;
        let vx_vy = velocity.x * velocity.y;
        let one_over_v_length_squared = 1.0 / v_length_squared;
        let gamma_over_v_length_squared = gamma * one_over_v_length_squared;
        let matrix_element = gamma_over_v_length_squared * vx_vy - one_over_v_length_squared * vx_vy;
        MVector{
            pos: Vector2D{
                x: MVector{
                    pos: Vector2D {
                        x: gamma_over_v_length_squared * vx_squared + one_over_v_length_squared * vy_squared,
                        y: matrix_element,
                    },
                    time: min_vx_gamma,
                },
                y: MVector{
                    pos: Vector2D {
                        x: matrix_element,
                        y: gamma_over_v_length_squared * vy_squared + one_over_v_length_squared * vx_squared,
                    },
                    time: min_vy_gamma,
                },
            },
            time: MVector{
                pos: Vector2D { x: min_vx_gamma, y: min_vy_gamma },
                time: gamma,
            },
        }
    }
}

#[test]
fn lorentz_invariant_interval() {
    let p = MVector { pos: Vector2D::new(1.0, 2.0), time: 3.0 };
    let v = Vector2D::new(0.6, 0.2);
    let p_prime = p.lorentz_transform(v);

    let s2 = p.length();
    let s2_prime = p_prime.length();

    assert!((s2 - s2_prime).abs() < 1e-6);
}