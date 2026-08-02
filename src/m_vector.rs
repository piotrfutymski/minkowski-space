use std::ops::{Add, Div, Mul, Sub};
use lazy_static::lazy_static;
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

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct MVector<T>{
    pub pos: Vector2D<T>,
    pub time: T,
}

impl<T> Add for MVector<T> where T: Add<T, Output = T> + Copy{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self{
            pos: self.pos + rhs.pos,
            time: self.time + rhs.time,
        }
    }
}

impl<T> Sub for MVector<T>  where T: Sub<T, Output=T> + Copy{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self{
            pos: self.pos - rhs.pos,
            time: self.time - rhs.time,
        }
    }
}

impl<T> Mul<T> for MVector<T> where T: Mul<T, Output=T> + Copy{
    type Output = MVector<T>;
    fn mul(self, rhs: T) -> Self::Output {
        Self{
            pos: self.pos * rhs,
            time: self.time * rhs
        }
    }
}

impl<T> Div<T> for MVector<T> where T: Div<T, Output=T> + Copy{
    type Output = MVector<T>;
    fn div(self, rhs: T) -> Self::Output {
        Self{
            pos: self.pos / rhs,
            time: self.time / rhs
        }
    }
}

impl MVector<f64> {

    pub fn new(time: f64, pos: Vector2D<f64>) -> Self{
        Self{
            pos,
            time,
        }
    }
    pub fn length_squared(&self) -> f64{
        self.time.powi(2) - self.pos.length_squared()
    }

    /// Absolute interval magnitude. Use `length_squared` and the causal
    /// predicates when the time-like/space-like sign matters.
    pub fn length(&self) -> f64{
        self.length_squared().abs().sqrt()
    }

    pub fn is_time_like(&self) -> bool{
        self.length_squared() > 0.0
    }

    pub fn is_time_or_light_like(&self) -> bool{
        self.length_squared() >= 0.0
    }

    pub fn is_space_like(&self) -> bool{
        self.length_squared() < 0.0
    }

    pub fn is_light_like(&self) -> bool{
        let interval = self.length_squared();
        interval.is_finite() && interval.abs() <= 1e-12 * (1.0 + self.time.abs().max(self.pos.length()).powi(2))
    }

    pub fn dot(&self, rhs: &MVector<f64>) -> f64{
        self.pos.x * rhs.pos.x + self.pos.y * rhs.pos.y + self.time * rhs.time
    }

    pub fn transform(&self, matrix: MVector<MVector<f64>>) -> Self {
        MVector{
            pos: Vector2D {
                x: matrix.pos.x.dot(self),
                y: matrix.pos.y.dot(self)
            },
            time: matrix.time.dot(self),
        }
    }

    pub fn zero() -> Self{
        Self{
            pos: Vector2D::new(0.0, 0.0),
            time: 0.0,
        }
    }

    pub fn lorentz_transform(&self, velocity: Vector2D<f64>) -> Self{
        self.transform(self.lorentz_transform_matrix(velocity))
    }

    pub fn lorentz_transform_matrix(&self, velocity: Vector2D<f64>) -> MVector<MVector<f64>>{
        let v_length_squared = velocity.length_squared();
        let gamma = 1.0/(1.0 - v_length_squared).sqrt();
        self.lorentz_transform_matrix_with_precalculated_gamma(velocity, gamma)
    }

    pub fn lorentz_transform_matrix_with_precalculated_gamma(&self, velocity: Vector2D<f64>, gamma: f64) -> MVector<MVector<f64>>{
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