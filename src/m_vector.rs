use std::ops::{Add, Div, Mul, Sub};
use vector2d::Vector2D;

#[derive(Copy, Clone, Default, Debug)]
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
        self.length_squared() == 0.0
    }

    pub fn zero() -> Self{
        Self{
            pos: Vector2D::new(0.0, 0.0),
            time: 0.0,
        }
    }

    pub fn lorentz_transform(&self, velocity: Vector2D<f64>) -> Self{
        let v_length = velocity.length();
        if v_length == 0.0 {
            return self.clone()
        }
        let gamma = 1.0/(1.0 - v_length * v_length).sqrt();

        let v_direction = velocity.normalise();

        let pos_parallel = v_direction * Vector2D::dot(v_direction, self.pos);
        let pos_perp = self.pos - pos_parallel;

        let pos_parallel_prime = (pos_parallel - velocity * self.time) * gamma;
        let pos_prime = pos_perp + pos_parallel_prime;
        let t_prime = gamma * (self.time - Vector2D::dot(velocity, self.pos));
        Self{
            pos: pos_prime,
            time: t_prime,
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