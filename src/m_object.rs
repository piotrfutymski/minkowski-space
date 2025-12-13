use vector2d::Vector2D;
use crate::m_vector::MVector;
use crate::photon::{Photon, PhotonEmittingPosition};
use crate::{MAX_SAFE_SPEED, UPDATE_RATIO};

pub struct MObject{

    constant_velocity: bool,
    radius: f64,

    tau: f64,
    m_pos: MVector<f64>,
    velocity: Vector2D<f64>,
    acceleration: Vector2D<f64>,

    t_from_last_update_in_base_frame: f64,
    constant_gamma: f64,
    constant_between_photons_vector: MVector<f64>,

    front_offset: MVector<f64>,
    back_offset: MVector<f64>,
    bottom_offset: MVector<f64>,
    top_offset: MVector<f64>,

}

impl Default for MObject{
    fn default() -> Self {
        Self::new(MVector::default(), Vector2D::default(), false, 0.0)
    }
}

impl MObject{
    pub(crate) fn new(initial_pos: MVector<f64>, initial_vel: Vector2D<f64>, constant_velocity: bool, radius: f64) -> Self{
        let mut res = Self{
            constant_velocity,
            radius,
            tau: 0.0,
            m_pos: initial_pos,
            velocity: initial_vel,
            acceleration: Default::default(),
            t_from_last_update_in_base_frame: 0.0,

            constant_gamma: 0.0,
            constant_between_photons_vector: Default::default(),

            front_offset: Default::default(),
            back_offset: Default::default(),
            bottom_offset: Default::default(),
            top_offset: Default::default(),
        };
        if constant_velocity {
            res.ready_constant_v()
        }
        res.update_offsets();
        res
    }

    pub(crate) fn process_tau(&mut self, tau: f64){
        let gamma = self.gamma();
        let mut rest_tau = tau;
        let mut update_ratio_in_base_frame = UPDATE_RATIO * gamma;
        while rest_tau > UPDATE_RATIO {
            rest_tau -= UPDATE_RATIO;
            self.m_pos = self.m_pos + MVector::new(update_ratio_in_base_frame, self.velocity * update_ratio_in_base_frame);
            if self.acceleration.length() > 0.0 {
                self.accelerate(UPDATE_RATIO);
            }
            update_ratio_in_base_frame = UPDATE_RATIO * gamma;
        }
        update_ratio_in_base_frame = rest_tau * gamma;
        self.m_pos = self.m_pos + MVector::new(update_ratio_in_base_frame, self.velocity * update_ratio_in_base_frame);
        if self.acceleration.length() > 0.0 {
            self.accelerate(rest_tau);
        }
        self.tau += tau;
    }

    pub(crate) fn process_time(&mut self, target_time: f64) -> Vec<Photon>{
        let delta = target_time - self.m_pos.time;
        if delta < 0.0{
            return vec![]
        }
        let gamma = self.gamma();
        if self.constant_velocity {
            self.tau += delta / gamma;
            self.m_pos = self.m_pos + MVector::new(delta, self.velocity * delta);
            vec![]
        } else {
            let mut res = vec![];
            let mut update_ratio_in_base_frame = UPDATE_RATIO * gamma;
            self.t_from_last_update_in_base_frame += delta;
            while self.check_for_next_update(update_ratio_in_base_frame) {
                self.m_pos = self.m_pos + MVector::new(update_ratio_in_base_frame, self.velocity * update_ratio_in_base_frame);
                if self.acceleration.length() > 0.0 {
                    self.accelerate(UPDATE_RATIO);
                }
                self.tau += UPDATE_RATIO;
                update_ratio_in_base_frame = UPDATE_RATIO * gamma;
                res.append(&mut self.emmit_all_photons())
            }
            res
        }
    }

    pub fn gamma(&self) -> f64{
        if self.constant_velocity {
            return self.constant_gamma
        }
        1.0/(1.0 - self.velocity.length_squared()).sqrt()
    }

    pub fn one_over_gamma(&self) -> f64{
        if self.constant_velocity {
            return 1.0/self.constant_gamma
        }
        (1.0 - self.velocity.length_squared()).sqrt()
    }

    pub fn calculate_between_photons_vector(&self) -> MVector<f64>{
        let gamma = self.gamma();
        let dt = UPDATE_RATIO * gamma;
        let dx = self.velocity * dt;
        MVector::new(dt, dx)
    }

    pub fn constant_velocity(&self) -> bool {
        self.constant_velocity
    }

    pub fn get_radius(&self) -> f64 {
        self.radius
    }

    pub fn get_tau(&self) -> f64 {
        self.tau
    }

    pub fn get_m_pos(&self) -> &MVector<f64> {
        &self.m_pos
    }

    pub fn get_velocity(&self) -> &Vector2D<f64> {
        &self.velocity
    }

    pub fn get_acceleration(&self) -> &Vector2D<f64> {
        &self.acceleration
    }

    pub fn set_velocity(&mut self, velocity: Vector2D<f64>) {
        if self.constant_velocity {
            return;
        }
        self.update_offsets();
        self.velocity = velocity;
    }

    pub fn set_acceleration(&mut self, acceleration: Vector2D<f64>) {
        if self.constant_velocity {
            return;
        }
        self.acceleration = acceleration;
    }

    pub(crate) fn emmit_all_photons(&mut self) -> Vec<Photon> {
        let mut res = vec![Photon::new(self.m_pos, PhotonEmittingPosition::CENTER)];
        if self.radius > 0.0 {
            res.reserve(4);
            res.push(Photon::new(self.m_pos + self.front_offset, PhotonEmittingPosition::FRONT));
            res.push(Photon::new(self.m_pos + self.back_offset, PhotonEmittingPosition::BACK));
            res.push(Photon::new(self.m_pos + self.bottom_offset, PhotonEmittingPosition::BOTTOM));
            res.push(Photon::new(self.m_pos + self.top_offset, PhotonEmittingPosition::TOP));
        }
        res
    }
}

impl MObject{

    fn check_for_next_update(&mut self, update_ratio_in_base_frame: f64) -> bool{
        if self.t_from_last_update_in_base_frame > update_ratio_in_base_frame{
            self.t_from_last_update_in_base_frame -= update_ratio_in_base_frame;
            return true
        }
        false
    }
    fn ready_constant_v(&mut self) {
        self.constant_gamma = 1.0/(1.0 - self.velocity.length_squared()).sqrt();
        self.constant_between_photons_vector = self.calculate_between_photons_vector();
    }
    fn update_offsets(&mut self){
        if self.radius > 0.0 {
            let gamma = self.gamma();
            let gamma_v = self.gamma() * self.velocity.length();
            let v_direction = match self.velocity.length_squared() {
                x if x < 0.001 => Vector2D::new(1.0, 0.0),
                _ => self.velocity.normalise()
            };
            self.front_offset = Self::offset_for_vec(gamma, gamma_v, v_direction, Vector2D::new(self.radius, 0.0));
            self.back_offset = Self::offset_for_vec(gamma, gamma_v, v_direction, Vector2D::new(-self.radius, 0.0));
            self.bottom_offset = Self::offset_for_vec(gamma, gamma_v, v_direction, Vector2D::new(0.0, -self.radius));
            self.top_offset = Self::offset_for_vec(gamma, gamma_v, v_direction, Vector2D::new(0.0, self.radius));
        }
    }

    fn offset_for_vec(gamma: f64, gamma_v: f64, v_direction: Vector2D<f64>, vec: Vector2D<f64>) -> MVector<f64>{
        let parallel_part = Vector2D::dot(v_direction, vec);
        let pos_parallel = v_direction * parallel_part;
        let pos_perp = vec - pos_parallel;
        let pos_parallel_prime = pos_parallel * gamma;
        let pos_prime = pos_perp + pos_parallel_prime;
        let t_prime = gamma_v * parallel_part;
        MVector::new(t_prime, pos_prime)
    }

    fn accelerate(&mut self, dt: f64){
        let dv = self.acceleration * dt;
        let speed = self.velocity.length();
        if speed == 0.0 {
            self.velocity = dv;
            return;
        }
        let current_v_direction = self.velocity.normalise();
        let dvx = Vector2D::dot(current_v_direction, dv);
        let dvy_vec = dv - current_v_direction * dvx;
        let dvy = dvy_vec.length();
        let one_over_gamma = self.one_over_gamma();
        let new_vx = (speed + dvx) / (1.0 + speed * dvx);
        let new_vy = one_over_gamma * dvy / (1.0 + speed * dvx);
        let new_v = current_v_direction * new_vx + dvy_vec.normalise() * new_vy;
        self.velocity = new_v;
        if self.velocity.length_squared() >= 1.0 {
            self.velocity = self.velocity.normalise() * MAX_SAFE_SPEED
        }
        self.update_offsets();
    }

}