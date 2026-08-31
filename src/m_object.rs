use crate::collision::CollisionMask;
use crate::config::{MotionMode, ObjectConfig, StartPosition};
use crate::m_vector::MVector;
use crate::m_world::ObjectSelection;
use crate::photon::{Photon, PhotonEmittingPosition};
use crate::{EPSILON, MAX_SAFE_SPEED};
use std::collections::HashMap;
use std::ops::Mul;
use vector2d::Vector2D;

/// A snapshot of an object's current physical state.
///
/// Positions are expressed in laboratory coordinates. `proper_time` is the
/// time elapsed along the object's world line, while the world itself advances
/// according to the observer's proper time.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ObjectState {
    /// Current spacetime position in laboratory coordinates.
    pub position: MVector<f64>,
    /// Proper time elapsed along the object's world line.
    pub proper_time: f64,
    /// Current velocity in laboratory coordinates and units where `c = 1`.
    pub velocity: Vector2D<f64>,
    /// Current acceleration used by the dynamic integrator.
    pub acceleration: Vector2D<f64>,
    /// Spatial radius used for extended-object rendering and collisions.
    pub radius: f64,
    /// Motion integration mode of the object.
    pub motion_mode: MotionMode,
    /// Lorentz factor calculated from the current velocity.
    pub gamma: f64,
    /// ID assigned when the object was registered.
    pub id: usize,
}
pub(crate) struct MObject {
    id: usize,

    motion_mode: MotionMode,
    radius: f64,

    tau: f64,
    m_pos: MVector<f64>,
    velocity: Vector2D<f64>,
    acceleration: Vector2D<f64>,

    t_from_last_update_in_base_frame: f64,
    constant_gamma: f64,
    constant_between_photons_vector: MVector<f64>,

    last_m_pos: MVector<f64>,

    front_offset: MVector<f64>,
    back_offset: MVector<f64>,
    bottom_offset: MVector<f64>,
    top_offset: MVector<f64>,

    proper_time_step: f64,
    collision_detection_enabled: bool,
    monitoring_collision_mask: CollisionMask,
    monitorable_collision_mask: CollisionMask,
}

impl MObject {
    pub(crate) fn new(
        object_config: ObjectConfig,
        update_ratio: f64,
        world_time: f64,
        id: usize,
    ) -> Self {
        let mut res = Self {
            id,
            motion_mode: object_config.motion_mode,
            radius: object_config.radius,
            tau: 0.0,
            m_pos: match object_config.position {
                StartPosition::Position(p) => p,
                StartPosition::PositionNow(p) => MVector::new(world_time, p),
            },
            velocity: object_config.velocity,
            acceleration: Default::default(),
            t_from_last_update_in_base_frame: 0.0,

            constant_gamma: 0.0,
            constant_between_photons_vector: Default::default(),

            last_m_pos: Default::default(),
            front_offset: Default::default(),
            back_offset: Default::default(),
            bottom_offset: Default::default(),
            top_offset: Default::default(),
            proper_time_step: update_ratio,
            collision_detection_enabled: true,
            monitoring_collision_mask: object_config.monitoring_collision_mask,
            monitorable_collision_mask: object_config.monitorable_collision_mask,
        };
        if object_config.motion_mode == MotionMode::AlwaysConstantVelocity {
            res.ready_constant_v()
        }
        res.update_offsets();
        res
    }

    pub(crate) fn process_as_observer_object_tau(
        &mut self,
        tau: f64,
        events_to_check: HashMap<usize, MVector<f64>>,
    ) -> Vec<(usize, MVector<f64>)> {
        let mut gamma = self.gamma();
        let mut rest_tau = tau;
        let mut dt = self.proper_time_step * gamma;
        let mut rest_events = events_to_check;
        let mut detections = Vec::new();

        self.last_m_pos = self.m_pos;

        while rest_tau > self.proper_time_step {
            let delta_pos = MVector::new(dt, self.velocity * dt);
            let found = Self::events_detection_check(&rest_events, &self.m_pos, &delta_pos);
            rest_events.retain(|id, _| !found.iter().any(|(found_id, _)| found_id == id));
            detections.extend(found);
            self.m_pos = self.m_pos + delta_pos;
            rest_tau -= self.proper_time_step;
            self.velocity_update(&mut gamma, &mut dt);
        }

        dt = rest_tau * gamma;
        let delta_pos = MVector::new(dt, self.velocity * dt);
        let found = Self::events_detection_check(&rest_events, &self.m_pos, &delta_pos);
        detections.extend(found);
        self.m_pos = self.m_pos + delta_pos;
        if self.acceleration.length() > 0.0 {
            self.accelerate(rest_tau);
        }
        self.tau += tau;
        detections
    }

    pub(crate) fn process_time(
        &mut self,
        target_time: f64,
        events_to_check: HashMap<usize, MVector<f64>>,
    ) -> (Vec<Photon>, Vec<(usize, MVector<f64>)>) {
        let delta = target_time - self.m_pos.time;
        if delta < 0.0 {
            return (vec![], vec![]);
        }

        self.last_m_pos = self.m_pos;

        let mut gamma = self.gamma();
        if self.motion_mode == MotionMode::AlwaysConstantVelocity {
            self.tau += delta / gamma;
            let delta_pos = MVector::new(delta, self.velocity * delta);
            let events_detections =
                Self::events_detection_check(&events_to_check, &self.m_pos, &delta_pos);
            self.m_pos = self.m_pos + delta_pos;
            (vec![], events_detections)
        } else {
            let mut res = vec![];
            let mut dt = self.proper_time_step * gamma;
            self.t_from_last_update_in_base_frame += delta;
            let mut rest_events = events_to_check;
            let mut all_events_detections = vec![];
            while self.check_for_next_update(dt) {
                let delta_pos = MVector::new(dt, self.velocity * dt);
                let events_detections =
                    Self::events_detection_check(&rest_events, &self.m_pos, &delta_pos);
                rest_events.retain(|id, _| {
                    !events_detections
                        .iter()
                        .any(|(detected_id, _)| detected_id == id)
                });
                all_events_detections.extend(events_detections);
                self.m_pos = self.m_pos + delta_pos;
                self.velocity_update(&mut gamma, &mut dt);
                self.tau += self.proper_time_step;
                res.append(&mut self.emmit_all_photons())
            }
            (res, all_events_detections)
        }
    }

    pub(crate) fn gamma(&self) -> f64 {
        if self.motion_mode == MotionMode::AlwaysConstantVelocity {
            return self.constant_gamma;
        }
        1.0 / (1.0 - self.velocity.length_squared()).sqrt()
    }

    pub(crate) fn one_over_gamma(&self) -> f64 {
        if self.motion_mode == MotionMode::AlwaysConstantVelocity {
            return 1.0 / self.constant_gamma;
        }
        (1.0 - self.velocity.length_squared()).sqrt()
    }

    pub(crate) fn calculate_between_photons_vector(&self) -> MVector<f64> {
        let gamma = self.gamma();
        let dt = self.proper_time_step * gamma;
        let dx = self.velocity * dt;
        MVector::new(dt, dx)
    }

    pub(crate) fn constant_velocity(&self) -> bool {
        self.motion_mode == MotionMode::AlwaysConstantVelocity
    }

    pub(crate) fn between_photons_vector(&self) -> &MVector<f64> {
        &self.constant_between_photons_vector
    }

    pub(crate) fn get_radius(&self) -> f64 {
        self.radius
    }

    pub(crate) fn get_tau(&self) -> f64 {
        self.tau
    }

    pub(crate) fn position(&self) -> &MVector<f64> {
        &self.m_pos
    }
    pub(crate) fn get_last_position(&self) -> &MVector<f64> {
        &self.last_m_pos
    }

    pub(crate) fn monitoring_collision_mask(&self) -> &CollisionMask {
        &self.monitoring_collision_mask
    }
    pub(crate) fn monitorable_collision_mask(&self) -> &CollisionMask {
        &self.monitorable_collision_mask
    }
    pub(crate) fn get_velocity(&self) -> &Vector2D<f64> {
        &self.velocity
    }

    pub(crate) fn get_acceleration(&self) -> &Vector2D<f64> {
        &self.acceleration
    }

    pub(crate) fn selection(&self) -> ObjectSelection {
        self.id.into()
    }

    pub(crate) fn is_collision_detection_enabled(&self) -> bool {
        self.collision_detection_enabled
    }

    pub(crate) fn set_velocity(&mut self, velocity: Vector2D<f64>) {
        if self.motion_mode == MotionMode::AlwaysConstantVelocity {
            return;
        }
        self.velocity = velocity;
        self.update_offsets();
    }

    pub(crate) fn set_acceleration(&mut self, acceleration: Vector2D<f64>) {
        if self.motion_mode == MotionMode::AlwaysConstantVelocity {
            return;
        }
        self.acceleration = acceleration;
    }

    pub(crate) fn set_collision_enabled(&mut self, collision_enabled: bool) {
        self.collision_detection_enabled = collision_enabled;
    }

    pub(crate) fn emmit_all_photons(&mut self) -> Vec<Photon> {
        let mut res = vec![Photon::new(self.m_pos, PhotonEmittingPosition::Center)];
        if self.radius > EPSILON {
            res.reserve(4);
            res.push(Photon::new(
                self.m_pos + self.front_offset,
                PhotonEmittingPosition::Front,
            ));
            res.push(Photon::new(
                self.m_pos + self.back_offset,
                PhotonEmittingPosition::Back,
            ));
            res.push(Photon::new(
                self.m_pos + self.bottom_offset,
                PhotonEmittingPosition::Bottom,
            ));
            res.push(Photon::new(
                self.m_pos + self.top_offset,
                PhotonEmittingPosition::Top,
            ));
        }
        res
    }

    pub(crate) fn get_proper_time_step(&self) -> f64 {
        self.proper_time_step
    }

    pub(crate) fn state(self: &MObject) -> ObjectState {
        ObjectState {
            position: self.m_pos,
            proper_time: self.tau,
            velocity: self.velocity,
            acceleration: self.acceleration,
            radius: self.radius,
            motion_mode: self.motion_mode,
            gamma: self.gamma(),
            id: self.id,
        }
    }

    pub(crate) fn get_detection_lines(&self) -> Vec<(MVector<f64>, MVector<f64>)> {
        let mut res = vec![(self.last_m_pos, self.m_pos)];
        if self.radius > EPSILON {
            res.reserve(4);
            res.push((
                self.last_m_pos + self.front_offset,
                self.m_pos + self.front_offset,
            ));
            res.push((
                self.last_m_pos + self.back_offset,
                self.m_pos + self.back_offset,
            ));
            res.push((
                self.last_m_pos + self.bottom_offset,
                self.m_pos + self.bottom_offset,
            ));
            res.push((
                self.last_m_pos + self.top_offset,
                self.m_pos + self.top_offset,
            ));
        }
        res
    }
}

impl MObject {
    fn velocity_update(&mut self, gamma: &mut f64, dt: &mut f64) {
        if self.acceleration.length() > 0.0 {
            self.accelerate(self.proper_time_step);
            *gamma = self.gamma();
        }
        *dt = self.proper_time_step * *gamma;
    }

    fn events_detection_check(
        events_to_check: &HashMap<usize, MVector<f64>>,
        x_0: &MVector<f64>,
        dx: &MVector<f64>,
    ) -> Vec<(usize, MVector<f64>)> {
        events_to_check
            .iter()
            .filter_map(|(id, e)| Self::event_detection_check(e, x_0, dx).map(|de| (*id, de)))
            .collect()
    }

    fn event_detection_check(
        e: &MVector<f64>,
        x_0: &MVector<f64>,
        dx: &MVector<f64>,
    ) -> Option<MVector<f64>> {
        let v = *x_0 - *e;
        let a = dx.length_squared();
        let b = 2.0 * (dx.time * v.time - dx.pos.x * v.pos.x - dx.pos.y * v.pos.y);
        let c = v.length_squared();
        let delta = b * b - 4.0 * a * c;
        if delta < 0.0 {
            return None;
        }
        let sqrt_delta = delta.sqrt();
        let x1 = (-b - sqrt_delta) / (2.0 * a);
        let x2 = (-b + sqrt_delta) / (2.0 * a);
        let mut x = x1;
        if x2 > x {
            x = x2;
        }
        if x > 1.0 {
            return None;
        }
        Some(*x_0 + *dx * x)
    }
    fn check_for_next_update(&mut self, update_ratio_in_base_frame: f64) -> bool {
        if self.t_from_last_update_in_base_frame > update_ratio_in_base_frame {
            self.t_from_last_update_in_base_frame -= update_ratio_in_base_frame;
            return true;
        }
        false
    }
    fn ready_constant_v(&mut self) {
        self.constant_gamma = 1.0 / (1.0 - self.velocity.length_squared()).sqrt();
        self.constant_between_photons_vector = self.calculate_between_photons_vector();
    }
    fn update_offsets(&mut self) {
        if self.radius > EPSILON {
            let gamma = self.gamma();
            let gamma_v = self.gamma() * self.velocity.length();
            let v_direction = match self.velocity.length_squared() {
                x if x < 0.001 => Vector2D::new(1.0, 0.0),
                _ => self.velocity.normalise(),
            };
            self.front_offset =
                Self::offset_for_vec(gamma, gamma_v, v_direction, Vector2D::new(self.radius, 0.0));
            self.back_offset = Self::offset_for_vec(
                gamma,
                gamma_v,
                v_direction,
                Vector2D::new(-self.radius, 0.0),
            );
            self.bottom_offset = Self::offset_for_vec(
                gamma,
                gamma_v,
                v_direction,
                Vector2D::new(0.0, -self.radius),
            );
            self.top_offset =
                Self::offset_for_vec(gamma, gamma_v, v_direction, Vector2D::new(0.0, self.radius));
        }
    }

    fn offset_for_vec(
        gamma: f64,
        gamma_v: f64,
        v_direction: Vector2D<f64>,
        vec: Vector2D<f64>,
    ) -> MVector<f64> {
        let parallel_part = Vector2D::dot(v_direction, vec);
        let pos_parallel = v_direction * parallel_part;
        let pos_perp = vec - pos_parallel;
        let pos_parallel_prime = pos_parallel * gamma;
        let pos_prime = pos_perp + pos_parallel_prime;
        let t_prime = gamma_v * parallel_part;
        MVector::new(t_prime, pos_prime)
    }

    fn accelerate(&mut self, dt: f64) {
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
        if self.velocity.length_squared() >= crate::MAX_SAFE_SPEED_SQUARED {
            self.velocity = self.velocity.normalise() * MAX_SAFE_SPEED
        }
        self.update_offsets();
    }
}
