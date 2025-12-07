use std::collections::{HashMap, VecDeque};
use vector2d::Vector2D;
use crate::m_vector::MVector;
use crate::m_object::MObject;
use crate::photon::{Photon, PhotonEmittingPosition};
use crate::UPDATE_RATIO;

#[derive(Clone, Debug, Default)]
pub struct PhotonCrossing{
    photon_emmit_pos: MVector<f64>,
    photon_emmit_pos_in_receiver_frame: MVector<f64>,
    time_from_catch: f64,
}

pub const LAST_PHOTONS_COUNT: usize = 2;
pub const LAST_RELATIVE_COUNT: usize = 1;
#[derive(Clone, Debug)]
pub struct TrackedSource {
    last_photons: VecDeque<PhotonCrossing>,
    relative_freq: Option<f64>,
    constant_velocity_dx: Option<MVector<f64>>,
    object_radius: f64,

    receiver_current_pos: MVector<f64>,
    receiver_v: Vector2D<f64>,

    t_between_last_photons: f64,
    v_source: MVector<f64>,
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct ReceiverData{
    pub(crate) m_pos: MVector<f64>,
    pub(crate) velocity: Vector2D<f64>
}
impl TrackedSource {
    fn new(first_photon: &Photon, source: &MObject, receiver: &ReceiverData) -> Self{
        let (constant_velocity_dx, object_radius) = {
            let mut constant_velocity_dx = None;
            if source.constant_velocity() {
                constant_velocity_dx = Some(source.calculate_between_photons_vector());
            }
            (constant_velocity_dx, source.get_radius())
        };
        let mut res = Self{
            last_photons: Default::default(),
            constant_velocity_dx,
            object_radius,
            receiver_current_pos: receiver.m_pos,
            receiver_v: receiver.velocity,
            t_between_last_photons: 1.0,
            v_source: Default::default(),
            relative_freq: None
        };
        let first_crossing = res.calculate_photon_crossing(&first_photon);
        res.last_photons.push_back(first_crossing);
        res
    }

    fn calculate_obj_properties(&mut self){
        if self.last_photons.len() >=2 {
            let newest = self.last_photons.back().expect("Checked in if");
            let oldest = self.last_photons.front().expect("Checked in if");
            self.t_between_last_photons = oldest.time_from_catch - newest.time_from_catch;
            self.v_source = (newest.photon_emmit_pos - oldest.photon_emmit_pos) / self.t_between_last_photons;
            self.relative_freq = Some(UPDATE_RATIO * (self.last_photons.len() - 1) as f64 / self.t_between_last_photons);
        }
    }

    fn relative_position(&self) -> Option<Vector2D<f64>>{
        let current_m_vector = self.current_m_vector()?;
        let emmit_minus_curr = current_m_vector - self.receiver_current_pos;
        let vec = emmit_minus_curr.lorentz_transform(self.receiver_v);
        Some(vec.pos)
    }

    fn current_m_vector(&self) -> Option<MVector<f64>> {
        if self.last_photons.len() >= 1 {
            let newest = self.last_photons.back().expect("Checked in if");
            let current_m_vector = newest.photon_emmit_pos + self.v_source * newest.time_from_catch;
            return Some(current_m_vector)
        }
        None
    }

    fn relative_frequency(&self) -> Option<f64>{
        self.relative_freq
    }

    fn insert_into(&mut self, photon: &Photon){
        let crossing = self.calculate_photon_crossing(photon);
        self.insert_new_crossing(crossing)
    }

    fn calculate_new_photons_for_constant_velocity(&mut self){
        if let Some(vec) = self.constant_velocity_dx{
            let mut new_photon_pos = self.last_photons.back().expect("checked").photon_emmit_pos + vec;
            while (self.receiver_current_pos - new_photon_pos).is_time_or_light_like() && self.receiver_current_pos.time > new_photon_pos.time {
                self.insert_new_crossing(self.calculate_photon_crossing_based_on_pos(new_photon_pos));
                new_photon_pos = new_photon_pos + vec;
            }
        }
    }
    fn insert_new_crossing(&mut self, crossing: PhotonCrossing){
        self.last_photons.push_back(crossing);
        if self.last_photons.len() > LAST_PHOTONS_COUNT{
            self.last_photons.pop_front();
        }
        self.calculate_obj_properties()
    }

    fn calculate_photon_crossing(&self, photon: &Photon) -> PhotonCrossing{
        self.calculate_photon_crossing_based_on_pos(photon.get_emmit_pos())
    }
    fn calculate_photon_crossing_based_on_pos(&self, photon_emmit_pos: MVector<f64>) -> PhotonCrossing{
        let emmit_minus_curr = photon_emmit_pos - self.receiver_current_pos;
        let photon_emmit_pos_in_receiver_frame = emmit_minus_curr.lorentz_transform(self.receiver_v);
        let time_from_catch = photon_emmit_pos_in_receiver_frame.time.abs() - photon_emmit_pos_in_receiver_frame.pos.length();
        PhotonCrossing{
            photon_emmit_pos,
            photon_emmit_pos_in_receiver_frame,
            time_from_catch,
        }
    }
}


pub struct ObjectTracker{

    last_visible_source: HashMap<PhotonEmittingPosition, TrackedSource>,
    waiting_photons_queue: HashMap<PhotonEmittingPosition, VecDeque<Photon>>,

    relative_visible_position: Vector2D<f64>,
    basis_x: Vector2D<f64>,
    basis_y: Vector2D<f64>,
    relative_frequency: f64,
    visible_m_vector: MVector<f64>,

    object_was_seen: bool,

}

impl ObjectTracker {
    pub fn get_relative_visible_position(&self) -> &Vector2D<f64> {
        &self.relative_visible_position
    }

    pub fn get_basis_x(&self) -> &Vector2D<f64> {
        &self.basis_x
    }

    pub fn get_basis_y(&self) -> &Vector2D<f64> {
        &self.basis_y
    }

    pub fn get_relative_frequency(&self) -> f64 {
        self.relative_frequency
    }

    pub fn get_visible_m_vector(&self) -> &MVector<f64> {
        &self.visible_m_vector
    }

    pub fn get_object_was_seen(&self) -> bool {
        self.object_was_seen
    }
}

impl ObjectTracker{

    pub(crate) fn new () -> Self{
        Self{
            last_visible_source: Default::default(),
            waiting_photons_queue: Default::default(),
            relative_visible_position: Default::default(),
            basis_x: Vector2D::new(1.0, 0.0),
            basis_y: Vector2D::new(0.0, 1.0),
            relative_frequency: 1.0,
            visible_m_vector: Default::default(),
            object_was_seen: false,
        }
    }
    pub(crate) fn recalculate_properties(&mut self, source: &MObject, receiver: &ReceiverData, delta_tau: f64) {
        self.last_visible_source.values_mut()
            .for_each(|v|{
                v.receiver_v = receiver.velocity;
                v.receiver_current_pos = receiver.m_pos;
                v.last_photons.iter_mut().for_each(|s|
                    {
                        s.time_from_catch += delta_tau;
                    })
            });
        self.process_new_photons(source, receiver);
        if let Some(properties) = self.calculate_properties(){
            (self.relative_visible_position, self.basis_x, self.basis_y, self.relative_frequency, self.visible_m_vector) = properties;
            self.object_was_seen = true
        }else{
            self.object_was_seen = false;
        }
    }

    pub(crate) fn track_photons(&mut self, emitted_photons: Vec<Photon>){
        emitted_photons.into_iter()
            .for_each(|emitted_photon|{
                let photon_emmit_type = emitted_photon.get_emmit_type();
                self.waiting_photons_queue
                    .entry(photon_emmit_type)
                    .or_insert_with(VecDeque::new)
                    .push_back(emitted_photon);
            })
    }
    fn process_new_photons(&mut self, source: &MObject, receiver: &ReceiverData){
        self.process_photons_of_type(source, receiver, PhotonEmittingPosition::CENTER);
        self.process_photons_of_type(source, receiver, PhotonEmittingPosition::BOTTOM);
        self.process_photons_of_type(source, receiver, PhotonEmittingPosition::TOP);
        self.process_photons_of_type(source, receiver, PhotonEmittingPosition::FRONT);
        self.process_photons_of_type(source, receiver, PhotonEmittingPosition::BACK);
    }

    fn process_photons_of_type(&mut self, source: &MObject, receiver: &ReceiverData, photon_emitting_position: PhotonEmittingPosition){
        while let Some(photon) = self.fetch_next_photon(receiver, photon_emitting_position) {
            self.last_visible_source.entry(photon_emitting_position)
                .and_modify(|last|last.insert_into(&photon)).or_insert(
                TrackedSource::new(&photon, source, receiver)
            );
        }
    }

    fn fetch_next_photon(&mut self, receiver: &ReceiverData, photon_emitting_position: PhotonEmittingPosition) -> Option<Photon>{
        if let Some(tracked_source) = self.last_visible_source.get_mut(&photon_emitting_position) && tracked_source.constant_velocity_dx.is_some(){
            tracked_source.calculate_new_photons_for_constant_velocity();
            None
        }else{
            let queue = self.waiting_photons_queue.get_mut(&photon_emitting_position)?;
            let is_first_photon_visible = {
                let first_photon = queue.front()?;
                (receiver.m_pos - first_photon.get_emmit_pos()).is_time_or_light_like()
            };
            if is_first_photon_visible {
                return queue.pop_front()
            }
            None
        }
    }

    fn calculate_properties(&self) -> Option<(Vector2D<f64>, Vector2D<f64>, Vector2D<f64>, f64, MVector<f64>)>{
        let last_visible_center = self.last_visible_source.get(&PhotonEmittingPosition::CENTER)?;
        let current_m_vector = last_visible_center.current_m_vector()?;
        let relative_pos = last_visible_center.relative_position()?;
        let (basis_x, basis_y) = self.calculate_transform(&relative_pos).unwrap_or((Vector2D::new(1.0, 0.0), Vector2D::new(0.0, 1.0)));
        Some((relative_pos, basis_x, basis_y, last_visible_center.relative_frequency()?, current_m_vector.into()))
    }

    fn calculate_transform(&self, center: &Vector2D<f64>) -> Option<(Vector2D<f64>, Vector2D<f64>)> {
        if let (Some(back), Some(front), Some(top), Some(bottom)) = (
            self.last_visible_source.get(&PhotonEmittingPosition::BACK),
            self.last_visible_source.get(&PhotonEmittingPosition::FRONT),
            self.last_visible_source.get(&PhotonEmittingPosition::TOP),
            self.last_visible_source.get(&PhotonEmittingPosition::BOTTOM),
        ) {
            let radius = back.object_radius;
            let a = [
                Vector2D::new(radius, 0.0),
                Vector2D::new(-radius, 0.0),
                Vector2D::new(0.0, radius),
                Vector2D::new(0.0, -radius),
            ];
            let b = [
                front.relative_position()? - *center,
                back.relative_position()? - *center,
                top.relative_position()? - *center,
                bottom.relative_position()? - *center,
            ];
            let mut sum_aa = [[0.0; 2]; 2];
            let mut sum_ab = [[0.0; 2]; 2];
            for i in 0..4 {
                let ax = a[i].x;
                let ay = a[i].y;
                let bx = b[i].x;
                let by = b[i].y;

                sum_aa[0][0] += ax * ax;
                sum_aa[0][1] += ax * ay;
                sum_aa[1][0] += ay * ax;
                sum_aa[1][1] += ay * ay;

                sum_ab[0][0] += bx * ax;
                sum_ab[0][1] += bx * ay;
                sum_ab[1][0] += by * ax;
                sum_ab[1][1] += by * ay;
            }

            let det = sum_aa[0][0] * sum_aa[1][1] - sum_aa[0][1] * sum_aa[1][0];
            if det.abs() < 1e-8 {
                return None;
            }

            let inv_aa = [
                [ sum_aa[1][1] / det, -sum_aa[0][1] / det ],
                [ -sum_aa[1][0] / det, sum_aa[0][0] / det ],
            ];

            let m00 = sum_ab[0][0] * inv_aa[0][0] + sum_ab[0][1] * inv_aa[1][0];
            let m01 = sum_ab[0][0] * inv_aa[0][1] + sum_ab[0][1] * inv_aa[1][1];
            let m10 = sum_ab[1][0] * inv_aa[0][0] + sum_ab[1][1] * inv_aa[1][0];
            let m11 = sum_ab[1][0] * inv_aa[0][1] + sum_ab[1][1] * inv_aa[1][1];

            let basis_x = Vector2D::new(m00, m10);
            let basis_y = Vector2D::new(m01, m11);
            return Some((basis_x, basis_y));
        }
        None
    }
}

