use rayon::iter::{IntoParallelRefMutIterator, ParallelBridge, ParallelIterator};
use std::collections::HashMap;
use std::sync::Arc;
use rayon::iter::IntoParallelRefIterator;
use vector2d::Vector2D;
use crate::collision::{Collision, CollisionCalculator, CollisionGroup};
use crate::config::{MotionMode, ObjectConfig, StartPosition, WorldConfig};
use crate::m_event::{EventDetection, MEvent};
use crate::m_object::{MObject, ObjectState};
use crate::m_vector::MVector;
use crate::object_tracker::{ObjectTracker, ReceiverData};
use crate::observation::{ObjectObservation, VisibleObjectObservation};

pub struct MWorld {

    config: WorldConfig,

    frame_object: MObject,

    registered_objects: HashMap<usize, (MObject, ObjectTracker)>,
    
    events: HashMap<usize, MEvent>,

    counter: usize
}

pub enum ProcessTimeCallback{
    Collision(Collision),
    EventDetection(EventDetection),
}

impl MWorld {

    pub fn new() -> Self{
        let config: WorldConfig = Default::default();
        Self{
            frame_object: MObject::new(ObjectConfig::default_with_group(config.frame_collision_group), config.proper_time_step, 0.0),
            config,
            registered_objects: Default::default(),
            events: Default::default(),
            counter: 0,
        }
    }

    pub fn register_object(&mut self, object_config: ObjectConfig) -> usize{
        let mut m_object = MObject::new(object_config, self.config.proper_time_step, self.frame_object.get_m_pos().time);
        let mut object_tracker = ObjectTracker::new();
        let id = self.counter;
        self.counter += 1;
        if object_config.motion_mode == MotionMode::AlwaysConstantVelocity {
            let photons = m_object.emmit_all_photons();
            object_tracker.track_photons(photons);
        }
        self.registered_objects.insert(id, (m_object, object_tracker));
        id
    }
    
    pub fn create_event(&mut self, event_position: Vector2D<f64>) -> usize {
        let id = self.counter;
        self.counter += 1;
        let m_event = MEvent::new(MVector::new(self.frame_object.get_m_pos().time, event_position), CollisionGroup::All);
        self.events.insert(id, m_event);
        id
    }

    pub fn unregister_object(&mut self, id: &usize) {
        self.registered_objects.remove(id);
    }

    pub fn object(&self, id: &usize) -> Option<ObjectState> {
        self
            .registered_objects
            .get(&id)
            .map(|e|e.0.state())
    }
    pub fn observe_object(&self, id: &usize) -> Option<ObjectObservation> {
        self
            .registered_objects
            .get(&id)
            .map(|e| match e.1.get_object_was_seen() {
                true => ObjectObservation::Visible(e.1.to_visible_observation()),
                false => ObjectObservation::NotVisible
            })
    }

    pub fn observe_visible_object(&self, id: &usize) -> Option<VisibleObjectObservation> {
        self
            .registered_objects
            .get(&id)
            .map(|e| match e.1.get_object_was_seen() {
                true => Some(e.1.to_visible_observation()),
                false => None
            }).flatten()
    }
    
    pub fn set_velocity(&mut self, id: &usize, velocity: Vector2D<f64>){
        if let Some(object) = self.registered_objects.get_mut(id){
            object.0.set_velocity(velocity);
        }
    }

    pub fn set_acceleration(&mut self, id: &usize, acceleration: Vector2D<f64>){
        if let Some(object) = self.registered_objects.get_mut(id){
            object.0.set_acceleration(acceleration);
        }
    }
    
    pub fn set_frame_velocity(&mut self, velocity: Vector2D<f64>){
        self.frame_object.set_velocity(velocity);
    }

    pub fn set_frame_acceleration(&mut self, acceleration: Vector2D<f64>){
        self.frame_object.set_acceleration(acceleration);
    }

    pub fn frame_tau(&self) -> f64 {
        self.frame_object.get_tau()
    }

    pub fn frame_position(&self) -> MVector<f64> {
        *self.frame_object.get_m_pos()
    }

    pub fn process_time(&mut self, delta: f64) -> Vec<ProcessTimeCallback> {
        self.frame_object.process_as_frame_object_tau(delta);
        let target_time = self.frame_object.get_m_pos().time;
        let receiver_data = Arc::new(ReceiverData{
            m_pos: *self.frame_object.get_m_pos(),
            velocity: *self.frame_object.get_velocity()
        });
        self.registered_objects
            .par_iter_mut()
            .for_each(|(_id, (object, tracker))|{
                let photons = object.process_time(target_time);
                tracker.track_photons(photons);
                tracker.recalculate_properties(&object, receiver_data.as_ref(), delta)
            });
        let res = CollisionCalculator { }.calculate_collisions()
            .into_iter()
            .map(|c|{ProcessTimeCallback::Collision(c)})
            .collect::<_>();
        res
    }
}