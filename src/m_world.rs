use rayon::iter::{IntoParallelRefMutIterator, ParallelBridge, ParallelIterator};
use std::collections::HashMap;
use std::sync::Arc;
use rayon::iter::IntoParallelRefIterator;
use vector2d::Vector2D;
use crate::config::{MotionMode, ObjectConfig, WorldConfig};
use crate::m_object::{MObject, ObjectState};
use crate::m_vector::MVector;
use crate::object_tracker::{ObjectTracker, ReceiverData};
use crate::observation::{ObjectObservation, VisibleObjectObservation};

pub struct MWorld {

    config: WorldConfig,

    frame_object: MObject,

    registered_objects: HashMap<usize, (MObject, ObjectTracker)>,

    counter: usize
}

impl MWorld {

    pub fn new() -> Self{
        let config: WorldConfig = Default::default();
        Self{
            frame_object: MObject::new(ObjectConfig::default_with_group(config.frame_collision_group), config.proper_time_step),
            config,
            registered_objects: Default::default(),
            counter: 0,
        }
    }

    pub fn register_object(&mut self, object_config: ObjectConfig) -> usize{
        let mut m_object = MObject::new(object_config, self.config.proper_time_step);
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
                true => ObjectObservation::Visible(VisibleObjectObservation {
                    relative_position: *e.1.get_relative_visible_position(),
                    basis_x: *e.1.get_basis_x(),
                    basis_y: *e.1.get_basis_y(),
                    relative_frequency: e.1.get_relative_frequency(),
                    visible_position: *e.1.get_visible_m_vector(),
                }),
                false => ObjectObservation::NotVisible
            })
    }

    pub fn observe_visible_object(&self, id: &usize) -> Option<VisibleObjectObservation> {
        self
            .registered_objects
            .get(&id)
            .map(|e| match e.1.get_object_was_seen() {
                true => Some(VisibleObjectObservation {
                    relative_position: *e.1.get_relative_visible_position(),
                    basis_x: *e.1.get_basis_x(),
                    basis_y: *e.1.get_basis_y(),
                    relative_frequency: e.1.get_relative_frequency(),
                    visible_position: *e.1.get_visible_m_vector(),
                }),
                false => None
            }).flatten()
    }

    pub fn get_object_mut(&mut self, id: &usize)-> Option<&mut MObject>{
        self.registered_objects.get_mut(id).map(|e|&mut e.0)
    }

    pub fn get_frame_object_mut(&mut self)-> &mut MObject{
        &mut self.frame_object
    }

    pub fn process_time(&mut self, delta: f64){
        self.frame_object.process_tau(delta);
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
            })
    }
}