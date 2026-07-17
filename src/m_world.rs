use rayon::iter::{IntoParallelRefMutIterator, ParallelBridge, ParallelIterator};
use std::collections::HashMap;
use std::sync::Arc;
use rayon::iter::IntoParallelRefIterator;
use vector2d::Vector2D;
use crate::config::{MotionMode, ObjectConfig, WorldConfig};
use crate::m_object::MObject;
use crate::m_vector::MVector;
use crate::object_tracker::{ObjectTracker, ReceiverData};

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

    pub fn get_object_with_properties(&self, id: &usize) -> Option<&(MObject, ObjectTracker)>{
        self.registered_objects.get(id)
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