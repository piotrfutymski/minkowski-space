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
use crate::observation::{EventObservation, ObjectObservation, VisibleObjectObservation};

/// Type alias for the `on_detection` callback.
pub type EventDetectionCallback = Box<dyn FnMut(&mut MWorld, &EventDetection)>;

pub struct MWorld {

    config: WorldConfig,

    frame_object: MObject,

    registered_objects: HashMap<usize, (MObject, ObjectTracker)>,
    
    events: HashMap<usize, MEvent>,

    /// Optional callbacks registered per event.
    event_callbacks: HashMap<usize, EventDetectionCallback>,

    counter: usize
}

pub enum ProcessTimeCallback{
    Collision(Collision),
}

impl MWorld {

    pub fn new() -> Self{
        let config: WorldConfig = Default::default();
        Self{
            frame_object: MObject::new(ObjectConfig::default_with_group(config.frame_collision_group), config.proper_time_step, 0.0),
            config,
            registered_objects: Default::default(),
            events: Default::default(),
            event_callbacks: Default::default(),
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
    
    /// Create a spacetime event without an `on_detection` callback.
    pub fn create_event(&mut self, event_position: Vector2D<f64>) -> usize {
        self.create_event_impl(event_position, None)
    }

    /// Create a spacetime event with an `on_detection` callback.
    ///
    /// The callback will be invoked each time an object in the world detects
    /// the event. It receives a mutable reference
    /// to the [`MWorld`] (so you can spawn objects, change velocities, etc.)
    /// and an [`EventDetection`] describing the detection.
    ///
    /// # Example
    ///
    /// ```
    /// use vector2d::Vector2D;
    /// use minkowski_space::m_event::DetectionObject;
    ///
    /// let mut world = minkowski_space::m_world::MWorld::new();
    /// world.create_event_with_callback(
    ///     Vector2D::new(0.0, 0.0),
    ///     |world, detection| {
    ///         match detection.detection_object {
    ///             DetectionObject::MObject(id) => println!("Event {} detected by object {}", detection.event_id, id),
    ///             DetectionObject::FrameObject => println!("Event {} detected by frame object", detection.event_id),
    ///         }
    ///     },
    /// );
    /// ```
    pub fn create_event_with_callback(
        &mut self,
        event_position: Vector2D<f64>,
        callback: impl FnMut(&mut MWorld, &EventDetection) + 'static,
    ) -> usize {
        self.create_event_impl(event_position, Some(Box::new(callback)))
    }

    /// Create a spacetime event at the given `MVector` position (time + space)
    /// without an `on_detection` callback.
    pub fn create_event_at(&mut self, event: MVector<f64>) -> usize {
        self.create_event_at_impl(event, None)
    }

    /// Create a spacetime event at the given `MVector` position (time + space)
    /// with an `on_detection` callback.
    ///
    /// See [`create_event_with_callback`](Self::create_event_with_callback) for details about the callback.
    pub fn create_event_with_callback_at(
        &mut self,
        event: MVector<f64>,
        callback: impl FnMut(&mut MWorld, &EventDetection) + 'static,
    ) -> usize {
        self.create_event_at_impl(event, Some(Box::new(callback)))
    }

    fn create_event_impl(
        &mut self,
        event_position: Vector2D<f64>,
        callback: Option<EventDetectionCallback>,
    ) -> usize {
        let id = self.counter;
        self.counter += 1;
        let m_event = MEvent::new(MVector::new(self.frame_object.get_m_pos().time, event_position), CollisionGroup::All);
        self.events.insert(id, m_event);
        if let Some(cb) = callback {
            self.event_callbacks.insert(id, cb);
        }
        id
    }

    fn create_event_at_impl(
        &mut self,
        event: MVector<f64>,
        callback: Option<EventDetectionCallback>,
    ) -> usize {
        let id = self.counter;
        self.counter += 1;
        let m_event = MEvent::new(event, CollisionGroup::All);
        self.events.insert(id, m_event);
        if let Some(cb) = callback {
            self.event_callbacks.insert(id, cb);
        }
        id
    }

    pub fn unregister_object(&mut self, id: &usize) {
        self.registered_objects.remove(id);
    }

    pub fn unregister_event(&mut self, id: &usize) {
        self.events.remove(id);
        self.event_callbacks.remove(id);
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

    /// Observes an event in the current frame of the world observer.
    ///
    /// An event is visible once the observer is inside its future light cone.
    /// The returned position is relative to the observer and is expressed in
    /// the observer's frame (the same convention as `observe_object`).
    pub fn observe_event(&self, id: &usize) -> Option<EventObservation> {
        self.events.get(id).map(|event| {
            let relative = self.frame_object.get_m_pos().clone() - event.position();
            if relative.is_time_or_light_like() && relative.time >= 0.0 {
                EventObservation::Visible(
                    relative.lorentz_transform(*self.frame_object.get_velocity()),
                )
            } else {
                EventObservation::NotVisible
            }
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
        CollisionCalculator { }.calculate_collisions()
            .into_iter()
            .map(|c|{ProcessTimeCallback::Collision(c)})
            .collect::<_>()
    }
}