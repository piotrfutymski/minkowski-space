use crate::collision::collision_calculator::CollisionCalculator;
use crate::collision::hashgrid::HashGrid;
use crate::collision::{Collision, CollisionGroup, CollisionObject};
use crate::config::{ConfigError, MotionMode, ObjectConfig, StartPosition, WorldConfig};
use crate::m_event::{DetectionObject, EventDetection, MEvent};
use crate::m_object::{MObject, ObjectState};
use crate::m_vector::MVector;
use crate::object_tracker::{ObjectTracker, ReceiverData};
use crate::observation::{EventObservation, ObjectObservation, VisibleObjectObservation};
use rayon::iter::IntoParallelRefIterator;
use rayon::iter::{IntoParallelRefMutIterator, ParallelBridge, ParallelIterator};
use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::Arc;
use vector2d::Vector2D;
use crate::CollisionGroupPair;

pub struct MWorld {
    config: WorldConfig,

    frame_object: MObject,

    registered_objects: HashMap<usize, (MObject, ObjectTracker)>,

    events: HashMap<usize, MEvent>,

    object_event_possible_to_detect: HashMap<usize, HashSet<usize>>,

    frame_event_possible_to_detect: HashSet<usize>,

    counter: usize,

    hash_grid: HashGrid,
}

pub enum ProcessTimeCallback {
    Collision(Collision),
    Event(EventDetection),
}

impl Default for MWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl MWorld {
    pub fn new() -> Self {
        Self::with_config(WorldConfig::default()).expect("default world configuration is valid")
    }

    pub fn with_config(mut config: WorldConfig) -> Result<Self, ConfigError> {
        config.validate()?;
        config.collision_pairs = config
            .collision_pairs
            .into_iter()
            .map(|pair| pair.canonical())
            .collect();
        let frame_config = ObjectConfig {
            radius: config.frame_collision_radius,
            ..ObjectConfig::default_with_group(config.frame_collision_group)
        };
        Ok(Self {
            frame_object: MObject::new(frame_config, config.proper_time_step, 0.0),
            hash_grid: HashGrid::init(&config),
            config,
            registered_objects: Default::default(),
            events: Default::default(),
            object_event_possible_to_detect: Default::default(),
            frame_event_possible_to_detect: Default::default(),
            counter: 1,
        })
    }

    pub fn try_register_object(
        &mut self,
        object_config: ObjectConfig,
    ) -> Result<usize, crate::config::ConfigError> {
        object_config.validate()?;
        let mut m_object = MObject::new(
            object_config,
            self.config.proper_time_step,
            self.frame_object.position().time,
        );
        let mut object_tracker = ObjectTracker::new();
        let id = self.counter;
        self.counter += 1;
        if object_config.motion_mode == MotionMode::AlwaysConstantVelocity {
            let photons = m_object.emmit_all_photons();
            object_tracker.track_photons(photons);
        }
        self.object_event_possible_to_detect.insert(
            id,
            self.events
                .iter()
                .filter(|e| {
                    e.1.collision_group().collision_group_matches(
                        m_object.collision_group(),
                        &self.config.collision_pairs,
                    )
                })
                .filter(|e| (*m_object.position() - *e.1.position()).is_space_like())
                .map(|e| *e.0)
                .collect(),
        );
        self.registered_objects
            .insert(id, (m_object, object_tracker));
        Ok(id)
    }

    /// Registers an object. Prefer [`Self::try_register_object`] when invalid
    /// user input must be reported to the caller.
    pub fn register_object(&mut self, object_config: ObjectConfig) -> usize {
        self.try_register_object(object_config)
            .unwrap_or_else(|_| usize::MAX)
    }

    /// Create a spacetime event without an `on_detection` callback.
    pub fn create_event(&mut self, event_position: Vector2D<f64>) -> usize {
        let event = MVector::new(self.frame_object.position().time, event_position);
        self.create_event_at_impl(event)
    }

    /// Create a spacetime event at the given `MVector` position (time + space)
    /// without an `on_detection` callback.
    pub fn create_event_at(&mut self, event: MVector<f64>) -> usize {
        self.create_event_at_impl(event)
    }

    fn create_event_at_impl(&mut self, event: MVector<f64>) -> usize {
        let id = self.counter;
        self.counter += 1;
        let m_event = MEvent::new(event, CollisionGroup::All);

        if self
            .frame_object
            .collision_group()
            .collision_group_matches(m_event.collision_group(), &self.config.collision_pairs)
        {
            self.frame_event_possible_to_detect.insert(id);
        }

        self.object_event_possible_to_detect
            .iter_mut()
            .filter(|(object_id, _)| {
                self.registered_objects
                    .get(object_id)
                    .is_some_and(|(object, _)| {
                        object.collision_group().collision_group_matches(
                            &m_event.collision_group(),
                            &self.config.collision_pairs,
                        )
                    })
            })
            .for_each(|(_, possible_events)| {
                possible_events.insert(id);
            });

        self.events.insert(id, m_event);
        id
    }

    pub fn unregister_object(&mut self, id: &usize) {
        self.registered_objects.remove(id);
        self.object_event_possible_to_detect.remove(id);
    }

    pub fn unregister_event(&mut self, id: &usize) {
        self.events.remove(id);
        self.object_event_possible_to_detect
            .iter_mut()
            .for_each(|s| {
                s.1.remove(id);
            });
        self.frame_event_possible_to_detect.remove(id);
    }

    pub fn object(&self, id: &usize) -> Option<ObjectState> {
        if *id == 0 {
            return Some(self.frame_object());
        }
        self.registered_objects.get(&id).map(|e| e.0.state())
    }

    pub fn frame_object(&self) -> ObjectState {
        self.frame_object.state()
    }

    pub fn event(&self, id: &usize) -> Option<MVector<f64>> {
        self.events.get(&id).map(|e| *e.position())
    }
    pub fn observe_object(&self, id: &usize) -> Option<ObjectObservation> {
        self.registered_objects
            .get(&id)
            .map(|e| match e.1.get_object_was_seen() {
                true => ObjectObservation::Visible(e.1.to_visible_observation()),
                false => ObjectObservation::NotVisible,
            })
    }

    /// Observes an event in the current frame of the world observer.
    ///
    /// An event is visible once the observer is inside its future light cone.
    /// The returned position is relative to the observer and is expressed in
    /// the observer's frame (the same convention as `observe_object`).
    pub fn observe_event(&self, id: &usize) -> Option<EventObservation> {
        self.events.get(id).map(|event| {
            let relative = *event.position() - self.frame_object.position().clone();
            if relative.is_time_or_light_like() {
                EventObservation::Visible(
                    relative.lorentz_transform(*self.frame_object.get_velocity()),
                )
            } else {
                EventObservation::NotVisible
            }
        })
    }

    pub fn observe_visible_object(&self, id: &usize) -> Option<VisibleObjectObservation> {
        self.registered_objects
            .get(&id)
            .map(|e| match e.1.get_object_was_seen() {
                true => Some(e.1.to_visible_observation()),
                false => None,
            })
            .flatten()
    }

    pub fn set_velocity(
        &mut self,
        id: &usize,
        velocity: Vector2D<f64>,
    ) -> Result<(), crate::config::ConfigError> {
        if *id == 0 {
            self.set_frame_velocity(velocity);
            return Ok(());
        }
        if !velocity.x.is_finite()
            || !velocity.y.is_finite()
            || velocity.length_squared() >= crate::MAX_SAFE_SPEED_SQUARED
        {
            return Err(ConfigError::SuperluminalVelocity);
        }
        if let Some(object) = self.registered_objects.get_mut(id) {
            if object.0.constant_velocity() {
                return Err(ConfigError::UnsupportedOperation(
                    "set_velocity on constant-velocity object",
                ));
            }
            object.0.set_velocity(velocity);
            return Ok(());
        }
        Err(ConfigError::UnsupportedOperation("unknown object"))
    }

    pub fn set_acceleration(&mut self, id: &usize, acceleration: Vector2D<f64>) {
        if *id == 0 {
            self.set_frame_acceleration(acceleration);
            return;
        }
        if let Some(object) = self.registered_objects.get_mut(id) {
            object.0.set_acceleration(acceleration);
        }
    }

    pub fn set_frame_velocity(&mut self, velocity: Vector2D<f64>) {
        self.frame_object.set_velocity(velocity);
    }

    pub fn set_frame_acceleration(&mut self, acceleration: Vector2D<f64>) {
        self.frame_object.set_acceleration(acceleration);
    }

    pub fn frame_tau(&self) -> f64 {
        self.frame_object.get_tau()
    }

    pub fn lab_time(&self) -> f64 {
        self.frame_object.position().time
    }

    pub fn frame_position(&self) -> MVector<f64> {
        *self.frame_object.position()
    }

    pub fn advance_by_proper_time(&mut self, delta: f64) -> Vec<ProcessTimeCallback> {
        let frame_events_to_check = self.get_frame_events_to_check();
        let frame_detections = self
            .frame_object
            .process_as_frame_object_tau(delta, frame_events_to_check);

        let target_time = self.lab_time();

        let receiver_data = Arc::new(ReceiverData {
            m_pos: *self.frame_object.position(),
            velocity: *self.frame_object.get_velocity(),
        });

        let events_to_check: HashMap<usize, HashMap<usize, MVector<f64>>> =
            self.get_events_to_check(target_time);

        let detected_events: Vec<(usize, usize, MVector<f64>)> =
            self.advance_objects_parallel(delta, target_time, receiver_data, events_to_check);

        self.hash_grid
            .build_grid(&self.frame_object, &self.registered_objects);

        let mut callbacks = self.get_event_callbacks(frame_detections, detected_events);
        callbacks.append(&mut CollisionCalculator { world: &self }.detect_collisions());

        callbacks
    }

    fn advance_objects_parallel(
        &mut self,
        delta: f64,
        target_time: f64,
        receiver_data: Arc<ReceiverData>,
        events_to_check: HashMap<usize, HashMap<usize, MVector<f64>>>,
    ) -> Vec<(usize, usize, MVector<f64>)> {
        self.registered_objects
            .par_iter_mut()
            .flat_map_iter(|(id, (object, tracker))| {
                let candidates = events_to_check.get(id).cloned().unwrap_or_default();
                let (photons, detected) = object.process_time(target_time, candidates);
                tracker.track_photons(photons);
                tracker.recalculate_properties(&object, receiver_data.as_ref(), delta);
                detected
                    .into_iter()
                    .map(move |(event_id, detection_position)| (*id, event_id, detection_position))
            })
            .collect()
    }

    fn get_events_to_check(
        &mut self,
        target_time: f64,
    ) -> HashMap<usize, HashMap<usize, MVector<f64>>> {
        self.registered_objects
            .keys()
            .map(|id| (*id, self.get_events_per_object_to_check(id, target_time)))
            .collect()
    }

    fn get_event_callbacks(
        &mut self,
        frame_detections: Vec<(usize, MVector<f64>)>,
        detected_events: Vec<(usize, usize, MVector<f64>)>,
    ) -> Vec<ProcessTimeCallback> {
        let mut res = Vec::new();
        for (event_id, event_detection_position) in frame_detections {
            self.frame_event_possible_to_detect.remove(&event_id);
            if let Some(callback) =
                self.handle_frame_event_detection(event_id, event_detection_position)
            {
                res.push(callback);
            }
        }
        for (object_id, event_id, event_detection_position) in detected_events {
            self.remove_object_event_possible_to_detect(object_id, event_id);
            if let Some(callback) =
                self.handle_event_detection(object_id, event_id, event_detection_position)
            {
                res.push(callback);
            }
        }
        res
    }
}

impl MWorld {
    pub(crate) fn get_registered_objects(&self) -> &HashMap<usize, (MObject, ObjectTracker)> {
        &self.registered_objects
    }

    pub(crate) fn get_frame_object(&self) -> &MObject {
        &self.frame_object
    }

    pub(crate) fn get_hash_grid(&self) -> &HashGrid {
        &self.hash_grid
    }

    pub(crate) fn configured_pairs(&self) -> &BTreeSet<CollisionGroupPair> {
        &self.config.collision_pairs
    }
}

impl MWorld {
    fn get_frame_events_to_check(&self) -> HashMap<usize, MVector<f64>> {
        self.frame_event_possible_to_detect
            .iter()
            .filter_map(|id| self.events.get(id).map(|event| (*id, *event.position())))
            .collect()
    }

    fn handle_frame_event_detection(
        &mut self,
        event_id: usize,
        event_detection_position: MVector<f64>,
    ) -> Option<ProcessTimeCallback> {
        if !self.events.contains_key(&event_id) {
            return None;
        }
        let detection = EventDetection {
            event_id,
            detection_object: DetectionObject::FrameObject,
            event_detection_position,
        };
        Some(ProcessTimeCallback::Event(detection))
    }

    fn remove_object_event_possible_to_detect(&mut self, object_id: usize, event_id: usize) {
        if let Some(possible_events) = self.object_event_possible_to_detect.get_mut(&object_id) {
            possible_events.remove(&event_id);
        }
    }

    fn handle_event_detection(
        &mut self,
        object_id: usize,
        event_id: usize,
        event_detection_position: MVector<f64>,
    ) -> Option<ProcessTimeCallback> {
        if !self.events.contains_key(&event_id) {
            return None;
        }
        let detection = EventDetection {
            event_id,
            detection_object: DetectionObject::MObject(object_id),
            event_detection_position,
        };
        Some(ProcessTimeCallback::Event(detection))
    }

    fn get_events_per_object_to_check(
        &self,
        object_id: &usize,
        target_time: f64,
    ) -> HashMap<usize, MVector<f64>> {
        if let (Some((object, _)), Some(events_possible_to_detect)) = (
            self.registered_objects.get(object_id),
            self.object_event_possible_to_detect.get(object_id),
        ) {
            return events_possible_to_detect
                .iter()
                .filter_map(|e| self.events.get(e).map(|v| (e, v)))
                .filter(|e| {
                    let mut event_to_object = *object.position() - *e.1.position();
                    let dt = target_time - object.position().time;
                    event_to_object.time += 2.0 * dt;
                    event_to_object.is_time_or_light_like()
                })
                .map(|e| (*e.0, *e.1.position()))
                .collect();
        }
        Default::default()
    }
}
