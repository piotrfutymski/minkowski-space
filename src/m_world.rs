use crate::collision::collision_calculator::CollisionCalculator;
use crate::collision::hashgrid::HashGrid;
use crate::collision::{Collision, CollisionGroup, CollisionPair};
use crate::config::{ConfigError, MotionMode, ObjectConfig, StartPosition, WorldConfig};
use crate::m_event::{DetectionObject, EventDetection, MEvent};
use crate::m_object::{MObject, ObjectState};
use crate::m_vector::MVector;
use crate::object_tracker::{ObjectTracker, ReceiverData};
use crate::observation::{EventObservation, ObjectObservation, VisibleObjectObservation};
use crate::{CollisionGroupPair, EPSILON};
use rayon::iter::IntoParallelRefIterator;
use rayon::iter::{IntoParallelRefMutIterator, ParallelBridge, ParallelIterator};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap, HashSet};
use std::sync::Arc;
use vector2d::Vector2D;

struct PendingCollision(Collision);

impl PartialEq for PendingCollision {
    fn eq(&self, other: &Self) -> bool {
        self.0.position.time.total_cmp(&other.0.position.time) == Ordering::Equal
    }
}

impl Eq for PendingCollision {}

impl PartialOrd for PendingCollision {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PendingCollision {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.position.time.total_cmp(&self.0.position.time)
    }
}

/// A simulation world with a laboratory frame and a moving observer.
///
/// Objects are stored and advanced in laboratory coordinates. The observer has
/// its own world line and proper time, and observations are reported in the
/// observer's instantaneous frame.
///
/// # Example
///
/// ```
/// use minkowski_space::{MWorld, Vector2D};
///
/// let mut world = MWorld::new();
/// world.set_observer_velocity(Vector2D::new(0.5, 0.0));
/// world.advance_by_proper_time(1.0);
///
/// assert!(world.lab_time() > world.observer_tau());
/// ```
pub struct MWorld {
    config: WorldConfig,

    observer_object: MObject,

    registered_objects: HashMap<usize, (MObject, ObjectTracker)>,

    events: HashMap<usize, MEvent>,

    event_possible_to_detect: HashMap<usize, HashSet<usize>>,

    counter: usize,

    hash_grid: HashGrid,

    active_collision_pairs: BTreeMap<CollisionPair, f64>,

    pending_collisions: BinaryHeap<PendingCollision>,
}

/// A notification produced while advancing the simulation.
///
/// Notifications are returned by [`MWorld::advance_by_proper_time`].
#[derive(Debug)]
pub enum ProcessTimeCallback {
    /// Two collision-enabled objects reached a collision.
    Collision(Collision),
    /// An object or the observer detected a spacetime event - it means that event is inside past light cone of some object
    Event(EventDetection),
}

/// Selects either the observer or a registered object.
///
/// The observer can also be selected by passing the integer `0` to methods
/// accepting `Into<ObjectSelection>`. Registered object IDs are returned by
/// [`MWorld::register_object`] and [`MWorld::try_register_object`].
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum ObjectSelection {
    /// The observer object.
    Observer,
    /// A registered object identified by its ID.
    Object(usize),
}

impl From<usize> for ObjectSelection {
    fn from(value: usize) -> Self {
        match value {
            0 => ObjectSelection::Observer,
            x => ObjectSelection::Object(x),
        }
    }
}

impl From<&usize> for ObjectSelection {
    fn from(value: &usize) -> Self {
        match value {
            0 => ObjectSelection::Observer,
            x => ObjectSelection::Object(*x),
        }
    }
}

impl Default for MWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl MWorld {
    /// Creates a world with [`WorldConfig::default`].
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::MWorld;
    ///
    /// let world = MWorld::new();
    /// assert_eq!(world.lab_time(), 0.0);
    /// ```
    pub fn new() -> Self {
        Self::with_config(WorldConfig::default()).expect("default world configuration is valid")
    }

    /// Creates a world using `config`.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError`] when the configuration contains invalid values,
    /// such as a non-positive time step or an invalid collision-group pair.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, WorldConfig};
    ///
    /// let world = MWorld::with_config(WorldConfig::default()).unwrap();
    /// assert_eq!(world.lab_time(), 0.0);
    /// ```
    pub fn with_config(mut config: WorldConfig) -> Result<Self, ConfigError> {
        config.validate()?;
        config.collision_pairs = config
            .collision_pairs
            .into_iter()
            .map(|pair| pair.canonical())
            .collect();
        let observer_config = ObjectConfig {
            radius: config.observer_collision_radius,
            ..ObjectConfig::default_with_group(config.observer_collision_group)
        };
        Ok(Self {
            observer_object: MObject::new(observer_config, config.proper_time_step, 0.0, 0),
            hash_grid: HashGrid::init(&config),
            config,
            registered_objects: Default::default(),
            events: Default::default(),
            event_possible_to_detect: HashMap::from([(0, HashSet::new())]),
            counter: 1,
            active_collision_pairs: BTreeMap::new(),
            pending_collisions: BinaryHeap::new(),
        })
    }

    /// Registers an object and returns its unique ID.
    ///
    /// The object's initial position is interpreted in laboratory coordinates.
    /// An object using [`MotionMode::AlwaysConstantVelocity`] starts with its
    /// complete constant-velocity trajectory prepared for observation.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError`] when `object_config` is invalid, for example
    /// when its velocity is superluminal or its radius is negative.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, ObjectConfig, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// let object_id = world.try_register_object(
    ///     ObjectConfig::at_position_with_const_speed(
    ///         Vector2D::new(0.0, 0.0),
    ///         Vector2D::new(0.5, 0.0),
    ///     ),
    /// ).unwrap();
    ///
    /// assert!(world.object(object_id).is_some());
    /// ```
    pub fn try_register_object(
        &mut self,
        object_config: ObjectConfig,
    ) -> Result<usize, ConfigError> {
        object_config.validate()?;
        let id = self.counter;
        let mut m_object = MObject::new(
            object_config,
            self.config.proper_time_step,
            self.observer_object.position().time,
            id,
        );
        let mut object_tracker = ObjectTracker::new();
        self.counter += 1;
        if object_config.motion_mode == MotionMode::AlwaysConstantVelocity {
            let photons = m_object.emmit_all_photons();
            object_tracker.track_photons(photons);
        }
        self.insert_event_possible_to_detect_for_new_object(id, &mut m_object);
        self.registered_objects
            .insert(id, (m_object, object_tracker));
        Ok(id)
    }

    /// Registers an object and returns its ID.
    ///
    /// Prefer [`Self::try_register_object`] when invalid user input must be
    /// reported to the caller. If registration fails, this convenience method
    /// returns `usize::MAX`.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, ObjectConfig, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// let id = world.register_object(ObjectConfig::at_position(
    ///     Vector2D::new(1.0, 0.0),
    /// ));
    /// assert_ne!(id, usize::MAX);
    /// ```
    pub fn register_object(&mut self, object_config: ObjectConfig) -> usize {
        self.try_register_object(object_config)
            .unwrap_or(usize::MAX)
    }

    /// Creates an event at the current laboratory time with a collision group.
    ///
    /// The returned ID can be passed to [`Self::event`], [`Self::observe_event`]
    /// and [`Self::unregister_event`].
    pub fn create_event_with_collision_group(
        &mut self,
        event_position: Vector2D<f64>,
        collision_group: CollisionGroup,
    ) -> usize {
        let event = MVector::new(self.lab_time(), event_position);
        self.create_event_at_impl(event, collision_group)
    }

    /// Creates an event at the current laboratory time.
    ///
    /// The event uses [`CollisionGroup::All`].
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// let event_id = world.create_event(Vector2D::new(1.0, 0.0));
    /// assert!(world.event(&event_id).is_some());
    /// ```
    pub fn create_event(&mut self, event_position: Vector2D<f64>) -> usize {
        let event = MVector::new(self.lab_time(), event_position);
        self.create_event_at_impl(event, CollisionGroup::All)
    }

    /// Creates an event at an explicit spacetime position.
    ///
    /// The `time` component of `event` is laboratory time. The event uses
    /// [`CollisionGroup::All`].
    pub fn create_event_at(&mut self, event: MVector<f64>) -> usize {
        self.create_event_at_impl(event, CollisionGroup::All)
    }

    /// Removes a registered object from the world.
    ///
    /// Passing [`ObjectSelection::Observer`] has no effect. Removing an object
    /// also removes its active collision pairs.
    pub fn unregister_object<T: Into<ObjectSelection>>(&mut self, id: T) {
        match id.into() {
            ObjectSelection::Object(id) => {
                self.registered_objects.remove(&id);
                self.event_possible_to_detect.remove(&id);
                self.active_collision_pairs
                    .retain(|pair, _| !pair.contains(ObjectSelection::Object(id)));
            }
            ObjectSelection::Observer => {}
        }
    }

    /// Removes an event from the world.
    ///
    /// Passing an unknown ID has no effect.
    pub fn unregister_event(&mut self, id: &usize) {
        self.events.remove(id);
        self.event_possible_to_detect.iter_mut().for_each(|s| {
            s.1.remove(id);
        });
    }

    /// Enables or disables collision detection for the selected object.
    ///
    /// This does not remove the object. Passing [`ObjectSelection::Observer`]
    /// changes collision detection for the observer.
    pub fn set_object_collision_enabled<T: Into<ObjectSelection>>(
        &mut self,
        id: T,
        collision_enabled: bool,
    ) {
        let selection = id.into();
        if !collision_enabled {
            self.active_collision_pairs
                .retain(|pair, _| !pair.contains(selection));
        }
        if let Some(object) = self.get_object_with_selection_mut(&selection) {
            object.set_collision_enabled(collision_enabled);
        }
    }

    /// Returns a snapshot of the selected object's current state in lab coordinates.
    /// Don't use this method for observation from observer frame - use [`self.observe_object`] instead
    /// Returns `None` when the selected registered object does not exist.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, ObjectConfig, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// let id = world.register_object(ObjectConfig::at_position(Vector2D::new(0.0, 0.0)));
    /// assert_eq!(world.object(id).unwrap().id, id);
    /// ```
    pub fn object<T: Into<ObjectSelection>>(&self, id: T) -> Option<ObjectState> {
        self.get_object_with_selection(&id.into())
            .map(|e| e.state())
    }

    /// Returns a snapshot of the observer's current state.
    ///
    /// The state is expressed in laboratory coordinates.
    pub fn observer_object(&self) -> ObjectState {
        self.observer_object.state()
    }

    /// Returns an event's position in laboratory coordinates.
    ///
    /// Returns `None` when the event ID is unknown.
    pub fn event(&self, id: &usize) -> Option<MVector<f64>> {
        self.events.get(id).map(|e| *e.position())
    }
    /// Returns the observer's current observation of a registered object.
    ///
    /// Returns `None` when the object ID is unknown. A known object can still
    /// return [`ObjectObservation::NotVisible`] if its emitted light has not
    /// reached the observer.
    pub fn observe_object(&self, id: &usize) -> Option<ObjectObservation> {
        self.registered_objects
            .get(id)
            .map(|e| match e.1.get_object_was_seen() {
                true => ObjectObservation::Visible(e.1.to_visible_observation()),
                false => ObjectObservation::NotVisible,
            })
    }

    /// Observes an event in the observer's current frame.
    ///
    /// Returns `None` when the event ID is unknown. A known event returns
    /// [`EventObservation::Visible`] when it lies in the observer's past light
    /// cone; otherwise it returns [`EventObservation::NotVisible`].
    ///
    /// The visible position is relative to the observer and expressed in the
    /// observer's frame.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{EventObservation, MWorld, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// let event_id = world.create_event(Vector2D::new(0.0, 0.0));
    ///
    /// assert!(matches!(
    ///     world.observe_event(&event_id),
    ///     Some(EventObservation::Visible(_))
    /// ));
    /// ```
    pub fn observe_event(&self, id: &usize) -> Option<EventObservation> {
        self.events.get(id).map(|event| {
            let relative = *event.position() - *self.observer_object.position();
            if relative.is_time_or_light_like() {
                EventObservation::Visible(
                    relative.lorentz_transform(*self.observer_object.get_velocity()),
                )
            } else {
                EventObservation::NotVisible
            }
        })
    }

    /// Returns whether an event is currently observable by the observer.
    ///
    /// Returns `false` when the event ID is unknown or when the event is
    /// outside the observer's past light cone.
    pub fn is_event_in_light_cone(&self, id: &usize) -> bool {
        self.events
            .get(id)
            .map(|event| {
                let relative = *event.position() - *self.observer_object.position();
                relative.is_time_or_light_like()
            })
            .unwrap_or(false)
    }

    /// Returns the visible observation of a registered object.
    ///
    /// Returns `None` when the object is unknown or has not yet been observed.
    /// Unlike [`Self::observe_object`], this method returns the observation
    /// directly instead of returning [`ObjectObservation::NotVisible`].
    pub fn observe_visible_object(&self, id: &usize) -> Option<VisibleObjectObservation> {
        self.registered_objects
            .get(id)
            .and_then(|e| match e.1.get_object_was_seen() {
                true => Some(e.1.to_visible_observation()),
                false => None,
            })
    }

    /// Sets the velocity of the observer or a dynamic registered object.
    ///
    /// The velocity is expressed in laboratory coordinates and in units where
    /// the speed of light is `1`.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError::SuperluminalVelocity`] for a non-finite or
    /// superluminal velocity. Returns [`ConfigError::UnsupportedOperation`]
    /// when the selected object does not exist or uses
    /// [`MotionMode::AlwaysConstantVelocity`].
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// world.set_velocity(0, Vector2D::new(0.4, 0.0)).unwrap();
    /// assert_eq!(world.observer_object().velocity, Vector2D::new(0.4, 0.0));
    /// ```
    pub fn set_velocity<T: Into<ObjectSelection>>(
        &mut self,
        id: T,
        velocity: Vector2D<f64>,
    ) -> Result<(), ConfigError> {
        if !velocity.x.is_finite()
            || !velocity.y.is_finite()
            || velocity.length_squared() >= crate::MAX_SAFE_SPEED_SQUARED
        {
            return Err(ConfigError::SuperluminalVelocity);
        }
        if let Some(object) = self.get_object_with_selection_mut(&id.into()) {
            if object.constant_velocity() {
                return Err(ConfigError::UnsupportedOperation(
                    "set_velocity on constant-velocity object",
                ));
            }
            object.set_velocity(velocity);
            return Ok(());
        }
        Err(ConfigError::UnsupportedOperation("unknown object"))
    }

    /// Sets the acceleration of the observer or a dynamic registered object.
    ///
    /// This operation has no effect for an unknown object or an object using
    /// [`MotionMode::AlwaysConstantVelocity`].
    pub fn set_acceleration<T: Into<ObjectSelection>>(
        &mut self,
        id: T,
        acceleration: Vector2D<f64>,
    ) {
        if let Some(object) = self.get_object_with_selection_mut(&id.into()) {
            object.set_acceleration(acceleration);
        }
    }

    /// Sets the observer's velocity in laboratory coordinates.
    ///
    /// The velocity is expressed in units where the speed of light is `1`.
    /// Invalid velocities are ignored; use [`Self::set_velocity`] when the
    /// error needs to be handled explicitly.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::{MWorld, Vector2D};
    ///
    /// let mut world = MWorld::new();
    /// world.set_observer_velocity(Vector2D::new(0.6, 0.0));
    /// assert_eq!(world.observer_object().velocity, Vector2D::new(0.6, 0.0));
    /// ```
    pub fn set_observer_velocity(&mut self, velocity: Vector2D<f64>) {
        self.set_velocity(0, velocity).ok();
    }
    /// Returns the observer's proper time since the beginning of the simulation.
    ///
    /// Proper time is measured along the observer's worldline.
    pub fn observer_tau(&self) -> f64 {
        self.observer_object.get_tau()
    }

    /// Returns the current time in the laboratory frame.
    pub fn lab_time(&self) -> f64 {
        self.observer_object.position().time
    }

    /// Returns the observer's current position in laboratory coordinates.
    pub fn observer_position(&self) -> MVector<f64> {
        *self.observer_object.position()
    }

    /// Advances the simulation by an amount of the observer's proper time.
    ///
    /// The observer is advanced first. Registered objects are then advanced to
    /// the resulting laboratory time. The returned notifications contain event
    /// detections and collisions found during this step.
    ///
    /// # Arguments
    ///
    /// * `delta` - The observer's proper-time increment. A negative value does
    ///   not advance the simulation.
    ///
    /// # Example
    ///
    /// ```
    /// use minkowski_space::MWorld;
    ///
    /// let mut world = MWorld::new();
    /// let callbacks = world.advance_by_proper_time(1.0);
    ///
    /// assert!(!callbacks.iter().any(|_| false));
    /// assert!(world.observer_tau() >= 1.0);
    /// ```
    pub fn advance_by_proper_time(&mut self, delta: f64) -> Vec<ProcessTimeCallback> {
        let observer_events_to_check = self.get_observer_events_to_check();
        let observer_detections = self
            .observer_object
            .process_as_observer_object_tau(delta, observer_events_to_check);

        let target_time = self.lab_time();

        let receiver_data = Arc::new(ReceiverData {
            m_pos: *self.observer_object.position(),
            velocity: *self.observer_object.get_velocity(),
        });

        let events_to_check: HashMap<usize, HashMap<usize, MVector<f64>>> =
            self.get_events_to_check(target_time);

        let detected_events: Vec<(usize, usize, MVector<f64>)> =
            self.advance_objects_parallel(delta, target_time, receiver_data, events_to_check);

        self.hash_grid
            .build_grid(&self.observer_object, &self.registered_objects);

        let mut callbacks = self.get_event_callbacks(observer_detections, detected_events);
        let collisions = self.detect_collisions();
        callbacks.extend(collisions);

        callbacks
    }
}

//Crate private
impl MWorld {
    pub(crate) fn get_registered_objects(&self) -> &HashMap<usize, (MObject, ObjectTracker)> {
        &self.registered_objects
    }

    pub(crate) fn get_object_with_selection(
        &self,
        selection: &ObjectSelection,
    ) -> Option<&MObject> {
        match selection {
            ObjectSelection::Object(id) => Some(&self.registered_objects.get(id)?.0),
            ObjectSelection::Observer => Some(&self.observer_object),
        }
    }

    pub(crate) fn get_object_with_selection_mut(
        &mut self,
        selection: &ObjectSelection,
    ) -> Option<&mut MObject> {
        match selection {
            ObjectSelection::Object(id) => Some(&mut self.registered_objects.get_mut(id)?.0),
            ObjectSelection::Observer => Some(&mut self.observer_object),
        }
    }

    pub(crate) fn get_hash_grid(&self) -> &HashGrid {
        &self.hash_grid
    }

    pub(crate) fn configured_pairs(&self) -> &BTreeSet<CollisionGroupPair> {
        &self.config.collision_pairs
    }
}

//Private
impl MWorld {
    fn detect_collisions(&mut self) -> Vec<ProcessTimeCallback> {
        let collisions = CollisionCalculator { world: self }.detect_collisions();
        let now = self.lab_time();

        for collision in collisions {
            let pair = CollisionPair::new(collision.object_a, collision.object_b);
            let was_active = self.active_collision_pairs.contains_key(&pair);
            let collision_time = collision.position.time;
            self.active_collision_pairs
                .insert(pair, collision_time + EPSILON);

            if !was_active && collision_time > now - EPSILON {
                self.pending_collisions.push(PendingCollision(collision));
            }
        }
        self.active_collision_pairs
            .retain(|_, keep_until| *keep_until > now);

        let mut callbacks = Vec::new();
        while self
            .pending_collisions
            .peek()
            .is_some_and(|pending| pending.0.position.time <= now)
        {
            let pending = self
                .pending_collisions
                .pop()
                .expect("pending collision disappeared after peek");
            callbacks.push(ProcessTimeCallback::Collision(pending.0));
        }

        callbacks
    }

    fn insert_event_possible_to_detect_for_new_object(
        &mut self,
        id: usize,
        m_object: &mut MObject,
    ) {
        if matches!(m_object.collision_group(), CollisionGroup::Empty) {
            return;
        }
        self.event_possible_to_detect.insert(
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
                tracker.recalculate_properties(object, receiver_data.as_ref(), delta);
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
        observer_detections: Vec<(usize, MVector<f64>)>,
        detected_events: Vec<(usize, usize, MVector<f64>)>,
    ) -> Vec<ProcessTimeCallback> {
        let mut res = Vec::new();
        for (event_id, event_detection_position) in observer_detections {
            self.remove_object_event_possible_to_detect(0, event_id);
            if let Some(callback) =
                self.handle_observer_event_detection(event_id, event_detection_position)
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
    fn create_event_at_impl(
        &mut self,
        event: MVector<f64>,
        collision_group: CollisionGroup,
    ) -> usize {
        let id = self.counter;
        self.counter += 1;
        let m_event = MEvent::new(event, collision_group);
        if matches!(m_event.collision_group(), CollisionGroup::Empty) {
            self.events.insert(id, m_event);
            return id;
        }
        let event_ids_to_insert: Vec<_> = self
            .event_possible_to_detect
            .iter()
            .map(|e| *e.0)
            .filter(|object_id| {
                self.get_object_with_selection(&object_id.into())
                    .is_some_and(|object| {
                        object.collision_group().collision_group_matches(
                            m_event.collision_group(),
                            &self.config.collision_pairs,
                        )
                    })
            })
            .collect();

        event_ids_to_insert.into_iter().for_each(|object_id| {
            self.event_possible_to_detect
                .entry(object_id)
                .and_modify(|s| {
                    s.insert(id);
                });
        });
        self.events.insert(id, m_event);
        id
    }
    fn get_observer_events_to_check(&self) -> HashMap<usize, MVector<f64>> {
        self.event_possible_to_detect
            .get(&0)
            .iter()
            .flat_map(|e| e.iter())
            .filter_map(|id| self.events.get(id).map(|event| (*id, *event.position())))
            .collect()
    }

    fn handle_observer_event_detection(
        &mut self,
        event_id: usize,
        event_detection_position: MVector<f64>,
    ) -> Option<ProcessTimeCallback> {
        if !self.events.contains_key(&event_id) {
            return None;
        }
        let detection = EventDetection {
            event_id,
            detection_object: DetectionObject::Observer,
            event_detection_position,
        };
        Some(ProcessTimeCallback::Event(detection))
    }

    fn remove_object_event_possible_to_detect(&mut self, object_id: usize, event_id: usize) {
        if let Some(possible_events) = self.event_possible_to_detect.get_mut(&object_id) {
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
            self.event_possible_to_detect.get(object_id),
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
