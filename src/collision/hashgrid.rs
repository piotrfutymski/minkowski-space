use crate::WorldConfig;
use crate::m_object::MObject;
use crate::object_tracker::ObjectTracker;
use std::collections::HashMap;
use vector2d::Vector2D;

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub(crate) struct HashCell {
    idx_x: u32,
    idx_y: u32,
}
pub(crate) struct HashGrid {
    cell_size: f64,
    object_grid: HashMap<HashCell, Vec<usize>>,
}

impl HashGrid {
    pub(crate) fn init(config: &WorldConfig) -> Self {
        Self {
            cell_size: config.spatial_hash_cell_size,
            object_grid: Default::default(),
        }
    }
    pub(crate) fn build_grid(
        &mut self,
        frame_object: &MObject,
        registered_objects: &HashMap<usize, (MObject, ObjectTracker)>,
    ) {
        self.object_grid
            .iter_mut()
            .for_each(|(_, vector)| vector.clear());
        self.insert_m_object(0, frame_object);
        registered_objects
            .iter()
            .for_each(|r| self.insert_m_object(*r.0, &r.1.0));
    }
    pub(crate) fn to_cell<T: Into<Vector2D<f64>>>(&self, pos: T) -> HashCell {
        let vec = pos.into();
        let idx_x = (vec.x / self.cell_size) as u32;
        let idx_y = (vec.y / self.cell_size) as u32;
        HashCell { idx_x, idx_y }
    }

    fn insert_m_object(&mut self, id: usize, object: &MObject) {
        let cell = self.to_cell(object.position());
        self.object_grid
            .entry(cell)
            .and_modify(|e| e.push(id))
            .or_insert(vec![id]);
    }
}
