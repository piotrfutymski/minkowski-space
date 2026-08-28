use crate::collision::hashgrid::HashGrid;
use crate::{MWorld, ProcessTimeCallback};

pub(crate) struct CollisionCalculator<'a> {
    pub(crate) world: &'a MWorld,
}

impl CollisionCalculator<'_> {
    pub(crate) fn detect_collisions(&self) -> Vec<ProcessTimeCallback> {
        vec![]
    }

    pub(crate) fn detect_collision_candidates(grid: &HashGrid, world: &MWorld) {}
}
