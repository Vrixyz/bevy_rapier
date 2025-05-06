use rapier::prelude::Voxels;

/// Read-only access to the properties of a [`Voxels`] shape.
#[derive(Copy, Clone)]
pub struct VoxelsView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Voxels,
}

/// Read-write access to the properties of a [`Voxels`].
pub struct VoxelsViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Voxels,
}
