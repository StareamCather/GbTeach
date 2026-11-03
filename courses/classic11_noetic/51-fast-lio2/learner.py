"""FAST-LIO2 practice: voxel indexing and point-to-plane pose Jacobian."""


def voxel_key(point, leaf_m):
    """Return integer grid indices for a world point (negative x/y/z allowed)."""
    # TODO: convert coordinates to cell indices with mathematical floor.
    raise NotImplementedError("Complete voxel_key")


def translation_jacobian(unit_normal_world):
    """Return dr/dt for r=n^T(Rp+t-q); the plane and rotation are fixed."""
    # TODO: differentiate the residual with respect to the three translations.
    raise NotImplementedError("Complete translation_jacobian")
