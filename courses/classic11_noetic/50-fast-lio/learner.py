"""Fill two short functions before running student_check.py."""


def point_stamp_ns(scan_start_ns, offset_ns):
    """Return the absolute acquisition stamp as an exact integer in ns."""
    # TODO: use the scan start and the relative acquisition offset.
    raise NotImplementedError("Complete point_stamp_ns")


def plane_residual(point_world, point_on_plane, unit_normal):
    """Return the signed orthogonal residual in meters for three 3-vectors."""
    # TODO: project the world-frame displacement onto the unit normal.
    raise NotImplementedError("Complete plane_residual")
