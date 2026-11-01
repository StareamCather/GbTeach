"""Actual Model.pose queries; no model writes, predictions or dynamics resets."""
import time
from .pause import get_control


def pose(world,name,deadline=None):
    """Return native xyz + SDK-derived RPY, or None for explicit SDK absence.

    The caller retains its existing runtime-only CLI path when the optional SDK
    is absent. A present SDK's build/query/protocol failure always propagates;
    it cannot trigger a replacement helper or a silent legacy retry.
    """
    if deadline is not None and deadline-time.monotonic()<=.3:
        raise RuntimeError('Native model pose query has no remaining wall budget')
    control=get_control(world,required=False)
    if control is None:return None
    remaining=3 if deadline is None else min(3,deadline-time.monotonic())
    if remaining<.3:raise RuntimeError('Native model pose initialization exhausted the wall budget')
    value=control.model_pose(name,timeout=remaining)
    return [value['pose']['position'][axis] for axis in ('x','y','z')]+value['rpy']
