"""Complete two short control equations; the rest of the simulator is supplied."""
import numpy as np


def altitude_acceleration(error,vertical_speed,integral):
    # TODO1: Kp=3, Kd=2.6, Ki=.15; ascent speed must produce downward correction.
    raise NotImplementedError('TODO1: altitude PID acceleration command')


def mix(total,roll_torque,pitch_torque,yaw_torque):
    L=.22; k=.018
    # TODO2: solve the four linear equations shown in lesson.md, return [front,left,rear,right].
    raise NotImplementedError('TODO2: signed plus-layout rotor mixer')
