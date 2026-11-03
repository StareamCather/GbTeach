"""Reference answers for the altitude PID and the signed plus mixer."""
import numpy as np


def altitude_acceleration(error, vertical_speed, integral):
    return 3.0*error-2.6*vertical_speed+.15*integral


def mix(total, roll_torque, pitch_torque, yaw_torque):
    arm = .22
    yaw_coefficient = .018
    return np.array([total/4-pitch_torque/(2*arm)+yaw_torque/(4*yaw_coefficient),
                     total/4+roll_torque/(2*arm)-yaw_torque/(4*yaw_coefficient),
                     total/4+pitch_torque/(2*arm)+yaw_torque/(4*yaw_coefficient),
                     total/4-roll_torque/(2*arm)-yaw_torque/(4*yaw_coefficient)])
