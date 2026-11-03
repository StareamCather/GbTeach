"""Cascaded hover controller and allocation diagnostics."""
import numpy as np

from reference import altitude_acceleration, mix


GRAVITY = 9.81
TARGET_Z = 1.0
ROTOR_MAX = 6.0
ARM_M = .22
REACTION_PER_THRUST_M = .018


def wrap(angle):
    return (angle+np.pi)%(2*np.pi)-np.pi


def rotor_wrench(rotors):
    front, left, rear, right = rotors
    return [float(front+left+rear+right), float(ARM_M*(left-right)),
            float(ARM_M*(rear-front)), float(REACTION_PER_THRUST_M*(front-left+rear-right))]


class Cascade:
    def __init__(self, functions=None):
        self.integral = 0.
        self.altitude = functions.altitude_acceleration if functions else altitude_acceleration
        self.mixer = functions.mix if functions else mix

    def compute(self, state, dt):
        state = np.asarray(state, dtype=float)
        if state.shape != (12,) or not np.isfinite(state).all() or not np.isfinite(dt) or dt <= 0:
            raise ValueError('Expected 12 finite native state components and a positive step interval')
        x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz = state
        z_error = TARGET_Z-z
        self.integral = float(np.clip(self.integral+z_error*dt, -.6, .6))
        az = float(np.clip(self.altitude(z_error, vz, self.integral), -4, 4))
        ax = float(np.clip(-1.4*x-1.8*vx, -2, 2))
        ay = float(np.clip(-1.4*y-1.8*vy, -2, 2))

        # Rotate the requested horizontal world acceleration into yaw-aligned axes.
        forward = np.cos(yaw)*ax+np.sin(yaw)*ay
        left = -np.sin(yaw)*ax+np.cos(yaw)*ay
        desired_pitch = float(np.clip(forward/GRAVITY, -.25, .25))
        desired_roll = float(np.clip(-left/GRAVITY, -.25, .25))
        total = float((GRAVITY+az)/max(.65, np.cos(roll)*np.cos(pitch)))
        roll_torque = float(.9*(desired_roll-roll)-.24*wx)
        pitch_torque = float(.9*(desired_pitch-pitch)-.24*wy)
        yaw_torque = float(.5*wrap(-yaw)-.2*wz)
        raw = np.asarray(self.mixer(total, roll_torque, pitch_torque, yaw_torque), dtype=float)
        if raw.shape != (4,) or not np.isfinite(raw).all():
            raise ValueError('Mixer must return four finite rotor thrusts')
        limited = np.clip(raw, 0, ROTOR_MAX)
        return {'target_z_m': TARGET_Z, 'altitude_error_m': float(z_error),
                'altitude_integral_m_s': self.integral, 'requested_world_acceleration_m_s2': [ax, ay, az],
                'desired_roll_rad': desired_roll, 'desired_pitch_rad': desired_pitch, 'desired_yaw_rad': 0.,
                'requested_wrench_n_nm': [total, roll_torque, pitch_torque, yaw_torque],
                'raw_rotors_n': raw.tolist(), 'rotors_n': limited.tolist(),
                'allocated_wrench_n_nm': rotor_wrench(limited),
                'saturated_rotors': np.flatnonzero(raw != limited).tolist()}

    def action(self, state, dt):
        return np.asarray(self.compute(state, dt)['rotors_n'])
