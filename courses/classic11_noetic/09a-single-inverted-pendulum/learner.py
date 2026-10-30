"""L09 选做 A：只完成三个 TODO；保存、切到 student，然后重载项目。

真实观测 observation = {x: m, xdot: m/s, q1: rad, q1dot: rad/s}。
q1=0 为向上直立，正角绕 +Y 让杆顶向 +X 倾斜。
目标状态 [target_x, 0, 0, 0]；K 已按本模型计算，无需实现 Riccati 求解。
"""
import numpy as np

K = np.array([-14.14213562, -15.95141643, -92.53361631, -22.61484036])
FORCE_LIMIT = 15.0


def state_error(observation, target_x=0.0):
    """TODO 1（约 2 行）：将 q1 包到 [-pi, pi)，返回四维误差数组。

    提示：wrap(a) = (a + pi) % (2*pi) - pi；误差是当前状态减目标。
    顺序必须是 [x-target_x, xdot, theta_up, q1dot]。
    """
    raise NotImplementedError("TODO 1: implement state_error in learner.py")


def feedback_force(error):
    """TODO 2（1 行）：按 u = -K @ error 返回未限幅的标量力（N）。"""
    raise NotImplementedError("TODO 2: implement feedback_force in learner.py")


def clip_force(force, force_limit=FORCE_LIMIT):
    """TODO 3（1 行）：双向限幅到 [-force_limit, +force_limit]，返回 float。"""
    raise NotImplementedError("TODO 3: implement clip_force in learner.py")


def control(observation, target_x=0.0):
    """100 Hz 的真实反馈流水线：状态误差 → 反馈力 → 双向限幅。"""
    return clip_force(feedback_force(state_error(observation, target_x)))
