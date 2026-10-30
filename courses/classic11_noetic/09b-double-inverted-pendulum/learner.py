"""L09 选做 B：串联双杆，只有小车水平力一个控制输入。

observation={x, xdot, q1, q1dot, q2, q2dot}；m/m·s⁻¹/rad/rad·s⁻¹。
q1是杆1相对上垂线角，q2是杆2相对杆1的原生关节角。
因此 theta2_abs=q1+q2，omega2_abs=q1dot+q2dot；正角绕+Y向+X倾。
只完成三个 TODO；保存后切 student 并重载，不需自己解 Riccati 方程。
"""
import numpy as np

K = np.array([7.07106781, 11.24116542, -161.66415569,
              3.24033851, 233.38706339, 31.37395393])
FORCE_LIMIT = 20.0


def state_error(observation, target_x=0.0):
    """TODO 1（约 3 行）：构造六维误差，两根杆都用绝对角/绝对角速度。

    顺序 [x-target_x, xdot, wrap(q1), q1dot, wrap(q1+q2), q1dot+q2dot]。
    wrap(a)=(a+pi)%(2*pi)-pi；第二杆先求和再包裹，q2不是其绝对角。
    """
    raise NotImplementedError("TODO 1: implement state_error and relative-to-absolute conversion in learner.py")


def feedback_force(error):
    """TODO 2（1 行）：按 u=-K@error 返回未限幅标量力，保留整体负号。"""
    raise NotImplementedError("TODO 2: implement feedback_force in learner.py")


def clip_force(force, force_limit=FORCE_LIMIT):
    """TODO 3（1 行）：使用传入限值，双向限幅，返回 float。"""
    raise NotImplementedError("TODO 3: implement clip_force in learner.py")


def control(observation, target_x=0.0):
    """真实 100 Hz 流水线：原生关节 → 六维误差 → 反馈力 → 双向限幅。"""
    return clip_force(feedback_force(state_error(observation, target_x)))
