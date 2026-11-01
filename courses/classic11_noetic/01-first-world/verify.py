#!/usr/bin/env python3
"""Verify a real fall using world.sdf in the learner's current workspace.

The application invokes this trusted source script with the learner's cwd.
Reading __file__ would incorrectly validate the untouched course template.
"""
import math
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from _control_support.models import pose as native_model_pose

DEADLINE = None


def run(*args):
    remaining = DEADLINE - time.monotonic()
    if remaining <= 0.1:
        raise RuntimeError("验收达到有界时间上限；请检查仿真实时因子和服务器日志")
    result = subprocess.run(
        args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        timeout=min(2.1, remaining),
    )
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()[-1200:]
        raise RuntimeError("命令失败：%s\n%s" % (" ".join(args), detail))
    return result.stdout


def numbers(text, count, label):
    try:
        values = [float(v) for v in (text or "").split()]
    except ValueError as exc:
        raise ValueError("%s 必须填写数值" % label) from exc
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError("%s 必须包含 %d 个有限数值" % (label, count))
    return values


def centered(element, label):
    if element.get("relative_to"):
        raise ValueError("本课限定中心对齐 box；%s 暂不使用 relative_to" % label)
    if any(abs(v) > 1e-8 for v in numbers(element.text, 6, label)):
        raise ValueError("本课要求 %s 为零，保持碰撞、显示、惯性中心对齐" % label)


def configuration():
    world_file = Path.cwd() / "world.sdf"
    world = ET.parse(str(world_file)).getroot().find("world")
    if world is None:
        raise ValueError("工作副本 world.sdf 缺少 world")
    block = world.find("model[@name='falling_block']")
    if block is None:
        raise ValueError("找不到 falling_block；本课请保留模型名称")
    if block.findtext("static", "false").strip().lower() in ("true", "1"):
        raise ValueError("falling_block 被设为 static，不能进行动态落体验收")
    model_pose = numbers(block.findtext("pose", "0 0 0 0 0 0"), 6, "模型 pose")
    if any(abs(v) > 1e-8 for v in model_pose[3:]):
        raise ValueError("本课先保持初始姿态为零；任意旋转的支撑几何留给后续课程")
    links = block.findall("link")
    if len(links) != 1:
        raise ValueError("本课限定一个 link 的中心对齐方块")
    link = links[0]
    for item, label in ((link, "link"), (link.find("collision"), "collision"),
                        (link.find("visual"), "visual"), (link.find("inertial"), "inertial")):
        if item is None:
            raise ValueError("方块缺少 %s" % label)
        offset = item.find("pose")
        if offset is not None:
            centered(offset, label + " pose")
    collisions = link.findall("collision")
    if len(collisions) != 1:
        raise ValueError("本课限定一个居中的 box collision")
    dimensions = numbers(collisions[0].findtext("geometry/box/size"), 3, "碰撞 box size")
    visible = numbers(link.findtext("visual/geometry/box/size"), 3, "显示 box size")
    if min(dimensions) <= 0 or any(abs(a-b) > 1e-8 for a,b in zip(dimensions, visible)):
        raise ValueError("碰撞和显示的 box size 必须正值且一致；改边长时请同步修改两处")
    gravity = numbers(world.findtext("gravity", "0 0 -9.8"), 3, "gravity")
    if abs(gravity[0]) > 1e-8 or abs(gravity[1]) > 1e-8 or gravity[2] >= 0:
        raise ValueError("本课采用竖直向下重力；gravity 的 x、y 应为0且 z<0")
    mass = float(link.findtext("inertial/mass", "nan"))
    inertia = link.find("inertial/inertia")
    if inertia is None or not math.isfinite(mass) or mass <= 0:
        raise ValueError("动态刚体需要正质量和有效惯性矩阵")
    xx, yy, zz = [float(inertia.findtext(n, "nan")) for n in ("ixx", "iyy", "izz")]
    xy, xz, yz = [float(inertia.findtext(n, "0")) for n in ("ixy", "ixz", "iyz")]
    determinant = xx*yy*zz + 2*xy*xz*yz - xx*yz*yz - yy*xz*xz - zz*xy*xy
    if not all(math.isfinite(v) for v in (xx,yy,zz,xy,xz,yz)) or xx <= 0 or xx*yy-xy*xy <= 0 or determinant <= 0:
        raise ValueError("惯性矩阵必须对称正定；参考页给出了均匀长方体的计算公式")
    sx, sy, sz = dimensions
    uniform = (mass*(sy*sy+sz*sz)/12.0,
               mass*(sx*sx+sz*sz)/12.0,
               mass*(sx*sx+sy*sy)/12.0)
    for name, measured, predicted in zip(("Ixx", "Iyy", "Izz"), (xx,yy,zz), uniform):
        if abs(measured-predicted) > max(abs(predicted)*1e-4, 1e-10):
            raise ValueError("均匀方块的%s配置不符：当前%.9g，按质量与尺寸应为%.9g kg·m²。改变尺寸时需同步更新惯性；落地现象正确不代表所有物理参数正确。" %
                             (name, measured, predicted))
    if max(abs(xy), abs(xz), abs(yz)) > max(max(uniform)*1e-4, 1e-10):
        raise ValueError("本课均匀、主轴对齐方块的惯性交叉项应接近0；请检查ixy、ixz、iyz")
    expected = dimensions[2] / 2.0
    print("配置：工作副本=%s；box=%s m；质量=%.4g kg" % (world_file, dimensions, mass), flush=True)
    print("惯性对角项：Ixx=%.6g、Iyy=%.6g、Izz=%.6g kg·m²" % (xx,yy,zz), flush=True)
    print("几何预测：地面z=0，中心高度=box高度/2=%.4f m" % expected, flush=True)
    return world.get("name", "default"), expected, dimensions[2]


def pose(world):
    raw=native_model_pose(world,"falling_block",DEADLINE)
    if raw is not None:return raw
    data = run("gz", "model", "-w", world, "-m", "falling_block", "-p")
    for line in reversed(data.splitlines()):
        fields = line.strip().split()
        if len(fields) == 6:
            try:
                values = [float(v) for v in fields]
                if all(math.isfinite(v) for v in values):
                    return values
            except ValueError:
                pass
    raise RuntimeError("服务器没有返回方块的实时六元组位姿：%r" % data[-400:])


def quaternion(raw):
    r, p, y = [value/2 for value in raw[3:]]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def confirm_lift(world, expected):
    """Observe the single lift publish without extending the physical budget."""
    expected_q = quaternion(expected)
    last = last_q = None
    while DEADLINE-time.monotonic() > .3:
        try:
            last = pose(world)
            last_q = quaternion(last)
        except Exception as error:
            raise RuntimeError("暂停后抬升只读确认失败：目标位置/四元数=%s/%s，最后实测=%s/%s；%s" %
                               (expected[:3], expected_q, last, last_q, error)) from error
        dot = abs(sum(a*b for a, b in zip(last_q, expected_q)))
        angle = 2*math.acos(max(-1., min(1., dot)))
        if (abs(last[2]-expected[2]) <= .03 and
                math.hypot(last[0]-expected[0], last[1]-expected[1]) <= .03 and angle <= .08):
            return last
        # The publish may precede application in Gazebo's update loop. Repeat
        # only actual pose reads; never resend the move or replace the helper.
        remaining = DEADLINE-time.monotonic()-.3
        if remaining > 0:
            time.sleep(min(.05, remaining))
    raise RuntimeError("暂停后抬升未在原截止时间内确认：目标位置/四元数=%s/%s，最后实测=%s/%s；移动只发布一次" %
                       (expected[:3], expected_q, last, last_q))


def prepare_pose_reader(world):
    """Warm the optional SDK and read actual pose before the physical budget.

    SDK discovery/build is synchronous and has its own longer inner timeouts.
    On its supported Linux runtime, an outer alarm also bounds cold compilation.
    This performs no world/model mutation and never replaces a failed helper.
    """
    global DEADLINE
    started = time.monotonic()
    DEADLINE = started + 20.0
    previous_handler = previous_timer = None

    def expired(signum, frame):
        raise RuntimeError("位姿读取准备超过20秒预算；未开始物理实验，不自动替换辅助进程")

    # Non-POSIX runtimes cannot start the optional native helper; their CLI
    # fallback is already bounded by run() and this same preparation deadline.
    if hasattr(signal, "setitimer"):
        previous_handler = signal.signal(signal.SIGALRM, expired)
        previous_timer = signal.setitimer(signal.ITIMER_REAL, 20.0)
    try:
        pose(world)
    finally:
        elapsed = time.monotonic() - started
        if previous_timer is not None:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous_handler)
            if previous_timer[0] > 0:
                signal.setitimer(signal.ITIMER_REAL, max(.001, previous_timer[0]-elapsed), previous_timer[1])
        print("位姿读取准备耗时：%.3f秒（独立20秒墙钟预算，含可选SDK冷编译和只读握手）" % elapsed, flush=True)


def paused_state(world):
    # Classic 11 accepts integer seconds for -d; '-d 0.2' is not portable.
    output = run("gz", "topic", "-e", "/gazebo/%s/world_stats" % world, "-d", "1")
    values = re.findall(r"\bpaused:\s*(true|false)\b", output)
    if not values:
        raise RuntimeError("world_stats 没有给出 paused 状态，无法可靠恢复原实验状态")
    return values[-1] == "true"


def set_paused(world, paused):
    run("gz", "topic", "-p", "/gazebo/%s/world_control" % world,
        "-m", "pause: %s" % ("true" if paused else "false"))


def verify():
    global DEADLINE
    world, expected, height = configuration()
    tolerance = max(0.02, height * 0.04)
    prepare_pose_reader(world)
    experiment_started = time.monotonic()
    DEADLINE = experiment_started + 8.0
    original_paused = paused_state(world)
    print("原暂停状态：%s；验收临时控制暂停，结束后恢复" % original_paused, flush=True)
    experiment_error = None
    try:
        set_paused(world, True)
        if not paused_state(world):
            raise RuntimeError("暂停请求后 world_stats 仍显示运行，未执行抬升")
        initial = pose(world)
        lift = max(2.0, expected + 1.0)
        target = [initial[0], initial[1], lift, 0., 0., 0.]
        run("gz", "model", "-w", world, "-m", "falling_block",
            "-x", str(initial[0]), "-y", str(initial[1]), "-z", str(lift),
            "-R", "0", "-P", "0", "-Y", "0")
        high = confirm_lift(world, target)
        print("已暂停并确认抬升：z=%.4f m；现在继续真实动力学仿真" % high[2], flush=True)
        set_paused(world, False)
        settled = []
        last = high
        while DEADLINE - time.monotonic() > 0.3:
            last = pose(world)
            now = time.monotonic()
            upright = max(abs(last[3]), abs(last[4])) < 0.08
            if abs(last[2] - expected) <= tolerance and upright:
                settled.append((now, last[2]))
                settled = settled[-4:]
                if len(settled) >= 3 and settled[-1][0]-settled[0][0] >= 0.25:
                    spread = max(p[1] for p in settled) - min(p[1] for p in settled)
                    if spread <= tolerance * 0.4:
                        if high[2] - last[2] < 0.3:
                            raise RuntimeError("高度变化不足，未证明真实下落")
                        print("PASS：预测=%.4f m；测量=%.4f m；容差=±%.4f m；连续稳定样本=%d" %
                              (expected, last[2], tolerance, len(settled)), flush=True)
                        return
            else:
                settled = []
            time.sleep(0.07)
        raise RuntimeError("未在限时内稳定落地：预测z=%.4f±%.4f m，最后位姿=%s。若已改文件，请先重启仿真以重新加载几何。" %
                           (expected, tolerance, last))
    except Exception as exc:
        experiment_error = exc
        raise
    finally:
        print("物理验收耗时：%.3f秒（原8秒墙钟总预算，恢复状态另计）" %
              (time.monotonic()-experiment_started), flush=True)
        # A separate bounded cleanup budget preserves pause state after failure.
        DEADLINE = time.monotonic() + 2.8
        try:
            set_paused(world, original_paused)
            if paused_state(world) != original_paused:
                raise RuntimeError("验收后未确认恢复原暂停状态，请检查运行日志")
            print("已恢复原暂停状态：%s" % original_paused, flush=True)
        except Exception as restore_error:
            if experiment_error is not None:
                print("附加恢复失败：%s；上面的实验失败原因仍保留" % restore_error,
                      file=sys.stderr, flush=True)
            else:
                raise


if __name__ == "__main__":
    try:
        verify()
    except Exception as exc:
        print("FAIL：%s" % exc, file=sys.stderr, flush=True)
        sys.exit(1)
