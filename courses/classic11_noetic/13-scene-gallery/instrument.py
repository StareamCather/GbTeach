"""Actual SDK frame analysis; rendering objectives require separate real GUI review."""
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time

OFFICIAL = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent)).resolve()
WORKSPACE = Path(os.environ.get("GAZEBOTEACH_WORKSPACE", Path.cwd())).resolve()
sys.path.insert(0, str(OFFICIAL))
from config import PROBES, digest, read_world, resources
sys.path.insert(0, str(OFFICIAL.parent/"_control_support"))
from pause import read_pause, set_pause


def directory():
    if not os.environ.get("GAZEBOTEACH_SESSION_DIR"):
        raise RuntimeError("请在已经启动的本课终端运行，缺少会话环境")
    return Path(os.environ["GAZEBOTEACH_SESSION_DIR"])/"control"


def strict_json(text):
    def invalid(x):
        raise ValueError("Non-finite JSON constant: "+x)
    return json.loads(text, parse_constant=invalid)


def status():
    path = directory()/"status.json"
    if not path.is_file():
        raise RuntimeError("真实SDK观察器尚未就绪，请查看初始化日志")
    return strict_json(path.read_text(encoding="utf-8"))


def near(actual, expected, label, tolerance=1e-7):
    if type(actual) not in (int, float) or not math.isfinite(actual) or abs(actual-expected) > tolerance:
        raise AssertionError("实际SDK字段不符："+label+" "+repr(actual)+" != "+repr(expected))


def vector(actual, expected, label, tolerance=1e-7):
    if not isinstance(actual, list) or len(actual) != len(expected):
        raise AssertionError("实际SDK向量长度不符："+label)
    for a, b in zip(actual, expected):
        near(a, b, label, tolerance)


def validate_model(m, parameters):
    h = m["heightmap"]
    if not isinstance(h["uri"], str) or not h["uri"].endswith("assets/heightmap.png"):
        raise AssertionError("Actual SDK heightmap URI differs")
    vector(h["size_m"], [4, 4, parameters["height_m"]], "heightmap Size")
    vector(h["pos_m"], [0, 0, 0], "heightmap Pos")
    vector(h["vertex_count"], [65, 65], "heightmap VertexCount", 0)
    near(h["sampling"], 1, "heightmap sampling", 0)
    if h["collision_name"] != "terrain::terrain_link::terrain_collision":
        raise AssertionError("Unexpected actual terrain collision identity")
    for entry, fixture, index in zip(h["indexed_heights"], PROBES[:4], [(16, 16), (48, 16), (16, 48), (48, 48)]):
        if entry["id"] != fixture[0]:
            raise AssertionError("Native heightmap indexed probe order differs")
        vector([entry["ix"], entry["iy"]], list(index), "native plateau vertex index", 0)
        near(entry["height_m"], parameters["height_m"]*fixture[3]/255., "indexed height", 1e-6)
    if len(h["indexed_heights"]) != 4:
        raise AssertionError("Missing actual indexed heights")
    near(m["physics_step_s"], .001, "physics dt")
    vector(m["gravity_m_s2"], [0, 0, -9.81], "gravity")
    if len(m["probes"]) != 6:
        raise AssertionError("Six native dynamic probes are required")
    for entry, fixture in zip(m["probes"], PROBES):
        if entry["id"] != fixture[0] or entry["is_static"] or entry["kinematic"] or not entry["gravity"]:
            raise AssertionError("Actual probe dynamics/topology differ")
        vector(entry["initial_position_m"], [fixture[1], fixture[2], 2], "probe initial position")
        near(entry["radius_m"], .1, "probe radius")
        near(entry["mass_kg"], .1, "probe mass")
    for s, y in zip(m["supports"], [-.9, .9]):
        vector(s["size_m"], [.6, .6, .6], "same support collisions")
        vector(s["world_pose"], [3.4, y, .3, 1, 0, 0, 0], "support world pose")
    near(m["actors"]["walker"]["mesh_node_count"], 31, "actual skin skeleton nodes", 0)
    if not any(a["name"] == "walking" and a["length_s"] > 0 for a in m["actors"]["walker"]["animations"]):
        raise AssertionError("Actual walking skeleton animation missing")
    near(m["actors"]["marker"]["mesh_node_count"], 0, "no-skin marker", 0)
    lights = {l["name"].split("::")[-1]: l for l in m["lights"]}
    if set(lights) != {"daylight", "point_lamp", "spot_lamp"}:
        raise AssertionError("Actual native light identities differ")
    rgba = lights["daylight"]["diffuse"]
    for key in ("r", "g", "b"):
        near(rgba.get(key, 0), parameters["daylight"], "native daylight diffuse", 1e-6)


def inspect():
    parameters = read_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf")
    assets = resources(WORKSPACE)
    s = status()
    wanted = {"world_sha256": digest(WORKSPACE/"world.sdf"), "source_sha256": digest(OFFICIAL/"gallery.cc"),
              "client_source_sha256": digest(OFFICIAL/"gallery_camera.cc"), "png_sha256": assets["heightmap"]["sha256"]}
    if any(s.get(k) != v for k, v in wanted.items()) or s.get("parameters") != parameters or s.get("assets") != assets:
        raise AssertionError("个人world/资源/source SHA已改变；必须显式重新加载 reload 后再观测或切视角")
    if s.get("run_id") != os.environ.get("GAZEBOTEACH_SESSION_TOKEN"):
        raise AssertionError("SDK状态来自其他会话")
    if not s.get("ready") or s.get("error"):
        raise AssertionError("真实观察器错误："+str(s.get("error")))
    validate_model(s["model"], parameters)
    return s


def frames_for(epoch, filename="gallery-frames.jsonl"):
    rows = []
    with (directory()/filename).open(encoding="utf-8") as source:
        for line in source:
            if not line.endswith("\n"):
                continue
            row = strict_json(line)
            if row["epoch"] == epoch:
                rows.append(row)
    return rows


def marker_position(t, cycle):
    times = [cycle*f for f in (0, 5/12, 1/2, 11/12, 1)]
    x = [-1., 1., 1., -1., -1.]
    for i in range(4):
        if t <= times[i+1]+1e-10:
            u = min(1., max(0., (t-times[i])/(times[i+1]-times[i])))
            return x[i]+(x[i+1]-x[i])*(3*u*u-2*u*u*u)
    raise AssertionError("Native marker ScriptTime outside current closed cycle")


def analyse(rows, p, model, tail_rows):
    cycle = p["actor_cycle_s"]
    if len(rows) < 2 or rows[0]["elapsed_s"] > .025 or rows[-1]["elapsed_s"] < 2.1*cycle:
        raise AssertionError("真实观测必须覆盖新reset起点及至少2.1个周期")
    for a, b in zip(rows, rows[1:]):
        if not 0 < b["sim_time_s"]-a["sim_time_s"] <= .021 or b["iteration"] <= a["iteration"] or b["epoch"] != a["epoch"]:
            raise AssertionError("实际帧不连续或跨复位epoch")
    settling = [r for r in rows if 3 <= r["elapsed_s"] <= 3.5+1e-9]
    if len(settling) < 20:
        raise AssertionError("缺少3..3.5sim秒真实静止检查窗")
    if len(tail_rows) < 201:
        raise AssertionError("缺少至少201个真实1ms落球尾部快照")
    for row in tail_rows:
        if row["run_id"] != rows[0]["run_id"] or row["epoch"] != rows[0]["epoch"] or not 3-1e-8 <= row["elapsed_s"] <= 3.5+1e-8:
            raise AssertionError("落球尾部快照不属于当前真实epoch/时间窗")
    for a, b in zip(tail_rows, tail_rows[1:]):
        if b["iteration"] != a["iteration"]+1 or abs((b["sim_time_s"]-a["sim_time_s"])-.001) > 1e-8:
            raise AssertionError("1kHz落球尾部缺步或时钟不连续")
    probe_metrics = []
    joint_good = [True]*len(tail_rows)
    for name, x, y, gray in PROBES:
        start = next(q for q in rows[0]["probes"] if q["id"] == name)
        if abs(start["position_m"][2]-2) > .001:
            raise AssertionError("原生models reset未恢复真实落球初态："+name)
        expected = .7 if gray is None else p["height_m"]*gray/255.+.1
        longest = consecutive = 0.
        max_z = max_xy = max_speed = 0.
        previous = None
        for index, row in enumerate(tail_rows):
            q = next(q for q in row["probes"] if q["id"] == name)
            zerr = abs(q["position_m"][2]-expected)
            xy = math.hypot(q["position_m"][0]-x, q["position_m"][1]-y)
            speed = math.sqrt(sum(v*v for v in q["velocity_m_s"]))
            good = zerr <= .01 and xy <= .03 and speed <= .03
            joint_good[index] = joint_good[index] and good
            consecutive = (consecutive+row["sim_time_s"]-previous) if good and previous is not None else 0.
            longest = max(longest, consecutive)
            previous = row["sim_time_s"] if good else None
            max_z, max_xy, max_speed = max(max_z, zerr), max(max_xy, xy), max(max_speed, speed)
        if longest < .2-1e-9:
            raise AssertionError("真实落球未连续.2sim秒稳定在预测碰撞高度："+name)
        probe_metrics.append({"id": name, "expected_center_z_m": expected, "max_z_error_m": max_z,
                              "max_xy_offset_m": max_xy, "max_speed_m_s": max_speed, "longest_settled_s": longest})
    joint_start = None
    joint_count = joint_longest_count = 0
    joint_longest = 0.
    for row, good in zip(tail_rows, joint_good):
        if good:
            if joint_start is None:
                joint_start = row["sim_time_s"]
            joint_count += 1
            span = row["sim_time_s"]-joint_start
            if span > joint_longest:
                joint_longest, joint_longest_count = span, joint_count
        else:
            joint_start, joint_count = None, 0
    if joint_longest < .2-1e-9 or joint_longest_count < 201:
        raise AssertionError("六个真实落球没有在同一连续.2sim秒/201快照窗口共同稳定")
    ray_max = 0.
    for row in settling:
        for r, fixture in zip(row["rays"], PROBES[:4]):
            expected = p["height_m"]*fixture[3]/255.
            if r["id"] != fixture[0] or r["entity"] != model["heightmap"]["collision_name"]:
                raise AssertionError("真实ray未命中terrain collision："+repr(r))
            vector([r["x_m"], r["y_m"]], [fixture[1]+.35, fixture[2]], "ray plateau location")
            ray_max = max(ray_max, abs(r["z_m"]-expected))
    if ray_max > 1e-4:
        raise AssertionError("真实射线高度超过1e-4m误差")
    marker_rows = [r["actors"]["marker"] for r in rows if r["actors"]["marker"]["has_animation_frame"]]
    walkers = [r["actors"]["walker"] for r in rows if r["actors"]["walker"]["has_animation_frame"]]
    if not marker_rows or not walkers or any(not a["active"] for a in marker_rows+walkers):
        raise AssertionError("Actor真实动画未持续active")
    error = 0.
    for a in marker_rows:
        expected = [marker_position(a["script_time_s"], cycle), -2.7, .15]
        error = max(error, math.dist(a["world_pose"][:3], expected))
        q = a["world_pose"][3:]
        if abs(abs(q[0])-1) > 1e-7 or max(abs(v) for v in q[1:]) > 1e-7:
            raise AssertionError("Marker方向不再恒定")
    if error > 1e-5:
        raise AssertionError("真实marker偏离零切线Hermite轨迹："+str(error))
    wrap_times = {}
    for name in ("marker", "walker"):
        active = [r for r in rows if r["actors"][name]["has_animation_frame"]]
        times = [b["sim_time_s"] for a, b in zip(active, active[1:])
                 if b["actors"][name]["script_time_s"] < a["actors"][name]["script_time_s"]-.1]
        if len(times) < 2 or any(abs((b-a)-cycle) > .08 for a, b in zip(times, times[1:])):
            raise AssertionError("真实Actor loop次数或周期不符："+name)
        wrap_times[name] = times
    span = max(a["world_pose"][0] for a in walkers)-min(a["world_pose"][0] for a in walkers)
    feet = {}
    for key in ("left_foot_relative_hips", "right_foot_relative_hips"):
        positions = [a[key][:3] for a in walkers if a["distinct_update"]]
        if len(positions) < 2:
            raise AssertionError("Missing actual distinct skeleton frames")
        change = max(math.dist(a, b) for i, a in enumerate(positions) for b in positions[i+1:])
        if change < .03:
            raise AssertionError("真实脚骨相对Hips没有明确运动："+key)
        feet[key+"_max_pairwise_position_change_m"] = change
    if span < 1.5:
        raise AssertionError("Walker真实水平移动不足1.5m")
    return {"probes": probe_metrics, "joint_longest_settled_s": joint_longest,
            "joint_longest_settled_samples": joint_longest_count, "max_ray_height_error_m": ray_max,
            "marker_max_position_error_m": error, "marker_loop_wraps": len(wrap_times["marker"]),
            "actual_actor_wrap_times_s": wrap_times, "probe_tail_samples": len(tail_rows),
            "probe_tail_actual_span_s": tail_rows[-1]["sim_time_s"]-tail_rows[0]["sim_time_s"],
            "marker_distinct_updates": sum(a["distinct_update"] for a in marker_rows),
            "walker_distinct_updates": sum(a["distinct_update"] for a in walkers),
            "walker_x_span_m": span, "bone_motion": feet, "actual_duration_s": rows[-1]["elapsed_s"]}


def experiment():
    initial = inspect()
    original = read_pause()
    cleanup = None
    try:
        set_pause(True)
        reset = subprocess.run(["gz", "world", "-w", "default", "-o"], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=4)
        if reset.returncode:
            raise RuntimeError("公开models reset失败："+reset.stderr)
        set_pause(False)
        deadline = time.monotonic()+140
        wanted = 2.1*initial["parameters"]["actor_cycle_s"]
        while True:
            current = status()
            if current["error"]:
                raise AssertionError("真实SDK观察失败："+current["error"])
            snapshot = current.get("snapshot", {})
            if (current["epoch"] > initial["epoch"] and not current["reset_pending"]
                    and snapshot.get("epoch") == current["epoch"] and snapshot.get("run_id") == initial["run_id"]
                    and snapshot.get("elapsed_s", -1) >= wanted):
                break
            if time.monotonic() > deadline:
                raise RuntimeError("等待原生reset和2.1周期实际观测超时；保留raw帧")
            time.sleep(.05)
        set_pause(True)
        final = inspect()
        rows = frames_for(current["epoch"])
        tail_rows = frames_for(current["epoch"], "probe-tail.jsonl")
        metrics = analyse(rows, initial["parameters"], initial["model"], tail_rows)
        result = {"passed": True, "parameters": initial["parameters"], "epoch": current["epoch"],
                  "world_sha256": initial["world_sha256"], "source_sha256": initial["source_sha256"],
                  "client_source_sha256": initial["client_source_sha256"], "png_sha256": initial["png_sha256"],
                  "actual_model": initial["model"], "assets": initial["assets"], "metrics": metrics,
                  "samples": len(rows), "first_actual_frame": rows[0], "last_actual_frame": rows[-1],
                  "reset_count_before": initial["reset_count"], "reset_count_after": final["reset_count"],
                  "raw_frames": str(directory()/"gallery-frames.jsonl"), "reset_events": str(directory()/"resets.jsonl"),
                  "probe_tail": str(directory()/"probe-tail.jsonl"),
                  "gui_visual_check": "not_performed", "visual_review_required": True}
    finally:
        cleanup = set_pause(original)
    result.update(original_pause=original, pause_cleanup=cleanup)
    (directory()/"verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def refresh_owned_window(window, application, read_geometry, initial, deadline):
    """Request one reversible refresh; a WM may legitimately ignore the resize."""
    def wait_size(size, required):
        until = min(deadline, time.monotonic()+.5)
        while True:
            application.processEvents()
            actual = read_geometry()
            if actual[2:] == size:
                return actual
            if time.monotonic() >= until:
                if required:
                    raise RuntimeError("owned窗口实际改变后未恢复原尺寸")
                return actual
            time.sleep(.01)

    try:
        window.resize(initial[2]+1, initial[3])
        resized = wait_size([initial[2]+1, initial[3]], False)
    finally:
        window.resize(initial[2], initial[3])
        application.processEvents()
    restored = wait_size(initial[2:], True)
    return resized, restored, resized[2:] == [initial[2]+1, initial[3]]


def activate_owned_gui():
    """Initialize only this session's new native window; camera ACK stays separate."""
    token = os.environ.get("GAZEBOTEACH_SESSION_TOKEN", "")
    if not re.fullmatch(r"[0-9a-f]{32}", token) or not os.environ.get("DISPLAY"):
        raise RuntimeError("激活窗口需要本课真实图形会话及有效Session token")
    if threading.current_thread() is not threading.main_thread():
        raise RuntimeError("窗口激活必须在CLI主线程执行")
    if os.environ.get("QT_QPA_PLATFORM", "").lower() in ("offscreen", "minimal"):
        raise RuntimeError("不能在offscreen/minimal环境激活真实Gazebo窗口")
    for name in ("xprop", "xwininfo"):
        if not shutil.which(name):
            raise RuntimeError("缺少实际窗口检查工具："+name)
    owned = []
    for process in Path("/proc").glob("[0-9]*"):
        try:
            if (process/"comm").read_text().strip() == "gzclient" and ("GAZEBOTEACH_SESSION_TOKEN="+token).encode() in (process/"environ").read_bytes().split(b"\0"):
                owned.append(int(process.name))
        except OSError:
            continue
    if len(owned) != 1:
        raise RuntimeError("本课必须恰有一个owned gzclient，实际："+repr(owned))
    pid = owned[0]
    start_identity = (Path("/proc")/str(pid)/"stat").read_text().rsplit(")", 1)[1].split()[19]
    deadline = time.monotonic()+3

    def output(arguments):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise RuntimeError("owned Gazebo窗口激活超过3秒预算")
        result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, timeout=min(1., remaining))
        if result.returncode:
            raise RuntimeError("窗口检查失败："+" ".join(arguments)+" "+result.stderr.strip())
        return result.stdout

    def geometry(xid):
        raw = output(["xwininfo", "-id", xid])
        values = []
        for field in ("Absolute upper-left X", "Absolute upper-left Y", "Width", "Height"):
            match = re.search(re.escape(field)+r":\s*(-?\d+)", raw)
            if not match:
                raise RuntimeError("无法读取owned窗口实际geometry："+field)
            values.append(int(match.group(1)))
        return values

    candidates = []
    for xid in re.findall(r"0x[0-9a-fA-F]+", output(["xprop", "-root", "_NET_CLIENT_LIST"])):
        properties = output(["xprop", "-id", xid, "_NET_WM_PID", "WM_STATE"])
        match = re.search(r"_NET_WM_PID\([^)]*\)\s*=\s*(\d+)", properties)
        if match and int(match.group(1)) == pid and "window state: Normal" in properties:
            actual = geometry(xid)
            if actual[2] >= 640 and actual[3] >= 360:
                candidates.append((actual[2]*actual[3], xid, actual))
    if not candidates:
        raise RuntimeError("尚无owned正常Gazebo主窗口；不会激活Splash或其他窗口")
    _, xid, before = max(candidates)
    from PyQt5 import QtGui
    application = QtGui.QGuiApplication.instance() or QtGui.QGuiApplication([])
    window = QtGui.QWindow.fromWinId(int(xid, 16))
    if window is None:
        raise RuntimeError("无法包装owned Gazebo XID")
    requested_at = time.monotonic()
    window.showNormal()
    window.raise_()
    window.requestActivate()
    method = "QWindow.showNormal/raise/requestActivate"
    if shutil.which("wmctrl"):
        output(["wmctrl", "-ia", xid])
        method += "+wmctrl owned XID"
    application.processEvents()
    normal = geometry(xid)

    resized, restored, resize_observed = refresh_owned_window(
        window, application, lambda: geometry(xid), normal, deadline)
    if restored[2:] != normal[2:]:
        raise RuntimeError("owned窗口尺寸刷新后未恢复实际原尺寸")
    current_identity = (Path("/proc")/str(pid)/"stat").read_text().rsplit(")", 1)[1].split()[19]
    if current_identity != start_identity:
        raise RuntimeError("窗口初始化期间gzclient进程身份已变化")
    return {"requested": True, "owned_pid": pid, "owned_XID": xid,
            "request_monotonic_s": requested_at, "method": method,
            "geometry_before": before, "geometry_normal": normal,
            "geometry_during_one_pixel_resize": resized, "geometry_restored": restored,
            "resize_observed": resize_observed,
            "camera_confirmation": "separate_actual_SDK_ACK_required"}


def view(station, timeout=4, activate=False):
    current = inspect()
    if not math.isfinite(timeout) or not 0 < timeout <= 8:
        raise ValueError("view timeout must be finite within (0,8]")
    activation = {"requested": bool(activate)}
    if activate:
        try:
            activation = activate_owned_gui()
        except Exception as error:
            result = {"passed": False, "station": station, "activation": activation,
                      "error": "owned GUI activation: "+type(error).__name__+": "+str(error),
                      "network_events": [], "gui_visual_check": "not_confirmed"}
            with (directory()/"camera-events.jsonl").open("a", encoding="utf-8") as out:
                out.write(json.dumps(result, ensure_ascii=False)+"\n")
            return result
    r = subprocess.run([str(WORKSPACE/"build/gallery_camera"), station, str(timeout)], text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2*timeout+7)
    events = [strict_json(line) for line in r.stdout.splitlines() if line.startswith("{")]
    success = [e for e in events if e.get("event") == "view_confirmed"]
    result = {"passed": r.returncode == 0 and len(success) == 1, "returncode": r.returncode, "station": station,
              "network_events": events, "stderr": r.stderr, "activation": activation, "server_lights": current["model"]["lights"],
              "world_sha256": current["world_sha256"], "gui_visual_check": "camera_pose_confirmed_pixels_require_review"}
    result["outer_timeout_s"] = 2*timeout+7
    with (directory()/"camera-events.jsonl").open("a", encoding="utf-8") as out:
        out.write(json.dumps(result, ensure_ascii=False)+"\n")
    return result
