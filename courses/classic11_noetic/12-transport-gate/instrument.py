"""Real network RPC/ACK transactions and independent physical gate checks."""
import csv
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import uuid

OFFICIAL = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent)).resolve()
WORKSPACE = Path(os.environ.get("GAZEBOTEACH_WORKSPACE", Path.cwd())).resolve()
sys.path.insert(0, str(OFFICIAL))
from config import digest, read_parameters, read_world, validate_model
sys.path.insert(0, str(OFFICIAL.parent/"_control_support"))
from pause import read_pause, set_pause


def directory():
    if not os.environ.get("GAZEBOTEACH_SESSION_DIR"):
        raise RuntimeError("请在本课已启动的实验终端中运行，缺少会话环境")
    return Path(os.environ["GAZEBOTEACH_SESSION_DIR"])/"control"


def _strict_json(text):
    def invalid(value):
        raise ValueError("Invalid non-finite JSON constant: "+value)
    return json.loads(text, parse_constant=invalid)


def native(mode, options, timeout=6):
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 12:
        raise ValueError("SDK wall timeout must be finite within (0,12] seconds")
    binary = WORKSPACE/"build/transport_client"
    if not binary.is_file():
        raise RuntimeError("SDK client尚未构建，请重新加载项目")
    args = [str(binary), mode, "--world", "default", "--timeout", str(timeout)]
    for key, value in options.items():
        if key == "omit-target":
            if value:
                args.append("--omit-target")
        elif value is not None:
            args += ["--"+key, str(value)]
    try:
        r = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout+6)
        code, output, error = r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired as e:
        code = 124
        output = e.stdout.decode("utf-8", "replace") if isinstance(e.stdout, bytes) else (e.stdout or "")
        error = "outer SDK client timeout; no command retry"
    events, other = [], []
    for line in output.splitlines():
        try:
            events.append(_strict_json(line))
        except (ValueError, TypeError):
            other.append(line)
    return {"passed": code == 0, "returncode": code, "network_events": events, "stderr": error,
            "other_output": other, "outer_timeout_s": timeout+6, "command_retried": False}


def rpc(request="gazeboteach_transport_snapshot", nonce=None, timeout=2):
    if request not in ("gazeboteach_transport_snapshot", "gazeboteach_transport_missing"):
        return {"passed": False, "returncode": 2, "transport_initialized": False, "published": False,
                "network_events": [], "reason": "query request is outside the read-only allowlist"}
    nonce = nonce or uuid.uuid4().hex
    result = native("query", {"request": request, "nonce": nonce,
                              "expected-run": os.environ.get("GAZEBOTEACH_SESSION_TOKEN", "")}, timeout)
    responses = [event for event in result["network_events"] if event.get("event") == "rpc_response"]
    result["query_nonce"] = nonce
    if result["passed"] and len(responses) == 1:
        result["rpc"] = responses[0]
        result["state"] = responses[0]["payload"]
    return result


def inspect():
    p = read_parameters(WORKSPACE/"parameters.json")
    read_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf")
    result = rpc()
    if not result["passed"] or "state" not in result:
        raise RuntimeError("真实只读RPC失败："+json.dumps(result, ensure_ascii=False))
    state = result["state"]
    current = {"config_sha256": digest(WORKSPACE/"parameters.json"), "world_sha256": digest(WORKSPACE/"world.sdf"),
               "source_sha256": digest(OFFICIAL/"transport_gate.cc"), "client_source_sha256": digest(OFFICIAL/"transport_client.cc")}
    if any(state.get(key) != value for key, value in current.items()):
        raise AssertionError("parameters/world/source/client SHA已改变，须显式重载 reload 后再控制")
    if state["parameters"] != p:
        raise AssertionError("实际加载参数与当前个人parameters.json不一致，请重载")
    validate_model(state["model"])
    if state["error"]:
        raise AssertionError("实际滑门错误："+state["error"])
    return result


def entity():
    result = native("entity", {}, 2)
    responses = [event for event in result["network_events"] if event.get("event") == "rpc_response"]
    if result["passed"] and len(responses) == 1:
        result["rpc"] = responses[0]
        result["model"] = responses[0]["payload"]
    return result


def entity_geometry(result, target=None):
    """Compose native Model.pose and Link.pose; never equate a local link pose to world."""
    m = result["model"]
    links = [link for link in m.get("link", []) if link.get("name", "").split("::")[-1] == "panel"]
    joints = [joint for joint in m.get("joint", []) if joint.get("name", "").split("::")[-1] == "slide_joint"]
    if m.get("name") != "transport_gate" or len(links) != 1 or len(joints) != 1 or len(joints[0].get("angle", [])) != 1:
        raise AssertionError("Native entity_info Model omitted the actual gate link/joint")
    def pose(value):
        if "position" not in value or "orientation" not in value:
            raise AssertionError("Native entity_info pose is missing")
        pos, rot = value["position"], value["orientation"]
        p = [float(pos.get(key, 0.)) for key in ("x", "y", "z")]
        q = [float(rot.get(key, 0.)) for key in ("w", "x", "y", "z")]
        if not all(math.isfinite(v) for v in p+q):
            raise AssertionError("Native entity_info has non-finite pose")
        close(sum(v*v for v in q), 1., "native entity quaternion norm", 1e-5)
        return p, q
    mp, mq = pose(m["pose"])
    lp, lq = pose(links[0]["pose"])
    def multiply(a, b):
        w, x, y, z = a
        t, u, v, k = b
        return [w*t-x*u-y*v-z*k, w*u+x*t+y*k-z*v, w*v-x*k+y*t+z*u, w*k+x*v-y*u+z*t]
    rotated = multiply(multiply(mq, [0.]+lp), [mq[0], -mq[1], -mq[2], -mq[3]])[1:]
    wp = [x+y for x, y in zip(mp, rotated)]
    wq = multiply(mq, lq)
    q = float(joints[0]["angle"][0])
    close(wp[0], q, "independent native entity world panel x versus joint q", 1e-5)
    close(wp[1], 0., "native entity world y", 1e-5)
    close(wp[2], .6, "native entity world z", 1e-5)
    close(abs(wq[0]), 1., "native entity world orientation", 1e-5)
    for value in wq[1:]:
        close(value, 0., "native entity world orientation", 1e-5)
    if target is not None and abs(q-target) > .005:
        raise AssertionError("Native entity_info endpoint is not near the actual requested target")
    return {"joint_q_m": q, "composed_panel_world_position_m": wp, "composed_panel_world_quaternion": wq,
            "target_m": target, "composed_model_and_link_pose": True}


def topic_info(topic):
    expanded = topic.replace("~/", "/gazebo/default/", 1) if topic.startswith("~/") else topic
    try:
        r = subprocess.run(["gz", "topic", "-w", "default", "-i", expanded], text=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=4)
        match = re.search(r"^Type:\s*(\S+)", r.stdout, re.MULTILINE)
        return {"returncode": r.returncode, "stdout": r.stdout, "stderr": r.stderr,
                "actual_type": match.group(1) if match else "", "outer_timeout_s": 4, "input_topic": topic, "expanded_topic": expanded}
    except subprocess.TimeoutExpired:
        return {"returncode": 124, "stdout": "", "stderr": "bounded gz topic introspection timeout", "actual_type": "", "outer_timeout_s": 4, "input_topic": topic, "expanded_topic": expanded}


def _ticket_path(path=None):
    return Path(path).expanduser().resolve() if path else directory()/"last-ticket.json"


def send(target=None, client=None, seq=1, nonce=None, run=None, epoch=None, verb="set_target", omit_target=False,
         topic="~/transport_lab/command", expect_type="gazebo.msgs.Request", wait="completed", ticket_path=None, timeout=6):
    initial = inspect()
    state = initial["state"]
    info = topic_info(topic)
    if info["actual_type"] != expect_type or info["actual_type"] != "gazebo.msgs.Request":
        return {"passed": False, "returncode": 2, "reason": "wrong topic/no known subscriber or wire type mismatch before publish",
                "published": False, "topic_info": info, "bootstrap_rpc": initial["rpc"], "network_events": []}
    target = state["parameters"]["open_target_m"] if target is None else target
    ticket = {"run_id": state["run_id"], "world": state["world"], "epoch": state["epoch"],
              "request_run_id": state["run_id"] if run is None else run, "request_epoch": state["epoch"] if epoch is None else epoch,
              "client_id": client or "cli-"+uuid.uuid4().hex[:12], "seq": seq, "nonce": nonce or uuid.uuid4().hex,
              "attempt_nonce": uuid.uuid4().hex,
              "target_text": str(target), "verb": verb, "omit_target": omit_target, "config_sha256": state["config_sha256"]}
    path = _ticket_path(ticket_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(ticket, ensure_ascii=False, indent=2), encoding="utf-8")
    options = {"expected-run": ticket["run_id"], "expected-epoch": ticket["epoch"], "run": ticket["request_run_id"],
               "epoch": ticket["request_epoch"], "client": ticket["client_id"], "seq": seq, "nonce": ticket["nonce"],
               "attempt": ticket["attempt_nonce"],
               "target": ticket["target_text"], "verb": verb, "omit-target": omit_target, "topic": topic,
               "expect-type": expect_type, "observed-type": info["actual_type"], "wait": wait}
    options.update({"baseline-received": state["counters"]["received"], "baseline-rejected": state["counters"]["rejected"]})
    result = native("send", options, timeout)
    result.update(ticket=ticket, ticket_path=str(path), topic_info=info, bootstrap_rpc=initial["rpc"])
    ticket["received_network_events"] = result["network_events"]
    path.write_text(json.dumps(ticket, ensure_ascii=False, indent=2), encoding="utf-8")
    if result["passed"]:
        result["postflight_rpc"] = inspect()["rpc"]
    return result


def wait_ticket(path=None, wait="completed", timeout=6):
    path = _ticket_path(path)
    ticket = _strict_json(path.read_text(encoding="utf-8"))
    initial = inspect()
    state = initial["state"]
    if ticket["run_id"] != state["run_id"] or ticket["epoch"] != state["epoch"]:
        return {"passed": False, "returncode": 3, "reason": "ticket boot/epoch cancelled by reset or reload", "ticket": ticket,
                "bootstrap_rpc": initial["rpc"], "network_events": [], "command_resent": False}
    result = native("wait", {"expected-run": ticket["run_id"], "expected-epoch": ticket["epoch"],
                             "client": ticket["client_id"], "seq": ticket["seq"], "nonce": ticket["nonce"],
                             "attempt": ticket["attempt_nonce"], "target": ticket["target_text"], "wait": wait}, timeout)
    result.update(ticket=ticket, ticket_path=str(path), command_resent=False, bootstrap_rpc=initial["rpc"])
    if result["passed"]:
        result["postflight_rpc"] = inspect()["rpc"]
    return result


def watch(seconds=2):
    initial = inspect()
    result = native("watch", {"expected-run": initial["state"]["run_id"], "expected-epoch": initial["state"]["epoch"]}, seconds)
    result["bootstrap_rpc"] = initial["rpc"]
    return result


def close(actual, expected, label, tolerance=1e-7):
    if not math.isfinite(actual) or abs(actual-expected) > tolerance:
        raise AssertionError("%s: observed=%g expected=%g" % (label, actual, expected))


def analyse(ticket, purpose, open_target):
    with (directory()/"gate.csv").open(newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle) if row["run_id"] == ticket["run_id"] and int(row["epoch"]) == ticket["epoch"]
                and row["client_id"] == ticket["client_id"] and int(row["seq"]) == ticket["seq"] and row["nonce"] == ticket["nonce"]
                and row["attempt_nonce"] == ticket["attempt_nonce"]]
    if not rows or len(rows) > 3000:
        raise AssertionError("真实事务必须在3000个1ms物理步内完成")
    target = float(ticket["target_text"])
    consecutive = 0
    last = None
    for index, row in enumerate(rows, 1):
        values = {key: float(value) for key, value in row.items() if key not in ("run_id", "client_id", "nonce", "attempt_nonce", "stage", "config_sha256")}
        if not all(math.isfinite(v) for v in values.values()):
            raise AssertionError("实际滑门CSV含非有限数")
        close(values["transaction_step"], index, "actual transaction step")
        close(values["transaction_elapsed_s"], index*.001, "explicit actual post-step interval")
        close(values["dt_s"], .001, "actual physics dt", 1e-9)
        close(values["target_m"], target, "actual loaded target")
        close(values["panel_x_m"], values["q_after_m"], "independent actual panel x versus slider q", 1e-5)
        close(values["panel_y_m"], 0., "actual panel y", 1e-5)
        close(values["panel_z_m"], .6, "actual panel z", 1e-5)
        for key, want in (("axis_x", 1.), ("axis_y", 0.), ("axis_z", 0.), ("panel_qx", 0.), ("panel_qy", 0.), ("panel_qz", 0.)):
            close(values[key], want, "actual fixed slider axis/orientation", 1e-5)
        close(abs(values["panel_qw"]), 1., "actual panel unit orientation", 1e-5)
        if abs(values["force_N"]) > 20+1e-7:
            raise AssertionError("真实native PD控制力超过20N")
        close(values["force_N"], values["pid_command_N"], "actual native force-input ledger", 1e-7)
        if last is not None:
            close(values["q_before_m"], last["q_after_m"], "actual pre/post position continuity", 1e-7)
        good = abs(values["q_after_m"]-target) <= .005 and abs(values["qd_after_m_s"]) <= .01
        consecutive = consecutive+1 if good else 0
        close(values["completion_consecutive_s"], consecutive*.001, "actual continuous completion interval", 1e-8)
        last = values
    if rows[-1]["stage"] != "completed" or consecutive < 200:
        raise AssertionError("completed缺少连续200个真实到位物理步")
    qvalues = [float(row["q_after_m"]) for row in rows]
    span = max(qvalues+[float(rows[0]["q_before_m"])])-min(qvalues+[float(rows[0]["q_before_m"])])
    if abs(last["q_after_m"]-target) > .005:
        raise AssertionError("实际事务终点没有到位")
    if purpose in ("open", "close"):
        initial_expected = 0. if purpose == "open" else open_target
        if abs(float(rows[0]["q_before_m"])-initial_expected) > .005 or span < open_target-.01:
            raise AssertionError("完整开关门缺少正确真实起点或open_target_m−0.01的实际运动跨度")
    return {"actual_samples": len(rows), "duration_s": len(rows)*.001, "target_m": target,
            "initial_q_m": float(rows[0]["q_before_m"]), "final_q_m": last["q_after_m"],
            "final_qd_m_s": last["qd_after_m_s"], "actual_q_span_m": span,
            "continuous_completed_samples": consecutive, "peak_control_force_N": max(abs(float(row["force_N"])) for row in rows)}


def experiment():
    initial = inspect()
    original = read_pause()
    transactions = []
    client = "verify-"+uuid.uuid4().hex[:12]
    try:
        set_pause(False)
        open_target = initial["state"]["parameters"]["open_target_m"]
        for seq, purpose, target in ((1, "prepare_closed", 0.), (2, "open", open_target), (3, "close", 0.)):
            before = entity()
            if not before["passed"]:
                raise AssertionError("实际entity_info初态查询失败："+json.dumps(before))
            before["independent_geometry"] = entity_geometry(before)
            result = send(target=target, client=client, seq=seq)
            result["role"] = purpose
            transactions.append(result)
            if not result["passed"]:
                raise AssertionError("真实network ACK事务失败："+json.dumps(result, ensure_ascii=False))
            matched = [event["payload"]["stage"] for event in result["network_events"] if event.get("event") == "ack" and event.get("matched")]
            if not all(stage in matched for stage in ("received", "applied", "completed")):
                raise AssertionError("本次新nonce缺少三层真实网络ACK")
            result["metrics"] = analyse(result["ticket"], purpose, open_target)
            result["entity_before"] = before
            result["entity_after"] = entity()
            if not result["entity_after"]["passed"]:
                raise AssertionError("实际entity_info终态查询失败")
            result["entity_after"]["independent_geometry"] = entity_geometry(result["entity_after"], target)
        final = inspect()
    finally:
        cleanup = set_pause(original)
        if not cleanup["confirmed"]:
            raise RuntimeError("Cannot confirm original pause after gate verification")
    result = {"passed": True, "parameters": initial["state"]["parameters"], "config_sha256": initial["state"]["config_sha256"],
              "world_sha256": initial["state"]["world_sha256"], "source_sha256": initial["state"]["source_sha256"],
              "client_source_sha256": initial["state"]["client_source_sha256"], "actual_model": initial["state"]["model"],
              "transactions": transactions, "initial_rpc": initial["rpc"], "final_rpc": final["rpc"],
              "original_pause": original, "pause_cleanup": cleanup, "telemetry_csv": str(directory()/"gate.csv"),
              "ack_jsonl": str(directory()/"ack-events.jsonl"), "state_jsonl": str(directory()/"state-events.jsonl")}
    (directory()/"verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result
