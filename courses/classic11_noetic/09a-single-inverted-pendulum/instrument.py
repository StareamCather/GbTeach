"""Real joint/force telemetry and independent, bounded multi-trial grading."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


OFFICIAL = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent)).resolve()
WORKSPACE = Path(os.environ.get("GAZEBOTEACH_WORKSPACE", Path.cwd())).resolve()
sys.path.insert(0,str(OFFICIAL.parent/"_control_support"))
from pause import read_pause, set_pause

# Sim-time subtraction can represent an exact 8 s control tick as 7.999999999999996.
# This tolerance only classifies time-window endpoints; it is 100,000x below one tick.
TIME_EPS = 1e-7


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def directory():
    if not os.environ.get("GAZEBOTEACH_SESSION_DIR"):
        raise RuntimeError("请在已启动的本课实验终端中运行；缺少本会话环境")
    return Path(os.environ["GAZEBOTEACH_SESSION_DIR"])/"control"


def status():
    path = directory()/"status.json"
    if not path.is_file():
        raise RuntimeError("本会话尚无原生控制桥状态；检查构建、插件加载和 runtime.log")
    return json.loads(path.read_text(encoding="utf-8"))


def unit_check():
    result = subprocess.run([sys.executable,str(OFFICIAL/"unit_probe.py"),str(WORKSPACE/"learner.py")],
                            text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=10)
    if result.returncode:
        raise AssertionError((result.stderr or result.stdout).strip())
    return json.loads(result.stdout.splitlines()[-1])


def inspect():
    result = status()
    result["current_learner_sha256"] = digest(WORKSPACE/"learner.py")
    result["configured_mode"] = json.loads((WORKSPACE/"runtime.json").read_text(encoding="utf-8"))["mode"]
    result["reload_required"] = (result["mode"] != result["configured_mode"] or
                                  (result["mode"] == "student" and result["source_sha256"] != result["current_learner_sha256"]) or
                                  result["world_sha256"] != digest(WORKSPACE/"world.sdf"))
    result["assignment_pass"] = False
    return result


def records(episode):
    with (directory()/"control.csv").open(newline="",encoding="utf-8") as handle:
        return [row for row in csv.DictReader(handle) if row.get("episode") == episode and row.get("controller_ok") in ("0","1")]


def evaluate(rows, mode, sha):
    if len(rows)<990:
        raise AssertionError("真实100 Hz回调样本不足："+str(len(rows)))
    fields=("sim_time_s","episode_time_s","x_m","xdot_m_s","q1_rad","q1dot_rad_s","theta_up_rad","pole_axis_x","pole_axis_z","python_force_N","command_force_N","disturbance_N","applied_force_N","callback_count","episode_callback_count")
    values=[{key:float(row[key]) for key in fields} for row in rows]
    if not all(math.isfinite(value) for row in values for value in row.values()):
        raise AssertionError("原生轨迹含非有限值")
    if any(row["mode"]!=mode or row["source_sha256"]!=sha or row["controller_ok"]!="1" for row in rows):
        raise AssertionError("实际执行模式、源文件SHA或控制异常状态不一致")
    for previous,current in zip(values,values[1:]):
        if not .0099<=current["episode_time_s"]-previous["episode_time_s"]<=.0101:
            raise AssertionError("控制周期不是0.01仿真秒")
        if current["episode_callback_count"]-previous["episode_callback_count"]!=1:
            raise AssertionError("每个控制时刻必须恰好执行一次学生Python")
    if any(abs(row["pole_axis_x"]-math.sin(row["q1_rad"]))>.002 or abs(row["pole_axis_z"]-math.cos(row["q1_rad"]))>.002 for row in values):
        raise AssertionError("SDK关节角与真实杆几何的直立零位/正方向不一致")
    peaks={"x_m":max(abs(row["x_m"]) for row in values),"theta_up_rad":max(abs(row["theta_up_rad"]) for row in values),
           "command_force_N":max(abs(row["command_force_N"]) for row in values)}
    if peaks["x_m"]>1.2 or peaks["theta_up_rad"]>.35 or peaks["command_force_N"]>15.000001:
        raise AssertionError("局部稳定/轨道/输入边界失败："+json.dumps(peaks))
    if any(abs(row["python_force_N"]-row["command_force_N"])>1e-6 for row in values):
        raise AssertionError("学生函数未自行限幅；原生安全限幅不能代替TODO3")
    tail=[row for row in values if 8-TIME_EPS<=row["episode_time_s"]<=10+TIME_EPS]
    rms={key:math.sqrt(sum(row[key]**2 for row in tail)/len(tail)) for key in ("x_m","xdot_m_s","theta_up_rad","q1dot_rad_s")}
    bounds={"x_m":.02,"xdot_m_s":.035,"theta_up_rad":.01,"q1dot_rad_s":.04}
    if any(rms[key]>bounds[key] for key in bounds):
        raise AssertionError("末2秒RMS未收敛："+json.dumps(rms))
    return {"samples":len(rows),"terminal_samples":len(tail),"terminal_window_s":[8,10],
            "time_endpoint_tolerance_s":TIME_EPS,"peaks":peaks,"terminal_RMS":rms,"terminal_RMS_bounds":bounds,
            "first_actual_state":{key:values[0][key] for key in ("x_m","xdot_m_s","q1_rad","q1dot_rad_s")},
            "source_sha256":sha,"mode":mode,"first_callback":values[0]["callback_count"],"last_callback":values[-1]["callback_count"]}


def _observe(seconds=3):
    """A raw physical experiment, including failing controllers; never assignment credit."""
    if not 1<=seconds<=15:
        raise ValueError("observe duration must be between 1 and 15 simulated seconds")
    initial=inspect()
    if initial["reload_required"]:
        raise AssertionError("模式/源文件已改变，请先重载项目")
    episode="observe-"+uuid.uuid4().hex
    path=directory()/"episode.request"
    temporary=path.with_suffix(".tmp")
    temporary.write_text("%s .1 0 .06 0 %.9g 0\n"%(episode,seconds),encoding="ascii")
    os.replace(str(temporary),str(path))
    set_pause(False)
    deadline=time.monotonic()+max(20,seconds*3)
    while True:
        current=status()
        if current["episode"]==episode and (current["error"] or current["episode_time_s"]>=seconds):
            break
        if time.monotonic()>=deadline:
            raise RuntimeError("Raw physical observation timed out; telemetry retained")
        time.sleep(.1)
    rows=records(episode)
    result={"assignment_pass":False,"observation_only":True,"episode":episode,"mode":current["mode"],
            "error":current["error"],"samples":len(rows),"source_sha256":current["source_sha256"],
            "peak_x_m":max(abs(float(row["x_m"])) for row in rows),
            "peak_theta_rad":max(abs(float(row["theta_up_rad"])) for row in rows),
            "telemetry_csv":str(directory()/"control.csv"),"status_json":str(directory()/"status.json")}
    return result


def _experiment(expected_mode="student", timeout=120):
    deadline=time.monotonic()+timeout
    state=inspect()
    if state["mode"]!=expected_mode or state["configured_mode"]!=expected_mode:
        raise AssertionError("当前为%s；作业验收只接受student。用python3 lab.py mode student后重载；参考演示请用demo-check。"%state["mode"])
    if state["reload_required"]:
        raise AssertionError("文件或模式已改变但运行中仍是旧代码，请先重载项目")
    if digest(WORKSPACE/"world.sdf")!=digest(OFFICIAL/"world.sdf"):
        raise AssertionError("本练习固定被控模型：请恢复world.sdf并重载，只修改learner.py的TODO")
    units=unit_check() if expected_mode=="student" else {"skipped":"reference demonstration is not an assignment pass"}
    source=WORKSPACE/"learner.py" if expected_mode=="student" else OFFICIAL/"reference_controller.py"
    sha=digest(source)
    if state["source_sha256"]!=sha:
        raise AssertionError("运行中的实际Python源SHA与当前文件不符，请重载")
    model=state["model"]
    for key,want,tolerance in (("cart_mass_kg",1,1e-9),("pole_mass_kg",.2,1e-9),("pole_Iyy_kg_m2",.0166866666666667,1e-10),
                               ("cart_damping",.05,1e-9),("pole_damping",.002,1e-9),("physics_step_s",.001,1e-10),("gravity_z",-9.81,1e-9)):
        if abs(model[key]-want)>tolerance:
            raise AssertionError("实际SDK模型参数不符："+key)
    set_pause(False)
    results=[]
    trials=[(.15,.08,0.),(-.15,-.08,0.),(.1,-.06,2.)]
    for index,(x,q,external) in enumerate(trials,1):
        episode="verify-"+uuid.uuid4().hex
        path=directory()/"episode.request"
        temporary=path.with_suffix(".tmp")
        temporary.write_text("%s %.12g 0 %.12g 0 10 %.12g\n"%(episode,x,q,external),encoding="ascii")
        os.replace(str(temporary),str(path))
        print("真实Gazebo试验%d/3：初值x=%+.2fm,theta=%+.2frad；4–4.25s扰动力%.1fN"%(index,x,q,external),flush=True)
        while True:
            if time.monotonic()>=deadline:
                raise RuntimeError("真实控制试验达到总时限；保留control.csv/status.json供诊断")
            current=status()
            if current["episode"]==episode:
                if current["error"]:
                    raise AssertionError("实际学生Python/物理控制失败："+current["error"])
                if current["episode_time_s"]>=10:
                    break
            time.sleep(.15)
        rows=[row for row in records(episode) if -TIME_EPS<=float(row["episode_time_s"])<=10+TIME_EPS]
        result=evaluate(rows,expected_mode,sha)
        first=result["first_actual_state"]
        if abs(first["x_m"]-x)>.001 or abs(first["q1_rad"]-q)>.001:
            raise AssertionError("实际回合未从要求的非零初值开始")
        # The native force window is half-open [4, 4.25). At its floating-point
        # boundary ticks either adjacent value is valid; strict interior/exterior
        # samples must still match the actual requested force, without rewriting data.
        disturbance_rows=[row for row in rows if 4+TIME_EPS<float(row["episode_time_s"])<4.25-TIME_EPS]
        if not disturbance_rows or any(abs(float(row["disturbance_N"])-external)>1e-9 for row in disturbance_rows):
            raise AssertionError("原生记录缺少要求的实际扰动力窗口")
        for row in rows:
            elapsed=float(row["episode_time_s"]);applied=float(row["disturbance_N"])
            if abs(elapsed-4)<=TIME_EPS or abs(elapsed-4.25)<=TIME_EPS:
                if min(abs(applied),abs(applied-external))>1e-9:
                    raise AssertionError("扰动端点记录不是相邻窗口的有效力值")
            elif (elapsed<4-TIME_EPS or elapsed>4.25+TIME_EPS) and abs(applied)>1e-9:
                raise AssertionError("原生记录在扰动窗口之外施加了外力")
        result.update({"episode":episode,"initial_x_m":x,"initial_theta_rad":q,"disturbance_N":external})
        results.append(result)
    if digest(source)!=sha:
        raise AssertionError("验收期间源文件发生变化，结果无效；保存并重载后重试")
    report={"passed":expected_mode=="student","demo_validated":expected_mode=="reference_demo","mode":expected_mode,
            "unit_checks":units,"trials":results,"actual_model":model,"telemetry_csv":str(directory()/"control.csv"),"status_json":str(directory()/"status.json"),
            "initialization":"actual joint position/velocity set only once per trial, then actual force control only"}
    return report


def _preserve_pause(function,output):
    original=read_pause()
    result=None
    try:
        result=function()
    finally:
        cleanup=set_pause(original)
        if not cleanup["confirmed"]:
            raise RuntimeError("Could not confirm original pause on cleanup")
    result["original_pause"]=original
    result["pause_cleanup"]=cleanup
    (directory()/output).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    return result


def observe(seconds=3):
    return _preserve_pause(lambda:_observe(seconds),"raw-observation.json")


def experiment(expected_mode="student",timeout=120):
    result=_preserve_pause(lambda:_experiment(expected_mode,timeout),
                           "student-verification.json" if expected_mode=="student" else "demo-validation.json")
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return result
