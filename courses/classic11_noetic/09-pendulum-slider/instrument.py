"""Passive joint measurements from native Position messages and actual clocks."""
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from kinematics import configuration, predictions
import transport as gz


def binary_path():
    path = Path.cwd()/"build/rig_sampler.json"
    if not path.is_file():
        raise RuntimeError("缺少原生关节测量器；请重载，或先运行python3 prepare.py")
    metadata = json.loads(path.read_text(encoding="utf-8"))
    expected = hashlib.sha256(Path(__file__).with_name("rig_sampler.cc").read_bytes()).hexdigest()
    binary = Path(metadata["binary"])
    if metadata.get("source_sha256") != expected or not binary.is_file():
        raise RuntimeError("原生关节测量器版本或缓存不符，请重载构建")
    return binary


def invoke(binary, config, mode):
    arguments = [str(binary), config["world"], mode]
    timeout = 9 if mode == "inspect" else 62
    if mode == "sample":
        arguments += [str(config["sample_steps"]), str(config["sample_count"]), "58"]
    timeout = min(timeout, gz.DEADLINE-time.monotonic())
    if timeout < .1:
        raise RuntimeError("关节仪器达到总deadline")
    timed_out = False
    try:
        process = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        stdout, stderr, returncode = process.stdout, process.stderr, process.returncode
    except subprocess.TimeoutExpired as error:
        timed_out = True
        stdout, stderr, returncode = error.stdout or "", error.stderr or "", -1
        if isinstance(stdout, bytes): stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes): stderr = stderr.decode("utf-8", "replace")
    records = []
    for line in stdout.splitlines():
        if line.startswith("{"):
            try: records.append(json.loads(line))
            except ValueError: break  # Preserve complete preceding raw records.
    return {"records": records, "returncode": returncode, "timed_out": timed_out, "stderr": stderr[-1500:]}


def named(entries, name):
    matches = [entry for entry in entries if str(entry.get("name", "")).split("::")[-1] == name]
    if len(matches) != 1:
        raise AssertionError("真实消息中需要唯一实体"+name)
    return matches[0]


def raw_pose(node):
    p, q = node["position"], node["orientation"]
    result = [float(p[key]) for key in ("x", "y", "z")]+[float(q[key]) for key in ("x", "y", "z", "w")]
    if not all(math.isfinite(value) for value in result) or abs(sum(v*v for v in result[3:])-1) > .002:
        raise AssertionError("真实pose非有限或非单位四元数")
    return result


def quaternion(raw):
    return gz.quaternion(raw)


def compare_pose(node, expected, label):
    actual = raw_pose(node)
    distance = math.sqrt(sum((a-b)**2 for a,b in zip(actual[:3],expected[:3])))
    angle = 2*math.acos(max(-1,min(1,abs(sum(a*b for a,b in zip(actual[3:],quaternion(expected)))))))
    if distance > .002 or angle > .002:
        raise AssertionError(label+"真实pose与当前SDF不符，请重载")


def check_initial(config, initial):
    physics = initial["physics"]
    if physics.get("type") != "ODE" or physics.get("solver_type") != "quick":
        raise AssertionError("实际physics后端/solver不符")
    for key in ("max_step_size", "iters"):
        gz.near(float(physics[key]),config["engine"][key],1e-9,"实际physics "+key)
    for axis,value in zip(("x","y","z"),config["engine"]["gravity"]):
        gz.near(float(physics["gravity"][axis]),value,1e-8,"实际重力")
    for model_name, field in (("pendulum_rig","pendulum_model"),("slider_rig","slider_model")):
        actual = initial[field]
        expected = config["models"][model_name]
        if actual.get("is_static") is not False:
            raise AssertionError(model_name+"真实模型不能static")
        compare_pose(actual["pose"],expected["pose"],model_name)
        if {str(link["name"]).split("::")[-1] for link in actual["link"]} != {expected["parent_link"],expected["child_link"]}:
            raise AssertionError(model_name+"真实link拓扑不符")
        for joint_name,saved in config["joints"][model_name].items():
            joint = named(actual["joint"],joint_name)
            if joint.get("type") != saved["type"].upper() or str(joint.get("parent","")).split("::")[-1] != saved["parent"] or str(joint.get("child","")).split("::")[-1] != saved["child"]:
                raise AssertionError(joint_name+"实际类型/parent/child拓扑不符")
            anchor = [0]*6
            if joint_name == "swing_joint":
                anchor = [0,0,config["pendulum"]["length_m"],0,-config["pendulum"]["initial_angle_rad"],0]
            compare_pose(joint["pose"],anchor,joint_name+"运行时child-relative joint.pose")
            if saved["type"] != "fixed":
                axis = joint["axis1"]
                for key,value in zip(("x","y","z"),saved["axis"]):
                    gz.near(float(axis["xyz"][key]),value,1e-8,joint_name+"实际axis")
                if axis.get("xyz_expressed_in","") != saved["expressed_in"]:
                    raise AssertionError(joint_name+"实际axis参考系不符")
                for key in ("limit_effort","limit_velocity"):
                    gz.near(float(axis[key]),-1,1e-8,joint_name+"实际"+key)
                for source,key in (("lower","limit_lower"),("upper","limit_upper"),("damping","damping"),("friction","friction")):
                    gz.near(float(axis[key]),saved[source],1e-8,joint_name+"实际"+key)
                if len(joint.get("angle",[])) != 1:
                    raise AssertionError(joint_name+"缺少实际Position角度/位移")
                gz.near(float(joint["angle"][0]),0,1e-5,joint_name+"reset后的q零位")
        for role in ("parent","child"):
            link=named(actual["link"],expected[role+"_link"])
            if link.get("kinematic") is not False or link.get("gravity") is not True:
                raise AssertionError(model_name+" "+role+"真实link需dynamic且gravity=true")
            compare_pose(link["pose"],expected[role]["pose"],model_name+"初始"+role)
            inertial=link["inertial"]
            gz.near(float(inertial["mass"]),expected[role]["mass_kg"],1e-7,model_name+"实际质量")
            compare_pose(inertial["pose"],[0]*6,model_name+"实际COM")
            for key,value in zip(("ixx","iyy","izz","ixy","ixz","iyz"),expected[role]["inertia"]):
                gz.near(float(inertial[key]),value,1e-8,model_name+"实际"+key)
    print("PASS：原生实际physics、fixed/revolute/prismatic拓扑、child-relative锚点、轴参考系/限位/阻尼/摩擦及child质量惯量匹配当前SDF",flush=True)


def world_position(model,link):
    x,y,z,w = model[3:]
    vx,vy,vz = link[:3]
    tx,ty,tz = 2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return [model[0]+vx+w*tx+y*tz-z*ty, model[1]+vy+w*ty+z*tx-x*tz,
            model[2]+vz+w*tz+x*ty-y*tx]


def check_world_orientation(model,link,expected,label):
    ax,ay,az,aw=model[3:];bx,by,bz,bw=link[3:]
    actual=[aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz]
    target=quaternion([0,0,0]+expected)
    angle=2*math.acos(max(-1,min(1,abs(sum(a*b for a,b in zip(actual,target))))))
    if angle>.005:raise AssertionError(label+"实际朝向违反本rig关节约束")


def derive_samples(config,records):
    samples = [dict(item) for item in records if item.get("kind")=="sample"]
    if len(samples)!=config["sample_count"]+1:
        raise AssertionError("原生采样数量不完整")
    baseline = samples[0]["stats"]
    theta0 = config["pendulum"]["initial_angle_rad"]
    for index,item in enumerate(samples):
        for key,value in item.items():
            if key.endswith("_pose"):
                if len(value)!=7 or not all(math.isfinite(v) for v in value) or abs(sum(v*v for v in value[3:])-1)>.002:
                    raise AssertionError(key+"真实pose非有限或四元数非单位")
        stats = item["stats"]
        item["t"] = stats["sim_time_s"]-baseline["sim_time_s"]
        if not stats["paused"] or stats["iterations"]-baseline["iterations"] != index*config["sample_steps"]:
            raise AssertionError("实际关节采样时间/iterations未形成指定的暂停步格")
        gz.near(item["t"],index*config["sample_interval_s"],1e-7,"实际采样时标")
        theta = theta0+item["pendulum_q_rad"]
        frame = world_position(item["pendulum_model_pose"],item["pendulum_frame_pose"])
        bob = world_position(item["pendulum_model_pose"],item["pendulum_bob_pose"])
        geometric = math.atan2(-(bob[0]-frame[0]),-(bob[2]-frame[2]))
        gz.near(theta,geometric,.005,"实际joint q与独立COM几何theta=theta0+q")
        item.update(theta_rad=theta,theta_geometry_rad=geometric)
        check_world_orientation(item["pendulum_model_pose"],item["pendulum_frame_pose"],[0,0,0],"fixed frame")
        check_world_orientation(item["pendulum_model_pose"],item["pendulum_bob_pose"],[0,theta,0],"revolute bob")
        for field in ("slider_guide_pose","slider_carriage_pose"):
            check_world_orientation(item["slider_model_pose"],item[field],[0,0,0],"prismatic "+field)
        length = math.sqrt(sum((a-b)**2 for a,b in zip(frame,bob)))
        gz.near(length,config["pendulum"]["length_m"],.003,"铰点到COM距离")
        guide = world_position(item["slider_model_pose"],item["slider_guide_pose"])
        carriage = world_position(item["slider_model_pose"],item["slider_carriage_pose"])
        gz.near(carriage[2]-guide[2],item["slider_q_m"],.005,"实际prismatic q与child相对位移")
        if math.hypot(carriage[0]-guide[0],carriage[1]-guide[1])>.005:
            raise AssertionError("滑轨出现超出轴向的自由位移")
        for point,expected,label in ((frame,config["models"]["pendulum_rig"]["pose"][:3],"frame"),
                                     (guide,config["models"]["slider_rig"]["pose"][:3],"guide")):
            if math.dist(point,expected)>.003:
                raise AssertionError(label+"未被真实fixed关节保持在世界锚点")
    return samples


def crossings(samples,direction):
    result=[]
    for a,b in zip(samples,samples[1:]):
        x,y=a["theta_rad"],b["theta_rad"]
        crossed=(x>0 and y<=0) if direction<0 else (x<0 and y>=0)
        if crossed:result.append(a["t"]+x/(x-y)*(b["t"]-a["t"]))
    return result


def analyze(config,samples):
    down,up=crossings(samples,-1),crossings(samples,1)
    periods=[b-a for times in (down,up) for a,b in zip(times,times[1:])]
    if len(periods)<2:
        raise AssertionError("真实joint时间序列没有至少两个同方向完整周期，不能用预测周期冒充实测")
    measured=statistics.mean(periods);predicted=config["pendulum"]["corrected_period_s"]
    tolerance=max(.02,predicted*.04)
    gz.near(measured,predicted,tolerance,"实际同向过零周期与物理摆小角/弱阻尼近似")
    angles=[item["theta_rad"] for item in samples]
    if min(angles)>-.07 or max(angles)<.07:
        raise AssertionError("实际摆未在鉛垂两侧形成可辨振幅")
    pendulum={"downward_zero_crossings_s":down,"upward_zero_crossings_s":up,"period_samples_s":periods,
              "measured_period_s":measured,"predicted_period_s":predicted,"period_tolerance_s":tolerance,
              "amplitude_range_rad":[min(angles),max(angles)]}
    early_index=min(range(len(samples)),key=lambda i:abs(samples[i]["t"]-config["slider"]["early_time_s"]))
    gz.near(samples[early_index]["t"],config["slider"]["early_time_s"],1e-7,"滑轨早期样本时标")
    early=samples[early_index]["slider_q_m"];target=config["slider"]["predicted_early_q_m"]
    gz.near(early,target,.015,"实际重力滑轨早期位移")
    lower=config["slider"]["lower_m"];upper=config["slider"]["upper_m"]
    if any(item["slider_q_m"]<lower-.01 or item["slider_q_m"]>upper+.01 for item in samples):
        raise AssertionError("真实prismatic轨迹明显越过安全限位")
    settled=[{"t":item["t"],"q_m":item["slider_q_m"]} for item in samples[-5:]]
    for item in settled:gz.near(item["q_m"],lower,.005,"实际下限位停靠")
    slider={"early_sample_index":early_index,"measured_early_q_m":early,"predicted_early_q_m":target,
            "settled_samples":settled,"last_q_m":samples[-1]["slider_q_m"],"lower_m":lower,"stop_tolerance_m":.005}
    print("PASS：真实joint过零周期%.7gs，物理摆预测%.7gs；滑轨t=.2s q=%.7gm，末端q=%.7gm/lower=%.6gm"%
          (measured,predicted,early,slider["last_q_m"],lower),flush=True)
    return pendulum,slider


def exception_record(error):
    """Keep the original CLI exit/timeout evidence when status can still be read."""
    record={"type":type(error).__name__,"message":str(error)}
    for key in ("cmd","returncode","stdout","stderr","timeout"):
        value=getattr(error,key,None)
        if isinstance(value,bytes):value=value.decode("utf-8","replace")
        if value is not None:record[key]=value
    return record


def restore_pause(world,target):
    """Read first, write at most once, and decide from confirmed actual state."""
    result={"target_paused":target,"command_sent":False,"confirmed":False}
    try:
        result["before"]=gz.stats(world)
        if result["before"]["paused"]==target:
            result.update(confirmed=True,action="already_matched")
            return result
        remaining=gz.DEADLINE-time.monotonic()
        command_timeout=min(2.5,remaining-2.5)  # Reserve one full bounded read-back.
        if command_timeout<=.1:raise RuntimeError("pause清理剩余时间不足以发送并核实状态，未发送变化命令")
        command=["gz","world","-w",world,"-p","1" if target else "0"]
        result.update(command_sent=True,action="change_once")
        try:
            process=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                                   timeout=command_timeout,check=True)
            result["command_result"]={"cmd":command,"returncode":process.returncode,
                                      "stdout":process.stdout,"stderr":process.stderr}
        except Exception as command_error:
            result["command_error"]=exception_record(command_error)
        # This is only a read, including after an abnormal CLI exit. Never
        # resend a state change that the server may already have applied.
        result["after"]=gz.stats(world)
        result["confirmed"]=result["after"]["paused"]==target
        if not result["confirmed"]:
            result["error"]="实际pause未恢复原值"
        elif "command_error" in result:
            result["transport_warning"]="pause命令异常退出，但有界只读查询已确认原pause"
            print("WARNING："+result["transport_warning"]+"；原始错误："+
                  json.dumps(result["command_error"],ensure_ascii=False),flush=True)
    except Exception as read_error:
        result["error"]="无法确认原pause：%s: %s"%(type(read_error).__name__,read_error)
        result["read_error"]=exception_record(read_error)
    return result


def experiment(workspace=None):
    config=configuration(workspace);binary=binary_path()
    gz.DEADLINE=time.monotonic()+73;started=time.monotonic();original=gz.stats(config["world"])
    result={"predictions":predictions(config),"original_pause":original["paused"],
            "state_restored":"pause_only_joint_link_pose_velocity_and_time_results_retained"}
    error=None;result["measurement_passed"]=False
    try:
        measurement_deadline=gz.DEADLINE
        gz.DEADLINE=min(measurement_deadline,time.monotonic()+6)
        try:
            result["initial_pause_confirmation"]=restore_pause(config["world"],True)
        finally:
            gz.DEADLINE=measurement_deadline
        confirmed=result["initial_pause_confirmation"]
        if not confirmed["confirmed"]:
            raise RuntimeError("未确认初始暂停："+confirmed.get("error","实际pause未匹配"))
        drained_before=confirmed.get("after",confirmed["before"])
        gz.run("gz","world","-w",config["world"],"-m","1")
        drained_after=gz.stats(config["world"])
        result["historical_input_drain"]={"requested_steps":1,"before":drained_before,"after":drained_after}
        if not drained_after["paused"] or drained_after["iterations"]!=drained_before["iterations"]+1:
            raise RuntimeError("历史输入排空单步未获真实iterations ACK，拒绝重复发送")
        gz.near(drained_after["sim_time_s"]-drained_before["sim_time_s"],config["dt_s"],1e-7,"排空单步实际sim时间")
        gz.run("gz","world","-w",config["world"],"-r")
        result["reset_baseline"]=gz.stats(config["world"])
        inspected=invoke(binary,config,"inspect");result["native_inspect"]=inspected
        if inspected["returncode"] or not inspected["records"]:raise RuntimeError("原生inspect失败："+inspected["stderr"])
        initial=inspected["records"][0];check_initial(config,initial)
        result["live_physics"]=initial["physics"]
        result["live_models"]={"pendulum_rig":initial["pendulum_model"],"slider_rig":initial["slider_model"]}
        sampled=invoke(binary,config,"sample");result["native_sampling"]=sampled
        result["samples"]=[item for item in sampled["records"] if item.get("kind")=="sample"]
        if sampled["returncode"]:raise RuntimeError("原生有限采样失败："+sampled["stderr"])
        result["samples"]=derive_samples(config,sampled["records"])
        result["pendulum_measurement"],result["slider_measurement"]=analyze(config,result["samples"])
        result["measurement_passed"]=True
    except Exception as caught:
        error=caught;result["error"]="%s: %s"%(type(caught).__name__,caught)
    finally:
        result["measurement_wall_s"]=time.monotonic()-started
        gz.DEADLINE=time.monotonic()+6
        result["cleanup"]=restore_pause(config["world"],original["paused"])
        if result["cleanup"]["confirmed"]:
            print("已确认原pause；保留被动关节试验后的位姿、速度和时钟，不恢复旧完整state。",flush=True)
    result.update(passed=result["measurement_passed"] and result["cleanup"]["confirmed"],
                  instrument_wall_s=time.monotonic()-started)
    if not result["cleanup"]["confirmed"]:
        result["cleanup_error"]=result["cleanup"].get("error","原pause未确认")
        if error is None:
            error=RuntimeError(result["cleanup_error"])
            result["error"]="RuntimeError: "+str(error)
    marker="MEASUREMENTS_JSON " if result["passed"] else "FAILED_MEASUREMENTS_JSON "
    print(marker+json.dumps(result,ensure_ascii=False,sort_keys=True),flush=True)
    if error is not None:raise error
    return result
