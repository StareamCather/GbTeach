"""Confirm actual pause; send at most one change command, including cleanup."""
import re
import subprocess
import time


def read_pause(timeout=3):
    result=subprocess.run(["gz","topic","-e","/gazebo/default/world_stats","-d","1"],
                          stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=timeout)
    if result.returncode:
        raise RuntimeError("world_stats read failed: "+result.stderr[-500:])
    values=re.findall(r"^\s*paused:\s*(true|false)\s*$",result.stdout,re.MULTILINE)
    if not values:
        raise RuntimeError("world_stats did not contain actual paused state")
    return values[-1]=="true"


def set_pause(target):
    before=read_pause()
    report={"target_paused":bool(target),"before_paused":before,"command_sent":False,"confirmed":False}
    if before==target:
        report["confirmed"]=True
        return report
    report["command_sent"]=True
    try:
        result=subprocess.run(["gz","world","-w","default","-p","1" if target else "0"],
                              stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=3)
        if result.returncode:
            report["command_warning"]="exit %d: %s"%(result.returncode,result.stderr[-500:])
    except subprocess.TimeoutExpired:
        report["command_warning"]="pause command timed out; no resend"
    deadline=time.monotonic()+4
    error="target not observed"
    while time.monotonic()<deadline:
        try:
            after=read_pause(timeout=min(3,max(.1,deadline-time.monotonic())))
            report["after_paused"]=after
            if after==target:
                report["confirmed"]=True
                return report
        except (RuntimeError,subprocess.TimeoutExpired) as failure:
            error=str(failure)
    raise RuntimeError("Pause readback did not confirm target %s: %s; %s"%(target,error,report))
