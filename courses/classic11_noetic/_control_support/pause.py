"""Confirm actual pause with one process-owned persistent Classic SDK client."""
import atexit
import hashlib
import json
import os
from pathlib import Path
import sys
import threading


_controls={}
_unavailable={}
_control_lock=threading.RLock()


def _write_metadata(control,directory):
    path=directory/'ownership.json'
    path.write_text(json.dumps(control.metadata(),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def close_controls():
    """Send EOF and join each owned SDK helper; retain its actual cleanup evidence."""
    with _control_lock:
        for key,(control,directory) in _controls.items():
            if key[0]==os.getpid():control.close();_write_metadata(control,directory)
        # A closed/poisoned controller stays cached. Unknown outcomes cannot cause
        # an automatic replacement process or command replay in this interpreter.


atexit.register(close_controls)


def _control(world='default',required=True):
    # Course commands run from a personal workspace with a course-only
    # PYTHONPATH. SOURCE_ROOT already identifies the installed app independently
    # of cwd; the __file__ layout also supports direct source-tree execution.
    root=Path(os.environ['GAZEBOTEACH_SOURCE_ROOT']).resolve() if os.environ.get('GAZEBOTEACH_SOURCE_ROOT') else Path(__file__).resolve().parents[3]
    if not (root/'gazeboteach/native_control.py').is_file() or not (root/'resources/native_control.cc').is_file():
        raise RuntimeError('Cannot locate the installed native control module/resources from GAZEBOTEACH_SOURCE_ROOT or the course support layout')
    if str(root) not in sys.path:sys.path.insert(0,str(root))
    from gazeboteach.native_control import try_create
    with _control_lock:
        key=(os.getpid(),os.environ.get('GAZEBO_MASTER_URI',''),world)
        if key in _unavailable:
            if required:raise RuntimeError('Persistent Classic SDK backend unavailable: '+json.dumps(_unavailable[key],ensure_ascii=False))
            return None
        if key not in _controls:
            suffix=hashlib.sha256((key[1]+'\0'+world).encode()).hexdigest()[:12]
            directory=Path.cwd()/'build'/('pause-control-'+str(os.getpid())+'-'+suffix)
            directory.mkdir(parents=True,exist_ok=True)
            control,diagnosis=try_create(root,dict(os.environ),world,directory)
            if control is None:
                _unavailable[key]=diagnosis
                if required:raise RuntimeError('Controller courses require the exact persistent Classic SDK backend: '+json.dumps(diagnosis,ensure_ascii=False))
                print('Native model query: explicit legacy CLI fallback: '+json.dumps(diagnosis,ensure_ascii=False),file=sys.stderr,flush=True)
                return None
            _controls[key]=(control,directory);_write_metadata(control,directory)
        return _controls[key][0]


def get_control(world='default',required=False):
    """Optional basic-course reads share the controller-course SDK lifetime."""
    return _control(world,required)


def control_metadata():
    """Read process identity/build evidence without replacing a failed helper."""
    return _control().metadata()


def read_pause(timeout=3):
    return _control().status(timeout=timeout)['stats']['paused']


def set_pause(target):
    control=_control();before=control.status()['stats']['paused']
    report={"target_paused":bool(target),"before_paused":before,"command_sent":False,"confirmed":False}
    if before==target:
        report["confirmed"]=True
        return report
    report["command_sent"]=True
    # Exactly one mutation. A failure poisons this persistent controller; neither
    # a legacy CLI fallback nor a new helper can silently retry its outcome.
    result=control.pause(bool(target))
    report.update(after_paused=result['stats']['paused'],confirmed=True)
    return report
