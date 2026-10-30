"""Independent six-state/relative-angle/feedback/clipping behavior probes."""
import hashlib
import json
from pathlib import Path
import sys
import types
import numpy as np

K = np.array([7.07106781, 11.24116542, -161.66415569, 3.24033851, 233.38706339, 31.37395393])


def check(path):
    raw=Path(path).read_bytes()
    module=types.ModuleType("_double_student_behavior_probe")
    module.__file__=str(Path(path).resolve())
    exec(compile(raw,module.__file__,"exec"),module.__dict__)
    def close(actual,expected,label):
        value=np.asarray(actual,dtype=float)
        if value.shape!=np.asarray(expected).shape or not np.all(np.isfinite(value)) or not np.allclose(value,expected,rtol=1e-7,atol=1e-8):
            raise AssertionError(label+": expected "+str(expected)+", observed "+str(actual))
    observations=[({"x":0.,"xdot":0.,"q1":0.,"q1dot":0.,"q2":0.,"q2dot":0.},0.,[0.,0.,0.,0.,0.,0.]),
                  ({"x":.1,"xdot":-.05,"q1":.02,"q1dot":.1,"q2":-.03,"q2dot":-.04},.04,[.06,-.05,.02,.1,-.01,.06]),
                  ({"x":-.1,"xdot":.2,"q1":2*np.pi+.12,"q1dot":-.3,"q2":-4*np.pi-.2,"q2dot":.13},.1,[-.2,.2,.12,-.3,-.08,-.17]),
                  ({"x":.03,"xdot":-.1,"q1":-.03,"q1dot":.07,"q2":.03,"q2dot":-.02},0.,[.03,-.1,-.03,.07,0.,.05])]
    for observation,target,expected in observations:
        close(module.state_error(observation,target),expected,"state_error/relative_to_absolute/order/wrap")
    vectors=[np.zeros(6),np.array([.03,-.11,.02,-.17,-.04,.09])]+[row*.01 for row in np.eye(6)]+[-row*.01 for row in np.eye(6)]
    for vector in vectors:
        close(module.feedback_force(vector),float(-K@vector),"feedback_force/sign/order")
    for cap in (20.,2.5):
        for value in (-100.,-20.,-1.,0.,1.,20.,100.):
            close(module.clip_force(value,cap),float(np.clip(value,-cap,cap)),"clip_force/bidirectional")
    for observation,target,expected in observations:
        close(module.control(observation,target),float(np.clip(-K@np.asarray(expected),-20,20)),"control/pipeline")
    close(module.clip_force(-1.),-1.,"valid negative one")
    return {"passed":True,"source_sha256":hashlib.sha256(raw).hexdigest(),"state_probes":4,"feedback_probes":len(vectors),"clip_probes":14,"pipeline_probes":4}


if __name__=="__main__":
    try:
        print(json.dumps(check(sys.argv[1]),ensure_ascii=False))
    except BaseException as error:
        print("TODO/behavior check failed: "+type(error).__name__+": "+str(error),file=sys.stderr)
        sys.exit(1)
