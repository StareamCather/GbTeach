"""Behavior probes, independent of TODO comments and of native safety clipping."""
import hashlib
import json
from pathlib import Path
import sys
import types
import numpy as np

K = np.array([-14.14213562, -15.95141643, -92.53361631, -22.61484036])


def check(path):
    raw = Path(path).read_bytes()
    module = types.ModuleType("_student_behavior_probe")
    module.__file__ = str(Path(path).resolve())
    exec(compile(raw, module.__file__, "exec"), module.__dict__)
    def close(actual, expected, label):
        value = np.asarray(actual, dtype=float)
        if value.shape != np.asarray(expected).shape or not np.all(np.isfinite(value)) or not np.allclose(value, expected, rtol=1e-7, atol=1e-8):
            raise AssertionError(label+": expected "+str(expected)+", observed "+str(actual))
    observations = [({"x":0.,"xdot":0.,"q1":0.,"q1dot":0.},0.,[0.,0.,0.,0.]),
                    ({"x":.14,"xdot":-.2,"q1":2*np.pi+.12,"q1dot":.31},.04,[.1,-.2,.12,.31]),
                    ({"x":-.1,"xdot":.2,"q1":-2*np.pi-.15,"q1dot":-.3},.1,[-.2,.2,-.15,-.3])]
    for observation,target,expected in observations:
        close(module.state_error(observation,target), expected, "state_error/order/wrap")
    vectors = [np.zeros(4),np.array([.03,-.11,.02,-.17])]+[row*.01 for row in np.eye(4)]+[-row*.01 for row in np.eye(4)]
    for vector in vectors:
        close(module.feedback_force(vector),float(-K@vector),"feedback_force/sign/order")
    for cap in (15.,2.5):
        for value in (-100.,-15.,-1.,0.,1.,15.,100.):
            close(module.clip_force(value,cap),float(np.clip(value,-cap,cap)),"clip_force/bidirectional")
    for observation,target,expected in observations:
        close(module.control(observation,target),float(np.clip(-K@np.asarray(expected),-15,15)),"control/pipeline")
    # The mathematically valid force -1 must not be confused with a Python C-API error sentinel.
    close(module.clip_force(-1.),-1.,"valid negative one")
    return {"passed":True,"source_sha256":hashlib.sha256(raw).hexdigest(),"state_probes":3,"feedback_probes":len(vectors),"clip_probes":14,"pipeline_probes":3}


if __name__ == "__main__":
    try:
        print(json.dumps(check(sys.argv[1]), ensure_ascii=False))
    except BaseException as error:
        print("TODO/behavior check failed: "+type(error).__name__+": "+str(error), file=sys.stderr)
        sys.exit(1)
