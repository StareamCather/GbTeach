"""Executed by the native bridge; loads the exact selected Python source bytes."""
import hashlib
import json
import os
from pathlib import Path
import sys
import types
import uuid
import numpy  # Deliberately check the actual embedded interpreter too.


def load_controller():
    workspace = Path(os.environ["GAZEBOTEACH_WORKSPACE"]).resolve()
    official = Path(os.environ["GAZEBOTEACH_LESSON_SOURCE"]).resolve()
    mode = json.loads((workspace/"runtime.json").read_text(encoding="utf-8"))["mode"]
    if mode not in ("student", "reference_demo"):
        raise ValueError("runtime.json mode must be student or reference_demo")
    source = workspace/"learner.py" if mode == "student" else official/"reference_controller.py"
    raw = source.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    name = "_gazeboteach_controller_"+digest+"_"+uuid.uuid4().hex
    module = types.ModuleType(name)
    module.__file__ = str(source)
    sys.modules[name] = module
    try:
        exec(compile(raw, str(source), "exec"), module.__dict__)
        callback = module.control
        if not callable(callback):
            raise TypeError("control must be callable")
    except BaseException:
        sys.modules.pop(name, None)
        raise
    # The C++ owner retains module and callback and removes this unique entry on teardown.
    return callback, mode, digest, str(source), module, numpy.__version__, hashlib.sha256((workspace/"world.sdf").read_bytes()).hexdigest()
