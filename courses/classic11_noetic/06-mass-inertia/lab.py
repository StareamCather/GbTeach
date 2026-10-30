#!/usr/bin/env python3
"""Predict current inertia, regenerate its display mesh, or run the instrument."""
import argparse
import datetime
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from instrument import experiment
from mechanics import configuration, predictions, scalar
from ring_mesh import write_ring


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("predict", "experiment", "rebuild-ring"))
    parser.add_argument("--report", help="把预测或实际测量另存JSON")
    args = parser.parse_args()
    workspace = Path.cwd().resolve()
    if args.action == "predict":
        result = predictions(configuration(workspace))
    elif args.action == "experiment":
        result = experiment(workspace)
    else:
        parameters = json.loads((workspace / "parameters.json").read_text(encoding="utf-8"))
        node = ET.parse(str(workspace / "world.sdf")).getroot().find("world/model[@name='flywheel_ring']/link/collision/geometry/cylinder")
        if node is None:
            raise ValueError("world.sdf缺少flywheel_ring圆柱包络")
        outer = scalar(node, "radius", "ring")
        length = scalar(node, "length", "ring")
        inner = float(parameters["ring_inner_radius_m"])
        if not 0 < inner < outer or length <= 0:
            raise ValueError("需要0<环内半径<外半径且厚度>0")
        target = workspace / "models/inertia_ring/meshes/ring.dae"
        backup = None
        if target.is_file():
            stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            backup = target.with_name("ring-backup-" + stamp + ".dae")
            shutil.copyfile(str(target), str(backup))
        write_ring(workspace, outer, inner, length)
        result = {"generated_mesh": str(target), "backup": str(backup) if backup else None,
                  "outer_radius_m": outer, "inner_radius_m": inner, "length_m": length}
        print("网格已按当前几何参数重建；world.sdf惯量未自动修改，请按公式核对并重载项目。")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.report:
        path = Path(args.report).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("已保存：" + str(path))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("FAIL：" + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
