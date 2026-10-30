#!/usr/bin/env python3
"""Select saved editor packages without editing or generating student models."""
import argparse
import datetime
import os
import sys
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from geometry import package, pose, room_geometry, table_geometry


def select(workspace, student):
    world_path = workspace / "world.sdf"
    tree = ET.parse(str(world_path))
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("当前练习缺少world")
    selected = []
    for instance, suffix, analyze in (("table_example", "table", table_geometry),
                                       ("room_example", "room", room_geometry)):
        matches = [node for node in world.findall("include") if node.findtext("name") == instance]
        if len(matches) != 1:
            raise ValueError("world需要唯一include实例：" + instance)
        node = matches[0]
        name = ("student_" if student else "reference_") + suffix
        data = package(workspace / "models" / name)
        details = analyze(data)
        raw = pose(node, instance)
        # Native editors recenter saved link/shape coordinates. Translate the
        # entire collision assembly until its lowest face is on world ground.
        raw[2] = -data["min_z"]
        selected.append((node, name, raw, details))
    # Both complete packages have passed before any mutation, including backup.
    backups = workspace / "world-backups"
    backups.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = backups / ("world-" + stamp + "-" + uuid.uuid4().hex[:8] + ".sdf")
    backup.write_bytes(world_path.read_bytes())
    temporary = world_path.with_name(".world-" + uuid.uuid4().hex + ".sdf")
    try:
        for node, name, raw, _ in selected:
            node.find("uri").text = "model://" + name
            entry = node.find("pose")
            if entry is None:
                entry = ET.SubElement(node, "pose")
            entry.text = " ".join("%.10g" % value for value in raw)
        tree.write(str(temporary), encoding="utf-8", xml_declaration=True)
        os.replace(str(temporary), str(world_path))
    finally:
        if temporary.exists():
            temporary.unlink()
    print("世界备份：" + str(backup))
    for node, name, raw, _ in selected:
        print("%s → model://%s；include.z=%.6gm（按collision最低面补偿）" %
              (node.findtext("name"), name, raw[2]))
    print("已更新当前world.sdf；请在应用中重载本课项目，使Gazebo实际读取保存模型。")
    print("学生模型文件未修改；自动几何检查不代表已完成GUI人工操作清单。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("show-paths", "use-exports", "use-reference"))
    arguments = parser.parse_args()
    workspace = Path.cwd().resolve()
    if arguments.action == "show-paths":
        print("当前练习：" + str(workspace))
        for name in ("student_table", "student_room"):
            print("GUI Save As 的最终模型目录：" + str(workspace / "models" / name))
        print("请在相应编辑器中实际创建并保存；本工具不会预填或覆盖学生模型。")
    else:
        select(workspace, arguments.action == "use-exports")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, ET.ParseError) as error:
        print("FAIL：" + str(error), file=sys.stderr)
        sys.exit(1)
