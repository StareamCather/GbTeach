"""Small offline reader for this lesson's static, horizontal box assemblies.

This deliberately is not a CAD/SDF frame-graph implementation. Model Editor
boxes and Building Editor segmented box walls (SDF 1.4--1.7) are supported.
All lengths and checks come from the current package, never reference constants.
"""
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

VERSIONS = ("1.4", "1.5", "1.6", "1.7")
# Native Classic Building exports serialize quarter turns as 1.5708 and half
# turns as 3.14159. Two independently rounded wall angles can differ from
# exact orthogonality by up to 1e-5 rad. This tolerance classifies wall axes;
# box coordinates, dimensions and doorway collision-volume checks stay exact.
AXIS_ANGLE_TOLERANCE = 1e-5


def numbers(text, count, label):
    try:
        values = [float(x) for x in (text or "").split()]
    except ValueError as error:
        raise ValueError(label + "必须是数值") from error
    if len(values) != count or not all(math.isfinite(x) for x in values):
        raise ValueError("%s需要%d个有限数值" % (label, count))
    return values


def pose(node, label):
    entry = node.find("pose")
    if entry is not None and (entry.get("relative_to") or entry.get("frame")):
        raise ValueError(label + "本课使用默认父坐标系，不支持额外frame/relative_to")
    values = numbers(node.findtext("pose", "0 0 0 0 0 0"), 6, label + " pose")
    if abs(values[3]) + abs(values[4]) > 1e-7:
        raise ValueError(label + "本课只支持水平box组合（roll/pitch=0，可改变yaw）")
    return values


def combine(parent, child):
    c, s = math.cos(parent[5]), math.sin(parent[5])
    return [parent[0] + c*child[0] - s*child[1],
            parent[1] + s*child[0] + c*child[1], parent[2] + child[2],
            0.0, 0.0, parent[5] + child[5]]


def inside(path, parent):
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def package(folder):
    folder = Path(folder).resolve()
    config = folder / "model.config"
    if not config.is_file():
        raise ValueError("模型包缺少model.config：" + str(config))
    metadata = ET.parse(str(config)).getroot()
    if metadata.tag != "model" or not metadata.findtext("name", "").strip():
        raise ValueError("model.config需要model根和name")
    entries = metadata.findall("sdf")
    if not entries or any(entry.get("version") not in VERSIONS for entry in entries):
        raise ValueError("本课支持model.config中SDF1.4至1.7的入口")
    entry = max(entries, key=lambda node: tuple(int(x) for x in node.get("version").split(".")))
    filename = (entry.text or "").strip()
    if not filename:
        raise ValueError("model.config的sdf元素缺少入口文件名")
    target = (folder / filename).resolve()
    if not inside(target, folder):
        raise ValueError("模型入口文件应位于本模型包目录内")
    if not target.is_file():
        raise ValueError("模型入口文件不存在：%s；检查model.config的sdf文本" % target)
    root = ET.parse(str(target)).getroot()
    if root.tag != "sdf" or root.get("version") != entry.get("version"):
        raise ValueError("model.config入口sdf版本与模型根版本不一致")
    models = root.findall("model")
    if len(models) != 1 or root.find("world") is not None:
        raise ValueError("本课模型包必须恰好描述一个model")
    model = models[0]
    if model.findtext("static", "false").strip().lower() not in ("true", "1"):
        raise ValueError("本课桌子和房间必须static=true")
    if model.findall("model") or model.findall("include") or model.findall("joint"):
        raise ValueError("本课仅验收静态box组合，不支持嵌套模型或关节")
    # An include pose replaces the saved model placement. Link and shape poses
    # remain model-relative; exported placement does not become a second offset.
    pose(model, "model")
    links, shapes = {}, {"collision": [], "visual": []}
    for link in model.findall("link"):
        name = link.get("name", "")
        if not name or name in links:
            raise ValueError("link名称缺失或重复")
        links[name] = pose(link, "link " + name)
        for kind in shapes:
            names = set()
            for shape in link.findall(kind):
                shape_name = shape.get("name", "")
                if not shape_name or shape_name in names:
                    raise ValueError("%s/%s名称缺失或重复" % (name, kind))
                names.add(shape_name)
                size = numbers(shape.findtext("geometry/box/size"), 3, name + "/" + kind + " box")
                if min(size) <= 0:
                    raise ValueError("box尺寸必须全部大于0")
                local = pose(shape, name + "/" + kind)
                shapes[kind].append({"link": name, "name": shape_name, "size": size,
                                     "local": local, "pose": combine(links[name], local)})
    if not links or min(len(shapes[kind]) for kind in shapes) < 2:
        raise ValueError("桌子和房间均需要多个实际box collision与visual")
    # Exporters may choose different shape names, so pair by geometry and pose.
    remaining = list(shapes["visual"])
    for collision in shapes["collision"]:
        match = next((visual for visual in remaining if same_box(collision, visual)), None)
        if match is None:
            raise ValueError("collision与visual的组合几何/位姿不一致：" + collision["link"])
        remaining.remove(match)
    if remaining:
        raise ValueError("存在没有对应collision的额外visual")
    return {"folder": folder, "entry": target, "model": model, "links": links,
            "collision": shapes["collision"], "visual": shapes["visual"],
            "min_z": min(box["pose"][2] - box["size"][2]/2 for box in shapes["collision"])}


def same_box(left, right):
    return (all(abs(a-b) < 1e-6 for a, b in zip(left["size"], right["size"])) and
            all(abs(a-b) < 1e-6 for a, b in zip(left["pose"][:3], right["pose"][:3])) and
            abs(math.sin((left["pose"][5]-right["pose"][5])/2)) < 1e-6)


def resource(uri, workspace, check_search=True):
    if not uri or not uri.startswith("model://"):
        raise ValueError("本课include需要model://本地模型包URI")
    name = uri[len("model://"):]
    if not name or name in (".", "..") or "/" in name or "\\" in name:
        raise ValueError("本课URI应指向单个模型包目录")
    expected = (Path(workspace) / "models" / name).resolve()
    if check_search:
        search = [Path(value).expanduser() for value in os.environ.get("GAZEBO_MODEL_PATH", "").split(os.pathsep) if value]
        selected = next((base/name for base in search if (base/name).is_dir()), None)
        if selected is None or selected.resolve() != expected:
            raise ValueError("URI未命中当前练习模型包%s；检查GAZEBO_MODEL_PATH" % expected)
    result = package(expected)
    result["uri"] = uri
    return result


def contains_xy(box, x, y, margin=0):
    dx, dy = x-box["pose"][0], y-box["pose"][1]
    c, s = math.cos(box["pose"][5]), math.sin(box["pose"][5])
    return (abs(c*dx+s*dy) <= box["size"][0]/2-margin+1e-7 and
            abs(-s*dx+c*dy) <= box["size"][1]/2-margin+1e-7)


def table_geometry(data):
    boxes = data["collision"]
    candidates = [box for box in boxes if box["size"][2] <= .35 and min(box["size"][:2]) >= .3]
    if not candidates:
        raise ValueError("桌子没有水平薄板桌面（厚度≤.35m、两侧≥.3m）")
    top = max(candidates, key=lambda box: box["size"][0]*box["size"][1])
    underside = top["pose"][2] - top["size"][2]/2
    legs = [box for box in boxes if box is not top and
            abs(box["pose"][2]+box["size"][2]/2-underside) < .025 and
            abs(box["pose"][2]-box["size"][2]/2-data["min_z"]) < .025 and
            contains_xy(top, box["pose"][0], box["pose"][1])]
    if len(legs) < 4:
        raise ValueError("桌面下至少需要4个接触桌底、底端对齐地面的box桌腿")
    return {"top": top, "legs": legs, "surface_z": top["pose"][2]+top["size"][2]/2,
            "drop_xy": top["pose"][:2]}


def room_geometry(data):
    """Find an actual open passage in a perimeter wall's 2-D occupancy grid.

    Each wall is expressed in its own horizontal tangent/normal coordinates.
    A door is an empty interval from floor upward, bounded by collision at
    either side and by a collision lintel above. Naming/box segmentation does
    not define the result; independent box face coordinates do.
    """
    boxes = data["collision"]
    floor = data["min_z"]
    # A native Building export can contain a ground-floor slab. The walking
    # surface is its top, whereas include grounding still uses its bottom.
    slabs = [box for box in boxes if box["size"][2] <= .3 and min(box["size"][:2]) >= .5 and
             abs(box["pose"][2]-box["size"][2]/2-floor) < .025]
    if slabs:
        floor = max(box["pose"][2]+box["size"][2]/2 for box in slabs)
    directions = []
    for box in boxes:
        sx, sy, _ = box["size"]
        if min(sx, sy) > .3 or max(sx, sy) < .3:
            continue
        angle = box["pose"][5] + (math.pi/2 if sy > sx else 0)
        angle = angle % math.pi
        if not any(abs(math.sin(angle-old)) < AXIS_ANGLE_TOLERANCE for old in directions):
            directions.append(angle)
    if len(directions) != 2 or abs(math.cos(directions[0]-directions[1])) > AXIS_ANGLE_TOLERANCE:
        raise ValueError("本课房间需要两组互相垂直的矩形墙体")
    found = []
    for angle in directions:
        c, s = math.cos(angle), math.sin(angle)
        projected = []
        for box in boxes:
            turn = box["pose"][5]-angle
            if abs(math.sin(turn*2)) > 2*AXIS_ANGLE_TOLERANCE:
                raise ValueError("本课房间墙体需要互相平行或垂直的水平box")
            sx, sy, sz = box["size"]
            tangent = abs(math.cos(turn))*sx + abs(math.sin(turn))*sy
            normal = abs(math.sin(turn))*sx + abs(math.cos(turn))*sy
            x, y, z = box["pose"][:3]
            u, v = c*x+s*y, -s*x+c*y
            projected.append((u-tangent/2, u+tangent/2, v-normal/2, v+normal/2, z-sz/2, z+sz/2))
        normal_min = min(item[2] for item in projected)
        normal_max = max(item[3] for item in projected)
        # Sample just inside each outside face, through the full wall thickness.
        perimeter_sides = 0
        for side, boundary in ((-1, normal_min), (1, normal_max)):
            near = [item for item in projected if (abs(item[2]-boundary) if side < 0 else abs(item[3]-boundary)) < .02]
            if not near:
                continue
            wall_thickness = min(item[3]-item[2] for item in near)
            if wall_thickness > .3:
                continue
            perimeter_sides += 1
            line = boundary - side*wall_thickness/2
            slices = [item for item in projected if item[2]+1e-7 < line < item[3]-1e-7]
            xs = sorted(set(round(value, 8) for item in slices for value in item[:2]))
            if len(xs) < 4:
                continue
            openings = []
            for left, right in zip(xs, xs[1:]):
                mid = (left+right)/2
                cover = [item for item in slices if item[0]-1e-7 <= mid <= item[1]+1e-7]
                low = min((item[4] for item in cover if item[5] > floor+.05), default=math.inf)
                if not math.isfinite(low) or low-floor < .8:
                    continue
                # Adjacent lintel blocks must not artificially split one door.
                if openings and abs(openings[-1][1]-left) < 1e-6 and abs(openings[-1][2]-low) < .025:
                    openings[-1] = (openings[-1][0], right, min(openings[-1][2], low))
                else:
                    openings.append((left, right, low))
            for left, right, low in openings:
                if right-left < .35:
                    continue
                # Neighbouring solid jambs must meet the floor at both sides.
                jambs = [any(item[0]-1e-5 <= point <= item[1]+1e-5 and item[4] <= floor+.025 and item[5] >= low-.025
                             for item in slices) for point in (left-.01, right+.01)]
                outer, inner = sorted((boundary, boundary-side*wall_thickness))
                blocked = any(item[1] > left+1e-5 and item[0] < right-1e-5 and
                              item[3] > outer+1e-5 and item[2] < inner-1e-5 and
                              item[5] > floor+1e-5 and item[4] < low-1e-5 for item in projected)
                if all(jambs) and not blocked:
                    found.append({"width": right-left, "height": low-floor, "wall_thickness": wall_thickness,
                                  "tangent_yaw": angle, "normal_coordinate": line,
                                  "tangent_interval": [left, right], "floor_z": floor})
        if perimeter_sides != 2:
            raise ValueError("房间缺少矩形周界的一侧墙体")
    if not found:
        raise ValueError("房间没有检测到两侧实墙及上方门楣围成的落地门洞（宽≥.35m、高≥.8m）")
    return max(found, key=lambda item: item["width"]*item["height"])
