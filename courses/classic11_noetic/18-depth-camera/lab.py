#!/usr/bin/env python3
"""Decode actual depth/cloud bytes; preserve raw data and their native metadata."""
import argparse
import copy
import json
import math
from pathlib import Path
import threading
import time
import numpy as np
from config import edit, prepared

DEPTH_TOPIC = "/lesson18/depth/image_raw"
INFO_TOPIC = "/lesson18/depth/camera_info"
CLOUD_TOPIC = "/lesson18/depth/points"
RGB_TOPIC = "/lesson18/rgb/image_raw"


class Client:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2
        self.rospy = rospy
        rospy.init_node("gazeboteach_depth_lab", anonymous=True, disable_signals=True)
        self.buffers = {key: {} for key in ("depth", "camera_info", "cloud", "rgb")}
        self.lock, self._latest_rgb = threading.Lock(), None
        self.subscriptions = [
            rospy.Subscriber(DEPTH_TOPIC, Image, lambda msg: self.image("depth", msg), queue_size=2, buff_size=8*1024*1024),
            rospy.Subscriber(INFO_TOPIC, CameraInfo, self.info, queue_size=30),
            rospy.Subscriber(CLOUD_TOPIC, PointCloud2, self.cloud, queue_size=1, buff_size=32*1024*1024),
            rospy.Subscriber(RGB_TOPIC, Image, lambda msg: self.image("rgb", msg), queue_size=2, buff_size=8*1024*1024)]

    def store(self, kind, row):
        with self.lock:
            self.buffers[kind][row["stamp_ns"]] = row
            if kind == "rgb":
                self._latest_rgb = row
            maximum = 8 if kind == "cloud" else 32
            while len(self.buffers[kind]) > maximum:
                self.buffers[kind].pop(next(iter(self.buffers[kind])))

    def image(self, kind, message):
        self.store(kind, {"stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
            "width": message.width, "height": message.height, "encoding": message.encoding,
            "is_bigendian": message.is_bigendian, "step": message.step,
            "data_length": len(message.data), "data": bytes(message.data)})

    def info(self, message):
        self.store("camera_info", {"stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
            "width": message.width, "height": message.height, "distortion_model": message.distortion_model,
            **{key: list(getattr(message, key)) for key in ("D", "K", "R", "P")},
            "binning_x": message.binning_x, "binning_y": message.binning_y,
            "roi": {key: getattr(message.roi, key) for key in ("x_offset", "y_offset", "width", "height", "do_rectify")}})

    def cloud(self, message):
        self.store("cloud", {"stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
            "width": message.width, "height": message.height, "point_step": message.point_step,
            "row_step": message.row_step, "is_bigendian": message.is_bigendian, "is_dense": message.is_dense,
            "fields": [{"name": item.name, "offset": item.offset, "datatype": item.datatype, "count": item.count} for item in message.fields],
            "data_length": len(message.data), "data": bytes(message.data)})

    def drain(self):
        with self.lock:
            keys = set(self.buffers["depth"]) & set(self.buffers["camera_info"]) & set(self.buffers["cloud"])
            result = []
            for stamp in sorted(keys):
                bundle = {"stamp_ns": stamp}
                for kind in ("depth", "camera_info", "cloud"):
                    row = dict(self.buffers[kind].pop(stamp))
                    if "data" in row:
                        bundle[kind+"_data"] = row.pop("data")
                    bundle[kind] = row
                if stamp in self.buffers["rgb"]:
                    row = dict(self.buffers["rgb"].pop(stamp))
                    bundle["rgb_data"] = row.pop("data")
                    bundle["rgb"] = row
                result.append(bundle)
        return result

    def latest_rgb(self):
        with self.lock:
            if self._latest_rgb is None:
                return None
            row = dict(self._latest_rgb)
            return {"rgb_data": row.pop("data"), "rgb": row}

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()

    def call(self, name, service_type, *args, timeout=4):
        self.rospy.wait_for_service(name, timeout=timeout)
        done, result = threading.Event(), []
        def worker():
            try:
                proxy = self.rospy.ServiceProxy(name, service_type)
                try:
                    result.append((True, proxy(*args)))
                finally:
                    proxy.close()
            except Exception as error:
                result.append((False, error))
            finally:
                done.set()
        threading.Thread(target=worker, daemon=True).start()
        if not done.wait(timeout):
            raise RuntimeError("ROS service wall deadline; control not resent: "+name)
        success, value = result[0]
        if not success:
            raise value
        if hasattr(value, "success") and not value.success:
            raise RuntimeError(name+": "+value.status_message)
        return value

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, paused):
        from std_srvs.srv import Empty
        if self.physics().pause != paused:
            self.call("/gazebo/"+("pause_physics" if paused else "unpause_physics"), Empty)
        if self.physics().pause != paused:
            raise RuntimeError("Actual pause state did not match request")

    def state(self):
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import GetModelState
        answer = self.call("/gazebo/get_model_state", GetModelState, "wall", "world")
        value = ModelState()
        value.model_name, value.reference_frame = "wall", "world"
        value.pose, value.twist = copy.deepcopy(answer.pose), copy.deepcopy(answer.twist)
        return value

    def apply(self, value):
        from gazebo_msgs.srv import SetModelState
        self.call("/gazebo/set_model_state", SetModelState, value)

    def move(self, x, y=0):
        from gazebo_msgs.msg import ModelState
        value = ModelState()
        value.model_name, value.reference_frame = "wall", "world"
        value.pose.position.x, value.pose.position.y, value.pose.position.z = x, y, 1
        value.pose.orientation.w = 1
        self.apply(value)
        actual = state_values(self.state())
        if max(abs(a-b) for a, b in zip(actual["position"], [x, y, 1])) > 1e-7:
            raise RuntimeError("Actual wall did not reach requested pose")
        return actual


def state_values(value):
    def vector(item):
        return [item.x, item.y, item.z]
    q = value.pose.orientation
    return {"position": vector(value.pose.position), "orientation": [q.x, q.y, q.z, q.w],
            "linear": vector(value.twist.linear), "angular": vector(value.twist.angular)}


def check_bundle(pair, config):
    image, info = pair["depth"], pair["camera_info"]
    width, height = config["width"], config["height"]
    if image["stamp_ns"] != info["stamp_ns"] or image["stamp_ns"] != pair["stamp_ns"]:
        raise RuntimeError("Image/CameraInfo native timestamps are not exactly paired")
    if image["frame"] != info["frame"] or image["frame"].lstrip("/") != "depth_optical_frame":
        raise RuntimeError("Unexpected depth image/calibration frame label")
    if any(item["width"] != width or item["height"] != height for item in (image, info)):
        raise RuntimeError("Actual image/calibration dimensions differ from prepared resolution")
    if image["encoding"] != "32FC1" or image["step"] != width*4 or image["is_bigendian"] != 0 or image["data_length"] != height*image["step"] or len(pair["depth_data"]) != image["data_length"]:
        raise RuntimeError("Actual Image encoding/stride/byte length differs from 32FC1 contract")
    ideal_hfov = math.radians(config["hfov_deg"])
    # Classic11 Camera::SetHFOV stores its angle using sdformat9 Param::Set<T>.
    # That round trip uses a default stringstream (six significant digits).
    # Validate the actual renderer-derived K rather than altering the message.
    # Camera.cc819-828; sdformat9 Param.hh305-311.
    render_hfov = float(format(ideal_hfov, ".6g"))
    focal = width/(2*math.tan(render_hfov/2))
    ideal_focal = width/(2*math.tan(ideal_hfov/2))
    cx, cy = (width+1)/2, (height+1)/2  # retain the stock Noetic convention
    expected = {"K": [focal, 0, cx, 0, focal, cy, 0, 0, 1],
                "P": [focal, 0, cx, 0, 0, focal, cy, 0, 0, 0, 1, 0],
                "R": [1, 0, 0, 0, 1, 0, 0, 0, 1], "D": [0]*5}
    if info["distortion_model"] != "plumb_bob":
        raise RuntimeError("Unexpected camera distortion model")
    for key, values in expected.items():
        if len(info[key]) != len(values) or any(not math.isfinite(actual) or not math.isclose(actual, expected, rel_tol=1e-7, abs_tol=1e-6) for actual, expected in zip(info[key], values)):
            difference = max((abs(actual-wanted) for actual, wanted in zip(info[key], values)), default=None)
            raise RuntimeError("Actual CameraInfo differs from current pinhole configuration: "+key+
                               "; actual="+str(info[key])+"; expected="+str(values)+"; max_abs_error="+str(difference)+
                               "; ideal_hfov_rad="+str(ideal_hfov)+"; render_hfov_6sig_rad="+str(render_hfov))
    pair["intrinsics_check"] = {"ideal_hfov_rad": ideal_hfov, "render_hfov_6sig_rad": render_hfov,
        "ideal_focal_px": ideal_focal, "expected_render_focal_px": focal,
        "actual_focal_px": [info["K"][0], info["K"][4]],
        "max_focal_error_px": max(abs(info["K"][0]-focal), abs(info["K"][4]-focal)),
        "policy": "Classic11 Camera::SetHFOV / sdformat9 Param::Set six-significant-digit round trip"}

    cloud = pair["cloud"]
    if cloud["stamp_ns"] != pair["stamp_ns"] or cloud["frame"] != image["frame"] or cloud["width"] != width or cloud["height"] != height:
        raise RuntimeError("Actual organized cloud does not match depth stamp/frame/dimensions")
    if cloud["point_step"] <= 0 or cloud["row_step"] < width*cloud["point_step"] or cloud["data_length"] != height*cloud["row_step"] or len(pair["cloud_data"]) != cloud["data_length"]:
        raise RuntimeError("Actual PointCloud2 point_step/row_step/byte length is inconsistent")
    fields = {item["name"]: item for item in cloud["fields"]}
    for name in ("x", "y", "z", "rgb"):
        field = fields.get(name)
        if field is None or field["datatype"] != 7 or field["count"] != 1 or not 0 <= field["offset"] <= cloud["point_step"]-4:
            raise RuntimeError("Unexpected actual PointCloud2 field layout: "+name)


def number(value):
    value = float(value)
    return value if math.isfinite(value) else ("nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf"))


def depth_array(bundle):
    image = bundle["depth"]
    dtype = ">f4" if image["is_bigendian"] else "<f4"
    return np.ndarray((image["height"], image["width"]), dtype=dtype, buffer=bundle["depth_data"], strides=(image["step"], 4))


def cloud_array(bundle):
    cloud = bundle["cloud"]
    fields = {item["name"]: item for item in cloud["fields"]}
    endian = ">" if cloud["is_bigendian"] else "<"
    dtype = np.dtype({"names": ["x", "y", "z", "rgb"], "formats": [endian+"f4"]*4,
                     "offsets": [fields[name]["offset"] for name in ("x", "y", "z", "rgb")], "itemsize": cloud["point_step"]})
    return np.ndarray((cloud["height"], cloud["width"]), dtype=dtype, buffer=bundle["cloud_data"],
                      strides=(cloud["row_step"], cloud["point_step"]))


def sample_coordinates(width, height):
    return {name: [round(u*width), round(v*height)] for name, u, v in
            (("foreground", .5, .5), ("wall_a", .7, .5), ("wall_b", .8, .5),
             ("near_probe", .25, .5), ("far_probe", .5, .075), ("no_return", .03, .05))}


def measure(bundle, config, wall_distance):
    check_bundle(bundle, config)
    depth, cloud = depth_array(bundle), cloud_array(bundle)
    valid = np.isfinite(depth)
    invalid = ~valid
    if not np.array_equal(np.isfinite(cloud["z"]), valid):
        raise RuntimeError("Actual cloud and depth invalid masks disagree")
    if not np.all(np.isnan(depth[invalid])) or not all(np.all(np.isnan(cloud[name][invalid])) for name in ("x", "y", "z")) or bundle["cloud"]["is_dense"]:
        raise RuntimeError("Stock Openni invalid depth/XYZ/is_dense semantics differ from this NaN fixture")
    if not np.any(valid):
        raise RuntimeError("No actual finite depth samples")
    if np.max(np.abs(cloud["z"][valid]-depth[valid])) > 1e-5:
        raise RuntimeError("Actual pointcloud Z differs from native depth buffer")
    height, width = depth.shape
    focal = bundle["camera_info"]["K"][0]
    u = np.arange(width, dtype=np.float64)[None, :]
    v = np.arange(height, dtype=np.float64)[:, None]
    with np.errstate(invalid="ignore"):
        predicted_x = depth*(u-(width-1)/2)/focal
        predicted_y = depth*(v-(height-1)/2)/focal
    helper_error = max(float(np.max(np.abs(cloud[name][valid]-expected[valid])))
                       for name, expected in (("x", predicted_x), ("y", predicted_y)))
    if helper_error > 1e-4:
        raise RuntimeError("Actual cloud XYZ does not match installed wrapper pixel-center convention")
    fields = {item["name"]: item for item in bundle["cloud"]["fields"]}
    samples = {}
    for name, (column, row) in sample_coordinates(width, height).items():
        z = float(depth[row, column])
        xyz = [float(cloud[axis][row, column]) for axis in ("x", "y", "z")]
        expected = {"foreground": 1.9, "wall_a": wall_distance-.05, "wall_b": wall_distance-.05,
                    "near_probe": None, "far_probe": 4.95 if config["max_depth_m"] > 4.95 else None,
                    "no_return": None}[name]
        if expected is None:
            if not math.isnan(z) or not all(math.isnan(value) for value in xyz):
                raise RuntimeError("Expected actual NaN software-cutoff/no-return sample: "+name)
        elif not math.isfinite(z) or abs(z-expected) > .03:
            raise RuntimeError("Actual axial depth differs from known rendered geometry: "+name+"="+str(z)+", expected="+str(expected))
        offset = row*bundle["cloud"]["row_step"]+column*bundle["cloud"]["point_step"]+fields["rgb"]["offset"]
        packed = bundle["cloud_data"][offset:offset+4]
        bits = int.from_bytes(packed, byteorder="big" if bundle["cloud"]["is_bigendian"] else "little")
        canonical_rgb = [(bits >> 16)&255, (bits >> 8)&255, bits&255]
        k_xyz = [(column-bundle["camera_info"]["K"][2])*z/focal,
                 (row-bundle["camera_info"]["K"][5])*z/focal, z]
        samples[name] = {"pixel": [column, row], "depth_m": number(z), "expected_depth_m": expected,
                         "xyz_m": [number(value) for value in xyz],
                         "radial_range_m": number(math.sqrt(sum(value*value for value in xyz))),
                         "k_backprojection_xyz_m": [number(value) for value in k_xyz],
                         "k_minus_cloud_xyz_m": [number(a-b) for a, b in zip(k_xyz, xyz)],
                         "raw_rgb_field_bytes": list(packed), "canonical_packed_rgb": canonical_rgb}
    if samples["wall_b"]["radial_range_m"] <= samples["wall_b"]["depth_m"]+.1:
        raise RuntimeError("Off-axis actual point does not distinguish radial range from axial Z")
    return {"samples": samples, "counts": {"finite": int(valid.sum()), "nan": int(np.isnan(depth).sum()),
                "positive_infinity": int(np.isposinf(depth).sum()), "negative_infinity": int(np.isneginf(depth).sum())},
            "cloud_helper_max_error_m": helper_error,
            "pointcloud_pixel_center": [(width-1)/2, (height-1)/2],
            "camera_info_principal_point": [bundle["camera_info"]["K"][2], bundle["camera_info"]["K"][5]]}


def pseudocolor(bundle, config):
    depth = depth_array(bundle)
    valid = np.isfinite(depth)
    safe = np.where(valid, depth, config["min_depth_m"])
    t = np.clip((safe-config["min_depth_m"])/(config["max_depth_m"]-config["min_depth_m"]), 0, 1)
    rgb = np.stack((255*(1-t), 255*(1-np.abs(2*t-1)), 255*t), axis=-1).astype(np.uint8)
    rgb[~valid] = [255, 0, 255]  # explicit invalid mask, not a measured RGB value
    return np.ascontiguousarray(rgb)


def save_depth_png(bundle, config, path):
    from PyQt5.QtGui import QImage
    rgb = pseudocolor(bundle, config)
    height, width = rgb.shape[:2]
    image = QImage(rgb.data, width, height, width*3, QImage.Format_RGB888).copy()
    if not image.save(str(path), "PNG"):
        raise RuntimeError("Could not save actual scalar-depth pseudocolor visualization")


def rgb_summary(actual):
    meta, data = actual["rgb"], actual["rgb_data"]
    if meta["encoding"] != "rgb8" or meta["step"] != meta["width"]*3 or len(data) != meta["step"]*meta["height"]:
        raise RuntimeError("Unexpected actual RGB auxiliary encoding/stride")
    rgb = np.ndarray((meta["height"], meta["width"], 3), dtype=np.uint8, buffer=data, strides=(meta["step"], 3, 1))
    coordinates = sample_coordinates(meta["width"], meta["height"])
    return {name: [int(value) for value in rgb[v, u]] for name, (u, v) in coordinates.items()}


def ready(config, timeout=25):
    client = Client()
    try:
        deadline = time.monotonic()+timeout
        last_error = None
        while time.monotonic() < deadline:
            for bundle in client.drain():
                try:
                    observed = measure(bundle, config, 3)
                    return {"ready": True, "stamp_ns": bundle["stamp_ns"], "depth": bundle["depth"],
                            "cloud": bundle["cloud"], "intrinsics_check": bundle["intrinsics_check"], **observed}
                except RuntimeError as error:
                    last_error = str(error)
            time.sleep(.02)
        raise RuntimeError("Actual depth/cloud readiness timed out: "+str(last_error))
    finally:
        client.close()


def observe(config, seconds):
    if not math.isfinite(seconds) or not .2 <= seconds <= 30:
        raise ValueError("Observation duration must bewithin[.2,30]wall seconds")
    client, result = Client(), {"triples": 0}
    try:
        distance = client.state().pose.position.x
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            for bundle in client.drain():
                result["latest"] = {"stamp_ns": bundle["stamp_ns"], **measure(bundle, config, distance)}
                result["triples"] += 1
            time.sleep(.02)
        return result
    finally:
        client.close()


def view(config, seconds=0, capture=None):
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QImage, QPixmap
    from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
    if not math.isfinite(seconds) or not 0 <= seconds <= 300:
        raise ValueError("Viewer seconds must bewithin[0,300];0meansuntilclosed")
    client, shown = Client(), {"frames": 0}
    app = QApplication([])
    window, label = QMainWindow(), QLabel("等待本课真实深度／点云／CameraInfo…")
    label.setAlignment(Qt.AlignCenter)
    window.setCentralWidget(label)
    window.setWindowTitle("GazeboTeach L18 · 深度伪彩（原始量为米，紫色无效）")
    window.resize(max(640, config["width"]), max(480, config["height"]+50))
    def update():
        bundles = client.drain()
        if not bundles:
            return
        bundle = bundles[-1]
        try:
            check_bundle(bundle, config)
            rgb = pseudocolor(bundle, config)
            height, width = rgb.shape[:2]
            image = QImage(rgb.data, width, height, width*3, QImage.Format_RGB888).copy()
            label.setPixmap(QPixmap.fromImage(image))
            window.statusBar().showMessage("32FC1 米 | 近 %.1f 红→远 %.1f 蓝 | 紫=NaN/无效 | t=%.3fs" % (config["min_depth_m"], config["max_depth_m"], bundle["stamp_ns"]/1e9))
            shown["frames"] += 1
            if capture is not None and shown["frames"] == 1:
                save_depth_png(bundle, config, capture)
        except Exception as error:
            shown["error"] = str(error)
            app.quit()
    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(75)
    window.show()
    if seconds:
        QTimer.singleShot(round(seconds*1000), app.quit)
    def no_image():
        if not shown["frames"]:
            shown["error"] = "No actual depth triple within15wall seconds"
            app.quit()
    QTimer.singleShot(15000, no_image)
    try:
        app.exec_()
    finally:
        client.close()
    if shown.get("error") or not shown["frames"]:
        raise RuntimeError(shown.get("error", "No actual frame displayed"))
    return {"actual_depth_frames_displayed": shown["frames"], "capture": str(capture) if capture else None}


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    cfg = commands.add_parser("config")
    cfg.add_argument("--width", type=int)
    cfg.add_argument("--max-depth", type=int)
    viewer = commands.add_parser("view")
    viewer.add_argument("--seconds", type=float, default=0)
    viewer.add_argument("--capture", type=Path)
    observer = commands.add_parser("observe")
    observer.add_argument("--seconds", type=float, default=3)
    wall = commands.add_parser("wall")
    wall.add_argument("--distance", type=float, required=True)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.width is None and args.max_depth is None:
            parser.error("config requires --width or --max-depth")
        result = {"config": edit(args.width, args.max_depth), "reload_required": True}
    else:
        config, record = prepared()
        if args.command == "ready":
            result = ready(config)
        elif args.command == "observe":
            result = observe(config, args.seconds)
        elif args.command == "view":
            result = view(config, args.seconds, args.capture)
        else:
            if not math.isfinite(args.distance) or not 2.5 <= args.distance <= 4:
                parser.error("wall center must bewithin[2.5,4]m")
            client, paused = Client(), None
            try:
                paused = client.physics().pause
                client.pause(True)
                result = {"wall": client.move(args.distance), "paused": paused}
            finally:
                try:
                    if paused is not None:
                        client.pause(paused)
                finally:
                    client.close()
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
