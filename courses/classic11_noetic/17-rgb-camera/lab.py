#!/usr/bin/env python3
"""Pair native Image/CameraInfo stamps and inspect real RGB byte buffers."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import numpy as np
from config import edit, prepared

IMAGE_TOPIC = "/lesson17/camera/image_raw"
INFO_TOPIC = "/lesson17/camera/camera_info"


class Client:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import Image, CameraInfo
        self.rospy = rospy
        rospy.init_node("gazeboteach_rgb_lab", anonymous=True, disable_signals=True)
        self.images, self.infos, self.lock = {}, {}, threading.Lock()
        self.subscriptions = [rospy.Subscriber(IMAGE_TOPIC, Image, self.image, queue_size=4, buff_size=4*1024*1024),
                              rospy.Subscriber(INFO_TOPIC, CameraInfo, self.info, queue_size=30)]

    def image(self, message):
        stamp = message.header.stamp.to_nsec()
        row = {"stamp_ns": stamp, "frame": message.header.frame_id, "width": message.width, "height": message.height,
               "encoding": message.encoding, "is_bigendian": message.is_bigendian, "step": message.step,
               "data_length": len(message.data), "data": bytes(message.data)}
        with self.lock:
            self.images[stamp] = row
            while len(self.images) > 32:
                self.images.pop(next(iter(self.images)))

    def info(self, message):
        stamp = message.header.stamp.to_nsec()
        row = {"stamp_ns": stamp, "frame": message.header.frame_id, "width": message.width, "height": message.height,
               "distortion_model": message.distortion_model, **{key: list(getattr(message, key)) for key in ("D", "K", "R", "P")},
               "binning_x": message.binning_x, "binning_y": message.binning_y,
               "roi": {key: getattr(message.roi, key) for key in ("x_offset", "y_offset", "width", "height", "do_rectify")}}
        with self.lock:
            self.infos[stamp] = row
            while len(self.infos) > 64:
                self.infos.pop(next(iter(self.infos)))

    def drain(self):
        with self.lock:
            pairs = []
            for stamp in sorted(set(self.images) & set(self.infos)):
                image, info = self.images.pop(stamp), self.infos.pop(stamp)
                data = image.pop("data")
                pairs.append({"stamp_ns": stamp, "image": image, "camera_info": info, "data": data})
        return pairs

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
        answer = self.call("/gazebo/get_model_state", GetModelState, "target_red", "world")
        value = ModelState()
        value.model_name, value.reference_frame = "target_red", "world"
        value.pose, value.twist = copy.deepcopy(answer.pose), copy.deepcopy(answer.twist)
        return value

    def apply(self, value):
        from gazebo_msgs.srv import SetModelState
        self.call("/gazebo/set_model_state", SetModelState, value)

    def move(self, x, y):
        from gazebo_msgs.msg import ModelState
        value = ModelState()
        value.model_name, value.reference_frame = "target_red", "world"
        value.pose.position.x, value.pose.position.y, value.pose.position.z = x, y, 1
        value.pose.orientation.w = 1
        self.apply(value)
        actual = state_values(self.state())
        if max(abs(a-b) for a, b in zip(actual["position"], [x, y, 1])) > 1e-7:
            raise RuntimeError("Actual target did not reach requested pose")
        return actual


def state_values(value):
    def vector(item):
        return [item.x, item.y, item.z]
    q = value.pose.orientation
    return {"position": vector(value.pose.position), "orientation": [q.x, q.y, q.z, q.w],
            "linear": vector(value.twist.linear), "angular": vector(value.twist.angular)}


def check_pair(pair, config):
    image, info = pair["image"], pair["camera_info"]
    width, height = config["width"], config["height"]
    if image["stamp_ns"] != info["stamp_ns"] or image["stamp_ns"] != pair["stamp_ns"]:
        raise RuntimeError("Image/CameraInfo native timestamps are not exactly paired")
    if image["frame"] != info["frame"] or image["frame"].lstrip("/") != "camera_optical_frame":
        raise RuntimeError("Unexpected RGB image/calibration frame label")
    if any(item["width"] != width or item["height"] != height for item in (image, info)):
        raise RuntimeError("Actual image/calibration dimensions differ from prepared resolution")
    if image["encoding"] != "rgb8" or image["step"] != width*3 or image["is_bigendian"] != 0 or image["data_length"] != height*image["step"] or len(pair["data"]) != image["data_length"]:
        raise RuntimeError("Actual Image encoding/stride/byte length differs from RGB8 contract")
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


def pixels(pair):
    image = pair["image"]
    # Honor row stride; never reinterpret rgb8 as BGR.
    rows = np.frombuffer(pair["data"], dtype=np.uint8).reshape(image["height"], image["step"])
    rgb = rows[:, :image["width"]*3].reshape(image["height"], image["width"], 3)
    result = {}
    channels = rgb.astype(np.uint16)
    for name, axis in (("red", 0), ("green", 1), ("blue", 2)):
        dominant = channels[:, :, axis]
        others = [channels[:, :, other] for other in range(3) if other != axis]
        mask = (dominant > 160) & (dominant > 2*others[0]) & (dominant > 2*others[1])
        v, u = np.nonzero(mask)
        result[name] = {"count": int(len(u)), "bbox": [int(u.min()), int(v.min()), int(u.max()), int(v.max())] if len(u) else None,
                        "centroid": [float(u.mean()), float(v.mean())] if len(u) else None}
    return result


def projection(info, x, y):
    fx, fy, cx, cy = info["K"][0], info["K"][4], info["K"][2], info["K"][5]
    corners = [(cx-fx*(y+dy)/(x+dx), cy-fy*dz/(x+dx))
               for dx in (-.02, .02) for dy in (-.3, .3) for dz in (-.3, .3)]
    left, top = min(p[0] for p in corners), min(p[1] for p in corners)
    right, bottom = max(p[0] for p in corners), max(p[1] for p in corners)
    return {"bbox": [left, top, right, bottom], "bbox_center": [(left+right)/2, (top+bottom)/2],
            "bbox_extent": [right-left, bottom-top]}


def save_png(pair, path):
    from PyQt5.QtGui import QImage
    image = pair["image"]
    actual = QImage(pair["data"], image["width"], image["height"], image["step"], QImage.Format_RGB888).copy()
    if not actual.save(str(path), "PNG"):
        raise RuntimeError("Could not save actual ROS image PNG")


def ready(config, timeout=20):
    client = Client()
    try:
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            for pair in client.drain():
                check_pair(pair, config)
                observed = pixels(pair)
                if observed["red"]["count"] > 100 and observed["green"]["count"] > 25 and observed["blue"]["count"] > 25:
                    return {"ready": True, "stamp_ns": pair["stamp_ns"], "image": pair["image"], "camera_info": pair["camera_info"],
                            "intrinsics_check": pair["intrinsics_check"], "pixels": observed}
            time.sleep(.02)
        raise RuntimeError("Actual RGB image/calibration/color readiness timed out; check graphical rendering")
    finally:
        client.close()


def observe(config, seconds):
    if not math.isfinite(seconds) or not .2 <= seconds <= 30:
        raise ValueError("Observation duration must be within[.2,30]wall seconds")
    client, rows = Client(), []
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            for pair in client.drain():
                check_pair(pair, config)
                rows.append({"stamp_ns": pair["stamp_ns"], "image": pair["image"], "camera_info": pair["camera_info"],
                             "intrinsics_check": pair["intrinsics_check"], "pixels": pixels(pair)})
            time.sleep(.02)
        return {"paired_messages": len(rows), "latest": rows[-1] if rows else None}
    finally:
        client.close()


def view(config, seconds=0, capture=None):
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QImage, QPixmap
    from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
    if not math.isfinite(seconds) or not 0 <= seconds <= 300:
        raise ValueError("Viewer duration must bewithin[0,300]s;0meansuntilclosed")
    client, captured = Client(), {"count": 0}
    app = QApplication([])
    window, label = QMainWindow(), QLabel("等待本课真实RGB图像与CameraInfo…")
    label.setAlignment(Qt.AlignCenter)
    window.setCentralWidget(label)
    window.setWindowTitle("GazeboTeach L17 · 真实RGB相机")
    window.resize(max(640, config["width"]), max(480, config["height"]+50))
    def update():
        pairs = client.drain()
        if not pairs:
            return
        pair = pairs[-1]
        try:
            check_pair(pair, config)
            image = pair["image"]
            actual = QImage(pair["data"], image["width"], image["height"], image["step"], QImage.Format_RGB888).copy()
            label.setPixmap(QPixmap.fromImage(actual))
            window.statusBar().showMessage("rgb8  %d×%d  step=%d  t=%.3fs  frame=%s" % (image["width"], image["height"], image["step"], pair["stamp_ns"]/1e9, image["frame"]))
            captured["count"] += 1
            if capture is not None and captured["count"] == 1:
                save_png(pair, capture)
        except Exception as error:
            captured["error"] = str(error)
            app.quit()
    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    window.show()
    if seconds:
        QTimer.singleShot(round(seconds*1000), app.quit)
    def no_image():
        if captured["count"] == 0:
            captured["error"] = "No actual RGB image in10wall seconds"
            app.quit()
    QTimer.singleShot(10000, no_image)
    try:
        app.exec_()
    finally:
        client.close()
    if captured.get("error"):
        raise RuntimeError(captured["error"])
    if not captured["count"]:
        raise RuntimeError("Viewer closed without an actual image")
    return {"actual_frames_displayed": captured["count"], "capture": str(capture) if capture else None}


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    cfg = commands.add_parser("config")
    cfg.add_argument("--width", type=int)
    cfg.add_argument("--fov", type=int)
    viewer = commands.add_parser("view")
    viewer.add_argument("--seconds", type=float, default=0)
    viewer.add_argument("--capture", type=Path)
    observer = commands.add_parser("observe")
    observer.add_argument("--seconds", type=float, default=3)
    mover = commands.add_parser("target")
    mover.add_argument("--x", type=float, required=True)
    mover.add_argument("--y", type=float, default=0)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.width is None and args.fov is None:
            parser.error("config requires --width or --fov")
        result = {"config": edit(args.width, args.fov), "reload_required": True}
    else:
        config, record = prepared()
        if args.command == "ready":
            result = ready(config)
        elif args.command == "observe":
            result = observe(config, args.seconds)
        elif args.command == "view":
            result = view(config, args.seconds, args.capture)
        else:
            if not math.isfinite(args.x) or not 2.5 <= args.x <= 4.5 or not math.isfinite(args.y) or not -.5 <= args.y <= .5:
                parser.error("Target center x must bewithin[2.5,4.5],ywithin[-.5,.5]m")
            client, paused = Client(), None
            try:
                paused = client.physics().pause
                client.pause(True)
                result = {"target": client.move(args.x, args.y), "paused": paused}
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
