#!/usr/bin/env python3
"""Scope the package overlay, then exec real roslaunch; never create another Master."""
import os
from pathlib import Path
import sys
import xmlrpc.client
from config import launch_argv, overlay, prepared, read_settings


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def main():
    workspace = Path(__file__).resolve().parent
    operation = sys.argv[1] if len(sys.argv) > 1 else "start"
    if operation not in ("start", "find", "nodes", "params"):
        raise ValueError("Use start/find/nodes/params")
    config = read_settings(workspace/"launch_settings.json")
    overrides = {}
    for value in sys.argv[2:]:
        key, separator, item = value.partition(":=")
        if not separator or key not in ("force_n", "robot_ns") or item not in (("2", "4") if key == "force_n" else ("cart_a", "cart_b")):
            raise ValueError("Inspect overrides must be force_n:=2/4 or robot_ns:=cart_a/cart_b")
        overrides[key] = item
    environment = overlay(workspace)
    if operation == "start":
        if overrides:
            raise ValueError("Save launch_settings.json and restart the platform experiment to change running arguments")
        prepared(workspace)
        if not environment.get("GAZEBOTEACH_SESSION_TOKEN"):
            raise RuntimeError("Start the experiment from GazeboTeach so its private Master/session are available")
        with xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"], transport=Transport()) as master:
            if master.getPid("/gazeboteach_l22_entry")[0] != 1:
                raise RuntimeError("The existing private Master is unavailable")
    argv = ["rospack", "find", "gazeboteach_l22"] if operation == "find" else launch_argv(config, operation, overrides)
    os.chdir(str(workspace))
    os.execvpe(argv[0], argv, environment)


if __name__ == "__main__":
    main()
