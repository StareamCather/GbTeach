"""Backend configuration must never imply that a Docker daemon is available."""

from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from gazeboteach.backends import DockerConnectionConfig, MountSpec, backend_descriptors, get_backend


class BackendContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def config(self, **changes):
        result = {"image": "osrf/ros:noetic-desktop-full", "host_workspace": str(self.root / "app")}
        result.update(changes)
        return result

    def test_configuration_round_trip_preserves_read_only_mount_intent(self):
        value = self.config(display=":0", context="default", container_name="gazeboteach-v1",
                            mounts=[{"source": str(self.root / "data"), "target": "/home/student/data",
                                     "read_only": False}])
        config = DockerConnectionConfig.from_dict(value)
        self.assertEqual(config, DockerConnectionConfig.from_dict(config.to_dict()))
        self.assertFalse(config.mounts[0].read_only)
        self.assertEqual(config.to_dict()["schema_version"], 1)

    def test_unknown_execution_options_and_schemas_are_rejected(self):
        for changes in ({"privileged": True}, {"command": "rm -rf /"}, {"schema_version": 2},
                        {"schema_version": True}, {"backend": "native"}, {"mounts": "not-list"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                DockerConnectionConfig.from_dict(self.config(**changes))

    def test_paths_and_identifiers_cannot_turn_into_root_mounts_or_options(self):
        for changes in ({"host_workspace": "relative"}, {"host_workspace": str(self.root.anchor)},
                        {"container_workspace": "/"}, {"container_workspace": "//"},
                        {"container_workspace": "/app/../etc"},
                        {"container_workspace": "/app\\etc"}, {"image": "--privileged"},
                        {"image": "osrf/ros; echo token"}, {"context": "--host"},
                        {"display": ":0\ncommand"}, {"container_name": "name --privileged"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                DockerConnectionConfig.from_dict(self.config(**changes))

    def test_overlapping_mounts_cannot_hide_workspace_or_each_other(self):
        for target in ("/workspace/GazeboTeach", "/workspace", "/workspace/GazeboTeach/build"):
            mounts = [{"source": str(self.root / "data"), "target": target}]
            with self.subTest(target=target), self.assertRaises(ValueError):
                DockerConnectionConfig.from_dict(self.config(mounts=mounts))

    def test_mount_defaults_to_read_only_and_rejects_boolean_strings(self):
        mount = MountSpec(str(self.root / "data"), "/data")
        self.assertTrue(mount.read_only)
        with self.assertRaises(ValueError):
            MountSpec(str(self.root / "data"), "/data", "false")

    def test_missing_docker_client_does_not_call_subprocess_or_claim_execution(self):
        with mock.patch("gazeboteach.backends.shutil.which", return_value=None), \
                mock.patch("gazeboteach.backends.subprocess.run") as execute:
            result = get_backend("docker").probe()
        execute.assert_not_called()
        self.assertFalse(result["cli"]["installed"])
        self.assertFalse(result["execution_available"])
        self.assertFalse(result["daemon_checked"])

    def test_probe_runs_client_version_only_and_reports_actual_timeout(self):
        completed = subprocess.CompletedProcess(["/tools/docker", "--version"], 0,
                                                 stdout="Docker version 27.0.0", stderr="")
        with mock.patch("gazeboteach.backends.shutil.which", return_value="/tools/docker"), \
                mock.patch("gazeboteach.backends.subprocess.run", return_value=completed) as execute:
            result = get_backend("docker").probe(timeout=2)
        self.assertEqual(execute.call_args[0][0], ["/tools/docker", "--version"])
        self.assertEqual(execute.call_args[1]["timeout"], 2)
        self.assertEqual(result["cli"]["version"], "Docker version 27.0.0")
        self.assertFalse(result["execution_available"])
        self.assertFalse(result["daemon_checked"])
        with mock.patch("gazeboteach.backends.shutil.which", return_value="/tools/docker"), \
                mock.patch("gazeboteach.backends.subprocess.run", side_effect=subprocess.TimeoutExpired("docker", 2)):
            result = get_backend("docker").probe(timeout=2)
        self.assertIn("timed out", result["cli"]["error"])

    def test_reserved_execution_is_explicit_even_for_valid_config(self):
        config = DockerConnectionConfig.from_dict(self.config())
        with self.assertRaises(NotImplementedError):
            get_backend("docker").create_session(self.root, None, configuration=config)
        self.assertFalse(get_backend("docker").describe()["execution_implemented"])
        self.assertEqual([item["id"] for item in backend_descriptors()], ["native", "docker"])

    def test_unknown_backend_and_unbounded_client_probe_are_rejected(self):
        with self.assertRaises(ValueError):
            get_backend("remote")
        for timeout in (0, -1, True, 11, "3"):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                get_backend("docker").probe(timeout)


if __name__ == "__main__":
    unittest.main()
