"""Exercise release provenance and installer behavior using an isolated Git repo."""
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ReleasePackagingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="gazeboteach-release-")
        cls.base = Path(cls.temporary.name)
        cls.repo = cls.base / "source"
        cls.repo.mkdir()
        for name in ("scripts/install_payload.sh", "launch.sh", "LICENSE"):
            destination = cls.repo / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes((ROOT / name).read_bytes().replace(b"\r\n", b"\n"))
        (cls.repo / "gazeboteach").mkdir()
        (cls.repo / "gazeboteach/__main__.py").write_text(
            "import json, sys\nprint(json.dumps({'fixture': True, 'args': sys.argv[1:]}))\n",
            encoding="utf-8",
        )
        for key in ["%02d" % value for value in range(1, 25)] + ["091", "092"]:
            folder = cls.repo / "courses/classic11_noetic" / (key + "-fixture")
            folder.mkdir(parents=True)
            (folder / "lesson.json").write_text(json.dumps({
                "id": key, "title": "Fixture " + key, "profile": "classic11_noetic"
            }), encoding="utf-8")
        cls.excluded = ("checkpoints/old.zip", ".env", "private.key", ".ssh/id_ed25519", "build/generated.txt")
        for name in cls.excluded:
            destination = cls.repo / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text("excluded fixture, no real secrets", encoding="utf-8")
        cls.git("init", "-q")
        cls.git("config", "user.name", "Release test")
        cls.git("config", "user.email", "release-test@example.invalid")
        cls.git("add", ".")
        cls.git("commit", "-qm", "Isolated release fixture")
        cls.commit = cls.git("rev-parse", "HEAD").strip()
        cls.output = cls.base / "artifacts"
        cls.build(cls.output)
        cls.installer = cls.output / "GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh"
        raw = cls.installer.read_bytes()
        cls.installer_python = raw.split(b"<<'GAZEBOTEACH_INSTALLER_PYTHON'\n", 1)[1].split(
            b"\nGAZEBOTEACH_INSTALLER_PYTHON\n", 1
        )[0].decode("utf-8")

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    @classmethod
    def git(cls, *args):
        return subprocess.check_output(["git", "-C", str(cls.repo), *args], text=True)

    @classmethod
    def build(cls, output):
        return subprocess.run([
            sys.executable, str(ROOT / "scripts/build_release.py"),
            "--repo", str(cls.repo), "--output", str(output)
        ], check=True, capture_output=True, text=True)

    def invoke(self, *args, installer=None, env=None):
        # Execute the exact embedded Python on every platform; Linux tests below
        # additionally execute the shell installer and installed wrapper directly.
        return subprocess.run([
            sys.executable, "-c", self.installer_python,
            str(installer or self.installer), *map(str, args)
        ], capture_output=True, text=True, env=env)

    def test_immutable_snapshot_reproducible_and_excludes_private_build_files(self):
        second = self.base / "artifacts-second"
        self.build(second)
        for first in self.output.iterdir():
            self.assertEqual(first.read_bytes(), (second / first.name).read_bytes())
        manifest = json.loads((self.output / "release-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["source_commit"], self.commit)
        self.assertEqual(manifest["course_count"], 26)
        self.assertTrue(all(name not in manifest["files"] for name in self.excluded))
        payload = self.output / "GazeboTeach-1.0.0-ubuntu20.04-x86_64.tar.gz"
        with tarfile.open(str(payload), "r:gz") as archive:
            launch = archive.getmember("GazeboTeach-1.0.0/launch.sh")
            self.assertEqual(launch.mode, 0o755)
            self.assertTrue(all(member.isfile() for member in archive.getmembers()))
        for line in (self.output / "SHA256SUMS").read_text(encoding="ascii").splitlines():
            expected, name = line.split("  ", 1)
            self.assertEqual(hashlib.sha256((self.output / name).read_bytes()).hexdigest(), expected)

    def test_check_is_nonmutating_and_tampered_payload_rejected(self):
        absent = self.base / "must-not-be-created"
        checked = self.invoke("--check", "--prefix", absent)
        self.assertEqual(checked.returncode, 0, checked.stderr)
        self.assertEqual(json.loads(checked.stdout)["source_commit"], self.commit)
        self.assertFalse(absent.exists())
        tampered = self.base / "tampered.sh"
        raw = bytearray(self.installer.read_bytes())
        raw[-10] ^= 1
        tampered.write_bytes(raw)
        rejected = self.invoke("--check", "--prefix", absent, installer=tampered)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("SHA256/length mismatch", rejected.stderr)
        self.assertFalse(absent.exists())

    @unittest.skipUnless(platform.system() == "Linux" and shutil.which("bash"), "requires Linux installer filesystem semantics")
    def test_install_wrapper_idempotence_modified_refusal_and_scoped_uninstall(self):
        prefix = self.base / "application with spaces"
        data = self.base / "personal-data"
        data.mkdir()
        (data / "progress.json").write_text("keep", encoding="utf-8")
        env = dict(os.environ, GAZEBOTEACH_DATA_DIR=str(data))
        for _ in range(2):
            installed = subprocess.run(["bash", str(self.installer), "--prefix", str(prefix), "--desktop"],
                                       capture_output=True, text=True, env=env)
            self.assertEqual(installed.returncode, 0, installed.stderr)
        entry = prefix / "bin/gazeboteach"
        launched = subprocess.run([str(entry), "--list"], capture_output=True, text=True, env=env)
        self.assertEqual(launched.returncode, 0, launched.stderr)
        self.assertEqual(json.loads(launched.stdout), {"fixture": True, "args": ["--list"]})
        source = prefix / "versions/1.0.0/gazeboteach/__main__.py"
        original = source.read_bytes()
        source.write_bytes(original + b"# personal modification\n")
        refused = self.invoke("--prefix", prefix, env=env)
        self.assertNotEqual(refused.returncode, 0)
        self.assertIn("was modified", refused.stderr)
        self.assertTrue(source.read_bytes().endswith(b"# personal modification\n"))
        source.write_bytes(original)
        other = prefix / "versions/0.9.0"
        other.mkdir()
        (other / "keep.txt").write_text("keep", encoding="utf-8")
        removed = self.invoke("--prefix", prefix, "--uninstall", env=env)
        self.assertEqual(removed.returncode, 0, removed.stderr)
        self.assertFalse((prefix / "versions/1.0.0").exists())
        self.assertFalse(entry.exists())
        self.assertFalse((prefix / "current").is_symlink())
        self.assertEqual((other / "keep.txt").read_text(), "keep")
        self.assertEqual((data / "progress.json").read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
