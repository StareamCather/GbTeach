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
        cls.v1_commit = cls.git("rev-parse", "HEAD").strip()
        for key in ["%02d" % value for value in range(25, 38)]:
            folder = cls.repo / "courses/classic11_noetic" / (key + "-fixture")
            folder.mkdir(parents=True)
            (folder / "lesson.json").write_text(json.dumps({
                "id": key, "title": "Fixture " + key, "profile": "classic11-noetic"
            }), encoding="utf-8")
        for lesson in (cls.repo / "courses").rglob("lesson.json"):
            item = json.loads(lesson.read_text(encoding="utf-8"))
            item["profile"] = "classic11-noetic"
            lesson.write_text(json.dumps(item), encoding="utf-8")
        cls.v2_documents = ("V2_INSTALL.md", "RELEASE_NOTES_V2.md", "V2_CURRICULUM.md", "V2_DOCKER.md", "ENVIRONMENT_V2.md")
        for name in ("resources/native_control.cc", "gazeboteach/native_control.py", "docs/NATIVE_CONTROL.md", *("docs/"+name for name in cls.v2_documents)):
            destination = cls.repo / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text("Release source fixture: "+name+"\n", encoding="utf-8")
        cls.git("add", ".")
        cls.git("commit", "-qm", "Complete V2 release fixture")
        cls.commit = cls.git("rev-parse", "HEAD").strip()
        cls.output = cls.base / "artifacts"
        cls.build(cls.output)
        cls.installer = cls.output / "GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh"
        cls.installer_python = cls.embedded_python(cls.installer)
        cls.add_expansion_fixtures()
        cls.add_compatibility_fixtures()

    @classmethod
    def add_expansion_fixtures(cls):
        additional = {
            ("classic11_noetic", "classic11-noetic"): ["%02d" % value for value in range(38, 54)],
            ("modern_ros2", "harmonic-jazzy"): ["01", "11", "22", "23", "24", "25", "26"],
        }
        for (folder_name, profile), lesson_ids in additional.items():
            for lesson_id in lesson_ids:
                folder = cls.repo / "courses" / folder_name / (lesson_id+"-fixture")
                folder.mkdir(parents=True)
                (folder/"lesson.json").write_text(json.dumps({
                    "id": lesson_id, "title": "Fixture "+lesson_id, "profile": profile
                }), encoding="utf-8")
        (cls.repo/"gazeboteach/backends.py").write_text("# Fixture backend\n", encoding="utf-8")
        version_file = cls.repo/"gazeboteach/__init__.py"
        version_file.write_bytes(b'__version__ = "2.0.0"\n')
        cls.git("add", ".")
        cls.git("commit", "-qm", "Expansion tested code fixture")
        cls.tested_expansion = cls.git("rev-parse", "HEAD").strip()
        cls.expansion_documents = ("V2_1_INSTALL.md", "EXPANSION_VALIDATION.md", "EXPANSION_VERIFICATION.json")
        for document in cls.expansion_documents[:2]:
            (cls.repo/"docs"/document).write_text("Expansion fixture "+document+"\n", encoding="utf-8")
        verification = cls.repo/"docs"/cls.expansion_documents[2]
        report = {"tested_code_commit": cls.tested_expansion, "profiles": {
            "classic11-noetic": {"passed": True, "course_count": 55, "cases": [{"id": k, "passed": True} for k in sorted({f"{i:02d}" for i in range(1, 54)} | {"091", "092"})]},
            "harmonic-jazzy": {"passed": True, "course_count": 7, "cases": [{"id": k, "passed": True} for k in ("01", "11", "22", "23", "24", "25", "26")]},
        }}
        verification.write_text(json.dumps(report), encoding="utf-8")
        version_file.write_bytes(b'__version__ = "2.1.0"\n')
        cls.git("add", ".")
        cls.git("commit", "-qm", "Expansion evidence fixture")
        cls.expansion_commit = cls.git("rev-parse", "HEAD").strip()
        report["profiles"]["harmonic-jazzy"]["passed"] = False
        verification.write_text(json.dumps(report), encoding="utf-8")
        cls.git("add", ".")
        cls.git("commit", "-qm", "Rejected evidence fixture")
        cls.failed_evidence_commit = cls.git("rev-parse", "HEAD").strip()
        report["profiles"]["harmonic-jazzy"]["passed"] = True
        verification.write_text(json.dumps(report), encoding="utf-8")
        application = cls.repo/"gazeboteach/__main__.py"
        application.write_bytes(application.read_bytes()+b"# Application changed after test\n")
        cls.git("add", ".")
        cls.git("commit", "-qm", "Rejected untested code fixture")
        cls.untested_code_commit = cls.git("rev-parse", "HEAD").strip()

    @classmethod
    def add_compatibility_fixtures(cls):
        cls.git("checkout", "-q", cls.expansion_commit)
        cls.git("tag", "v2.1.0")
        for lesson in (cls.repo/"courses/modern_ros2").glob("*/lesson.json"):
            item = json.loads(lesson.read_text(encoding="utf-8"))
            item["variants"] = {"fortress-humble": {"world": "world.fortress.sdf"}}
            lesson.write_text(json.dumps(item), encoding="utf-8")
            (lesson.parent/"world.fortress.sdf").write_text("<sdf version='1.9'/>\n", encoding="utf-8")
        cls.git("add", ".")
        cls.git("commit", "-qm", "Compatibility tested code fixture")
        cls.tested_compatibility = cls.git("rev-parse", "HEAD").strip()
        cls.compatibility_documents = ("V2_2_INSTALL.md", "V2_2_VERIFICATION.json")
        (cls.repo/"docs/V2_2_INSTALL.md").write_text("Compatibility installation fixture\n", encoding="utf-8")
        cls.compatibility_report = {
            "schema_version": 1, "passed": True,
            "tested_code_commit": cls.tested_compatibility,
            "classic_baseline_commit": cls.expansion_commit,
            "profiles": {
                "classic11-noetic": {"passed": True, "course_count": 55, "cases": [{"id": "01", "passed": True}]},
                **{profile: {"passed": True, "course_count": 7,
                             "cases": [{"id": key, "passed": True} for key in ("01", "11", "22", "23", "24", "25", "26")]}
                   for profile in ("harmonic-jazzy", "fortress-humble")},
            },
        }
        cls.compatibility_proof = cls.repo/"docs/V2_2_VERIFICATION.json"
        cls.compatibility_proof.write_text(json.dumps(cls.compatibility_report), encoding="utf-8")
        (cls.repo/"gazeboteach/__init__.py").write_bytes(b'__version__ = "2.2.0"\n')
        cls.git("add", ".")
        cls.git("commit", "-qm", "Complete compatibility release fixture")
        cls.compatibility_commit = cls.git("rev-parse", "HEAD").strip()
        cls.compatibility_rejections = []
        mutations = (
            ("passed evidence for fortress-humble", "failed-modern"),
            ("representative regression", "missing-classic"),
            ("historical V2.1 verification was changed", "historical-proof"),
            ("Classic course files differ", "changed-classic"),
            ("application or course changes", "untested-modern"),
            ("lacks its tracked world", "missing-world"),
        )
        for expected, mutation in mutations:
            cls.git("checkout", "-q", cls.compatibility_commit)
            report = json.loads(json.dumps(cls.compatibility_report))
            if mutation == "failed-modern":
                report["profiles"]["fortress-humble"]["cases"][0]["passed"] = False
                cls.compatibility_proof.write_text(json.dumps(report), encoding="utf-8")
            elif mutation == "missing-classic":
                report["profiles"]["classic11-noetic"]["cases"] = []
                cls.compatibility_proof.write_text(json.dumps(report), encoding="utf-8")
            elif mutation == "historical-proof":
                path = cls.repo/"docs/EXPANSION_VERIFICATION.json"
                path.write_bytes(path.read_bytes()+b"\n")
            elif mutation == "changed-classic":
                (cls.repo/"courses/classic11_noetic/01-fixture/unaccepted.txt").write_text("changed\n")
            elif mutation == "untested-modern":
                (cls.repo/"courses/modern_ros2/01-fixture/lab.py").write_text("# changed after test\n")
            elif mutation == "missing-world":
                cls.git("rm", "courses/modern_ros2/01-fixture/world.fortress.sdf")
            cls.git("add", ".")
            cls.git("commit", "-qm", "Rejected compatibility fixture "+mutation)
            cls.compatibility_rejections.append((cls.git("rev-parse", "HEAD").strip(), expected))

    @staticmethod
    def embedded_python(installer):
        raw = installer.read_bytes()
        return raw.split(b"<<'GAZEBOTEACH_INSTALLER_PYTHON'\n", 1)[1].split(
            b"\nGAZEBOTEACH_INSTALLER_PYTHON\n", 1
        )[0].decode("utf-8")

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    @classmethod
    def git(cls, *args):
        return subprocess.check_output(["git", "-C", str(cls.repo), *args], text=True)

    @classmethod
    def build(cls, output, *arguments):
        return subprocess.run([
            sys.executable, str(ROOT / "scripts/build_release.py"),
            "--repo", str(cls.repo), "--output", str(output), "--revision", cls.commit, *arguments
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
        self.assertEqual(manifest["course_count"], 39)
        self.assertEqual(manifest["version"], "2.0.0")
        self.assertIn("resources/native_control.cc", manifest["files"])
        for name in self.v2_documents:
            committed = subprocess.check_output(["git", "-C", str(self.repo), "show", self.commit+":docs/"+name])
            self.assertEqual((self.output / name).read_bytes(), committed)
        self.assertTrue(all(name not in manifest["files"] for name in self.excluded))
        payload = self.output / "GazeboTeach-2.0.0-ubuntu20.04-x86_64.tar.gz"
        with tarfile.open(str(payload), "r:gz") as archive:
            launch = archive.getmember("GazeboTeach-2.0.0/launch.sh")
            self.assertEqual(launch.mode, 0o755)
            self.assertTrue(all(member.isfile() for member in archive.getmembers()))
        for line in (self.output / "SHA256SUMS").read_text(encoding="ascii").splitlines():
            expected, name = line.split("  ", 1)
            self.assertEqual(hashlib.sha256((self.output / name).read_bytes()).hexdigest(), expected)

    def test_v2_rejects_incomplete_snapshot_and_v1_remains_explicitly_buildable(self):
        with self.assertRaises(subprocess.CalledProcessError) as caught:
            self.build(self.base / "incomplete", "--revision", self.v1_commit)
        self.assertIn("39 distinct projects", caught.exception.stderr)
        legacy = self.base / "legacy"
        self.build(legacy, "--version", "1.0.0", "--revision", self.v1_commit)
        manifest = json.loads((legacy / "release-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["course_count"], 26)
        self.assertEqual(manifest["source_commit"], self.v1_commit)

    def test_expansion_packages_both_profiles_and_verified_docs(self):
        output = self.base/"expansion-artifacts"
        self.build(output, "--version", "2.1.0", "--revision", self.expansion_commit)
        manifest = json.loads((output/"release-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["source_commit"], self.expansion_commit)
        self.assertEqual(manifest["course_count"], 62)
        self.assertEqual(manifest["course_counts_by_profile"], {"classic11-noetic": 55, "harmonic-jazzy": 7})
        self.assertEqual({row["profile"] for row in manifest["courses"] if row["id"] == "01"},
                         {"classic11-noetic", "harmonic-jazzy"})
        self.assertEqual(manifest["tested_platforms"]["harmonic-jazzy"]["version"], "24.04")
        self.assertEqual(manifest["verification"]["tested_code_commit"], self.tested_expansion)
        for name in self.expansion_documents:
            committed = subprocess.check_output(["git", "-C", str(self.repo), "show", self.expansion_commit+":docs/"+name])
            self.assertEqual((output/name).read_bytes(), committed)
        report = output/"EXPANSION_VERIFICATION.json"
        self.assertEqual(manifest["verification"]["sha256"], hashlib.sha256(report.read_bytes()).hexdigest())
        for name in ("GazeboTeach-2.1.0-linux-x86_64.sh", "GazeboTeach-2.1.0-linux-x86_64.tar.gz",
                     "release-manifest.json", *self.expansion_documents):
            self.assertIn("  "+name+"\n", (output/"SHA256SUMS").read_text(encoding="ascii"))
        installer = output/"GazeboTeach-2.1.0-linux-x86_64.sh"
        check = subprocess.run([sys.executable, "-c", self.embedded_python(installer), str(installer), "--check"],
                               capture_output=True, text=True, check=True)
        self.assertEqual(json.loads(check.stdout)["course_count"], 62)

    def test_expansion_requires_correct_inventory_passed_report_and_tested_code(self):
        cases = [
            (self.commit, "55 Classic/Noetic and 7 Harmonic/Jazzy"),
            (self.failed_evidence_commit, "passed evidence for harmonic-jazzy"),
            (self.untested_code_commit, "application or course changes"),
        ]
        for revision, message in cases:
            with self.subTest(revision=revision), self.assertRaises(subprocess.CalledProcessError) as caught:
                self.build(self.base/"rejected-expansion", "--version", "2.1.0", "--revision", revision)
            self.assertIn(message, caught.exception.stderr)

    def test_compatibility_includes_declared_variants_and_six_verified_assets(self):
        output = self.base/"compatibility-artifacts"
        self.build(output, "--version", "2.2.0", "--revision", self.compatibility_commit)
        manifest = json.loads((output/"release-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["course_count"], 69)
        self.assertEqual(manifest["project_count"], 62)
        self.assertEqual(manifest["course_counts_by_profile"],
                         {"classic11-noetic": 55, "harmonic-jazzy": 7, "fortress-humble": 7})
        self.assertEqual(manifest["tested_platforms"]["fortress-humble"]["version"], "22.04")
        self.assertEqual(manifest["verification"]["tested_code_commit"], self.tested_compatibility)
        self.assertEqual(manifest["verification"]["classic_baseline_commit"], self.expansion_commit)
        self.assertEqual(manifest["verification"]["classic_baseline_verification_sha256"],
                         manifest["files"]["docs/EXPANSION_VERIFICATION.json"]["sha256"])
        self.assertEqual({item["profile"] for item in manifest["courses"] if item["id"] == "01"},
                         {"classic11-noetic", "harmonic-jazzy", "fortress-humble"})
        expected_assets = {"GazeboTeach-2.2.0-linux-x86_64.sh", "GazeboTeach-2.2.0-linux-x86_64.tar.gz",
                           "release-manifest.json", "SHA256SUMS", *self.compatibility_documents}
        self.assertEqual({path.name for path in output.iterdir()}, expected_assets)
        installer = output/"GazeboTeach-2.2.0-linux-x86_64.sh"
        checked = subprocess.run([sys.executable, "-c", self.embedded_python(installer), str(installer), "--check"],
                                 check=True, capture_output=True, text=True)
        self.assertEqual(json.loads(checked.stdout)["course_count"], 69)
        second = self.base/"compatibility-repeat"
        self.build(second, "--version", "2.2.0", "--revision", self.compatibility_commit)
        for path in output.iterdir():
            self.assertEqual(path.read_bytes(), (second/path.name).read_bytes())

    def test_compatibility_rejects_missing_proof_changed_baseline_and_untested_code(self):
        for revision, message in self.compatibility_rejections:
            with self.subTest(revision=revision), self.assertRaises(subprocess.CalledProcessError) as caught:
                self.build(self.base/"rejected-compatibility", "--version", "2.2.0", "--revision", revision)
            self.assertIn(message, caught.exception.stderr)

    def test_dirty_tracked_source_is_rejected(self):
        target = self.repo / "LICENSE"
        original = target.read_bytes()
        try:
            target.write_bytes(original+b"\nfixture dirty change\n")
            with self.assertRaises(subprocess.CalledProcessError) as caught:
                self.build(self.base / "dirty")
            self.assertIn("Commit tracked changes", caught.exception.stderr)
        finally:
            target.write_bytes(original)

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
        source = prefix / "versions/2.0.0/gazeboteach/__main__.py"
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
        self.assertFalse((prefix / "versions/2.0.0").exists())
        self.assertFalse(entry.exists())
        self.assertFalse((prefix / "current").is_symlink())
        self.assertEqual((other / "keep.txt").read_text(), "keep")
        self.assertEqual((data / "progress.json").read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
