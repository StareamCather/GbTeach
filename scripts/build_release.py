#!/usr/bin/env python3
"""Build an offline source application release from one immutable Git commit."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import tarfile


V2_COURSE_IDS = {"%02d" % value for value in range(1, 38)} | {"091", "092"}
V2_DOCUMENTS = ("V2_INSTALL.md", "RELEASE_NOTES_V2.md", "V2_CURRICULUM.md", "V2_DOCKER.md", "ENVIRONMENT_V2.md")
EXPANSION_COURSE_IDS = {
    "classic11-noetic": {"%02d" % value for value in range(1, 54)} | {"091", "092"},
    "harmonic-jazzy": {"01", "11", "22", "23", "24", "25", "26"},
}
EXPANSION_VERIFICATION = "EXPANSION_VERIFICATION.json"
EXPANSION_DOCUMENTS = ("V2_1_INSTALL.md", "EXPANSION_VALIDATION.md", EXPANSION_VERIFICATION)


def git(root, *arguments, **kwargs):
    return subprocess.check_output(["git", "-C", str(root), *arguments], **kwargs)


def excluded(name):
    parts = PurePosixPath(name).parts
    forbidden = {".git", ".deploy", ".runtime", "checkpoints", "__pycache__", ".pytest_cache", "build", "devel", "install", ".ssh", ".aws", ".gnupg"}
    leaf = parts[-1].lower()
    return bool(set(parts) & forbidden) or leaf.endswith((".pyc", ".pyo", ".log", ".bag", ".bundle", ".pem", ".key")) or leaf in {".env", "id_rsa", "id_ed25519", "id_ecdsa", "id_dsa"} or leaf.startswith(".env.")


def snapshot(root, revision):
    commit = git(root, "rev-parse", "--verify", revision+"^{commit}").decode().strip()
    rows = git(root, "ls-tree", "-rz", commit).split(b"\0")
    entries = []
    for row in rows:
        if not row:
            continue
        metadata, raw_name = row.split(b"\t", 1)
        mode, kind, oid = metadata.decode().split()
        name = raw_name.decode("utf-8")
        if excluded(name):
            continue
        path = PurePosixPath(name)
        if path.is_absolute() or ".." in path.parts or "\\" in name or ":" in name or name != path.as_posix():
            raise ValueError("Unsafe tracked path: "+name)
        if kind != "blob" or mode not in ("100644", "100755"):
            raise ValueError("Release accepts only tracked regular files, no symlinks/submodules: "+name)
        entries.append((name, oid, 0o755 if mode == "100755" or name.endswith(".sh") else 0o644))
    raw = subprocess.run(["git", "-C", str(root), "cat-file", "--batch"], input=("\n".join(oid for _, oid, _ in entries)+"\n").encode(), stdout=subprocess.PIPE, check=True).stdout
    stream, files = io.BytesIO(raw), {}
    for name, oid, mode in entries:
        actual_oid, kind, size = stream.readline().decode().strip().split()
        if actual_oid != oid or kind != "blob":
            raise ValueError("Git blob identity mismatch")
        content = stream.read(int(size))
        if len(content) != int(size) or stream.read(1) != b"\n":
            raise ValueError("Truncated Git blob")
        files[name] = (content, mode)
    return commit, files


def expansion_verification(root, commit, files, lessons, version):
    expected = {(profile, lesson_id) for profile, ids in EXPANSION_COURSE_IDS.items() for lesson_id in ids}
    actual = {(lesson["profile"], lesson["id"]) for lesson in lessons}
    if len(lessons) != len(expected) or actual != expected:
        raise RuntimeError("V2.1 requires exactly 55 Classic/Noetic and 7 Harmonic/Jazzy projects by (profile, id)")
    for name in ("resources/native_control.cc", "gazeboteach/native_control.py", "gazeboteach/backends.py",
                 *("docs/"+document for document in EXPANSION_DOCUMENTS)):
        if name not in files:
            raise RuntimeError("V2.1 tracked source payload is incomplete: "+name)
    content = files["docs/"+EXPANSION_VERIFICATION][0]
    report = json.loads(content.decode("utf-8"))
    tested = report.get("tested_code_commit") if isinstance(report, dict) else None
    if not isinstance(tested, str) or not re.fullmatch(r"[0-9a-f]{40}", tested):
        raise RuntimeError("V2.1 verification needs a full tested_code_commit SHA")
    if subprocess.run(["git", "-C", str(root), "merge-base", "--is-ancestor", tested, commit],
                      stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode != 0:
        raise RuntimeError("V2.1 tested_code_commit is not an ancestor of the selected release")
    changed = set(git(root, "diff", "--name-only", "-z", tested, commit).decode("utf-8").split("\0"))
    version_path = "gazeboteach/__init__.py"
    if version_path in changed:
        before = git(root, "show", tested+":"+version_path)
        after = files.get(version_path)
        expected, count = re.subn(rb'(?m)^__version__ = "2\.0\.0"$',
                                  b'__version__ = "'+version.encode("ascii")+b'"', before)
        before_mode = git(root, "ls-tree", tested, "--", version_path).split(b" ", 1)[0]
        if count == 1 and after == (expected, 0o644) and before_mode == b"100644":
            changed.remove(version_path)
    if any(path and path != "README.md" and not path.startswith(("docs/", "scripts/", "tests/")) for path in changed):
        raise RuntimeError("V2.1 release contains application or course changes after tested_code_commit")
    profiles = report.get("profiles")
    if not isinstance(profiles, dict) or set(profiles) != set(EXPANSION_COURSE_IDS):
        raise RuntimeError("V2.1 verification must describe exactly both course profiles")
    for profile, course_ids in EXPANSION_COURSE_IDS.items():
        evidence = profiles[profile]
        cases = evidence.get("cases") if isinstance(evidence, dict) else None
        if (not isinstance(evidence, dict) or evidence.get("passed") is not True
                or type(evidence.get("course_count")) is not int or evidence["course_count"] != len(course_ids)
                or not isinstance(cases, list) or len(cases) != len(course_ids)
                or {case.get("id") for case in cases if isinstance(case, dict) and case.get("passed") is True} != course_ids):
            raise RuntimeError("V2.1 verification lacks passed evidence for "+profile)
    return {"asset": EXPANSION_VERIFICATION, "sha256": hashlib.sha256(content).hexdigest(),
            "tested_code_commit": tested}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--revision", default="HEAD")
    parser.add_argument("--version", default="2.0.0")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    version = re.fullmatch(r"([0-9]+)\.([0-9]+)\.[0-9]+(?:-[A-Za-z0-9.-]+)?", args.version)
    if not version:
        raise ValueError("Use a path-safe semantic version")
    major, minor = map(int, version.groups())
    is_v2_legacy = (major, minor) == (2, 0)
    is_expansion = (major, minor) == (2, 1)
    if major != 1 and not (is_v2_legacy or is_expansion):
        raise ValueError("Supported release lines are 1.x, 2.0.x and 2.1.x")
    root = args.repo.resolve()
    if git(root, "status", "--porcelain", "--untracked-files=no").strip():
        raise RuntimeError("Commit tracked changes before building a release; payload bytes come from the selected Git commit")
    commit, files = snapshot(root, args.revision)
    for required in ("launch.sh", "gazeboteach/__main__.py", "scripts/install_payload.sh", "LICENSE"):
        if required not in files:
            raise RuntimeError("Commit release scripts/application first; missing tracked payload: "+required)
    lessons = []
    for name, (content, _) in files.items():
        if name.startswith("courses/") and name.endswith("/lesson.json"):
            item = json.loads(content.decode("utf-8"))
            lessons.append({key: item[key] for key in ("id", "title", "profile")})
    verification = None
    documents = ()
    if is_v2_legacy:
        documents = V2_DOCUMENTS
        if len(lessons) != 39 or {lesson["id"] for lesson in lessons} != V2_COURSE_IDS:
            raise RuntimeError("V2 requires exactly L01-L37 plus 091/092: 39 distinct projects")
        for required in ("resources/native_control.cc", "gazeboteach/native_control.py", "docs/NATIVE_CONTROL.md", *("docs/"+name for name in V2_DOCUMENTS)):
            if required not in files:
                raise RuntimeError("V2 tracked source payload is incomplete: "+required)
        if any(lesson["profile"] != "classic11-noetic" for lesson in lessons):
            raise RuntimeError("V2 projects must declare the Classic 11 / ROS Noetic profile")
    elif is_expansion:
        documents = EXPANSION_DOCUMENTS
        verification = expansion_verification(root, commit, files, lessons, args.version)
    elif len(lessons) < 26:
        raise RuntimeError("V1 requires the 26 delivered projects (L01–L24 plus two pendulum additions)")
    manifest = {"application": "GazeboTeach", "schema_version": 1, "version": args.version,
        "source_commit": commit, "source_tree": git(root, "rev-parse", commit+"^{tree}").decode().strip(),
        "tested_platform": {"os": "Ubuntu", "version": "20.04", "architecture": "x86_64", "gazebo": "Classic 11", "ros": "Noetic"},
        "course_count": len(lessons), "courses": sorted(
            lessons, key=(lambda value: (value["profile"], value["id"])) if is_expansion else (lambda value: value["id"])),
        "files": {name: {"size": len(content), "sha256": hashlib.sha256(content).hexdigest(), "mode": mode}
            for name, (content, mode) in sorted(files.items())},
        "exclusions": "Git history, checkpoints, caches, generated builds, private keys and host dependencies are not included."}
    if is_v2_legacy:
        manifest["tested_platform"].update({"python": "3.8", "qt": "5.12", "gazebo": "Classic 11.15.1", "gazebo_plugins": "2.9.3"})
        manifest["compatibility"] = {
            "native_control": "Optional persistent helper requires the exact Classic 11.15.1 SDK/runtime and C++ toolchain; unsupported capability is explicitly legacy-cli.",
            "cpp_courses": "Exact SDK/runtime and each lesson's declared tools, ROS packages and plugin requirements apply.",
            "docker": "Backend/configuration interface and manual setup guide only; automatic course container execution and container end-to-end validation are not provided.",
            "modern_gazebo_ros2": "Reading and diagnostics remain available; modern Gazebo and ROS 2 do not implement the Classic/Noetic course API.",
            "validation": "Consult the separately published V2_VERIFICATION.md/json; packaging does not certify experiment outcomes."}
    elif is_expansion:
        del manifest["tested_platform"]
        manifest["tested_platforms"] = {
            "classic11-noetic": {"os": "Ubuntu", "version": "20.04", "architecture": "x86_64",
                                  "gazebo": "Classic 11.15.1", "ros": "Noetic", "environment": "ikaros native"},
            "harmonic-jazzy": {"os": "Ubuntu", "version": "24.04", "architecture": "x86_64",
                               "gazebo": "Harmonic (Gazebo Sim 8.15.0)", "ros": "ROS 2 Jazzy",
                               "environment": "ikaros isolated GPU container"},
        }
        manifest["course_counts_by_profile"] = {profile: len(ids) for profile, ids in EXPANSION_COURSE_IDS.items()}
        manifest["verification"] = verification
        manifest["compatibility"] = {
            "profiles": "Select the matching simulator and ROS profile. Shared course numbers are scoped by profile.",
            "host_dependencies": "This source package does not install or bundle host dependencies. Run --diagnose and check the course requirements.",
            "classic_sdk": "Native Classic control and SDK courses require their declared exact runtime, SDK and toolchain.",
            "lio_dependencies": "A50-A53 build pinned external upstream sources separately. Their archives and built binaries are not bundled.",
            "docker": "The isolated modern test environment was prepared externally. The installer does not provision or run containers.",
            "validation": "Consult EXPANSION_VERIFICATION.json for the tested code commit and scope. Packaging checks file integrity.",
        }
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True)+"\n").encode("utf-8")
    epoch = int(git(root, "show", "-s", "--format=%ct", commit).decode().strip())
    folder = "GazeboTeach-"+args.version
    buffer = io.BytesIO()
    with gzip.GzipFile(fileobj=buffer, mode="wb", mtime=epoch, filename="") as compressed:
        with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
            for name, (content, mode) in sorted({**files, "release-manifest.json": (manifest_bytes, 0o644)}.items()):
                info = tarfile.TarInfo(folder+"/"+name)
                info.size, info.mode, info.mtime = len(content), mode, epoch
                info.uid, info.gid, info.uname, info.gname = 0, 0, "", ""
                archive.addfile(info, io.BytesIO(content))
    payload = buffer.getvalue()
    template = files["scripts/install_payload.sh"][0].decode("utf-8")
    values = {"@@VERSION@@": args.version, "@@PAYLOAD_SHA256@@": hashlib.sha256(payload).hexdigest(),
        "@@MANIFEST_SHA256@@": hashlib.sha256(manifest_bytes).hexdigest(), "@@PAYLOAD_LENGTH@@": str(len(payload))}
    for key, value in values.items():
        if key not in template:
            raise RuntimeError("Installer template lacks "+key)
        template = template.replace(key, value)
    installer = template.encode("utf-8")+b"\n__GAZEBOTEACH_PAYLOAD__\n"+payload
    output = (args.output or root/"build/releases"/args.version).resolve()
    output.mkdir(parents=True, exist_ok=True)
    base = folder+("-linux-x86_64" if is_expansion else "-ubuntu20.04-x86_64")
    assets = {base+".sh": installer, base+".tar.gz": payload, "release-manifest.json": manifest_bytes}
    assets.update({name: files["docs/"+name][0] for name in documents})
    sums = []
    for name, content in assets.items():
        destination = output/name
        temporary = destination.with_name(destination.name+".tmp")
        temporary.write_bytes(content)
        temporary.replace(destination)
        if name.endswith(".sh"):
            destination.chmod(0o755)
        sums.append(hashlib.sha256(content).hexdigest()+"  "+name)
    (output/"SHA256SUMS").write_bytes(("\n".join(sums)+"\n").encode("ascii"))
    print(json.dumps({"version": args.version, "source_commit": commit, "course_count": len(lessons),
        "file_count": len(files), "output": str(output), "assets": [*assets, "SHA256SUMS"]}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
