#!/bin/bash
# GazeboTeach self-extracting installer template. Render with build_release.py.
set -eu
command -v python3 >/dev/null 2>&1 || { printf '%s\n' 'Python 3 is required; no host packages were changed.' >&2; exit 2; }
exec python3 - "$0" "$@" <<'GAZEBOTEACH_INSTALLER_PYTHON'
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sys
import tarfile
import tempfile

VERSION = "@@VERSION@@"
PAYLOAD_SHA = "@@PAYLOAD_SHA256@@"
MANIFEST_SHA = "@@MANIFEST_SHA256@@"
PAYLOAD_LENGTH = int("@@PAYLOAD_LENGTH@@")
APP = "GazeboTeach"
WRAPPER = b'#!/bin/bash\n# GazeboTeach managed application entry, layout 1\nset -e\nexport PYTHONDONTWRITEBYTECODE=1\nPREFIX="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"\nexec "$PREFIX/current/launch.sh" "$@"\n'


def fail(message):
    raise RuntimeError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def safe_name(name):
    path = PurePosixPath(name)
    if not name or path.is_absolute() or ".." in path.parts or "\\" in name or ":" in name or path.as_posix() != name:
        fail("Unsafe archive path: "+name)
    return path


def verified_archive(filename):
    raw = Path(filename).read_bytes()
    marker = b"\n__GAZEBOTEACH_PAYLOAD__\n"
    if marker not in raw:
        fail("No embedded payload; execute a built release installer")
    payload = raw.split(marker, 1)[1]
    if len(payload) != PAYLOAD_LENGTH or digest(payload) != PAYLOAD_SHA:
        fail("Embedded payload SHA256/length mismatch; nothing was installed")
    archive = tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz")
    members, seen = {}, set()
    root = "GazeboTeach-"+VERSION
    for member in archive.getmembers():
        path = safe_name(member.name)
        if not path.parts or path.parts[0] != root or len(path.parts) < 2 or member.type not in (tarfile.REGTYPE, tarfile.AREGTYPE):
            fail("Only regular files under the single release directory are accepted")
        name = PurePosixPath(*path.parts[1:]).as_posix()
        if name in seen:
            fail("Duplicate archive member: "+name)
        seen.add(name)
        members[name] = member
        if len(members) > 20000 or sum(value.size for value in members.values()) > 512*1024*1024:
            fail("Archive exceeds the bounded source-package size")
    if "release-manifest.json" not in members:
        fail("Release manifest missing")
    manifest_bytes = archive.extractfile(members["release-manifest.json"]).read()
    if digest(manifest_bytes) != MANIFEST_SHA:
        fail("Release manifest SHA256 mismatch")
    manifest = json.loads(manifest_bytes.decode("utf-8"))
    if manifest.get("application") != APP or manifest.get("version") != VERSION or manifest.get("schema_version") != 1:
        fail("Release manifest identity differs")
    if set(members) != set(manifest["files"]) | {"release-manifest.json"}:
        fail("Archive file set differs from release manifest")
    contents = {}
    for name, info in manifest["files"].items():
        safe_name(name)
        member = members[name]
        if member.size != info["size"] or member.mode != info["mode"] or info["mode"] not in (0o644, 0o755):
            fail("Archive size/mode differs: "+name)
        content = archive.extractfile(member).read()
        if digest(content) != info["sha256"]:
            fail("Archive file SHA256 mismatch: "+name)
        contents[name] = content
    archive.close()
    contents["release-manifest.json"] = manifest_bytes
    return manifest, contents


def checked_directory(path, prefix):
    if path.is_symlink() or prefix not in path.resolve().parents:
        fail("Managed directory escapes installation prefix: "+str(path))
    path.mkdir(parents=True, exist_ok=True)
    if path.stat().st_uid != os.getuid():
        fail("Managed directory is not owned by this user")


def verify_installed(target, manifest, contents):
    if target.is_symlink() or not target.is_dir():
        fail("Installed version is not a real directory")
    expected = set(contents)
    actual = set()
    for path in target.rglob("*"):
        if path.is_symlink():
            fail("Installed version contains an unexpected symlink")
        if path.is_file():
            name = path.relative_to(target).as_posix()
            if "__pycache__" in path.parts and path.suffix == ".pyc":
                continue
            actual.add(name)
    if actual != expected:
        fail("Installed version has user-added/missing files; refusing overwrite/removal")
    for name, expected_bytes in contents.items():
        if digest((target/name).read_bytes()) != digest(expected_bytes):
            fail("Installed version was modified; refusing overwrite/removal: "+name)


def atomic_bytes(path, content, mode=0o644):
    if path.is_symlink() or (path.exists() and path.read_bytes() != content):
        fail("Refusing to replace an unrelated/modified managed file: "+str(path))
    descriptor, temporary_name = tempfile.mkstemp(prefix=".managed-", dir=str(path.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
        temporary.chmod(mode)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()


def current_target(prefix):
    link = prefix/"current"
    if not link.exists() and not link.is_symlink():
        return None
    if not link.is_symlink():
        fail("current is not a managed symlink")
    target = link.resolve()
    if target.parent != (prefix/"versions").resolve() or target.is_symlink():
        fail("current points outside the version directory")
    return target


def set_current(prefix, target):
    temporary = prefix/(".current-"+str(os.getpid()))
    try:
        temporary.symlink_to("versions/"+target.name)
        os.replace(str(temporary), str(prefix/"current"))
    finally:
        if temporary.is_symlink():
            temporary.unlink()


def desktop_bytes(prefix):
    command = str(prefix/"bin/gazeboteach").replace("%", "%%")
    for character in ("\\", '"', "`", "$"):
        command = command.replace(character, "\\"+character)
    return ('# GazeboTeach managed desktop entry, layout 1\n[Desktop Entry]\nType=Application\nName=GazeboTeach\nComment=Gazebo Classic 11 / ROS Noetic teaching projects\nExec="'+command+'"\nTerminal=false\nCategories=Education;Science;\n').encode("utf-8")


def main():
    parser = argparse.ArgumentParser(description="GazeboTeach "+VERSION+" source application installer; never runs apt/sudo or deletes personal learning data.")
    parser.add_argument("--prefix", default=str(Path.home()/".local/opt/GazeboTeach"))
    parser.add_argument("--check", action="store_true", help="Verify embedded archive, paths and every file without installing")
    parser.add_argument("--desktop", action="store_true", help="Create an optional desktop file under PREFIX/share/applications only")
    parser.add_argument("--uninstall", action="store_true", help="Remove only this exact unmodified installed version, preserving user data and other versions")
    args = parser.parse_args(sys.argv[2:])
    manifest, contents = verified_archive(sys.argv[1])
    if args.check:
        print(json.dumps({"checked": True, "version": VERSION, "source_commit": manifest["source_commit"], "course_count": manifest["course_count"], "file_count": len(manifest["files"])}))
        return
    if platform.system() != "Linux":
        fail("Installation requires Linux; --check can validate this source package without installing")
    requested = Path(args.prefix).expanduser().absolute()
    if requested.is_symlink():
        fail("Installation prefix itself must not be a symlink")
    prefix = requested.resolve()
    if prefix == Path("/") or prefix == Path.home().resolve():
        fail("Choose a dedicated private application directory as --prefix")
    storage = Path(os.environ.get("GAZEBOTEACH_DATA_DIR", str(Path.home()/".local/share/GazeboTeach"))).expanduser().resolve()
    if prefix == storage or storage in prefix.parents or (prefix/"versions") == storage or (prefix/"versions") in storage.parents:
        fail("Installation prefix/version directory overlaps personal learning data")
    marker = prefix/".gazeboteach-install-root.json"
    marker_bytes = (json.dumps({"application": APP, "layout": 1}, sort_keys=True)+"\n").encode()
    if prefix.exists():
        if prefix.stat().st_uid != os.getuid() or (any(prefix.iterdir()) and (not marker.is_file() or marker.is_symlink() or marker.read_bytes() != marker_bytes)):
            fail("Nonempty --prefix is not an owned GazeboTeach installation")
    else:
        if args.uninstall:
            fail("Installation prefix does not exist")
        prefix.mkdir(parents=True)
    import fcntl
    if (prefix/".install.lock").is_symlink():
        fail("Installer lock must not be a symlink")
    with (prefix/".install.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        atomic_bytes(marker, marker_bytes)
        versions, binary = prefix/"versions", prefix/"bin"
        checked_directory(versions, prefix)
        checked_directory(binary, prefix)
        target, entry = versions/VERSION, binary/"gazeboteach"
        current = current_target(prefix)
        if args.uninstall:
            verify_installed(target, manifest, contents)
            if target.resolve().parent != versions.resolve():
                fail("Version removal leaves the intended directory")
            shutil.rmtree(str(target))
            if current == target:
                (prefix/"current").unlink()
                if entry.is_file() and not entry.is_symlink() and entry.read_bytes() == WRAPPER:
                    entry.unlink()
                desktop = prefix/"share/applications/gazeboteach.desktop"
                if not (prefix/"share").is_symlink() and not desktop.parent.is_symlink() and prefix in desktop.resolve().parents and desktop.is_file() and not desktop.is_symlink() and desktop.read_bytes() == desktop_bytes(prefix):
                    desktop.unlink()
            print(json.dumps({"uninstalled": True, "version": VERSION, "prefix": str(prefix), "personal_data_preserved": str(storage), "other_versions_preserved": True}))
            return
        if entry.exists() or entry.is_symlink():
            if entry.is_symlink() or entry.read_bytes() != WRAPPER:
                fail("Existing application entry was modified/unrelated")
        if target.exists() or target.is_symlink():
            verify_installed(target, manifest, contents)
        else:
            stage = Path(tempfile.mkdtemp(prefix=".stage-", dir=str(versions)))
            try:
                for name, content in contents.items():
                    destination = stage/name
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_bytes(content)
                    destination.chmod(manifest["files"].get(name, {"mode": 0o644})["mode"])
                verify_installed(stage, manifest, contents)
                os.replace(str(stage), str(target))
            finally:
                if stage.exists() and stage.resolve().parent == versions.resolve():
                    shutil.rmtree(str(stage))
        atomic_bytes(entry, WRAPPER, 0o755)
        if args.desktop:
            folder = prefix/"share/applications"
            checked_directory(prefix/"share", prefix)
            checked_directory(folder, prefix)
            atomic_bytes(folder/"gazeboteach.desktop", desktop_bytes(prefix))
        set_current(prefix, target)
        warnings = []
        release = Path("/etc/os-release").read_text() if Path("/etc/os-release").exists() else ""
        if not re.search(r'^ID=\"?ubuntu\"?$', release, re.M) or not re.search(r'^VERSION_ID=\"?20\.04\"?$', release, re.M) or platform.machine() not in ("x86_64", "amd64"):
            warnings.append("This host is outside the tested Ubuntu 20.04 x86_64 platform. Reading/diagnostics may work; Classic 11/Noetic experiments are not modern Gazebo/ROS 2 courses.")
        print(json.dumps({"installed": True, "version": VERSION, "source_commit": manifest["source_commit"], "prefix": str(prefix),
            "entry": str(entry), "previous_current": str(current) if current else None, "warnings": warnings,
            "diagnose": str(entry)+" --diagnose", "personal_data": str(storage),
            "note": "System Python/PyQt5/Gazebo/ROS/compiler dependencies are not bundled or installed. Other versions and personal exercises are preserved."}, ensure_ascii=False))


try:
    main()
except Exception as error:
    print("GazeboTeach installer: "+str(error), file=sys.stderr)
    sys.exit(2)
GAZEBOTEACH_INSTALLER_PYTHON
exit 2
