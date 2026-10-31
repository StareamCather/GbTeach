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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--revision", default="HEAD")
    parser.add_argument("--version", default="1.0.0")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?", args.version):
        raise ValueError("Use a path-safe semantic version")
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
    if len(lessons) < 26:
        raise RuntimeError("V1 requires the 26 delivered projects (L01–L24 plus two pendulum additions)")
    manifest = {"application": "GazeboTeach", "schema_version": 1, "version": args.version,
        "source_commit": commit, "source_tree": git(root, "rev-parse", commit+"^{tree}").decode().strip(),
        "tested_platform": {"os": "Ubuntu", "version": "20.04", "architecture": "x86_64", "gazebo": "Classic 11", "ros": "Noetic"},
        "course_count": len(lessons), "courses": sorted(lessons, key=lambda value: value["id"]),
        "files": {name: {"size": len(content), "sha256": hashlib.sha256(content).hexdigest(), "mode": mode}
            for name, (content, mode) in sorted(files.items())},
        "exclusions": "Git history, checkpoints, caches, generated builds, private keys and host dependencies are not included."}
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
    base = folder+"-ubuntu20.04-x86_64"
    assets = {base+".sh": installer, base+".tar.gz": payload, "release-manifest.json": manifest_bytes}
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
