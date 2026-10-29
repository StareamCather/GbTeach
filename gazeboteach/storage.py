"""Private practice copies, template fingerprints and profile-scoped records."""

import datetime
import hashlib
import json
import os
import shutil
import tempfile
import threading
from pathlib import Path

_METADATA = ".gazeboteach-template.json"
_IGNORED = {"__pycache__", ".git", "build", "devel", "install", ".pytest_cache"}


def now():
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def data_directory():
    override = os.environ.get("GAZEBOTEACH_DATA_DIR")
    return Path(override).expanduser().resolve() if override else Path.home() / ".local/share/GazeboTeach"


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def _files(folder):
    result = {}
    folder = Path(folder).resolve()
    if not folder.is_dir():
        return result
    for path in sorted(folder.rglob("*")):
        relative = path.relative_to(folder)
        if path.name == _METADATA or any(part in _IGNORED for part in relative.parts):
            continue
        if path.is_file():
            if folder not in path.resolve().parents:
                raise ValueError("Template symlink leaves the lesson directory")
            result[relative.as_posix()] = _hash(path)
    return result


def _fingerprint(files):
    return hashlib.sha256(json.dumps(files, sort_keys=True).encode("utf-8")).hexdigest()


def _workspace_file(folder, relative):
    target = (folder / relative).resolve()
    if folder.resolve() not in target.parents:
        raise ValueError("Template entry leaves the practice directory")
    return target


def _read_metadata(folder):
    try:
        data = json.loads((folder / _METADATA).read_text(encoding="utf-8"))
        return data if isinstance(data.get("files"), dict) else {}
    except (OSError, ValueError, TypeError, AttributeError):
        return {}


def _write_metadata(folder, data):
    handle, temporary = tempfile.mkstemp(prefix=".template-", dir=str(folder))
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
        os.replace(temporary, str(folder / _METADATA))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _copy_file(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".update-", dir=str(target.parent))
    os.close(descriptor)
    try:
        shutil.copy2(str(source), temporary)
        os.replace(temporary, str(target))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class Storage:
    def __init__(self, base=None):
        self.base = (Path(base) if base else data_directory()).expanduser().resolve()
        self.base.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()

    def workspace(self, lesson):
        return self.base / "workspaces" / lesson.profile / lesson.id

    def template_status(self, lesson):
        target = self.workspace(lesson)
        source = _files(lesson.folder)
        actual = _files(target)
        metadata = _read_metadata(target)
        baseline = metadata.get("files", {})
        comparison = baseline if metadata else source
        modified = sorted(name for name in set(comparison) | set(actual)
                          if comparison.get(name) != actual.get(name))
        conflicts = set(metadata.get("conflicts", []))
        for name in set(source) | set(baseline):
            current = actual.get(name)
            if (source.get(name) != baseline.get(name)
                    and current != baseline.get(name) and current != source.get(name)):
                conflicts.add(name)
        return {
            "source_fingerprint": _fingerprint(source),
            "workspace_fingerprint": _fingerprint(actual) if target.exists() else None,
            "updated": bool(target.exists() and metadata and
                            metadata.get("fingerprint") != _fingerprint(source)),
            "modified": modified if target.exists() else [],
            "conflicts": sorted(name for name in conflicts if name in modified),
            "prepared": target.exists(),
            "baseline_unknown": bool(target.exists() and not metadata),
        }

    def prepare(self, lesson):
        """Refresh only unchanged template files; never overwrite student edits."""
        with self.lock:
            target = self.workspace(lesson)
            source_files = _files(lesson.folder)
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                staging = Path(tempfile.mkdtemp(prefix=".prepare-", dir=str(target.parent))).resolve()
                try:
                    copied = staging / "lesson"
                    copied.mkdir()
                    for relative in source_files:
                        _copy_file(lesson.folder / relative, _workspace_file(copied, relative))
                    _write_metadata(copied, {"files": source_files, "fingerprint": _fingerprint(source_files),
                                             "conflicts": [], "prepared_at": now()})
                    os.replace(str(copied), str(target))
                finally:
                    # This exact staging directory was created beneath the intended workspace.
                    if target.parent.resolve() in staging.parents:
                        shutil.rmtree(str(staging), ignore_errors=True)
                return target
            metadata = _read_metadata(target)
            baseline = metadata.get("files", {})
            conflicts = set(metadata.get("conflicts", []))
            changed = []
            for relative, source_hash in source_files.items():
                dest = _workspace_file(target, relative)
                current_hash = _hash(dest)
                old_hash = baseline.get(relative)
                if current_hash == source_hash:
                    conflicts.discard(relative)
                    continue
                if ((relative not in baseline and current_hash is None)
                        or (relative in baseline and current_hash == old_hash)):
                    _copy_file(lesson.folder / relative, dest)
                    changed.append(relative)
                    conflicts.discard(relative)
                elif source_hash != old_hash:
                    conflicts.add(relative)
            for relative in set(baseline) - set(source_files):
                dest = _workspace_file(target, relative)
                current_hash = _hash(dest)
                if current_hash == baseline[relative] and current_hash is not None:
                    dest.unlink()
                    changed.append(relative)
                    conflicts.discard(relative)
                elif current_hash is not None:
                    conflicts.add(relative)
            _write_metadata(target, {
                "files": source_files, "fingerprint": _fingerprint(source_files),
                "conflicts": sorted(conflicts), "prepared_at": now(), "updated_files": sorted(changed),
            })
            return target

    def restore(self, lesson):
        with self.lock:
            target = self.workspace(lesson)
            backup = None
            if target.exists():
                stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                backup = self.base / "backups" / (lesson.profile + "-" + lesson.id + "-" + stamp)
                backup.parent.mkdir(parents=True, exist_ok=True)
                target.rename(backup)
            try:
                return self.prepare(lesson), backup
            except Exception:
                if backup and not target.exists():
                    backup.rename(target)
                raise

    def record(self, lesson_id, event, profile="classic11-noetic", **details):
        if hasattr(lesson_id, "profile"):
            profile, lesson_id = lesson_id.profile, lesson_id.id
        lesson_id = str(lesson_id)
        if ":" in lesson_id:
            profile, lesson_id = lesson_id.split(":", 1)
        entry = {"time": now(), "lesson": lesson_id, "profile": profile,
                 "key": profile + ":" + lesson_id, "event": event, "details": details}
        with self.lock:
            with (self.base / "experiments.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(entry, ensure_ascii=False) + "\n")
        return entry

    def records(self, limit=100):
        path = self.base / "experiments.jsonl"
        if not path.exists():
            return []
        result = []
        for line in path.read_text(encoding="utf-8").splitlines()[-limit:]:
            try:
                record = json.loads(line)
                if isinstance(record, dict):
                    result.append(record)
            except ValueError:
                pass
        return result

    def progress(self):
        result = {}
        for record in self.records(10000):
            if record.get("event") in ("read", "started", "verified"):
                key = record.get("key") or record.get("profile", "classic11-noetic") + ":" + str(record["lesson"])
                result.setdefault(key, set()).add(record["event"])
        return result
