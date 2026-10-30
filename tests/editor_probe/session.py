"""Bounded real GUI session for editor QA, with private INI and owned cleanup.

Preload the save parent into the private GUI property tree: Classic's save
dialog otherwise writes the default home INI even with GAZEBO_GUI_INI_FILE.
The original home GUI INI hash must remain unchanged at session exit.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import Lesson, load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes


def fingerprint(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--io', required=True)
    parser.add_argument('--lesson')
    parser.add_argument('--seconds', type=float, default=1800)
    arguments = parser.parse_args()
    folder = Path(arguments.io).resolve()
    if ROOT / '.deploy' not in folder.parents:
        raise ValueError('QA control directory must stay within this repository .deploy')
    folder.mkdir(parents=True, exist_ok=False)
    if arguments.lesson:
        lesson = next(x for x in load_catalog(ROOT)[0] if x.id == arguments.lesson)
    else:
        fixture = ROOT / 'tests/fixtures/diagnostic'
        lesson = Lesson(fixture, json.loads((fixture / 'lesson.json').read_text()))
    lesson.data['gui_args'] = ['--gui-client-plugin',
                             str(ROOT / '.deploy/editor-probe-build/libGazeboTeachEditorProbe.so')]
    session = LessonSession(ROOT, lesson, Storage(folder / 'data'))
    session.env['GAZEBOTEACH_EDITOR_IO'] = str(folder)
    logs = folder / 'gazebo-log'
    logs.mkdir()
    session.env['GAZEBO_LOG_PATH'] = str(logs)
    ini = folder / 'gui.ini'
    ini.write_text('[model_paths]\nfilenames={}\n'.format(session.workspace / 'models'))
    session.env['GAZEBO_GUI_INI_FILE'] = str(ini)
    original_ini = Path.home() / '.gazebo/gui.ini'
    original_fingerprint = fingerprint(original_ini)
    record = {'workspace': str(session.workspace), 'token': session.token,
              'pid': os.getpid(), 'runtime_log': str(session.log_path),
              'home_gui_ini_before': original_fingerprint}
    try:
        session.start(gui=True, terminals=False)
        record['running'] = True
        (folder / 'session.json').write_text(json.dumps(record, indent=2))
        deadline = time.monotonic() + arguments.seconds
        while time.monotonic() < deadline and not (folder / 'stop').exists():
            if not session.running:
                raise RuntimeError('Owned Gazebo server stopped unexpectedly')
            time.sleep(.2)
    except BaseException as exc:
        record['error'] = '{}: {}'.format(type(exc).__name__, exc)
        raise
    finally:
        session.stop()
        record['running'] = False
        record['leftovers'] = live_token_processes(session.token)
        record['home_gui_ini_after'] = fingerprint(original_ini)
        record['home_gui_ini_unchanged'] = record['home_gui_ini_after'] == original_fingerprint
        (folder / 'session.json').write_text(json.dumps(record, indent=2))
        assert not record['leftovers'], record['leftovers']
        assert record['home_gui_ini_unchanged'], 'Editor test changed the user GUI INI'


if __name__ == '__main__':
    main()
