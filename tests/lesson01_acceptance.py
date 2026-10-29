"""L01 real scenarios: baseline, changed geometry, rejected wrong inertia.

Each case starts the actual world from a private practice copy. Configuration
changes alone never count as success: the canonical verifier must observe a
real fall and restore the simulation's original pause state.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes


def paused(session):
    result = subprocess.run(['gz', 'topic', '-e', '/gazebo/default/world_stats', '-d', '1'],
                            env=session.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=5)
    values = re.findall(r'\bpaused:\s*(true|false)', result.stdout)
    if result.returncode or not values:
        raise AssertionError('No actual pause state from world_stats')
    return values[-1] == 'true'


def edit_box(world_path, side, initial_z, inertia_side=None):
    tree = ET.parse(str(world_path))
    block = tree.getroot().find("world/model[@name='falling_block']")
    block.find('pose').text = '0 0 {} 0 0 0'.format(initial_z)
    for location in ['link/collision/geometry/box/size', 'link/visual/geometry/box/size']:
        block.find(location).text = '{} {} {}'.format(side, side, side)
    inertia = (side if inertia_side is None else inertia_side) ** 2 / 6.0
    for name in ['ixx', 'iyy', 'izz']:
        block.find('link/inertial/inertia/' + name).text = str(inertia)
    tree.write(str(world_path), encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--report')
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    lesson = next(x for x in lessons if x.key == 'classic11-noetic:01')
    report = {'passed': False, 'scenarios': []}
    sessions = []
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l01-') as directory:
        base = Path(directory)
        try:
            standard = LessonSession(ROOT, lesson, storage=Storage(base / 'standard'))
            sessions.append(standard)
            standard.start(gui=False, terminals=False)
            standard.pause(True)
            assert standard.verify()['passed'] is True
            assert paused(standard) is True, 'Verifier must restore original paused state'
            report['scenarios'].append({'name': 'standard_0.5m_cube', 'expected_center_z': 0.25,
                                        'original_paused': True, 'pause_restored': True,
                                        'evidence': standard.log_path.read_text(encoding='utf-8', errors='replace')[-7000:]})
            standard.stop()

            storage = Storage(base / 'challenge')
            workspace = storage.prepare(lesson)
            edit_box(workspace / 'world.sdf', 0.8, 4.0)
            challenge = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(challenge)
            challenge.start(gui=False, terminals=False)
            assert challenge.verify()['passed'] is True
            assert paused(challenge) is False, 'Verifier must restore original running state'
            report['scenarios'].append({'name': 'challenge_0.8m_cube_from_4m',
                                        'expected_center_z': 0.4, 'original_paused': False,
                                        'pause_restored': True,
                                        'evidence': challenge.log_path.read_text(encoding='utf-8', errors='replace')[-7000:]})

            # A plausible positive inertia for the old 0.5 m cube is still wrong
            # for a uniform 0.8 m cube. It must not pass by simply settling.
            edit_box(workspace / 'world.sdf', 0.8, 4.0, inertia_side=0.5)
            rejected = challenge.verify()
            assert rejected['passed'] is False, 'Wrong size/inertia combination was accepted'
            assert paused(challenge) is False
            report['scenarios'].append({'name': 'wrong_inertia_rejected', 'passed': True,
                                        'evidence': challenge.log_path.read_text(encoding='utf-8', errors='replace')[-2000:]})
            report['passed'] = True
        except Exception as error:
            report['error'] = '{}: {}'.format(type(error).__name__, error)
            report['log_tails'] = [s.log_path.read_text(encoding='utf-8', errors='replace')[-16000:]
                                   for s in sessions if s.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason='L01 scenario test finished')
            leftovers = {s.token: live_token_processes(s.token) for s in sessions}
            report['leftovers'] = leftovers
            if any(leftovers.values()):
                report['passed'] = False
            if args.report:
                path = Path(args.report).resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
