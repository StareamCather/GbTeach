#!/usr/bin/env python3
"""Finite real L31 stride-one regression, separate from the frozen formal test.

One independent 200-tick pulse first proves native time, iterations, gravity
and cube motion. Its 200 cumulative records must retain exactly ordinals
73..200 in the bounded 128-record window. After a real full-world reset, run
the actual course verifier and inspect its intermediate 200-tick evidence plus
a separate native wire observer spanning the verifier's final reset. Only
temporary Storage and the created session token are managed by this probe.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import first, parsed_model
from tests.lesson31_acceptance import (advance, checkpoint, events, hashes,
    lifecycle, mapped_library, native_scene, physics, plugin, position, require,
    run, save, sha, stamp, stats, stop_watcher, watcher, wire)

CONFIG = {'gravity_m_s2': 9.8, 'record_every_steps': 1}
STEP_NS, STEPS = 1000000, 200


def retained_window(state, native_world, initial_z):
    require(native_world['paused'] and native_world['iterations'] == STEPS
            and native_world['sim_time_ns'] == STEPS*STEP_NS,
            'Independent native world does not establish exactly 200 paused ticks')
    require(state['epoch_updates'] == state['epoch_record_count'] == STEPS,
            'Stride-one cumulative callback/record count is not 200')
    samples = state['recent_records']
    expected = list(range(73, 201))
    require(len(samples) == 128 and [v['epoch_updates'] for v in samples] == expected,
            'Bounded stride-one window is not exactly the last real ordinals 73..200')
    checks = []
    for sample, n in zip(samples, expected):
        require(sample['phase'] == 'world_update_begin_prephysics'
                and sample['world_iterations'] == n and sample['sim_time_ns'] == n*STEP_NS,
                'Bounded window shifted actual prephysics timestamps or world iteration labels')
        # A prediction is explicitly separate from each measured field. Native
        # Physics and the completed native Model response are checked outside.
        predicted = initial_z-9.8*.001*.001*(n-1)*n/2
        require(abs(sample['z_m']-predicted) < 3e-6
                and abs(sample['actual_gravity_z']+9.8) < 1e-10,
                'Actual retained prephysics sample disagrees with its labelled physical context')
        checks.append({'ordinal': n, 'actual_source_time_ns': sample['sim_time_ns'],
                       'measured_prephysics_z_m': sample['z_m'],
                       'separate_ode_prephysics_prediction_z_m': predicted})
    return {'passed': True, 'cumulative_epoch_records': state['epoch_record_count'],
            'retained_window_length': len(samples), 'retained_ordinals': expected,
            'measured_samples': samples, 'independent_clock_and_prediction_checks': checks,
            'distinction': 'Cumulative 200 records and retained 128 records are different quantities.'}


def full_reset(session, artifacts, label, state_before):
    sent = wire(session, artifacts, label+'-single-control', 'control', '--reset-all')
    require(first(first(parsed_model(sent['raw_protobuf']), 'reset'), 'all') == 'true',
            'Native full-reset bytes did not explicitly request all=true')
    deadline, polls = time.monotonic()+7, []
    while True:
        current = plugin(wire(session, artifacts, label+'-state-'+str(len(polls)), 'snapshot'))
        polls.append(current)
        if current['world_reset_event_count'] == state_before['world_reset_event_count']+1:
            break
        require(time.monotonic() < deadline, 'Real owned-world reset was not observed; reset was not replayed')
    native = stats(wire(session, artifacts, label+'-native-world', 'stats'))
    body = checkpoint(session, artifacts, label+'-native')[1]
    require(native['paused'] and native['sim_time_ns'] == native['iterations'] == 0,
            'Full reset did not return independent native time and iterations to zero')
    require(abs(position(body)[2]-2) < 1e-10 and current['epoch_updates'] == current['epoch_record_count'] == 0
            and current['recent_records'] == [] and current['updates'] == state_before['updates']
            and current['record_count'] == state_before['record_count']
            and current['reset_count'] == 0 and current['load_count'] == current['init_count'] == 1,
            'Full world reset confused bounded history, cumulative counters or process lifecycle')
    return {'passed': True, 'single_native_reset': sent, 'native_world_after': native,
            'native_cube_after': body, 'plugin_after': current, 'completion_polls': polls}


def independent_verifier_wire(rows, epoch, model_id, initial_z):
    """Keep source arrival order and preserve the actual reset regression."""
    clocks, poses, samples = [], [], []
    native_reset_regressions = 0
    for row in rows:
        source = row.get('source')
        if source == 'world_stats':
            value = stats(row)
            if clocks and value['sim_time_ns'] < clocks[-1]['sim_time_ns']:
                native_reset_regressions += 1
            clocks.append(value)
        elif source == 'pose':
            packet = parsed_model(row['raw_protobuf'])
            bodies = [p for p in packet.get('pose', []) if int(first(p, 'id', '-1')) == model_id]
            if not bodies:
                continue
            require(len(bodies) == 1 and str(first(bodies[0], 'name')).split('::')[-1] == 'falling_cube',
                    'Independent course-verifier pose ID/name changed')
            poses.append({'sim_time_ns': stamp(packet, 'time'),
                          'position_m': position({'pose': [bodies[0]]}), 'raw_wire': row})
        elif source == 'plugin':
            state = plugin(row)
            if state['epoch'] == epoch and state['epoch_updates'] == STEPS:
                samples.append(state)
    require(any(c['paused'] and c['sim_time_ns'] == STEPS*STEP_NS and c['iterations'] == STEPS for c in clocks),
            'Separate native observer did not witness verifier run at exactly 200 ticks')
    final_poses = [p for p in poses if p['sim_time_ns'] == STEPS*STEP_NS]
    require(final_poses, 'Separate native pose observer did not receive verifier final 200ms cube')
    predicted = initial_z-9.8*.001*.001*STEPS*(STEPS+1)/2
    require(all(abs(p['position_m'][2]-predicted) < 3e-6 for p in final_poses),
            'Separate native pose evidence does not establish verifier actual 200-step fall')
    require(samples, 'Separate Any wire observer did not receive verifier 200th actual sample')
    actual = samples[-1]
    target_world = next(c for c in clocks if c['paused'] and c['sim_time_ns'] == STEPS*STEP_NS and c['iterations'] == STEPS)
    window = retained_window(actual, target_world, initial_z)
    require(native_reset_regressions >= 1 and clocks[-1]['sim_time_ns'] == clocks[-1]['iterations'] == 0,
            'Separate observer did not preserve the verifier final real native clock reset')
    return {'passed': True, 'native_stats_in_original_arrival_order': clocks,
            'native_pose_packets_in_original_arrival_order': poses, 'actual_200th_plugin_wire_state': actual,
            'native_200ms_cube_poses': final_poses, 'native_clock_reset_regressions_preserved': native_reset_regressions,
            'window_crosscheck': window}


def actual_course_verifier(session, artifacts, before, model_id, initial_z):
    process, wire_path, stop = watcher(session, artifacts, 'course-verifier-independent-wire')
    report = artifacts/'course-verifier-original-report.json'
    log = artifacts/'course-verifier-stdout-stderr.log'
    try:
        code = run(session, ['python3', 'verify.py', '--report', str(report)], log, timeout=180)
        # The tool normally resets at the end. Give this independent subscriber
        # a bounded opportunity to receive the corresponding native zero clock.
        deadline = time.monotonic()+4
        while True:
            rows, _ = events(wire_path, live=True)
            clocks = [stats(r) for r in rows if r.get('source') == 'world_stats']
            if (clocks and any(c['sim_time_ns'] == STEPS*STEP_NS for c in clocks)
                    and clocks[-1]['sim_time_ns'] == clocks[-1]['iterations'] == 0):
                break
            require(time.monotonic() < deadline, 'Course verifier final 200-tick/reset native sequence was not captured')
            time.sleep(.04)
    finally:
        stop_watcher(process, stop)
        # Keep the course tool's original JSONL before temporary Storage exits,
        # even when its own assertions failed.
        if report.exists():
            original = json.loads(report.read_text(encoding='utf-8'))
            path = Path(original.get('raw_events_file', ''))
            if path.is_file():
                shutil.copy2(str(path), str(artifacts/'course-verifier-original-events.jsonl'))
    require(code == 0 and report.exists(), 'Actual course verifier failed; inspect its preserved report/log')
    original = json.loads(report.read_text(encoding='utf-8'))
    require(original.get('passed') is True, 'Actual course verifier did not return passed')
    phases = original.get('phases', [])
    require(len(phases) == 5, 'Actual course verifier lost intermediate measurements before its final reset')
    expected = [1, 10, 100, 199, 200]
    phase_checks = []
    for phase, n in zip(phases, expected):
        native = phase['after']
        actual_time = int(native['sim_time'].get('sec', 0))*1000000000+int(native['sim_time'].get('nsec', 0))
        require(native['paused'] and actual_time == n*STEP_NS and int(native['iterations']) == n,
                'Course tool intermediate native clock/iteration data do not establish its actual run')
        body_z = float(phase['native_cube']['pose']['position']['z'])
        g = float(phase['native_physics']['gravity']['z'])
        predicted = initial_z-9.8*.001*.001*n*(n+1)/2
        require(abs(g+9.8) < 1e-10 and abs(body_z-predicted) < 3e-6,
                'Course tool retained native physical measurements disagree with expected motion')
        phase_checks.append({'ticks': n, 'actual_native_sim_time_ns': actual_time,
                             'actual_native_cube_z_m': body_z, 'actual_native_gravity_z': g})
    rows, diagnostics = events(wire_path)
    separate = independent_verifier_wire(rows, before['epoch'], model_id, initial_z)
    tool_state = phases[-1]['state']
    wire_state = separate['actual_200th_plugin_wire_state']
    require(tool_state['recent_records'] == original['samples'] == wire_state['recent_records']
            and tool_state['epoch_record_count'] == wire_state['epoch_record_count'] == STEPS
            and tool_state['record_count'] == wire_state['record_count'] == before['record_count']+STEPS
            and tool_state['updates'] == wire_state['updates'] == before['updates']+STEPS,
            'Course verifier cumulative/window values differ from separately received actual Any bytes')
    final_world = stats(wire(session, artifacts, 'course-verifier-independent-final-world', 'stats'))
    final, final_body = checkpoint(session, artifacts, 'course-verifier-independent-final')
    require(final_world['paused'] and final_world['sim_time_ns'] == final_world['iterations'] == 0
            and abs(position(final_body)[2]-initial_z) < 1e-10 and final['epoch'] == before['epoch']+1
            and final['epoch_updates'] == final['epoch_record_count'] == 0 and final['recent_records'] == []
            and final['record_count'] == before['record_count']+STEPS and final['updates'] == before['updates']+STEPS
            and final['reset_count'] == 0, 'Independent final reset state differs from actual course verification effects')
    physics(session, artifacts, 'course-verifier-independent-final-physics', 9.8)
    archived_wire = artifacts/'course-verifier-original-events.jsonl'
    require(archived_wire.is_file(), 'Course verifier original internal raw wire file was not preserved')
    return {'passed': True, 'course_verifier_report': str(report), 'course_verifier_report_sha256': sha(report),
            'course_verifier_original_raw_jsonl': str(archived_wire), 'course_verifier_original_raw_sha256': sha(archived_wire),
            'course_verifier_log': str(log), 'retained_intermediate_native_checks': phase_checks,
            'separate_wire_crosscheck': separate, 'independent_raw_jsonl': str(wire_path),
            'independent_raw_jsonl_sha256': sha(wire_path), 'wire_diagnostics': diagnostics,
            'independent_final_world': final_world, 'independent_final_plugin': final,
            'observer_end': {'returncode': process.poll(), 'stop_file': str(stop), 'normal_stop_file_exit': True}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    output = args.report.expanduser().resolve()
    artifacts = output.with_name(output.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(v for v in lessons if v.id == '31' and v.profile == 'classic11-noetic')
    source, started, session = hashes(lesson.folder), time.monotonic(), None
    result = {'passed': False, 'parameters': CONFIG, 'test_algorithm': __doc__, 'source_sha256': source,
              'graphical_checks_performed': False}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l31-stride1-') as temporary:
        try:
            storage = Storage(Path(temporary)/'stride1')
            workspace = storage.prepare(lesson)
            save(workspace/'parameters.json', CONFIG)
            session = LessonSession(ROOT, lesson, storage=storage)
            session.start(gui=False, terminals=False)
            record = json.loads((workspace/'build/system_lab.json').read_text(encoding='utf-8'))
            require(record['source_sha256']['system_plugin.cc'] == source['system_plugin.cc']
                    and all(sha(workspace/p) == expected for p, expected in record['source_sha256'].items())
                    and sha(workspace/'parameters.json') == record['config_sha256'], 'Prepared actual source/config SHA differs')
            result['actual_process_mapping'] = mapped_library(session, artifacts, 'stride1', record)
            initial_world = stats(wire(session, artifacts, 'initial-native-world', 'stats'))
            initial, initial_body = checkpoint(session, artifacts, 'initial')
            require(initial_world['paused'] and initial_world['sim_time_ns'] == initial_world['iterations'] == 0,
                    'Fresh stride-one world did not start truly paused at zero')
            require(initial['updates'] == initial['record_count'] == 0 and initial['record_every_steps'] == 1,
                    'Fresh actual process state has wrong stride or previous physics')
            result['lifecycle'] = lifecycle(initial)
            result['initial_native_scene'] = native_scene(session, artifacts, 'initial-scene')
            physics(session, artifacts, 'initial-native-physics', 9.8)
            initial_z = position(initial_body)[2]
            require(abs(initial_z-2) < 1e-10, 'Fresh native cube height differs')
            process, path, stop = watcher(session, artifacts, 'independent-single-200-tick-wire')
            try:
                final_world, pulse = advance(session, artifacts, 'independent-single-200-tick', initial_world, STEPS)
                final, final_body = checkpoint(session, artifacts, 'independent-200-tick')
                actual_physics = physics(session, artifacts, 'independent-200-tick-physics', 9.8)
                expected_z = initial_z-9.8*.001*.001*STEPS*(STEPS+1)/2
                require(abs(position(final_body)[2]-expected_z) < 3e-6
                        and int(first(final_body, 'id')) == int(first(initial_body, 'id')),
                        'Independent native Model does not establish the actual 200-step fall')
                window = retained_window(final, final_world, initial_z)
                require(final['updates'] == final['record_count'] == STEPS, 'Fresh cumulative total differs from native tick count')
            finally:
                stop_watcher(process, stop)
            rows, diagnostics = events(path)
            require(any(v.get('source') == 'pose' for v in rows), 'Independent 200-tick observer did not retain native pose bytes')
            result['independent_200_tick_regression'] = {'passed': True, 'single_native_step_pulse': pulse,
                'actual_native_world': final_world, 'actual_native_cube': final_body, 'actual_native_physics': actual_physics,
                'actual_plugin_state': final, 'window_check': window, 'raw_jsonl': str(path),
                'raw_jsonl_sha256': sha(path), 'wire_diagnostics': diagnostics,
                'observer_end': {'returncode': process.poll(), 'stop_file': str(stop), 'normal_stop_file_exit': True}}
            save(artifacts/'independent-200-tick-analysis.json', result['independent_200_tick_regression'])
            reset = full_reset(session, artifacts, 'independent-reset-before-course-verifier', final)
            result['full_reset_before_course_verifier'] = reset
            result['actual_course_verifier_regression'] = actual_course_verifier(
                session, artifacts, reset['plugin_after'], int(first(initial_body, 'id')), initial_z)
            shutil.copy2(str(workspace/'build/system_lab.json'), str(artifacts/'prepared-system_lab.json'))
            shutil.copy2(str(workspace/'parameters.json'), str(artifacts/'original-personal-parameters.json'))
            result['passed'] = True
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            if session:
                try:
                    session.stop(reason='L31 isolated stride-one regression cleanup')
                    shutil.copy2(str(session.log_path), str(artifacts/'runtime.log'))
                    shutil.copy2(str(session.session_dir/'session.json'), str(artifacts/'session.json'))
                except Exception as error:
                    result['cleanup_errors'].append(repr(error))
                result['leftovers'] = {session.token: live_token_processes(session.token)}
            else:
                result['leftovers'] = {}
            result['source_templates_unchanged'] = hashes(lesson.folder) == source
            if result['cleanup_errors'] or any(result['leftovers'].values()) or not result['source_templates_unchanged']:
                result['passed'] = False
            result['elapsed_seconds'] = round(time.monotonic()-started, 3)
            save(output, result)
    print(json.dumps({k: v for k, v in result.items() if k in ('passed', 'error', 'elapsed_seconds', 'leftovers', 'source_templates_unchanged')},
                     ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
