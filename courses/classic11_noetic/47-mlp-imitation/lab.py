#!/usr/bin/env python3
"""Actual expert episodes -> supervised MLP -> actual unseen-target rollouts."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import uuid

import numpy as np
from network import MLP
from plant import NORMALIZATION, Plant, expert, features, require


INITIALS = [(-.8, 0), (.8, 0), (-.55, 1), (.55, -1), (-.3, -.8),
            (.3, .8), (-.7, -.2), (.7, .2), (-.15, 1.3), (.15, -1.3)]
TRAIN_EPISODES = list(range(8))
HELD_OUT_EPISODES = [8, 9]
UNSEEN_TARGETS = [-.3, .3]


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_learner():
    # The trusted source verifier must still use the PERSONAL exercise file.
    path = Path('learner.py').resolve()
    spec = importlib.util.spec_from_file_location('gazeboteach_personal_a47_learner', str(path))
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def split_episodes(rows):
    episodes = np.asarray([row['episode'] for row in rows])
    train = np.isin(episodes, TRAIN_EPISODES); held = np.isin(episodes, HELD_OUT_EPISODES)
    require(not np.any(train & held) and np.all(train | held), 'Episode split is incomplete or overlapping')
    require(set(episodes[train].tolist()) == set(TRAIN_EPISODES), 'Missing training episodes')
    require(set(episodes[held].tolist()) == set(HELD_OUT_EPISODES), 'Missing held-out episodes')
    return train, held


def observation_row(episode, step, state, target, action, after):
    return {'episode': episode, 'step': step, 'target_m': target,
            'x_m': state['x'], 'v_m_s': state['v'], 'features': features(state, target).tolist(),
            'expert_force_n': action, 'expert_action_normalized': action/NORMALIZATION['force_scale_n'],
            'before': after['transition']['before'], 'after': after['transition']['after'],
            'transition': after['transition'], 'independent_native_readback': after['independent_native_readback']}


def collect(plant, path):
    rows = []
    try:
        for episode, (x, v) in enumerate(INITIALS):
            state = plant.reset(x, v); target = 0.0
            for step in range(60):
                action = expert(state, target); after = plant.step(action, 20)
                rows.append(observation_row(episode, step, state, target, action, after))
                state = after
            print('Expert episode %d/10: %d real transitions' % (episode+1, len(rows)), flush=True)
    finally:
        write_json(path, {'schema': 2, 'normalization': NORMALIZATION,
                          'expert': {'law': 'clip(6*(target-x)-3*v, -8, 8) N', 'target_m': 0.0},
                          'train_episode_ids': TRAIN_EPISODES, 'held_out_episode_ids': HELD_OUT_EPISODES,
                          'rows': rows, 'complete': len(rows) == 600})
    return rows


def rollout(plant, policy, path):
    trials = []
    try:
        for target in UNSEEN_TARGETS:
            state = plant.reset(0.0, 0.0); rows = []
            trial = {'initial_x_m': 0.0, 'initial_v_m_s': 0.0, 'target_m': target,
                     'actual_initial_state': state['native'], 'reset_native_reply': state['native_reply'],
                     'setting': 'unseen_absolute_target_with_translation_prior', 'trace': rows}
            trials.append(trial)
            for step in range(170):
                inputs = features(state, target)
                normalized = float(policy(inputs))
                require(np.isfinite(normalized), 'MLP produced a non-finite action')
                unclipped = NORMALIZATION['force_scale_n']*normalized
                action = float(np.clip(unclipped, -8, 8))
                after = plant.step(action, 20)
                rows.append({'step': step, 'target_m': target, 'before': after['transition']['before'],
                             'features': inputs.tolist(), 'network_output_normalized': normalized,
                             'unclipped_network_force_n': unclipped, 'actual_force_n': action,
                             'after': after['transition']['after'], 'transition': after['transition'],
                             'independent_native_readback': after['independent_native_readback']})
                state = after
            tail = rows[-30:]
            trial['terminal_error_rms_m'] = float(np.sqrt(np.mean([(row['after']['x_m']-target)**2 for row in tail])))
            trial['terminal_velocity_rms_m_s'] = float(np.sqrt(np.mean([row['after']['v_m_s']**2 for row in tail])))
            trial['peak_abs_x_m'] = float(max(abs(row['after']['x_m']) for row in rows))
            trial['actual_position_change_m'] = state['x']
            print('MLP target %.2f m: terminal error RMS %.5f m' % (target, trial['terminal_error_rms_m']), flush=True)
    finally:
        write_json(path, {'schema': 2, 'normalization': NORMALIZATION, 'trials': trials})
    return trials


def train_and_evaluate(student=False):
    learner = load_learner() if student else None
    if learner: check_student(learner)
    directory = Path('results')/('run-'+uuid.uuid4().hex); directory.mkdir(parents=True)
    result = {'schema': 2, 'passed': False, 'algorithm': 'behavior_cloning_numpy_tanh_mlp',
              'assignment_pass': False, 'execution_mode': 'student' if student else 'reference_demo',
              'run_directory': str(directory), 'normalization': NORMALIZATION,
              'split': {'unit': 'whole_episode', 'train_episode_ids': TRAIN_EPISODES,
                        'held_out_episode_ids': HELD_OUT_EPISODES, 'training_target_m': 0.0},
              'generalization': {'unseen_target_m': UNSEEN_TARGETS, 'initial_x_m': 0.0,
                                 'claim': 'Translation-structured unseen absolute goals, not arbitrary out-of-distribution dynamics'},
              'physics': 'Real Gazebo ODE joint only; Python has no dynamics integration or expert fallback in evaluation',
              'source': {'lab_sha256': file_hash(__file__), 'network_sha256': file_hash(Path(__file__).with_name('network.py'))},
              'data': str(directory/'expert.json'), 'policy': str(directory/'imitation-policy.npz'),
              'traces': str(directory/'rollouts.json'), 'report': str(directory/'report.json')}
    plant = None; started = time.monotonic()
    if student: result['source']['personal_learner_sha256'] = file_hash('learner.py')
    try:
        with Plant() as plant:
            result['native_endpoint'] = plant.endpoint
            result['build'] = plant.build_metadata
            result['native_initial_state'] = plant.begin['after']
            rows = collect(plant, directory/'expert.json')
            result['expert_data_sha256'] = file_hash(directory/'expert.json')
            X = np.asarray([row['features'] for row in rows]); y = np.asarray([row['expert_action_normalized'] for row in rows])
            train, held = split_episodes(rows)
            result.update(training_rows=int(train.sum()), held_out_episode_rows=int(held.sum()))
            model = MLP(); history = []; force_scale = NORMALIZATION['force_scale_n']
            forward = (lambda inputs: np.asarray(learner.forward(inputs, model.W1, model.b1, model.W2, model.b2))) if learner else model.forward
            before = float(np.mean((forward(X[held])-y[held])**2))*force_scale**2
            for epoch in range(2600):
                prediction = forward(X[train])
                derivative = learner.mse_derivative(prediction, y[train]) if learner else 2*(prediction-y[train])
                norm = model.backward(X[train], derivative, .025)
                if epoch % 200 == 0 or epoch == 2599:
                    loss = float(np.mean((prediction-y[train])**2))
                    history.append({'epoch': epoch, 'training_mse_normalized': loss, 'gradient_norm_before_clip': float(norm)})
                    print('Training %d/2600: normalized MSE %.7f' % (epoch+1, loss), flush=True)
            after = float(np.mean((forward(X[held])-y[held])**2))*force_scale**2
            result.update(test_mse_before=before, test_mse_after=after, mse_unit='N^2', loss_history=history)
            model.save(directory/'imitation-policy.npz', normalization=NORMALIZATION)
            write_json(directory/'training.json', {'normalization': NORMALIZATION, 'training_rows': int(train.sum()),
                       'held_out_rows': int(held.sum()), 'train_episode_ids': TRAIN_EPISODES,
                       'held_out_episode_ids': HELD_OUT_EPISODES, 'test_mse_before_n2': before,
                       'test_mse_after_n2': after, 'history': history,
                       'hyperparameters': {'epochs': 2600, 'learning_rate': .025, 'gradient_clip_global_norm': 5,
                                           'seed': 7, 'architecture': [2, 12, 1], 'activation': 'tanh_then_linear'}})
            # Freeze the checkpoint, reload it, then use only the learner/reference MLP.
            saved = MLP().load(directory/'imitation-policy.npz')
            model.W1, model.b1, model.W2, model.b2 = saved.W1, saved.b1, saved.W2, saved.b2
            trials = rollout(plant, lambda inputs: forward(inputs)[0], directory/'rollouts.json')
            result.update(test_mse_before=before, test_mse_after=after, mse_unit='N^2',
                          loss_history=history, physical_trials=trials,
                          real_gazebo_transitions=plant.samples, native_final_state=plant.state()['native'],
                          policy_sha256=file_hash(directory/'imitation-policy.npz'))
            result['rollouts_sha256'] = file_hash(directory/'rollouts.json')
            require(before > 0 and after < before*.15 and after < .15,
                    'Held-out whole-episode force MSE did not improve enough: %.6f N^2' % after)
            require(all(trial['terminal_error_rms_m'] < .07 and trial['terminal_velocity_rms_m_s'] < .12
                        and trial['peak_abs_x_m'] < .95
                        and abs(trial['actual_position_change_m']) > .2 for trial in trials),
                    'Actual MLP closed-loop trials did not reach the unseen targets')
        require(plant.close_report['confirmed'], 'Native pause restoration was not confirmed')
        result['passed'] = True; result['assignment_pass'] = bool(student)
        return result
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        if plant is not None:
            result['real_gazebo_transitions'] = plant.samples
            result['auxiliary_reset_physics_steps'] = plant.auxiliary_reset_physics_steps
            result['reset_observations'] = plant.resets
            result['total_native_physics_steps'] = plant.samples*20+plant.auxiliary_reset_physics_steps
            result['readback_semantics'] = 'Separate state RPC reads the actual UpdateEnd cache version; root acceptance additionally checks independent WorldStatistics'
            result['pause_restoration'] = plant.close_report
        result['elapsed_wall_s'] = time.monotonic()-started
        write_json(directory/'report.json', result)
        write_json('results/imitation-report.json', result)
        write_json('build/verification.json', result)
        print(json.dumps({key: result.get(key) for key in ('passed', 'assignment_pass', 'real_gazebo_transitions',
                         'test_mse_after', 'report', 'error')}, ensure_ascii=False), flush=True)


def check_student(learner=None):
    learner = learner or load_learner()
    model = MLP(); X = np.array([[.2, -.3], [-.7, .4]])
    np.testing.assert_allclose(learner.forward(X, model.W1, model.b1, model.W2, model.b2), model.forward(X), atol=1e-10)
    np.testing.assert_allclose(learner.mse_derivative(np.array([2., -1.]), np.array([1., 3.])), np.array([2., -8.]))
    return {'student_algebra_checks': True, 'assignment_pass': False,
            'reason': 'Physical student rollout is still required; the exercise file is never filled automatically'}


def ready():
    with Plant() as plant:
        require(plant.source_sha256, 'Native source identity missing')
        result = {'ready': True, 'native': plant.begin['after'], 'endpoint': plant.endpoint,
                  'source_sha256': plant.source_sha256}
    require(plant.close_report['confirmed'], 'Readiness did not restore original pause')
    result['pause_restoration'] = plant.close_report
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['demo', 'student', 'student-check', 'ready'])
    args = parser.parse_args()
    if args.command == 'student-check': print(json.dumps(check_student(), ensure_ascii=False))
    elif args.command == 'ready': print(json.dumps(ready(), ensure_ascii=False))
    else: train_and_evaluate(args.command == 'student')
