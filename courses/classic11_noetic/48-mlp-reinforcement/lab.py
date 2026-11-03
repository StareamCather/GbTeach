#!/usr/bin/env python3
"""Finite-horizon REINFORCE on the Gazebo ODE slider."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import uuid

import numpy as np
from network import MLP
from plant import Plant, require
from reference import mean_score, reward_to_go


GAMMA = .98
SIGMA = 1.4
TRAIN_HORIZON = 40
EVAL_HORIZON = 120
BATCH_EPISODES = 4
MODEL_SEED = 19
TRAIN_SEED = 29
EVALUATION = ((-.55, 2047), (.55, 2048))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_learner():
    path = Path('learner.py').resolve()
    spec = importlib.util.spec_from_file_location('gazeboteach_personal_a48_learner', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_student(module=None):
    module = module or load_learner()
    np.testing.assert_allclose(module.reward_to_go([1., 2., 3.], .5), [2.75, 3.5, 3.])
    np.testing.assert_allclose(module.mean_score(np.array([2., -1.]), np.array([1., 1.]), 2), [.25, -.5])
    return {'student_algebra_checks': True, 'assignment_pass': False,
            'reason': 'A real on-policy physical update is still required'}


def episode(plant, policy, path, *, phase, index, initial_x, noise_seed, horizon, checkpoint, update=None):
    rng = np.random.default_rng(noise_seed)
    record = {'phase': phase, 'episode': index, 'update': update, 'requested_initial_x_m': initial_x,
              'requested_initial_v_m_s': 0., 'noise_seed': noise_seed, 'horizon': horizon,
              'policy_checkpoint': str(checkpoint), 'rows': [], 'complete': False}
    try:
        state = plant.reset(initial_x)
        record['reset_native_reply'] = state['native_reply']
        record['actual_initial_state'] = state['native']
        for step in range(horizon):
            features = [-state['x'], state['v']]
            mean = float(policy.forward(features)[0])
            noise = float(rng.standard_normal())
            raw = mean+SIGMA*noise
            require(np.isfinite(raw), 'Gaussian policy produced a nonfinite action')
            force = float(8*np.tanh(raw/8))
            after = plant.step(force, 20)
            reward = -(after['x']**2+.03*after['v']**2+.001*force**2)
            record['rows'].append({'step': step, 'features': features, 'mean_n': mean,
                                   'noise_standard_normal': noise, 'raw_action_n': raw, 'applied_force_n': force,
                                   'reward': reward, 'before': after['transition']['before'],
                                   'after': after['transition']['after'], 'transition': after['transition'],
                                   'independent_native_readback': after['independent_native_readback']})
            require(abs(after['x']) <= 1.205, 'Rail boundary violated')
            state = after
        rewards = [row['reward'] for row in record['rows']]
        record.update(complete=True, undiscounted_return=float(sum(rewards)),
                      discounted_return=float(np.dot(GAMMA**np.arange(horizon), rewards)))
        return record
    finally:
        with path.open('a', encoding='utf-8') as output:
            output.write(json.dumps(record, allow_nan=False, separators=(',', ':'))+'\n')


def batch_derivative(batches, baseline, returns_function=reward_to_go, score_function=mean_score):
    returns = np.array([returns_function([row['reward'] for row in trial], GAMMA) for trial in batches])
    raw = np.array([[row['raw_action_n'] for row in trial] for trial in batches])
    means = np.array([[row['mean_n'] for row in trial] for trial in batches])
    advantage = returns-baseline
    scores = np.asarray(score_function(raw.ravel(), means.ravel(), SIGMA)).reshape(raw.shape)
    derivative = scores*advantage*(GAMMA**np.arange(raw.shape[1]))
    require(np.isfinite(derivative).all(), 'Nonfinite REINFORCE derivative')
    return derivative.ravel(), returns


def weights(model):
    return np.concatenate([getattr(model, key).ravel() for key in ('W1', 'b1', 'W2', 'b2')])


def evaluate(plant, model, path, phase, checkpoint):
    summaries = []
    for index, (x, seed) in enumerate(EVALUATION):
        record = episode(plant, model, path, phase=phase, index=index, initial_x=x,
                         noise_seed=seed, horizon=EVAL_HORIZON, checkpoint=checkpoint)
        summaries.append({key: value for key, value in record.items() if key != 'rows'})
    return summaries


def paired_evaluation(before, after):
    return [{'requested_initial_x_m': old['requested_initial_x_m'], 'noise_seed': old['noise_seed'],
             'discounted_return_before': old['discounted_return'], 'discounted_return_after': new['discounted_return'],
             'discounted_delta': new['discounted_return']-old['discounted_return'],
             'undiscounted_return_before': old['undiscounted_return'],
             'undiscounted_return_after': new['undiscounted_return'],
             'undiscounted_delta': new['undiscounted_return']-old['undiscounted_return']}
            for old, new in zip(before, after)]


def train_and_evaluate(student=False, updates=6):
    require(type(updates) is int and 1 <= updates <= 30, 'Choose 1..30 bounded updates')
    learner = load_learner() if student else None
    if learner:
        check_student(learner)
    returns_function = learner.reward_to_go if learner else reward_to_go
    score_function = learner.mean_score if learner else mean_score
    model = MLP(seed=MODEL_SEED)
    rng = np.random.default_rng(TRAIN_SEED)
    initial_weights = weights(model)
    baseline = np.zeros(TRAIN_HORIZON)
    directory = Path('results')/('run-'+uuid.uuid4().hex)
    directory.mkdir(parents=True)
    trajectory_path = directory/'on-policy-trajectories.jsonl'
    initial_path = directory/'policy-000.npz'
    report_path = directory/'report.json'
    model.save(initial_path)
    result = {'schema': 2, 'passed': False, 'assignment_pass': False,
              'execution_mode': 'student' if student else 'reference_demo', 'updates': updates,
              'algorithm': 'Gaussian REINFORCE with reward-to-go and previous-batch time baseline',
              'run_directory': str(directory), 'report': str(report_path), 'trajectory_log': str(trajectory_path),
              'initial_policy': str(initial_path), 'initial_policy_sha256': file_hash(initial_path),
              'history': [], 'evaluation': {}, 'convergence_claim': False,
              'config': {'gamma': GAMMA, 'sigma_n': SIGMA, 'training_horizon': TRAIN_HORIZON,
                         'evaluation_horizon': EVAL_HORIZON, 'batch_episodes': BATCH_EPISODES,
                         'model_seed': MODEL_SEED, 'training_seed': TRAIN_SEED, 'rng': 'NumPy default_rng PCG64',
                         'action_limit_n': 8, 'features': ['-x_m', 'v_m_s'], 'learning_rate': .035,
                         'reward': '-(x_next^2 + .03*v_next^2 + .001*force^2)', 'global_gradient_norm_clip': 5},
              'source': {'lab_sha256': file_hash(__file__), 'network_sha256': file_hash(Path(__file__).with_name('network.py'))}}
    if student:
        result['source']['personal_learner_sha256'] = file_hash('learner.py')
    plant = None
    started = time.monotonic()
    try:
        with Plant() as plant:
            result.update(native_endpoint=plant.endpoint, build=plant.build_metadata,
                          native_initial_state=plant.begin['after'])
            before = evaluate(plant, model, trajectory_path, 'eval_before', initial_path)
            result['evaluation']['before'] = before
            checkpoint = initial_path
            for update in range(1, updates+1):
                records = []
                for index in range(BATCH_EPISODES):
                    initial_x = (-1 if index % 2 else 1)*float(rng.uniform(.3, .7))
                    seed = int(rng.integers(0, 2**32))
                    records.append(episode(plant, model, trajectory_path, phase='train', index=index,
                                           initial_x=initial_x, noise_seed=seed, horizon=TRAIN_HORIZON,
                                           checkpoint=checkpoint, update=update))
                batches = [record['rows'] for record in records]
                inputs = np.asarray([row['features'] for trial in batches for row in trial])
                derivative, returns = batch_derivative(batches, baseline, returns_function, score_function)
                gradient_norm = float(model.backward(inputs, derivative, .035, ascent=True))
                require(np.isfinite(weights(model)).all() and np.isfinite(gradient_norm) and gradient_norm > 0,
                        'REINFORCE update must produce a finite nonzero gradient')
                next_checkpoint = directory/('policy-%03d.npz' % update)
                model.save(next_checkpoint)
                summary = {'update': update, 'policy_before': str(checkpoint), 'policy_after': str(next_checkpoint),
                           'baseline_used': baseline.tolist(), 'gradient_norm': gradient_norm,
                           'mean_discounted_return': float(np.mean([record['discounted_return'] for record in records])),
                           'transitions': len(inputs)}
                baseline = np.mean(returns, axis=0)
                summary['baseline_for_next_batch'] = baseline.tolist()
                result['history'].append(summary)
                checkpoint = next_checkpoint
                print(json.dumps({'progress': summary}, ensure_ascii=False), flush=True)
            final_model = MLP().load(checkpoint)
            result.update(final_policy=str(checkpoint), final_policy_sha256=file_hash(checkpoint))
            after = evaluate(plant, final_model, trajectory_path, 'eval_after', checkpoint)
            pairs = paired_evaluation(before, after)
            result['evaluation'].update(after=after, paired=pairs)
            mean_before = float(np.mean([trial['discounted_return'] for trial in before]))
            mean_after = float(np.mean([trial['discounted_return'] for trial in after]))
            change = float(np.linalg.norm(weights(final_model)-initial_weights))
            result.update(weight_change_l2=change, evaluation_initial_return=mean_before,
                          evaluation_final_return=mean_after, evaluation_improved=mean_after>mean_before,
                          evaluation_return_kind='mean discounted return of the two paired trials',
                          native_final_state=plant.state()['native'])
            require(change > 1e-6, 'Physical policy gradients did not change the MLP')
            require(plant.samples == updates*BATCH_EPISODES*TRAIN_HORIZON+4*EVAL_HORIZON,
                    'Native action transition count differs from the experiment schedule')
            require(plant.auxiliary_reset_physics_steps == updates*BATCH_EPISODES+4,
                    'Native reset count differs from the experiment schedule')
        require(plant.close_report['confirmed'], 'Native pause restoration was not confirmed')
        result.update(passed=True, assignment_pass=bool(student))
        return result
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        if plant is not None:
            result.update(real_gazebo_transitions=plant.samples, pause_restoration=plant.close_report,
                          auxiliary_reset_physics_steps=plant.auxiliary_reset_physics_steps,
                          total_native_physics_steps=plant.samples*20+plant.auxiliary_reset_physics_steps,
                          reset_observations=plant.resets)
        if trajectory_path.is_file():
            result['trajectory_sha256'] = file_hash(trajectory_path)
        result['elapsed_wall_s'] = time.monotonic()-started
        write_json(report_path, result)
        write_json('results/reinforce-report.json', result)
        write_json('build/verification.json', result)
        print(json.dumps({key: result.get(key) for key in ('passed', 'assignment_pass', 'real_gazebo_transitions',
                         'evaluation_improved', 'report', 'error')}, ensure_ascii=False), flush=True)


def ready():
    with Plant() as plant:
        result = {'ready': True, 'native': plant.begin['after'], 'endpoint': plant.endpoint,
                  'source_sha256': plant.source_sha256}
    require(plant.close_report['confirmed'], 'Readiness did not restore original pause')
    result['pause_restoration'] = plant.close_report
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['demo', 'student', 'student-check', 'ready'])
    parser.add_argument('--updates', type=int, default=6)
    args = parser.parse_args()
    if args.command == 'student-check':
        print(json.dumps(check_student(), ensure_ascii=False))
    elif args.command == 'ready':
        print(json.dumps(ready(), ensure_ascii=False))
    else:
        train_and_evaluate(args.command == 'student', args.updates)
