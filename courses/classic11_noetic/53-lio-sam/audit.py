"""Audit native rolling scan inputs and compare estimator poses to offline truth."""
from bisect import bisect_left
from decimal import Decimal
import json
import math

import numpy as np


SCAN_PERIOD_NS = 100_000_000
RING_COUNT = 16


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def vector(values, length, label):
    result = np.asarray(values, dtype=float)
    require(result.shape == (length,) and np.isfinite(result).all(), 'Invalid '+label)
    return result


def quaternion(values):
    result = vector(values, 4, 'quaternion')
    length = np.linalg.norm(result)
    require(abs(length-1.) < .01, 'Quaternion is not close to unit length')
    return result/length


def native_records(path, done):
    scans, truth = [], []
    with path.open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
                stamp = record['stamp_ns']
                require(type(stamp) is int and stamp > 0, 'Invalid simulator acquisition timestamp')
                if record['type'] == 'scan':
                    rows = record['points']
                    require(len(rows) >= 32, 'Rolling scan has too few hits')
                    previous = -1
                    rings = set()
                    for x, y, z, reflectivity, ring, offset in rows:
                        require(all(math.isfinite(value) for value in (x, y, z)), 'Non-finite lidar hit')
                        require(type(reflectivity) is int and 0 <= reflectivity <= 255, 'Invalid reflectivity')
                        require(type(ring) is int and 0 <= ring < RING_COUNT, 'Invalid laser ring')
                        require(type(offset) is int and previous <= offset < SCAN_PERIOD_NS and offset >= 0,
                                'Point time is invalid or out of acquisition order')
                        previous = offset
                        rings.add(ring)
                    require(len(rings) >= 2 and previous > rows[0][5], 'Scan is not a multi-ring rolling sweep')
                    if scans:
                        require(stamp > scans[-1]['end_ns'], 'Native scans overlap or time moved backwards')
                    record['end_ns'] = stamp+previous
                    scans.append(record)
                elif record['type'] == 'truth':
                    vector(record['position'], 3, 'truth position')
                    quaternion(record['orientation'])
                    require(not truth or stamp > truth[-1]['stamp_ns'], 'Non-increasing truth timestamps')
                    truth.append(record)
                else:
                    raise RuntimeError('Unknown capture record type')
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                raise RuntimeError('Native capture line %d: %s' % (line_number, error)) from error
    require(len(scans) == done['scans'] and len(truth) == done['truth_samples'], 'Capture marker counts disagree')
    require(len(scans) >= 20 and len(truth) >= 100, 'Incomplete lidar/truth capture')
    return scans, truth


def slerp(left, right, fraction):
    dot = float(np.dot(left, right))
    if dot < 0:
        right = -right
        dot = -dot
    if dot > .9995:
        mixed = left+fraction*(right-left)
        return mixed/np.linalg.norm(mixed)
    angle = np.arccos(np.clip(dot, -1., 1.))
    return (np.sin((1-fraction)*angle)*left+np.sin(fraction*angle)*right)/np.sin(angle)


def multiply(a, b):
    return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3], b[:3]), a[3]*b[3]-np.dot(a[:3], b[:3])]


def rotate(q, xyz):
    cross = 2*np.cross(q[:3], xyz)
    return xyz+q[3]*cross+np.cross(q[:3], cross)


def interpolate_truth(stamps, truth, stamp):
    right = bisect_left(stamps, stamp)
    if right < len(stamps) and stamps[right] == stamp:
        exact = truth[right]
        return vector(exact['position'], 3, 'truth position'), quaternion(exact['orientation'])
    if right == 0 or right == len(stamps) or stamps[right]-stamps[right-1] > 40_000_000:
        return None
    left_pose, right_pose = truth[right-1], truth[right]
    fraction = (stamp-stamps[right-1])/(stamps[right]-stamps[right-1])
    p0, p1 = np.asarray(left_pose['position']), np.asarray(right_pose['position'])
    return p0+fraction*(p1-p0), slerp(quaternion(left_pose['orientation']), quaternion(right_pose['orientation']), fraction)


def trajectory_metrics(truth, odometry):
    """Match at processed scan time, then remove one initial SE(3) gauge transform."""
    stamps = [row['stamp_ns'] for row in truth]
    matched = []
    previous = -1
    for row in odometry:
        stamp = row['stamp_ns']
        require(stamp > previous, 'Odometry timestamps are not strictly increasing')
        previous = stamp
        sample = interpolate_truth(stamps, truth, stamp)
        if sample is not None:
            matched.append((stamp, vector(row['position'], 3, 'estimated position'),
                            quaternion(row['orientation']), *sample))
    require(len(matched) >= 2, 'Insufficient time overlap between odometry and offline truth')
    _, estimate0, q_estimate0, truth0, q_truth0 = matched[0]
    rotation = multiply(q_estimate0, q_truth0*np.array([-1., -1., -1., 1.]))
    translation = estimate0-rotate(rotation, truth0)
    rows = []
    for stamp, estimate, q_estimate, target, q_target in matched:
        aligned_position = rotate(rotation, target)+translation
        aligned_orientation = multiply(rotation, q_target)
        angle = 2*np.arccos(np.clip(abs(np.dot(aligned_orientation, q_estimate)), 0., 1.))
        rows.append({'stamp_ns': stamp, 'position_estimate_m': estimate.tolist(),
                     'orientation_estimate_xyzw': q_estimate.tolist(),
                     'position_truth_aligned_m': aligned_position.tolist(),
                     'orientation_truth_aligned_xyzw': aligned_orientation.tolist(),
                     'position_error_m': float(np.linalg.norm(estimate-aligned_position)),
                     'orientation_error_rad': float(angle)})
    estimate_positions = np.asarray([pair[1] for pair in matched])
    truth_positions = np.asarray([pair[3] for pair in matched])
    summary = {'alignment': 'first_matched_pose_SE3_no_scale_fit', 'anchor_stamp_ns': matched[0][0],
               'alignment_rotation_xyzw': rotation.tolist(), 'alignment_translation_m': translation.tolist(),
               'matched': len(rows), 'unmatched': len(odometry)-len(rows),
               'position_rmse_m': float(np.sqrt(np.mean([row['position_error_m']**2 for row in rows]))),
               'orientation_rmse_rad': float(np.sqrt(np.mean([row['orientation_error_rad']**2 for row in rows]))),
               'final_position_error_m': rows[-1]['position_error_m'],
               'truth_path_length_m': float(np.sum(np.linalg.norm(np.diff(truth_positions, axis=0), axis=1))),
               'estimate_path_length_m': float(np.sum(np.linalg.norm(np.diff(estimate_positions, axis=0), axis=1))),
               'truth_max_displacement_m': float(np.max(np.linalg.norm(truth_positions-truth0, axis=1))),
               'estimate_max_displacement_m': float(np.max(np.linalg.norm(estimate_positions-estimate0, axis=1)))}
    return summary, rows


EXTRINSIC_T_IL = np.array([.04165, .02326, -.0284])


def lidar_truth(imu_truth):
    """Apply the surveyed IMU-to-LiDAR lever arm before scoring LiDAR poses."""
    return [dict(row, position=(vector(row['position'], 3, 'native IMU-link position')
                               +rotate(quaternion(row['orientation']), EXTRINSIC_T_IL)).tolist())
            for row in imu_truth]


def physical_revisit(truth, min_age_s=15.):
    positions = np.array([row['position'] for row in truth])
    eligible = [index for index, row in enumerate(truth)
                if row['stamp_ns']-truth[0]['stamp_ns'] >= int(min_age_s*1e9)]
    require(eligible, 'Capture does not contain the required revisit interval')
    distances = np.linalg.norm(positions-positions[0], axis=1)
    index = min(eligible, key=lambda idx: distances[idx])
    return {'initial_stamp_ns': truth[0]['stamp_ns'], 'revisit_stamp_ns': truth[index]['stamp_ns'],
            'revisit_separation_s': (truth[index]['stamp_ns']-truth[0]['stamp_ns'])/1e9,
            'revisit_distance_m': float(distances[index]),
            'physical_distance_travelled_m': float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())}


def parse_factor_log(path):
    markers = {'GT_LIO_SAM_GRAPH': 'graph', 'GT_LIO_SAM_LOOP': 'loop',
               'GT_LIO_SAM_IMU': 'imu', 'GT_LIO_SAM_IMU_RESET': 'imu_reset',
               'GT_LIO_SAM_PROJECTION': 'projection', 'GT_LIO_SAM_FEATURE': 'feature'}
    events = {kind: [] for kind in markers.values()}
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        matches = [marker for marker in markers if marker+' ' in line]
        if not matches:
            continue
        require(len(matches) == 1, 'Interleaved LIO-SAM factor diagnostic lines')
        marker = matches[0]
        kind = markers[marker]
        fields = dict(token.split('=', 1) for token in line.split(marker+' ', 1)[1].split() if '=' in token)
        time_key = 'start' if kind == 'projection' else 'stamp'
        row = {'stamp_ns': int(Decimal(fields[time_key])*1_000_000_000)}
        require(row['stamp_ns'] > 0, 'Invalid LIO-SAM diagnostic timestamp')
        if kind == 'graph':
            row.update({key: int(fields[key]) for key in ('key', 'prior', 'odom', 'gps', 'loop')})
            require(min(row[key] for key in ('key', 'prior', 'odom', 'gps', 'loop')) >= 0,
                    'Invalid mapping graph factor count')
        elif kind == 'loop':
            row.update({key: int(fields[key]) for key in ('from', 'to')})
            for key in ('from_stamp', 'to_stamp'):
                row[key+'_ns'] = int(Decimal(fields[key])*1_000_000_000)
            for key in ('pre_xyz', 'post_xyz', 'pre_to_xyz', 'post_to_xyz'):
                row[key] = vector([float(value) for value in fields[key].split(',')], 3, key).tolist()
            for key in ('fitness', 'pre_residual', 'post_residual'):
                row[key] = float(fields[key])
                require(math.isfinite(row[key]) and row[key] >= 0, 'Invalid loop diagnostic '+key)
            require(row['from'] > row['to'] >= 0 and row['stamp_ns'] >= row['from_stamp_ns'] > row['to_stamp_ns'] > 0,
                    'Invalid applied loop factor key/time order')
        elif kind == 'imu':
            row.update({key: int(fields[key]) for key in ('key', 'imu', 'bias', 'pose', 'samples', 'accepted')})
            row['integrated_s'] = float(fields['integrated'])
            require(row['accepted'] in (0, 1) and row['key'] >= 0 and min(row[key] for key in ('imu','bias','pose','samples')) >= 0
                    and math.isfinite(row['integrated_s']) and row['integrated_s'] >= 0,
                    'Invalid IMU preintegration factor diagnostic')
        elif kind == 'imu_reset':
            row.update(key=int(fields['key']), reason=fields['reason'])
            require(row['reason'] in ('initial', 'periodic', 'failure'), 'Unknown IMU optimizer reset reason')
        elif kind == 'projection':
            row.update({key: int(fields[key]) for key in ('input', 'projected', 'imu', 'deskew')})
            row['end_ns'] = int(Decimal(fields['end'])*1_000_000_000)
            require(row['end_ns'] > row['stamp_ns'] and 0 <= row['projected'] <= row['input']
                    and row['imu'] >= 0 and row['deskew'] in (0, 1), 'Invalid native projection diagnostic')
        else:
            row.update(corners=int(fields['corners']), surfaces=int(fields['surfaces']))
            require(row['corners'] >= 0 and row['surfaces'] >= 0, 'Invalid native feature count')
        events[kind].append(row)
    return events


def graph_summary(events, scans, loop_enabled):
    graph, loops, imu = events['graph'], events['loop'], events['imu']
    require(len(graph) >= 20 and len(imu) >= 20, 'Insufficient mapping and IMU graph updates')
    starts = {row['stamp_ns'] for row in scans}
    by_key = {row['key']: row['stamp_ns'] for row in graph}
    require([row['key'] for row in graph] == list(range(len(graph)))
            and all(row['stamp_ns'] in starts for row in graph),
            'Mapping factor keys or capture timestamps are inconsistent')
    totals = {key: sum(row[key] for row in graph) for key in ('prior', 'odom', 'gps', 'loop')}
    require(totals['prior'] == 1 and totals['odom'] == len(graph)-1 and totals['gps'] == 0,
            'Mapping graph must contain one prior, odometry factors and zero GPS factors')
    require(totals['loop'] == len(loops), 'Applied loop factors do not match actual graph batches')
    require(bool(loops) is loop_enabled, 'Loop-on requires an applied iSAM factor; loop-off requires none')
    for edge in loops:
        require(by_key.get(edge['from']) == edge['from_stamp_ns'] and by_key.get(edge['to']) == edge['to_stamp_ns']
                and edge['stamp_ns'] in starts, 'Applied loop keys do not resolve to recorded native scans')
        require(edge['from_stamp_ns']-edge['to_stamp_ns'] >= 12_000_000_000
                and edge['fitness'] <= .300001, 'Applied loop fails stated temporal/ICP criterion')
    accepted_imu = [row for row in imu if row['accepted'] and row['imu'] > 0
                    and row['samples'] > 0 and row['integrated_s'] > 0]
    require(len(accepted_imu) >= 20, 'Too few accepted real IMU preintegration factors')
    return {'mapping_batches': len(graph), 'mapping_factor_totals': totals,
            'first_applied_loop_stamp_ns': min((row['stamp_ns'] for row in loops), default=None),
            'imu_graph_optimization_events': len(imu), 'imu_accepted_factor_events': len(accepted_imu),
            'imu_factor_totals_accepted': {key: sum(row[key] for row in accepted_imu) for key in ('imu', 'bias', 'pose')},
            'imu_integrated_samples_accepted': sum(row['samples'] for row in accepted_imu),
            'imu_resets': events['imu_reset']}


def front_end_summary(events, scans):
    raw = {row['stamp_ns']: row for row in scans}
    projected, features = events['projection'], events['feature']
    require(len(projected) >= 50 and len(features) >= 50, 'Insufficient native projection/feature events')
    for row in projected:
        stamp = row['stamp_ns']
        require(stamp in raw and abs(row['end_ns']-raw[stamp]['end_ns']) <= 1000
                and row['input'] == len(raw[stamp]['points']), 'Projection event disagrees with acquired sweep')
    active = [row for row in projected if row['deskew'] and row['imu'] > 0 and row['projected'] > 0]
    require(len(active) >= 50, 'Insufficient actual IMU-backed deskew events')
    projection_stamps = {row['stamp_ns'] for row in projected}
    require(all(row['stamp_ns'] in projection_stamps for row in features), 'Features do not resolve to projected scans')
    require(sum(row['corners'] for row in features) > 0 and sum(row['surfaces'] for row in features) > 0,
            'Native feature extraction produced no corner or surface points')
    return {'projected_scans': len(projected), 'deskewed_scans': len(active), 'feature_scans': len(features),
            'median_corner_points': float(np.median([row['corners'] for row in features])),
            'median_surface_points': float(np.median([row['surfaces'] for row in features]))}


def same_stamps(left, right):
    before = {row['stamp_ns']: row for row in left}
    after = {row['stamp_ns']: row for row in right}
    shared = sorted(set(before)&set(after))
    return [before[stamp] for stamp in shared], [after[stamp] for stamp in shared]
