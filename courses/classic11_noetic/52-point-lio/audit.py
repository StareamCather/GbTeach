"""Audit native rolling scan inputs and compare estimator poses to offline truth."""
from bisect import bisect_left
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


STAMP_TOLERANCE_NS = 1000


def gap_stats(stamps):
    gaps_ms = np.diff(stamps)/1e6
    return {'median_ms': float(np.median(gaps_ms)) if len(gaps_ms) else None,
            'p95_ms': float(np.percentile(gaps_ms, 95)) if len(gaps_ms) else None}


def odometry_frequency(rows, publication_calls):
    """Measure independent header and wall-receipt timelines, never subtract between clocks."""
    require(len(rows) >= 2, 'Too few received odometry messages')
    header = [row['stamp_ns'] for row in rows]
    receipt = [row['received_monotonic_ns'] for row in rows]
    sequences = [row['seq'] for row in rows]
    require(all(a <= b for a, b in zip(header, header[1:])), 'Odometry header timestamp regressed')
    require(all(a <= b for a, b in zip(receipt, receipt[1:])), 'Monotonic receipt timestamp regressed')
    unique = sorted(set(header))
    require(len(unique) >= 2 and receipt[-1] > receipt[0], 'Odometry timelines have insufficient span')
    require(publication_calls >= len(rows), 'More received odometry messages than mapper publish calls')
    sequence_steps = [(b-a) % (2**32) for a, b in zip(sequences, sequences[1:])]
    require(all(0 < step < 2**31 for step in sequence_steps), 'Odometry sequence regressed or repeated')
    return {'mapper_publish_calls': publication_calls, 'received_messages': len(rows),
            'unobserved_publish_calls': publication_calls-len(rows),
            'received_fraction_of_publish_calls': len(rows)/publication_calls,
            'missing_header_sequence_numbers_between_received_messages': sum(step-1 for step in sequence_steps),
            'distinct_header_times': len(unique), 'duplicate_header_times': len(rows)-len(unique),
            'first_header_ns': unique[0], 'last_header_ns': unique[-1],
            'header_unique_hz': (len(unique)-1)*1e9/(unique[-1]-unique[0]),
            'receipt_average_hz': (len(receipt)-1)*1e9/(receipt[-1]-receipt[0]),
            'unique_header_gaps': gap_stats(unique), 'wall_receipt_gaps': gap_stats(receipt),
            'drop_scope': 'Unobserved mapper publish calls may reflect transport/subscriber loss or late delivery. The difference alone does not identify where.'}


def matched_stamp(sorted_stamps, stamp):
    index = bisect_left(sorted_stamps, stamp)
    options = sorted_stamps[max(0, index-1):index+1]
    if not options:
        return None
    candidate = min(options, key=lambda time: abs(time-stamp))
    return candidate if abs(candidate-stamp) <= STAMP_TOLERANCE_NS else None


def compare_publication_modes(scan_rows, point_rows):
    # Check cross-mode difference only when both modes emitted essentially
    # the same sensor timestamp; keep earlier last-point gaps separately.
    by_stamp = {row['stamp_ns']: row for row in point_rows}
    stamps = sorted(by_stamp)
    matches, missing = [], []
    for row in scan_rows:
        stamp = row['stamp_ns']
        matched = matched_stamp(stamps, stamp)
        if matched is None:
            index = bisect_left(stamps, stamp)
            nearest_earlier = stamp-stamps[index-1] if index else None
            missing.append({'scan_end_ns': stamp, 'gap_to_previous_point_ns': nearest_earlier})
            continue
        other = by_stamp[matched]
        delta = np.linalg.norm(vector(row['position'], 3, 'scan estimate')
                               -vector(other['position'], 3, 'point estimate'))
        rotation_delta = 2*np.arccos(np.clip(abs(np.dot(quaternion(row['orientation']),
                                                      quaternion(other['orientation']))), 0., 1.))
        matches.append({'scan_end_ns': stamp, 'point_stamp_ns': matched, 'difference_ns': matched-stamp,
                        'position_difference_m': float(delta), 'orientation_difference_rad': float(rotation_delta)})
    return {'matched_scan_ends': len(matches), 'unmatched_scan_ends': len(missing),
            'tolerance_ns': STAMP_TOLERANCE_NS,
            'position_difference_rmse_m': (float(np.sqrt(np.mean([row['position_difference_m']**2 for row in matches])))
                                           if matches else None),
            'matched': matches, 'unmatched': missing}
