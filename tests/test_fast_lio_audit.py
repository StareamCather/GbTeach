"""Tests for A50 timing validation and gauge-aware trajectory scoring."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/50-fast-lio/audit.py'
SPEC = importlib.util.spec_from_file_location('gazeboteach_a50_audit', str(SOURCE))
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def pose(t, x, y=0., heading=0.):
    return {'stamp_ns': t, 'position': [x, y, 0.],
            'orientation': [0., 0., math.sin(heading/2), math.cos(heading/2)]}


class FastLioAuditTests(unittest.TestCase):
    def test_interpolation_removes_initial_frame_rotation_but_not_position_drift(self):
        truth = [pose(10_000_000, 0.), pose(20_000_000, 1.), pose(30_000_000, 2.)]
        estimate = [pose(15_000_000, 2., 4.5, math.pi/2), pose(25_000_000, 2., 5.7, math.pi/2)]
        summary, records = audit.trajectory_metrics(truth, estimate)
        self.assertAlmostEqual(records[0]['position_error_m'], 0.)
        self.assertAlmostEqual(records[1]['position_error_m'], .2)
        self.assertAlmostEqual(summary['position_rmse_m'], math.sqrt(.02))
        self.assertAlmostEqual(summary['orientation_rmse_rad'], 0., places=6)

    def test_clock_gap_is_not_interpolated(self):
        truth = [pose(10_000_000, 0.), pose(60_000_000, 1.)]
        self.assertIsNone(audit.interpolate_truth([10_000_000, 60_000_000], truth, 30_000_000))

    def test_bad_point_offsets_fail_before_repackaging(self):
        rows = [[1., 0., 0., 0, index%16, index*1000] for index in range(32)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'native.jsonl'
            for invalid in (100_000_000, -1, 2_000):
                rows[-1][5] = invalid
                path.write_text(json.dumps({'type': 'scan', 'stamp_ns': 1, 'points': rows})+'\n', encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError, 'Point time'):
                    audit.native_records(path, {'scans': 1, 'truth_samples': 0})


if __name__ == '__main__':
    unittest.main()
