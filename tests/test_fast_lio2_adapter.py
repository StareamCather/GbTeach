"""Focused checks for A51 wire and timing pairing semantics."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock


COURSE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/51-fast-lio2'


def load(name):
    spec = importlib.util.spec_from_file_location('gazeboteach_a51_'+name, str(COURSE/(name+'.py')))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


with mock.patch.dict(sys.modules, {'audit': load('audit')}):
    adapter = load('adapter')


class FastLio2AdapterTests(unittest.TestCase):
    def test_velodyne_wire_points_keep_actual_ring_and_seconds(self):
        reader = types.SimpleNamespace(create_cloud=lambda header, fields, points: (header, fields, points))
        rospy = types.SimpleNamespace(Time=lambda seconds, ns: (seconds, ns))
        scan = {'stamp_ns': 6_112_345_678, 'points': [[1., -2., 3., 17, 8, 25_000_000]]}
        header, fields, points = adapter.velodyne_cloud(
            rospy, reader, types.SimpleNamespace, types.SimpleNamespace, scan)
        self.assertEqual(header.stamp, (6, 112_345_678))
        self.assertEqual([(field.name, field.offset, field.datatype) for field in fields],
                         [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7), ('time', 16, 7), ('ring', 20, 4)])
        self.assertEqual(points, [(1., -2., 3., 17., .025, 8)])

    def test_timing_matches_exact_scan_starts_even_when_modes_skip_different_warmup(self):
        def log(start, last, pre_time, map_time, missing_count_index=None):
            lines = []
            for index in range(start, last):
                stamp = '12.%09d' % (index*5_000_000)
                lines.extend(['GT_FAST_LIO2_PRE stamp=%s points=500 seconds=%.9f' % (stamp, pre_time),
                              'GT_FAST_LIO2_SCAN start=%s end=12.997000000 points=500 downsampled=400 effective=100 map_points=%d seconds=%.9f'
                              % (stamp, -1 if index == missing_count_index else 999, map_time)])
            return '\n'.join(lines)+'\n'
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'log.txt'
            path.write_text(log(0, 21, .0004, .002, missing_count_index=1), encoding='ascii')
            pre, proc = adapter.parse_diagnostics(path)
            path.write_text(log(1, 22, .001, .0015), encoding='ascii')
            other_pre, other_proc = adapter.parse_diagnostics(path)
        summary, rows = adapter.compare_timing({'preprocess': pre, 'processed': proc},
                                                {'preprocess': other_pre, 'processed': other_proc})
        self.assertEqual(summary['paired_scans'], 20)
        self.assertEqual(rows[0]['scan_start_ns'], 12_005_000_000)
        self.assertAlmostEqual(summary['direct']['sum_measured_ms']['median'], 2.4)
        self.assertAlmostEqual(summary['feature']['sum_measured_ms']['median'], 2.5)
        self.assertIsNone(rows[0]['direct']['map_points'])
        self.assertEqual(summary['direct']['map_points']['available_count'], 19)
        self.assertEqual(summary['direct']['map_points']['median'], 999)

    def test_other_negative_point_counts_are_rejected(self):
        pre = 'GT_FAST_LIO2_PRE stamp=12.000000000 points=500 seconds=0.000400000\n'
        scan = ('GT_FAST_LIO2_SCAN start=12.000000000 end=12.097000000 points=500 '
                'downsampled=400 effective=100 map_points=999 seconds=0.002000000\n')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'log.txt'
            for invalid in (scan.replace('map_points=999', 'map_points=-2'),
                            scan.replace('effective=100', 'effective=-1'),
                            scan.replace('downsampled=400', 'downsampled=-1')):
                path.write_text(pre+invalid, encoding='ascii')
                with self.assertRaisesRegex(RuntimeError, 'point count'):
                    adapter.parse_diagnostics(path)


if __name__ == '__main__':
    unittest.main()
