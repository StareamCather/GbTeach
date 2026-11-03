"""Timestamp, receipt and count invariants for A52's independent audits."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


COURSE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/52-point-lio'


def load(name):
    spec = importlib.util.spec_from_file_location('gazeboteach_a52_'+name, str(COURSE/(name+'.py')))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


audit = load('audit')
with mock.patch.dict(sys.modules, {'audit': audit}):
    adapter = load('adapter')


def pose(stamp, receipt, seq, x=0.):
    return {'stamp_ns': stamp, 'received_monotonic_ns': receipt, 'seq': seq,
            'position': [x, 0., 0.], 'orientation': [0., 0., 0., 1.]}


class PointLioAdapterTests(unittest.TestCase):
    def test_header_spacing_and_receipt_are_independent_and_loss_is_observed(self):
        messages = [pose(10_000_000, 1_000_000_000, 50), pose(10_000_000, 1_000_100_000, 51),
                    pose(11_000_000, 1_100_000_000, 54)]
        result = audit.odometry_frequency(messages, 6)
        self.assertEqual(result['distinct_header_times'], 2)
        self.assertEqual(result['duplicate_header_times'], 1)
        self.assertEqual(result['unobserved_publish_calls'], 3)
        self.assertEqual(result['missing_header_sequence_numbers_between_received_messages'], 2)
        self.assertAlmostEqual(result['header_unique_hz'], 1000.)
        self.assertAlmostEqual(result['receipt_average_hz'], 20.)

    def test_scan_end_matching_never_compares_nearby_but_distinct_columns(self):
        scan = [pose(110_000_000, 1, 0, 1.), pose(210_000_000, 2, 1, 3.)]
        point = [pose(110_000_050, 3, 0, 1.1), pose(209_500_000, 4, 1, 3.)]
        result = audit.compare_publication_modes(scan, point)
        self.assertEqual(result['matched_scan_ends'], 1)
        self.assertEqual(result['unmatched_scan_ends'], 1)
        self.assertAlmostEqual(result['position_difference_rmse_m'], .1)
        self.assertEqual(result['unmatched'][0]['gap_to_previous_point_ns'], 500_000)

    def test_native_counts_follow_the_selected_publication_rule(self):
        line = ('GT_POINT_LIO_SCAN start=12.000000000 end=12.099500000 groups=200 updates=185 '
                'publish_calls=%d saturated_axes=0 seconds=0.024000000\n')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'estimator.log'
            path.write_text(line % 185, encoding='ascii')
            self.assertEqual(adapter.parse_scan_diagnostics(path, True)[12_000_000_000]['groups'], 200)
            with self.assertRaisesRegex(RuntimeError, 'publication count'):
                adapter.parse_scan_diagnostics(path, False)
            path.write_text(line % 1, encoding='ascii')
            self.assertEqual(adapter.parse_scan_diagnostics(path, False)[12_000_000_000]['updates'], 185)


if __name__ == '__main__':
    unittest.main()
