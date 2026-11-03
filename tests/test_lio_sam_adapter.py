"""LIO-SAM loop evidence and same-physical-time comparison."""
import importlib.util
from pathlib import Path
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/53-lio-sam/audit.py'
SPEC = importlib.util.spec_from_file_location('gazeboteach_a53_audit', str(SOURCE))
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class LioSamAuditTests(unittest.TestCase):
    def test_lever_arm_follows_native_imu_rotation(self):
        pose = {'stamp_ns': 1, 'position': [1., 2., 3.], 'orientation': [0., 0., 1., 0.]}
        shifted, = audit.lidar_truth([pose])
        self.assertAlmostEqual(shifted['position'][0], 1.-.04165)
        self.assertAlmostEqual(shifted['position'][1], 2.-.02326)
        self.assertAlmostEqual(shifted['position'][2], 3.-.0284)

    def test_factor_totals_require_corresponding_applied_loop_event(self):
        scans = [{'stamp_ns': (index+1)*1_000_000_000} for index in range(20)]
        mapping = ['GT_LIO_SAM_GRAPH stamp=%d.000000000 key=%d prior=%d odom=%d gps=0 loop=%d'
                   % (i+1, i, int(i==0), int(i>0), int(i==19)) for i in range(20)]
        imu = ['GT_LIO_SAM_IMU stamp=%d.000000000 key=%d imu=1 bias=1 pose=1 integrated=0.100000000 samples=20 accepted=1'
               % (i+1, i) for i in range(20)]
        edge = ('GT_LIO_SAM_LOOP stamp=20.000000000 from=18 to=0 from_stamp=19.000000000 '
                'to_stamp=1.000000000 fitness=0.02 pre_xyz=0.1,0,0 post_xyz=0.05,0,0 '
                'pre_to_xyz=0,0,0 post_to_xyz=0,0,0 pre_residual=0.1 post_residual=0.05')
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp)/'log.txt'
            log.write_text('\n'.join(mapping+imu), encoding='ascii')
            with self.assertRaisesRegex(RuntimeError, 'graph batches'):
                audit.graph_summary(audit.parse_factor_log(log), scans, True)
            log.write_text('\n'.join(mapping+imu+[edge]), encoding='ascii')
            result = audit.graph_summary(audit.parse_factor_log(log), scans, True)
            self.assertEqual(result['mapping_factor_totals'], {'prior': 1, 'odom': 19, 'gps': 0, 'loop': 1})
            self.assertEqual(result['first_applied_loop_stamp_ns'], 20_000_000_000)
            self.assertEqual(result['imu_integrated_samples_accepted'], 400)
            with self.assertRaisesRegex(RuntimeError, 'loop-off'):
                audit.graph_summary(audit.parse_factor_log(log), scans, False)

    def test_trajectory_comparison_uses_intersection_not_index_pairing(self):
        left = [{'stamp_ns': stamp} for stamp in (1, 2, 4, 7)]
        right = [{'stamp_ns': stamp} for stamp in (2, 3, 7, 8)]
        a, b = audit.same_stamps(left, right)
        self.assertEqual([row['stamp_ns'] for row in a], [2, 7])
        self.assertEqual(a, b)


if __name__ == '__main__':
    unittest.main()
