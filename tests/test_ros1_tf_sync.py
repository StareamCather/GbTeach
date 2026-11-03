"""A45 geometry and clock-boundary checks; actual TF/message_filters need smoke45."""
import importlib.util
import math
from pathlib import Path
import tempfile
import time
import types
import unittest
import xml.etree.ElementTree as ET
from unittest import mock


FOLDER=Path(__file__).resolve().parents[1]/'courses/classic11_noetic/45-ros1-tf-synchronization'


def load():
    spec=importlib.util.spec_from_file_location('gazeboteach_a45_test_instrument',str(FOLDER/'instrument.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class TFAndSync(unittest.TestCase):
    def setUp(self):self.module=load()

    def test_nonzero_point_requires_rotation_then_translation(self):
        R=self.module.rotation(self.module.quaternion([0,0,math.pi/2]))
        actual=self.module.transform_point(R,[.1,.15,.75],[.2,0,0])
        self.assertLess(self.module.distance(actual,[.1,.35,.75]),1e-12)
        self.assertGreater(self.module.distance(actual,[.3,.15,.75]),.28,'Translation-only implementation must fail this case')

    def test_rotation_has_an_independent_known_quaternion_oracle(self):
        fixed=[0,0,math.sqrt(.5),math.sqrt(.5)]
        expected=[[0,-1,0],[1,0,0],[0,0,1]]
        actual=self.module.rotation(fixed)
        for row_a,row_b in zip(actual,expected):
            for a,b in zip(row_a,row_b):self.assertAlmostEqual(a,b,places=12)
        for a,b in zip(self.module.quaternion([0,0,math.pi/2]),fixed):self.assertAlmostEqual(a,b,places=12)
        # This oracle does not call either quaternion conversion helper.
        self.assertEqual(self.module.transform_point(expected,[.1,.15,.75],[0,.2,0]),[-.1,.15,.75])

    def test_mount_variant_is_read_from_the_actual_personal_sdf(self):
        document=ET.parse(str(FOLDER/'world.sdf'))
        document.find("world/model[@name='workshop']/link[@name='probe']/pose").text='.1 .15 .10 0 0 .3'
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'world.sdf';document.write(str(path),encoding='utf-8')
            mount=self.module.read_mount(path)
        pose=mount['probe_pose_relative_body'];self.assertEqual(pose,[.1,.15,.1,0.,0.,.3])
        point=self.module.transform_point(self.module.rotation(self.module.quaternion(pose[3:])),
            [.1,.15,mount['rail_pose'][2]+pose[2]],[.2,0,0])
        self.assertLess(self.module.distance(point,[.1+.2*math.cos(.3),.15+.2*math.sin(.3),.8]),1e-12)

    def test_quaternion_sign_does_not_create_a_false_orientation_error(self):
        q=self.module.quaternion([.2,-.3,.4]);negative=[-v for v in q]
        self.assertLess(self.module.angle_between(q,negative),1e-7)

    def test_paused_clock_duplicates_and_slow_tick_catchup_do_not_create_samples(self):
        cadence=self.module.Cadence()
        self.assertFalse(cadence.due(0),'An uninitialized use_sim_time read is not an observation timestamp')
        stamps=[100_000_000,100_000_000,159_000_000,160_000_000,400_000_000,401_000_000,459_000_000,460_000_000]
        actual=[stamp for stamp in stamps if cadence.due(stamp)]
        self.assertEqual(actual,[100_000_000,160_000_000,400_000_000,460_000_000])
        self.assertTrue(cadence.due(10_000_000),'A documented clock rewind starts a fresh cadence epoch')
        self.assertFalse(cadence.due(10_000_000))

    def test_configured_cadence_prevents_neighbor_frame_alias_in_negative_ats_case(self):
        for slop in (.02,.004,.012):
            config=self.module.configuration(types.SimpleNamespace(get_param=lambda name,default:slop if name=='~slop_s' else default))
            stamps=[i*self.module.SYNC_PERIOD for i in range(8)]
            matches=[(i,j) for i,a in enumerate(stamps) for j,b in enumerate(stamps)
                     if abs((a)-(b+config['offset_sim_s']))<slop-1e-12]
            if slop>.012:self.assertEqual(matches,[(i,i) for i in range(8)])
            else:self.assertEqual(matches,[],'At/above slop is an expected mismatch, not a neighboring-frame match')

    def test_boolean_and_unsafe_timing_settings_are_rejected(self):
        for key,bad in [('delay_sim_s',True),('delay_sim_s',-.1),('offset_sim_s',.04),('slop_s',float('nan')),('slop_s',.03)]:
            with self.assertRaises(RuntimeError):
                self.module.configuration(types.SimpleNamespace(get_param=lambda name,default,k=key,b=bad:b if name=='~'+k else default))

    def test_tf_lookup_uses_only_immediate_reads_and_a_wall_deadline(self):
        calls=[]
        class Buffer:
            def lookup_transform(self,parent,child,stamp,timeout):
                calls.append((stamp,timeout));raise LookupError('no TF while simulated time is paused')
        rospy=types.SimpleNamespace(Time=lambda value:value,Duration=lambda value:value)
        start=time.monotonic()
        with self.assertRaises(self.module.WallDeadline):self.module.lookup(rospy,Buffer(),seconds=.035)
        self.assertLess(time.monotonic()-start,.5)
        self.assertTrue(calls);self.assertTrue(all(pair==(0,0) for pair in calls))

    def observations(self):
        raw={'pose':[],'companion':[],'companion_offset':[]};pairs={'baseline_exact':[],'offset_exact':[],'offset_approx':[]};traces=[]
        for i in range(8):
            stamp=1_000_000_000+i*60_000_000;source=500+i
            pose_seq,companion_seq,shadow_seq=100+i,3+i,40+i
            raw['pose'].append({'seq':pose_seq,'stamp_ns':stamp,'receipt_monotonic_ns':stamp+100_000_000})
            raw['companion'].append({'seq':companion_seq,'stamp_ns':stamp,'receipt_monotonic_ns':stamp+145_000_000})
            raw['companion_offset'].append({'seq':shadow_seq,'stamp_ns':stamp+12_000_000,'receipt_monotonic_ns':stamp+146_000_000})
            traces.append({'source_observation_seq':source,'query_stamp_ns':stamp,
                'pose_publisher_seq':pose_seq,'pose_stamp_ns':stamp,
                'companion_publisher_seq':companion_seq,'companion_stamp_ns':stamp,
                'shadow_publisher_seq':shadow_seq,'shadow_stamp_ns':stamp+12_000_000,
                'pose_publish_call_sim_ns':stamp+1_000_000,'companion_publish_call_sim_ns':stamp+40_000_000,
                'shadow_publish_call_sim_ns':stamp+40_000_000})
            pairs['baseline_exact'].append({'pose_seq':pose_seq,'companion_seq':companion_seq,'pose_stamp_ns':stamp,'companion_stamp_ns':stamp})
        return raw,pairs,traces

    def test_source_identity_works_with_different_real_publisher_header_counters(self):
        raw,pairs,traces=self.observations();counts=self.module.annotate_sources(raw,pairs,traces)
        self.assertEqual(counts['baseline_exact']['source_verified'],8)
        for row in pairs['baseline_exact']:
            self.assertNotEqual(row['pose_seq'],row['companion_seq'])
            self.assertEqual(row['pose_source_observation_seq'],row['companion_source_observation_seq'])

    def test_cross_observation_pair_is_rejected_even_when_each_packet_is_real(self):
        raw,pairs,traces=self.observations()
        pairs['baseline_exact'][0]['companion_seq']=traces[1]['companion_publisher_seq']
        pairs['baseline_exact'][0]['companion_stamp_ns']=traces[1]['companion_stamp_ns']
        with self.assertRaisesRegex(RuntimeError,'neighboring source observation'):
            self.module.annotate_sources(raw,pairs,traces)

    def test_delay_needs_actual_source_sim_evidence_and_separate_positive_wall_median(self):
        raw,pairs,traces=self.observations()
        metrics=self.module.delivery_metrics(raw,traces,{'delay_sim_s':.03})
        self.assertAlmostEqual(metrics['median_receive_wall_lag_s'],.045)
        self.assertTrue(all(row['actual_source_stamp_to_companion_publish_sim_s']>=.03 for row in metrics['source_publication_traces']))
        traces[0]['companion_publish_call_sim_ns']=traces[0]['query_stamp_ns']
        with self.assertRaisesRegex(RuntimeError,'delay was not actually observed'):
            self.module.delivery_metrics(raw,traces,{'delay_sim_s':.03})

    def test_positive_sim_delay_does_not_allow_nonpositive_wall_delivery_evidence(self):
        raw,pairs,traces=self.observations()
        for pose,companion in zip(raw['pose'],raw['companion']):companion['receipt_monotonic_ns']=pose['receipt_monotonic_ns']-1
        with self.assertRaisesRegex(RuntimeError,'positive median'):
            self.module.delivery_metrics(raw,traces,{'delay_sim_s':.03})

    def test_uncertain_clear_is_not_replayed_by_final_cleanup(self):
        with mock.patch.object(self.module,'clear') as clear:
            with self.assertRaisesRegex(RuntimeError,'automatic cleanup replay.*skipped'):
                self.module.final_clear(object(),self.module.WallDeadline('/gazebo/clear_joint_forces'))
            clear.assert_not_called()


if __name__=='__main__':unittest.main()
