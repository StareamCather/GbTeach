"""A46 CLI/lifecycle boundaries; actual bag data/playback is tested on ikaros."""
import importlib.util
import io
from pathlib import Path
import signal
import tempfile
import types
import unittest
from unittest import mock


SOURCE=Path(__file__).resolve().parents[1]/'courses/classic11_noetic/46-ros1-bag-diagnostics/instrument.py'


def load():
    spec=importlib.util.spec_from_file_location('gazeboteach_a46_test_instrument',str(SOURCE))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class BagBoundaries(unittest.TestCase):
    def setUp(self):
        self.module=load();self.topics=['/workshop46/state','/workshop46/diagnostics','/clock']
        self.mapping={topic:'/workshop46/replay_unique/'+('recorded_clock' if topic=='/clock' else topic.rsplit('/',1)[1]) for topic in self.topics}

    def test_real_player_command_explicitly_isolates_recorded_clock_and_topics(self):
        command=self.module.playback_command('/private/motion.bag',self.topics,self.mapping,2,'owned_player')
        self.assertEqual(command[:2],['rosbag','play']);self.assertNotIn('--clock',command)
        index=command.index('--topics');self.assertEqual(command[index+1:],self.topics)
        self.assertLess(command.index('/private/motion.bag'),index)
        for topic in self.topics:self.assertIn(topic+':='+self.mapping[topic],command[:index])
        self.assertIn('--wait-for-subscribers',command)

    def test_internal_namespace_separates_disabled_player_clock_from_recorded_clock(self):
        command=self.module.playback_command('/private/motion.bag',self.topics,self.mapping,1,'owned_player')
        internal='/workshop46/replay_unique/player_internal'
        namespace='__ns:='+internal
        self.assertIn(namespace,command[:command.index('--topics')])
        self.assertNotEqual(internal+'/clock',self.mapping['/clock'])
        self.assertIn('/clock:='+self.mapping['/clock'],command)
        self.assertNotIn('--clock',command);self.assertNotIn('--keep-alive',command)

    def test_missing_clock_remap_or_live_topic_destination_is_rejected(self):
        for mapping in ({key:value for key,value in self.mapping.items() if key!='/clock'},
                        {**self.mapping,'/clock':'/clock'},
                        {**self.mapping,'/workshop46/state':'/workshop46/state'}):
            with self.assertRaises(RuntimeError):self.module.playback_command('bag',self.topics,mapping,1,'owned')

    def test_record_is_explicit_and_does_not_depend_on_ros_duration_for_shutdown(self):
        command=self.module.record_command('/private/run_unique/motion.bag',self.topics,'owned_recorder')
        self.assertEqual(command[:2],['rosbag','record']);self.assertNotIn('--duration',command)
        self.assertIn('/private/run_unique/motion.bag',command)
        self.assertTrue(all(topic in command for topic in self.topics))

    def test_active_bag_is_never_treated_as_a_completed_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'motion.bag';path.write_bytes(b'not accepted as bag data')
            Path(str(path)+'.active').write_bytes(b'active')
            with self.assertRaisesRegex(RuntimeError,'active is not success'):self.module.audit_bag(path,self.topics)

    def test_freshness_levels_have_exact_warn_error_boundaries_and_recover(self):
        expected=[(0,0),(.249,0),(.25,1),(.699,1),(.7,2),(2,2),(0,0)]
        for age,level in expected:self.assertEqual(self.module.health_level(age,.25,.7),level)
        with self.assertRaises(RuntimeError):self.module.health_level(float('nan'),.25,.7)

    def test_unowned_group_is_never_signalled(self):
        process=object.__new__(self.module.BagProcess);process.closed=False;process.stream=io.BytesIO()
        process.members=mock.Mock(side_effect=RuntimeError('unowned process'));process.pgid=123;process.report={}
        with mock.patch.object(self.module.os,'killpg',create=True) as kill,\
             mock.patch.object(self.module.signal,'SIGKILL',9,create=True):
            with self.assertRaisesRegex(RuntimeError,'unowned'):process.stop()
            kill.assert_not_called()

    def test_owned_stop_uses_sigint_first_and_reaps_wrapper(self):
        process=object.__new__(self.module.BagProcess);process.closed=False;process.stream=io.BytesIO()
        process.members=mock.Mock(side_effect=[[{'pid':123}],[],[],[],[]]);process.pgid=123;process.report={}
        process.process=types.SimpleNamespace(wait=mock.Mock(return_value=0),poll=lambda:0)
        with mock.patch.object(self.module.os,'killpg',create=True) as kill,\
             mock.patch.object(self.module.signal,'SIGKILL',9,create=True):
            report=process.stop()
            kill.assert_called_once_with(123,signal.SIGINT)
        self.assertEqual(report['remaining_owned_group'],[]);process.process.wait.assert_called_once()

    def test_uncertain_clear_is_not_replayed(self):
        with mock.patch.object(self.module,'clear') as clear:
            with self.assertRaisesRegex(RuntimeError,'automatic cleanup replay'):
                self.module.final_clear(object(),self.module.WallDeadline('/gazebo/clear_joint_forces'))
            clear.assert_not_called()


if __name__=='__main__':unittest.main()
