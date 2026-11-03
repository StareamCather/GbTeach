"""Installed graph tools must be reported, not borrowed from a runtime fallback."""
import tempfile
from pathlib import Path
import unittest
from unittest import mock

from gazeboteach.environment import detect_environment
from gazeboteach.compatibility import lesson_compatibility
from tests.test_core import environment,make_lesson


class ROSGraphToolDiscovery(unittest.TestCase):
    def test_rosnode_and_rosmsg_are_probed_and_enable_the_matching_lesson(self):
        def which(name):
            return '/fixture/bin/'+name if name in ('rosnode','rosmsg') else None
        with mock.patch('gazeboteach.environment.shutil.which',side_effect=which), \
             mock.patch('gazeboteach.environment._output',return_value=''), \
             mock.patch('gazeboteach.environment.detect_development_capabilities',return_value={}), \
             mock.patch('gazeboteach.modern.detect_modern',return_value={'profiles':[]}):
            snapshot=detect_environment()
        for name in ('rosnode','rosmsg'):
            self.assertEqual(snapshot['tools'][name],'/fixture/bin/'+name)
        with tempfile.TemporaryDirectory() as directory:
            lesson=make_lesson(Path(directory)/'graph',requires_ros=True,requires_tools=['rosnode','rosmsg'])
            known=environment()
            known['tools'].update({name:snapshot['tools'][name] for name in ('rosnode','rosmsg')})
            self.assertEqual(lesson_compatibility(lesson,known,gui=False,terminals=False)['status'],'ready')


if __name__=='__main__':unittest.main()
