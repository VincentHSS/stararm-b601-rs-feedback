import copy
import math
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from stararm_rs_feedback.config import Config, NAMES
from stararm_rs_feedback.preview import PreviewObserver, PreviewSession, main
from test_rs_feedback import pair, FakeMonitor, Clock
from test_unloaded import LIMITS, Q


class PreviewTests(unittest.TestCase):
    def make(self):
        return PreviewObserver(copy.deepcopy(LIMITS))

    def test_signed_ripple_cancels_and_constant_load_is_preserved(self):
        for load in (0., 1.5, -1.5):
            o = self.make()
            g = list(o.model.evaluate(Q)[0]) + [0.]
            for j in range(101):
                tau = list(g)
                tau[2] += load + .6 * math.sin(2*math.pi*j*.05*4)
                o.update(list(Q), [.07]*6+[0.], tau, 100+j*.05)
            self.assertIn('steady 1s', o.status)
            self.assertAlmostEqual(o.residual_nm[2], load, places=8)
            self.assertEqual(o.loads, [0.]*7)

    def test_irregular_intervals_use_time_weighting(self):
        o = self.make()
        g = list(o.model.evaluate(Q)[0])+[0.]
        t = 100.
        for j in range(35):
            tau = list(g);tau[2] += 2*(t-100)
            o.update(list(Q), [0.]*7, tau, t)
            t += (.04, .11, .07)[j%3]
        # Exact average of a linear signal over the last one-second interval.
        self.assertAlmostEqual(o.residual_nm[2], 2*(o.last-100-.5), places=8)

    def test_movement_and_gaps_clear_available_result(self):
        o = self.make();g = list(o.model.evaluate(Q)[0])+[0.]
        for j in range(30):o.update(list(Q), [0.]*7, g, 100+j*.05)
        self.assertIn('steady 1s', o.status)
        q=list(Q);q[2]+=.1
        o.update(q, [0.]*7, g, 101.5)
        self.assertIn('unavailable', o.status)
        o.update(list(Q),[0.]*7,g,102.)
        self.assertNotIn('steady 1s',o.status)
        self.assertEqual(o.residual_nm,[0.]*6)

    def test_gripper_motion_high_speed_and_limits_reject_stationarity(self):
        for mode in ('gripper','speed','limit'):
            with self.subTest(mode=mode):
                o=self.make();g=list(o.model.evaluate(Q)[0])+[0.]
                for j in range(31):
                    q=list(Q);v=[0.]*7
                    if mode=='gripper':q[6]=j*.01
                    if mode=='speed':v[2]=.6
                    if mode=='limit':q[1]=0.
                    o.update(q,v,g,100+j*.05)
                self.assertNotIn('steady 1s',o.status)

    def test_force_flags_rejected_before_device_import(self):
        for flag in ('--feedback-enable','--feedback-enable=true','--feedback-config=x','--collect-unloaded=x'):
            with patch('stararm_rs_feedback.preview.importlib.import_module') as imported:
                with self.assertRaises(SystemExit):main([flag])
                imported.assert_not_called()

    def test_preview_session_never_writes_leader_force_or_unlock(self):
        leader,robot=pair();robot.config.joint_limits=copy.deepcopy(LIMITS)
        clock=Clock()
        with PreviewSession(leader,robot,Config(),monitor_factory=FakeMonitor,clock=clock):
            for j in range(40):
                clock.now=100+j*.05
                robot.send_action(leader.get_action())
        self.assertEqual(leader.bus.calls,[])
        self.assertEqual(len(robot.sent),40)

    def test_launcher_restores_official_loop_and_argv_on_error(self):
        original=lambda teleop,robot:None
        def fail():raise RuntimeError('official error')
        module=NS(teleop_loop=original,main=fail)
        saved=sys.argv
        with patch('stararm_rs_feedback.preview.importlib.import_module',return_value=module):
            with self.assertRaisesRegex(RuntimeError,'official error'):main(['--fps=30'])
        self.assertIs(module.teleop_loop,original)
        self.assertIs(sys.argv,saved)


if __name__=='__main__':unittest.main()
