import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from stararm_rs_feedback.can_feedback import State
from stararm_rs_feedback.config import NAMES
from stararm_rs_feedback.controller import Feedback
from stararm_rs_feedback.gravity import GravityModel
from stararm_rs_feedback.shadow import ShadowPolicy
from stararm_rs_feedback.trial import (POLICY, TrialSpec, TrialObserver, TrialSession,
                                       check_can, main, trial_config)
from stararm_rs_feedback.unloaded import robot_signature
from test_rs_feedback import pair, FakeMonitor, Clock
from test_unloaded import LIMITS, Q


class MutableMonitor(FakeMonitor):
    def snapshot(self):
        return getattr(self, 'states', None) or super().snapshot()


def setup(root):
    leader, robot = pair()
    robot.config.id = 'follower1'
    robot.config.joint_limits = copy.deepcopy(LIMITS)
    robot.config.gravity_compensation = False
    robot.calibration_fpath = root/'calibration.json'
    robot.calibration_fpath.write_text('{}')
    data = dict(format='stararm-rs-residual-trial-v2', experimental=True,
                upright_stock_gripper=True, robot_signature=robot_signature(robot),
                model_sha256=GravityModel().sha256, policy=copy.deepcopy(POLICY))
    path = root/'spec.json'
    path.write_text(json.dumps(data))
    return leader, robot, path


class TrialTests(unittest.TestCase):
    def test_exact_fourfold_mapping_through_controller_for_every_candidate_power(self):
        leader, robot = pair()
        o = TrialObserver(LIMITS)
        f = Feedback(leader.bus, leader.config.joint_ids, trial_config(), True, o)
        for p in range(151):
            # Inject at the policy boundary to exercise the real adapter conversion.
            with patch.object(o.policy, 'update', return_value=[0,p,p,p,0,0,0]):
                ratios = o.update(list(Q), [0.]*7, [0.]*7, 100+p*.03)
            f.damping_loads = dict(zip(NAMES, ratios))
            f.apply(100+p*.03)
            for i,n in enumerate(NAMES):
                self.assertEqual(f.sent[n][:2], (0x12,4*p) if i in (1,2,3) and p else (0x10,0))

    def test_trial_steady_signal_scales_offline_policy_and_memory_does_not_cross_gaps(self):
        o, oracle = TrialObserver(LIMITS), ShadowPolicy()
        g = list(o.model.evaluate(Q)[0])+[0.]
        for j in range(120):
            now=100+j*.05; tau=list(g)
            tau[1]-=2.8; tau[2]+=4.8; tau[3]+=2.6
            o.update(list(Q), [0.]*7, tau, now)
            expected=oracle.update(o.residual_nm,now,o.valid)
            self.assertEqual(o.ordinary_power_mw, [4*p for p in expected])
        self.assertTrue(all(o.ordinary_power_mw[i]>0 for i in (1,2,3)))
        q=list(Q);q[2]+=.05
        o.update(q,[0.]*7,tau,106.)
        self.assertFalse(o.valid);self.assertTrue(any(o.loads))
        self.assertIn('RECENT LOAD',o.status)
        o.update(list(Q),[0.]*7,tau,107.)
        self.assertFalse(o.valid);self.assertEqual(o.loads,[0.]*7)
        q=list(Q);q[1]=0.
        o.update(q,[0.]*7,tau,107.05)
        self.assertFalse(o.valid);self.assertEqual(o.loads,[0.]*7)

    def test_session_real_writes_empty_loaded_unloaded_and_monitor(self):
        for enabled in (False,True):
            with self.subTest(enabled=enabled), tempfile.TemporaryDirectory() as t:
                leader,robot,path=setup(Path(t)); clock=Clock()
                with TrialSession(leader,robot,trial_config(),spec=TrialSpec(path),enabled=enabled,
                                  monitor_factory=MutableMonitor,clock=clock) as s:
                    g=list(GravityModel().evaluate(Q)[0])+[0.]
                    for j in range(240):
                        clock.now=100+j*.05
                        tau=list(g)
                        if 60<=j<140:
                            tau[1]-=2.8;tau[2]+=4.8;tau[3]+=2.6
                        s.monitor.states={n:State(Q[i],.6 if j==140 and i==2 else 0.,tau[i],25.,2,0,clock.now)
                                          for i,n in enumerate(NAMES)}
                        action=leader.get_action()
                        action.update({n+'.pos':math.degrees(Q[i])/s.directions[n]
                                       for i,n in enumerate(NAMES)})
                        robot.send_action(action)
                        if j in (59,140,239):
                            self.assertTrue(all(x[:2]==(0x10,0) for x in s.feedback.sent.values()))
                        if j==139:
                            self.assertTrue(all(1<=s.feedback.sent[NAMES[i]][1]<=600 for i in (1,2,3)))
                            self.assertTrue(all(s.feedback.sent[NAMES[i]][:2]==(0x10,0) for i in (0,4,5,6)))
                    self.assertFalse(s.feedback.holding)
                if enabled:
                    self.assertTrue(any(m==0x12 and p>0 for _,m,p in leader.bus.calls))
                    self.assertTrue(all(0<=p<=600 for _,_,p in leader.bus.calls))
                    self.assertTrue(all(m==0x10 for _,m,_ in leader.bus.calls[-7:]))
                else:
                    self.assertEqual(leader.bus.calls,[])

    def test_original_stall_holds_all_joints_and_freezes_follower_then_releases(self):
        with tempfile.TemporaryDirectory() as t:
            leader,robot,path=setup(Path(t));clock=Clock()
            with TrialSession(leader,robot,trial_config(),spec=TrialSpec(path),enabled=True,
                              monitor_factory=MutableMonitor,clock=clock) as s:
                tau=list(GravityModel().evaluate(Q)[0])+[0.];tau[0]=20.
                s.monitor.states={n:State(Q[i],0.,tau[i],25.,2,0,100.) for i,n in enumerate(NAMES)}
                def send(j,target):
                    clock.now=100+j*.05
                    a=leader.get_action()
                    a.update({n+'.pos':math.degrees(Q[i])/s.directions[n] for i,n in enumerate(NAMES)})
                    a['shoulder_pan.pos']=target
                    return robot.send_action(a)
                for j in range(25):send(j,30.)
                self.assertTrue(s.feedback.holding)
                self.assertTrue(all(x[:2]==(0x11,1500) for x in s.feedback.sent.values()))
                send(25,70.)
                self.assertEqual(robot.sent[-1]['shoulder_pan.pos'],30.)
                # Real follower converges to the held target; recovery is based on actual feedback.
                old=s.monitor.states[NAMES[0]]
                s.monitor.states[NAMES[0]]=State(math.radians(30),old.vel,old.torq,25.,2,0,100.)
                for j in range(26,36):send(j,70.)
                self.assertFalse(s.feedback.holding)
                send(36,70.)
                self.assertEqual(robot.sent[-1]['shoulder_pan.pos'],70.)

    def test_identity_and_policy_rejected_before_leader_writes(self):
        with tempfile.TemporaryDirectory() as t:
            leader,robot,path=setup(Path(t));spec=TrialSpec(path)
            robot.calibration_fpath.write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError,'identity mismatch'):
                with TrialSession(leader,robot,trial_config(),spec=spec,enabled=True,
                                  monitor_factory=FakeMonitor):pass
            self.assertEqual(leader.bus.calls,[])
            d=json.loads(path.read_text());d['policy']['max_ordinary_power_mw']=150
            path.write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,'candidate policy'):TrialSpec(path)

    def test_can_preflight_up_down_bitrate_and_bus_off(self):
        config=NS(port='can0',can_adapter='socketcan')
        for flags,bitrate,state,ok in [(['UP'],1000000,'ERROR-ACTIVE',True),
                                      ([],1000000,'STOPPED',False),
                                      (['UP'],500000,'ERROR-ACTIVE',False),
                                      (['UP'],1000000,'BUS-OFF',False)]:
            raw=[dict(flags=flags,linkinfo=dict(info_kind='can',info_data=dict(
                bittiming=dict(bitrate=bitrate),state=state)))]
            with self.subTest(raw=raw), patch('stararm_rs_feedback.trial.subprocess.run',
                                              return_value=NS(stdout=json.dumps(raw))):
                if ok:check_can(config)
                else:
                    with self.assertRaisesRegex(ValueError,'before arm connection'):check_can(config)

    def loaded_observer(self):
        o=TrialObserver(copy.deepcopy(LIMITS))
        tau=list(o.model.evaluate(Q)[0])+[0.]
        tau[1]-=2.8;tau[2]+=4.8;tau[3]+=2.6
        for j in range(81):o.update(list(Q),[0.]*7,tau,100+j*.05)
        self.assertTrue(all(o.ordinary_power_mw[i]>300 for i in (1,2,3)))
        return o,tau

    def test_recent_load_survives_normal_motion_but_expires_without_refresh(self):
        o,tau=self.loaded_observer();powers=list(o.ordinary_power_mw)
        for j in range(1,62):
            q=list(Q);q[2]+=.03+.002*j
            o.update(q,[0.]*7,tau,104+j*.05)
            if j<60:
                self.assertEqual(o.ordinary_power_mw,powers)
                self.assertFalse(o.valid)
                self.assertIn('RECENT LOAD',o.status)
        self.assertEqual(o.loads,[0.]*7)
        self.assertIsNone(o.memory)

    def test_recent_load_releases_on_pose_gripper_speed_limit_or_leader_gap(self):
        for mode in ('pose','gripper','speed','limit','leader_gap'):
            with self.subTest(mode=mode):
                o,tau=self.loaded_observer();q=list(Q);v=[0.]*7
                if mode=='pose':q[2]+=math.radians(21)
                if mode=='gripper':q[6]+=math.radians(4)
                if mode=='speed':v[2]=.6
                if mode=='limit':q[1]=0.
                if mode=='leader_gap':o.invalidate('Leader feedback gap')
                o.update(q,v,tau,104.05)
                self.assertEqual(o.loads,[0.]*7)
                self.assertIsNone(o.memory)

    def test_confirmed_unloaded_clears_memory_and_unconfirmed_motion_cannot_create_it(self):
        o,tau=self.loaded_observer()
        g=list(o.model.evaluate(Q)[0])+[0.]
        for j in range(1,45):o.update(list(Q),[0.]*7,g,104+j*.05)
        self.assertTrue(o.valid);self.assertEqual(o.loads,[0.]*7);self.assertIsNone(o.memory)
        for j in range(1,40):
            q=list(Q);q[2]+=.003*j
            o.update(q,[0.]*7,tau,106.2+j*.05)
            self.assertEqual(o.loads,[0.]*7)
        self.assertIsNone(o.memory)

    def test_session_sends_remembered_damping_and_missing_leader_releases_it(self):
        with tempfile.TemporaryDirectory() as t:
            leader,robot,path=setup(Path(t));clock=Clock()
            with TrialSession(leader,robot,trial_config(),spec=TrialSpec(path),enabled=True,
                              monitor_factory=MutableMonitor,clock=clock) as s:
                tau=list(GravityModel().evaluate(Q)[0])+[0.]
                tau[1]-=2.8;tau[2]+=4.8;tau[3]+=2.6
                for j in range(101):
                    clock.now=100+j*.05
                    q=list(Q)
                    if j>80:q[2]+=.003*(j-80)
                    s.monitor.states={n:State(q[i],0.,tau[i],25.,2,0,clock.now) for i,n in enumerate(NAMES)}
                    a=leader.get_action()
                    a.update({n+'.pos':math.degrees(q[i])/s.directions[n] for i,n in enumerate(NAMES)})
                    robot.send_action(a)
                self.assertIn('RECENT LOAD',s.feedback.observer.status)
                self.assertTrue(all(s.feedback.sent[NAMES[i]][0]==0x12 for i in (1,2,3)))
                leader.bus.reliable=False;clock.now+=.05
                count=len(robot.sent)
                robot.send_action(leader.get_action())
                self.assertEqual(len(robot.sent),count)
                self.assertTrue(all(x[:2]==(0x10,0) for x in s.feedback.sent.values()))

    def test_launcher_preconnect_failure_restores_factory_loop_argv(self):
        with tempfile.TemporaryDirectory() as t:
            leader,robot,path=setup(Path(t))
            original=lambda teleop,robot:None
            factory=lambda cfg:robot
            connections=[]
            def official_main():
                module.make_robot_from_config(robot.config)
                connections.append('connect both arms')
            module=NS(teleop_loop=original,make_robot_from_config=factory,main=official_main)
            saved=sys.argv
            for failure in ('CAN','identity'):
                with self.subTest(failure=failure):
                    if failure=='identity':robot.calibration_fpath.write_text('new')
                    with patch('stararm_rs_feedback.trial.check_can',side_effect=ValueError('CAN down')), \
                         patch('stararm_rs_feedback.trial.importlib.import_module',return_value=module):
                        with self.assertRaises(ValueError):main(['--trial-spec',str(path),'--feedback-enable'])
                    self.assertEqual(connections,[])
                    self.assertIs(module.teleop_loop,original)
                    self.assertIs(module.make_robot_from_config,factory)
                    self.assertIs(sys.argv,saved)

    def test_forbidden_flags_rejected_before_lerobot_import(self):
        for flag in ('--feedback-config=x','--feedback-enable=true','--collect-unloaded=x'):
            with patch('stararm_rs_feedback.trial.importlib.import_module') as imported:
                with self.assertRaises(SystemExit):main(['--trial-spec','not-opened.json',flag])
                imported.assert_not_called()

    def test_fault_cleanup_unlocks_and_restores_methods_and_sdk(self):
        with tempfile.TemporaryDirectory() as t:
            leader,robot,path=setup(Path(t));clock=Clock()
            original_get,original_send=leader.get_action,robot.send_action
            with self.assertRaisesRegex(RuntimeError,'test fault'):
                with TrialSession(leader,robot,trial_config(),spec=TrialSpec(path),enabled=True,
                                  monitor_factory=FakeMonitor,clock=clock) as s:
                    robot.send_action(leader.get_action())
                    raise RuntimeError('test fault')
            self.assertTrue(robot._skip_safe_zero_on_disconnect)
            self.assertTrue(all(m==0x10 for _,m,_ in leader.bus.calls[-7:]))
            self.assertIs(leader.get_action,original_get);self.assertIs(robot.send_action,original_send)
            self.assertEqual(leader.bus.zero_samples,30)
            self.assertTrue(s.monitor.closed)


if __name__=='__main__':unittest.main()
