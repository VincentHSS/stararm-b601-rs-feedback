import copy
import csv
import itertools
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import patch
from dataclasses import replace

from stararm_rs_feedback.calibrate import fit, read_capture
from stararm_rs_feedback.config import Config, NAMES, load
from stararm_rs_feedback.controller import Feedback
from stararm_rs_feedback.gravity import GravityModel
from stararm_rs_feedback.unloaded import (ALGORITHM, Observer, Profile, Recorder, digest, robot_signature, within)
from stararm_rs_feedback.session import Session
from stararm_rs_feedback.launcher import main
from test_rs_feedback import pair, FakeMonitor, states, Clock

LIMITS = dict(zip(NAMES, [[-145,145],[0,170],[0,200],[-80,90],[-90,90],[-130,130],[0,270]]))
Q = [0., .8, 1., .2, 0., 0., 0.]


def raw_profile():
    return dict(algorithm=ALGORITHM, model_sha256=GravityModel().sha256,
                robot_signature='a'*64, joint_limits_deg=copy.deepcopy(LIMITS),
                gravity_axes=[1,2,3], coefficients=[[0.,1.,0.,0.] for _ in range(6)],
                sigma_nm=[.03]*6, validated_poses_rad=[list(Q)])


def sample_file(path, *, run='train', load_nm=0., q_offset=0., signature='a'*64, single=False):
    model = GravityModel()
    count, t = 0, 100.
    with path.open('x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t']+[f'{key}{i}' for key in ('q','v','tau') for i in range(1,8)])
        poses = list(itertools.product((.45,.85,1.25),(.5,.9,1.3),(-.5,0.,.5)))
        if single:poses = [poses[0]]*27
        for a,b,c in poses:
            q = [0.,a+q_offset,b+q_offset,c+q_offset,0.,0.,0.]
            g, _ = model.evaluate(q)
            for j in range(20):
                tau = [.1+(1 if i==0 else .9)*g[i]+.005*math.sin(count*.7+i) for i in range(6)]+[0.]
                tau[2] += load_nm
                writer.writerow([t]+q+[0.]*7+tau)
                count += 1;t += .1
    meta = dict(algorithm=ALGORITHM, confirmed_unloaded=True, upright_stock_gripper=True,
                complete=True, run_id=run, csv_sha256=digest(path.read_bytes()),
                model_sha256=model.sha256, sample_count=count,
                robot_signature=signature, joint_limits_deg=LIMITS)
    path.with_suffix('.meta.json').write_text(json.dumps(meta))
    return path


class GravityTests(unittest.TestCase):
    def test_rs_gravity_matches_finite_difference_of_potential(self):
        m = GravityModel()
        for q in ([0.]*6, [.2,.5,.7,-.3,.2,-.1], [-.5,1.1,.3,.7,-.4,.8]):
            tau, _ = m.evaluate(q)
            for i in range(6):
                a, b = list(q), list(q);a[i]+=1e-6;b[i]-=1e-6
                self.assertAlmostEqual(tau[i],(m.evaluate(a)[1]-m.evaluate(b)[1])/2e-6,places=6)

    def test_includes_finger_mass_and_uses_raw_motor_angles(self):
        m=GravityModel();self.assertGreater(m.bodies['gripper_left'][0],0)
        q=[.2,.5,.7,-.3,.2,-.1]
        self.assertNotEqual(m.evaluate(q)[0],m.evaluate([q[0],q[1],-q[2],-q[3],-q[4],q[5]])[0])

    def test_matches_independent_official_numpy_implementation(self):
        # 官方 gravity.py / compute_generalized_gravity 在同一 URDF、零指爪位移下的值。
        expected=[0.,.9857586748667421,6.278131204249115,1.9948813474213376,
                  .016844050826735405,-.0008108330035335125]
        actual=GravityModel().evaluate([.2,.5,.7,-.3,.2,-.1])[0]
        for a,b in zip(actual,expected):self.assertAlmostEqual(a,b,places=9)


class ObserverTests(unittest.TestCase):
    def make(self):return Observer(Profile(data=raw_profile(),require_validated=False),Config().residual_reference_nm)

    def feed(self,o,start,count,extra=0.,q=None):
        q=list(Q) if q is None else q
        tau=list(o.profile.model.evaluate(q)[0])+[0.];tau[2]+=extra
        for j in range(count):o.update(q,[0.]*7,tau,start+j*.05)
        return o.loads

    def test_unloaded_model_predicts_zero_haptic_output(self):
        o=self.make();self.feed(o,100,50)
        self.assertEqual(o.loads,[0.]*7);self.assertEqual(o.status,'steady residual')

    def test_external_residual_adds_damping_and_putdown_releases(self):
        o=self.make();self.feed(o,100,40,extra=1.)
        self.assertGreater(o.loads[2],.3);self.assertEqual(o.loads[6],0)
        self.feed(o,102,70);self.assertEqual(o.loads,[0.]*7)

    def test_short_residual_spike_does_not_activate(self):
        o=self.make();self.feed(o,100,20);self.feed(o,101,3,extra=2.);self.feed(o,101.15,10)
        self.assertEqual(o.loads,[0.]*7)

    def test_unvalidated_pose_clears_damping(self):
        o=self.make();self.feed(o,100,40,extra=1.)
        q=list(Q);q[1]+=.3;self.feed(o,102,1,extra=1.,q=q)
        self.assertEqual(o.loads,[0.]*7);self.assertEqual(o.status,'outside validated poses')

    def test_motion_eventually_releases_confirmed_feedback(self):
        o=self.make();self.feed(o,100,40,extra=1.)
        tau=list(o.profile.model.evaluate(Q)[0])+[0.];tau[2]+=1.
        for j in range(100):o.update(list(Q),[.2]*6+[0.],tau,102+j*.05)
        self.assertEqual(o.loads,[0.]*7);self.assertEqual(o.status,'moving/settling')

    def test_dropout_cannot_complete_prior_confirmation(self):
        o=self.make();self.feed(o,100,20);self.feed(o,101,3,extra=1.);self.feed(o,102,1,extra=1.)
        self.assertEqual(o.loads,[0.]*7)

    def test_coverage_does_not_combine_axis_ranges(self):
        a,b=list(Q),list(Q);a[1]+=.5;b[2]+=.5
        self.assertFalse(within([Q[0],a[1],b[2],*Q[3:]],[a,b]))

    def test_unlock_at_no_residual_and_stall_overrides(self):
        leader,robot=pair();o=self.make()
        f=Feedback(leader.bus,leader.config.joint_ids,Config(),True,observer=o)
        action=dict.fromkeys((n+'.pos' for n in NAMES),0.)
        for t in (100.,100.51,101.02):
            action['shoulder_pan.pos']=30.;f.update(action,states(20.),robot.config.joint_directions,t)
        self.assertTrue(f.holding)
        self.assertTrue(all(mode==0x11 for _,mode,_ in leader.bus.calls[-7:]))
        f.triggers.clear();f.apply(102.)
        self.assertTrue(all(mode==0x10 and power==0 for _,mode,power in leader.bus.calls[-7:]))


class FittingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.train=sample_file(self.root/'train.csv')
        self.check=sample_file(self.root/'check.csv',run='check',q_offset=.003)

    def tearDown(self):self.tmp.cleanup()

    def test_independent_empty_validation_and_profile_integrity(self):
        d=fit(self.train,self.check);self.assertTrue(d['validated'],d['validation']);Profile(data=d)
        self.assertEqual(d['validation']['false_activation_frames'],0)
        profile=self.root/'profile.json';profile.write_text(json.dumps(d))
        cfg=self.root/'cfg.json';cfg.write_text(json.dumps({'unloaded_profile':'profile.json'}))
        self.assertEqual(load(cfg).unloaded_profile,str(profile))
        d['coefficients'][2][0]+=.5
        with self.assertRaisesRegex(ValueError,'integrity'):Profile(data=d)

    def test_held_object_in_check_is_not_learned_as_empty(self):
        check=sample_file(self.root/'loaded.csv',run='loaded',load_nm=1.,q_offset=.003)
        d=fit(self.train,check);self.assertFalse(d['validated'])
        self.assertGreater(d['validation']['false_activation_frames'],0)
        with self.assertRaisesRegex(ValueError,'validation'):Profile(data=d)

    def test_same_run_wrong_identity_single_pose_tamper_rejected(self):
        for mode in ('same','identity','single','tamper'):
            with self.subTest(mode=mode):
                if mode=='same':other=self.train
                elif mode=='identity':other=sample_file(self.root/'id.csv',run='other',signature='b'*64)
                elif mode=='single':other=sample_file(self.root/'single.csv',run='other',single=True)
                else:
                    other=sample_file(self.root/'tamper.csv',run='other');other.write_text(other.read_text()+'\n')
                with self.assertRaises(ValueError):fit(self.train,other)

    def test_model_tampering_rejected(self):
        d=fit(self.train,self.check);d['model_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'URDF'):Profile(data=d)

    def test_friction_fit_recovers_known_parameters_from_independent_motion(self):
        model=GravityModel()
        paths=[]
        for label in ('motion_train','motion_check'):
            path=self.root/(label+'.csv');paths.append(path)
            v_last=None
            with path.open('x',newline='') as f:
                writer=csv.writer(f)
                writer.writerow(['t']+[f'{k}{i}' for k in ('q','v','tau') for i in range(1,8)])
                for j in range(1000):
                    x=j*.1+(.01 if label.endswith('check') else 0.)
                    q=[0.,.85+.35*math.sin(.18*x),.9+.35*math.sin(.27*x+.7),
                       .1+.5*math.sin(.19*x+1.3),0.,0.,0.]
                    v=[0.,.35*.18*math.cos(.18*x),.35*.27*math.cos(.27*x+.7),
                       .5*.19*math.cos(.19*x+1.3),0.,0.,0.]
                    vv=list(v) if v_last is None else [(a+b)/2 for a,b in zip(v_last,v)]
                    v_last=vv
                    g=model.evaluate(q)[0]
                    tau=[.1+.9*g[i]+.4*vv[i]+.08*math.tanh(vv[i]/.03) for i in range(6)]+[0.]
                    writer.writerow([100+j*.1]+q+v+tau)
            meta=dict(algorithm=ALGORITHM,confirmed_unloaded=True,upright_stock_gripper=True,
                      complete=True,run_id=label,csv_sha256=digest(path.read_bytes()),
                      model_sha256=model.sha256,sample_count=1000,robot_signature='a'*64,
                      joint_limits_deg=LIMITS)
            path.with_suffix('.meta.json').write_text(json.dumps(meta))
        d=fit(*paths)
        for i in (1,2,3):
            for actual,expected in zip(d['coefficients'][i],[.1,.9,.4,.08]):
                self.assertAlmostEqual(actual,expected,places=5)


class CaptureTests(unittest.TestCase):
    def robot(self,root):
        leader,robot=pair();robot.config.id='follower1';robot.config.joint_limits=copy.deepcopy(LIMITS)
        robot.config.gravity_compensation=False;robot.calibration_fpath=root/'calibration.json'
        robot.calibration_fpath.write_text('{}');return leader,robot

    def test_distinct_fresh_samples_and_hash(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);_,robot=self.robot(root);path=root/'empty.csv';r=Recorder(path,robot)
            r.update(states(),100.);r.update(states(),100.1);self.assertEqual(r.meta['sample_count'],1)
            r.close(complete=True)
            self.assertEqual(json.loads(path.with_suffix('.meta.json').read_text())['csv_sha256'],digest(path.read_bytes()))
            with self.assertRaises(FileExistsError):Recorder(path,robot)

    def test_recalibration_invalidates_identity(self):
        with tempfile.TemporaryDirectory() as t:
            _,robot=self.robot(Path(t));first=robot_signature(robot)
            robot.calibration_fpath.write_text('{"new_zero":1}');self.assertNotEqual(first,robot_signature(robot))

    def test_fault_capture_unusable(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);_,robot=self.robot(root);path=root/'empty.csv';r=Recorder(path,robot)
            r.update(states(),100.);r.close(complete=False)
            with self.assertRaisesRegex(ValueError,'incomplete'):read_capture(path)

    def test_explicit_confirmation_checked_before_importing_lerobot(self):
        with patch('stararm_rs_feedback.launcher.importlib.import_module') as imp:
            for args in (['--collect-unloaded','unused.csv'],['--collect-unloaded','unused.csv','--confirm-unloaded','--feedback-enable']):
                with self.assertRaises(SystemExit):main(args)
                imp.assert_not_called()

    def test_capture_session_adds_no_leader_force_writes(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);leader,robot=self.robot(root)
            with Session(leader,robot,Config(),collect_path=root/'empty.csv',monitor_factory=FakeMonitor,clock=Clock()):
                robot.send_action(leader.get_action())
            self.assertFalse(leader.bus.calls)
            self.assertTrue(json.loads((root/'empty.meta.json').read_text())['complete'])

    def test_profile_mismatch_blocks_before_device_connect_and_restores_factory(self):
        def loop(teleop,robot):pass
        factory=lambda cfg:NS()
        module=NS(teleop_loop=loop,make_robot_from_config=factory)
        module.main=lambda:module.make_robot_from_config(NS())
        with patch('stararm_rs_feedback.unloaded.Profile') as cls:
            with patch('stararm_rs_feedback.launcher.load',return_value=Config(unloaded_profile='unused')):
                with patch('stararm_rs_feedback.launcher.importlib.import_module',return_value=module):
                    cls.return_value.check_robot.side_effect=ValueError('identity mismatch')
                    with self.assertRaisesRegex(ValueError,'identity mismatch'):main([])
        self.assertIs(module.make_robot_from_config,factory)
        self.assertIs(module.teleop_loop,loop)

    def test_profile_session_empty_pose_unlocks_while_normal_following_continues(self):
        from stararm_rs_feedback.unloaded import canonical
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);leader,robot=self.robot(root)
            sig=robot_signature(robot)
            train=sample_file(root/'train.csv',signature=sig)
            check=sample_file(root/'check.csv',run='check',q_offset=.003,signature=sig)
            data=fit(train,check)
            path=root/'profile.json';path.write_text(json.dumps(data))
            cfg=Config(unloaded_profile=str(path));clock=Clock()
            q=[0.,.85,.9,0.,0.,0.,0.]
            gravity=GravityModel().evaluate(q)[0]
            snapshot={n:replace(states()[n],pos=q[i],torq=.1+.9*gravity[i] if i<6 else 0.)
                      for i,n in enumerate(NAMES)}
            with Session(leader,robot,cfg,enabled=True,monitor_factory=FakeMonitor,clock=clock) as s:
                s.monitor.snapshot=lambda:snapshot
                for j in range(45):
                    clock.now=100+j*.05
                    a=leader.get_action()
                    a.update({n+'.pos':math.degrees(q[i])/robot.config.joint_directions[n]
                              for i,n in enumerate(NAMES)})
                    robot.send_action(a)
                self.assertFalse(s.feedback.holding)
                self.assertEqual(s.feedback.observer.status,'steady residual')
            self.assertEqual(len(robot.sent),45)
            self.assertTrue(all(mode==0x10 and power==0 for _,mode,power in leader.bus.calls))


if __name__=='__main__':unittest.main()
