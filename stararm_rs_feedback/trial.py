"""Explicit experimental, low-power residual feedback for this calibrated RS pair.

This is a hardware trial of the offline candidate, not an accepted unloaded
calibration. The read-only preview entry point retains its original semantics.
"""
import argparse
import functools
import importlib
import json
import logging
import math
from pathlib import Path
import re
import subprocess
import sys

from .config import Config, NAMES
from .gravity import GravityModel
from .launcher import install_runtime_wrapper
from .preview import PreviewObserver
from .session import Session
from .shadow import ShadowPolicy, DEADZONE_NM, EXCESS_FULL_SCALE_NM
from .unloaded import robot_signature

log = logging.getLogger(__name__)
POLICY = dict(deadzone_nm=DEADZONE_NM, excess_full_scale_nm=EXCESS_FULL_SCALE_NM,
              max_ordinary_power_mw=600, confirmation_s=.3, smoothing_s=.15,
              motion_memory_s=3., motion_pose_delta_deg=20., gripper_delta_deg=3.)


class TrialSpec:
    def __init__(self, path):
        self.data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
        d = self.data
        if (not isinstance(d, dict) or d.get('format') != 'stararm-rs-residual-trial-v2'
                or d.get('experimental') is not True
                or d.get('upright_stock_gripper') is not True
                or d.get('model_sha256') != GravityModel().sha256
                or d.get('policy') != POLICY
                or not re.fullmatch('[0-9a-f]{64}', str(d.get('robot_signature', '')))):
            raise ValueError('Invalid RS trial spec, model hash, or fixed candidate policy')

    def check_robot(self, robot):
        if robot_signature(robot) != self.data['robot_signature']:
            raise ValueError('RS trial identity mismatch: calibration, mapping or robot settings changed; '
                             'do not reuse this trial spec on a different setup')


def check_can(config):
    """Read-only check before either arm connects; never configures the interface."""
    port = config.port
    if config.can_adapter != 'socketcan' or not re.fullmatch('[a-zA-Z0-9_-]+', port):
        raise ValueError('RS residual trial requires a SocketCAN interface such as can0')
    hint = (f'Check: ip -details link show {port}\n'
            f'If no other controller is running, configure CAN:\n'
            f'  sudo ip link set {port} down\n'
            f'  sudo ip link set {port} type can bitrate 1000000\n'
            f'  sudo ip link set {port} up')
    try:
        result = subprocess.run(['ip', '-json', '-details', 'link', 'show', 'dev', port],
                                check=True, capture_output=True, text=True, timeout=3)
        records = json.loads(result.stdout)
        if len(records) != 1:
            raise ValueError('Expected one CAN interface')
        entry = records[0]
        info = entry.get('linkinfo', {})
        details = info.get('info_data', {})
        if ('UP' not in entry.get('flags', []) or info.get('info_kind') != 'can'
                or details.get('bittiming', {}).get('bitrate') != 1000000
                or details.get('state') in ('BUS-OFF', 'STOPPED', 'SLEEPING')):
            raise ValueError('CAN must be UP at 1 Mbps and not bus-off')
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError) as e:
        raise ValueError(f'CAN preflight failed before arm connection: {e}\n{hint}') from e


def trial_config():
    # A one-mW deadband preserves the integer candidate mapping exactly.
    # Stall HOLD remains the separate original 1500 mW path.
    return Config(power_max_mw=600, power_deadband_mw=1).validate()


class TrialObserver(PreviewObserver):
    def __init__(self, limits):
        super().__init__(limits)
        self.policy = ShadowPolicy()
        self.valid = False
        self.ordinary_power_mw = [0]*7
        self.memory = None
        self.status = 'RS TRIAL: warming up; uncalibrated'

    def invalidate(self, reason):
        super().invalidate(reason)
        self.policy = ShadowPolicy()
        self.valid = False
        self.loads = [0.]*7
        self.ordinary_power_mw = [0]*7
        self.memory = None
        self.status = 'RS TRIAL: ' + reason + '; ordinary damping released'

    def update(self, q, v, tau, now):
        super().update(q, v, tau, now)
        self.valid = self.status == 'PREVIEW: steady 1s signed mean (uncalibrated)'
        # Keep the original offline policy unchanged. The physical trial has
        # four times its power, with a separate, bounded recent-load memory.
        powers = [4*p for p in self.policy.update(self.residual_nm, now, self.valid)]
        above = self.valid and any(abs(self.residual_nm[i]) > DEADZONE_NM[i] for i in (1,2,3))
        if self.valid and not above:
            powers = [0]*7
            self.memory = None
        elif self.valid and any(powers):
            self.memory = (now, list(q), list(powers))

        if self.memory is not None:
            when, pose, remembered = self.memory
            reason = None
            if now-when > POLICY['motion_memory_s']:
                reason = 'recent load expired; stop to recheck'
            elif any(abs(q[i]-pose[i]) > math.radians(POLICY['motion_pose_delta_deg']) for i in range(6)):
                reason = 'pose changed more than 20deg; stop to recheck'
            elif abs(q[6]-pose[6]) > math.radians(POLICY['gripper_delta_deg']):
                reason = 'gripper moved; stop to recheck'
            elif max(abs(x) for x in v[:6]) > .5:
                reason = 'motor speed exceeds 0.5rad/s; stop to recheck'
            if reason:
                self.memory = None
                powers = [0]*7
                self.status = 'RS TRIAL: ' + reason + '; ordinary damping released'
            elif not self.valid or (above and not any(powers)):
                powers = list(remembered)
                self.status = (f'RS TRIAL: RECENT LOAD {now-when:.1f}/3.0s; '
                               'using prior stationary load, current residual unavailable')

        self.ordinary_power_mw = powers
        # Feedback uses min + ratio*(max-min). Zero ratio means unlock;
        # a tiny positive ratio represents a genuine one-mW request.
        self.loads = [0. if p == 0 else max(1e-12, (p-1)/599.)
                      for p in self.ordinary_power_mw]
        self.status = self.status.replace('PREVIEW: steady 1s signed mean (uncalibrated)',
                                          'RS TRIAL: steady residual; uncalibrated')
        self.status = self.status.replace('PREVIEW:', 'RS TRIAL:', 1)
        return self.loads


class TrialSession(Session):
    def __init__(self, *args, spec, **kwargs):
        self.spec = spec
        super().__init__(*args, **kwargs)

    def __enter__(self):
        if self.collect_path or self.cfg.unloaded_profile or self.cfg != trial_config():
            raise ValueError('Residual trial requires the fixed trial config without capture/profile')
        self.spec.check_robot(self.robot)
        result = super().__enter__()
        try:
            self.feedback.observer = TrialObserver(self.robot.config.joint_limits)
            log.warning('EXPERIMENTAL RS TRIAL v2: ordinary damping J2/J3/J4 <=600mW; '
                        'stationary window 1s + confirmation 0.3s; recent-load memory <=3s / 20deg. '
                        'Gripper change, high speed, limits or feedback gaps release ordinary damping. '
                        'Independent original stall HOLD can request 1500mW on all joints. '
                        'Leader output %s.', 'ENABLED' if self.enabled else 'DISABLED')
            return result
        except BaseException:
            self._skip_fault_zero()
            self.close()
            raise

    def _log(self, states, errors, now):
        o = self.feedback.observer
        if self.csv:
            for i, n in enumerate(NAMES):
                mode, power, _ = self.feedback.sent[n]
                self.csv.writerow([now, n, states[n].torq, math.degrees(states[n].pos),
                                   errors[n], self.feedback.loads[n], hex(mode), power, self.enabled,
                                   self.feedback.damping_loads[n],
                                   o.residual_nm[i] if o.valid and i<6 else '', o.status])
        if now >= self.next_log:
            mode = ('HOLD' if self.feedback.holding else
                    'FREE' if all(v[0] == 0x10 for v in self.feedback.sent.values()) else 'DAMPING')
            powers = ', '.join(f'J{i+1}={self.feedback.sent[n][1]}mW' for i,n in enumerate(NAMES))
            log.info('RS TRIAL %s%s | %s | %s', mode,
                     '' if self.enabled else ' (simulated; no Leader output)', powers, o.status)
            if o.valid:
                log.info('RS TRIAL residual: %s', ', '.join(
                    f'J{i+1}={o.residual_nm[i]:+.3f}Nm' for i in (1,2,3)))
            self.next_log = now+1.
            if self.csv_file:
                self.csv_file.flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Experimental RS residual feedback; monitor by default',
                                     allow_abbrev=False)
    parser.add_argument('--trial-spec', required=True, help='Fixed identity-bound experimental trial JSON')
    parser.add_argument('--feedback-enable', action='store_true', help='Enable physical Leader output')
    parser.add_argument('--feedback-log', help='Create a NEW CSV, never overwrite')
    args, remaining = parser.parse_known_args(argv)
    if any(x.startswith(('--feedback', '--collect', '--confirm-unloaded', '--trial')) for x in remaining):
        parser.error('Use only trial-spec, feedback-enable and feedback-log for this fixed trial')
    if args.feedback_log and Path(args.feedback_log).exists():
        parser.error('Trial log exists; choose a new file')
    spec = TrialSpec(args.trial_spec)
    module = importlib.import_module('lerobot.scripts.lerobot_teleoperate')
    original, original_factory = module.teleop_loop, module.make_robot_from_config
    saved_argv = sys.argv
    try:
        def checked_factory(config):
            robot = original_factory(config)
            spec.check_robot(robot)  # Official factory is hardware-inert; both connects follow it.
            check_can(config)
            return robot

        module.make_robot_from_config = checked_factory
        install_runtime_wrapper(module, trial_config(), enabled=args.feedback_enable,
                                log_path=args.feedback_log,
                                session_factory=functools.partial(TrialSession, spec=spec))
        sys.argv = [saved_argv[0], *remaining]
        module.main()
    finally:
        module.teleop_loop = original
        module.make_robot_from_config = original_factory
        sys.argv = saved_argv


if __name__ == '__main__':
    main()
