"""Read-only Leader preview of a one-second signed gravity residual.

Follower teleoperation remains active. This is NOT a calibrated load estimator;
this entry point deliberately has no option to enable Leader force output.
"""
import argparse
import collections
import importlib
import logging
import math
from pathlib import Path
import sys

from .config import Config, NAMES
from .gravity import GravityModel
from .launcher import install_runtime_wrapper
from .session import Session
from .unloaded import vector

log = logging.getLogger(__name__)


class PreviewObserver:
    def __init__(self, limits):
        self.limits = limits
        self.model = GravityModel()
        self.history = collections.deque()
        self.last = None
        self.loads = [0.] * 7
        self.residual_nm = [0.] * 6
        self.status = 'PREVIEW: warming up'

    def invalidate(self, reason):
        self.history.clear()
        self.last = None
        self.residual_nm = [0.] * 6
        self.status = 'PREVIEW: ' + reason

    def update(self, q, v, tau, now):
        vector(q, 7); vector(v, 7); vector(tau, 7)
        if not math.isfinite(now):
            raise ValueError('Invalid preview time')
        if self.last is not None and not 0 < now-self.last <= .25:
            self.invalidate('feedback gap')
        self.last = now
        outside = [f'J{i+1}={math.degrees(q[i]):.1f}deg '
                   f'(interior {self.limits[n][0]+2:.1f}..{self.limits[n][1]-2:.1f}deg)'
                   for i, n in enumerate(NAMES[:6])
                   if not math.radians(self.limits[n][0]+2) <= q[i]
                   <= math.radians(self.limits[n][1]-2)]
        if outside:
            self.invalidate('near joint limit: ' + ', '.join(outside))
            return self.loads
        gravity, _ = self.model.evaluate(q)
        residual = [a-b for a, b in zip(tau[:6], gravity)]
        self.history.append((now, list(q), list(v), residual))
        cutoff = now-1.
        while len(self.history)>1 and self.history[1][0] <= cutoff:
            self.history.popleft()
        stable = (now-self.history[0][0] >= 1.
                  and all(max(r[1][i] for r in self.history)-min(r[1][i] for r in self.history)
                          <= math.radians(.7 if i<6 else 3.) for i in range(7))
                  and max(abs(x) for r in self.history for x in r[2][:6]) <= .5)
        if not stable:
            if now-self.history[0][0] < 1.:
                reason = f'collecting stationary window ({now-self.history[0][0]:.1f}/1.0s)'
            else:
                changes = [(i, math.degrees(max(r[1][i] for r in self.history)
                                           -min(r[1][i] for r in self.history))) for i in range(7)]
                details = [f'J{i+1} span={span:.1f}deg' for i, span in changes
                           if span > (.7 if i<6 else 3.)]
                if max(abs(x) for r in self.history for x in r[2][:6]) > .5:
                    details.append('motor speed exceeds 0.5rad/s')
                reason = ', '.join(details)
            self.status = 'PREVIEW: moving/settling; residual unavailable: ' + reason
            self.residual_nm = [0.] * 6
            return self.loads
        # Integrate the signed residual over exactly one second. Clip the oldest
        # interval and interpolate its boundary, so irregular FPS cannot bias it.
        total = [0.] * 6
        for a, b in zip(self.history, list(self.history)[1:]):
            start = max(cutoff, a[0])
            dt = b[0]-start
            if dt <= 0:
                continue
            fraction = (start-a[0])/(b[0]-a[0])
            for i in range(6):
                left = a[3][i]+fraction*(b[3][i]-a[3][i])
                total[i] += .5*(left+b[3][i])*dt
        self.residual_nm = total
        self.status = 'PREVIEW: steady 1s signed mean (uncalibrated)'
        return self.loads


class PreviewSession(Session):
    def __enter__(self):
        if self.enabled or self.collect_path or self.cfg.unloaded_profile:
            raise ValueError('Preview requires no Leader output, capture, or calibrated profile')
        result = super().__enter__()
        try:
            self.feedback.observer = PreviewObserver(self.robot.config.joint_limits)
            log.warning('RESIDUAL PREVIEW ONLY: Leader force output disabled; '
                        'follower teleoperation ACTIVE; values include model error and friction')
            return result
        except BaseException:
            self._skip_fault_zero()
            self.close()
            raise

    def _log(self, states, errors, now):
        # Keep the ordinary diagnostic CSV schema, but do not describe zero
        # simulated damping as an accepted empty-arm result.
        if self.csv:
            o = self.feedback.observer
            for i, n in enumerate(NAMES):
                self.csv.writerow([now, n, states[n].torq, math.degrees(states[n].pos),
                                   errors[n], self.feedback.loads[n], '', '', False, '',
                                   o.residual_nm[i] if i<6 and 'steady 1s' in o.status else '', o.status])
        if now >= self.next_log:
            o = self.feedback.observer
            values = (', '.join(f'J{i+1}={x:+.3f}Nm' for i,x in enumerate(o.residual_nm))
                      if 'steady 1s' in o.status else 'wait for a stationary one-second window')
            log.info('%s | %s | NO LEADER OUTPUT', o.status, values)
            if 'near joint limit' in o.status:
                log.info('PREVIEW: manually choose a safe interior pose; zero/home pose '
                         'does not provide a valid residual. Keep all joints still for at least 1s.')
            self.next_log = now+1.
            if self.csv_file:
                self.csv_file.flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Uncalibrated signed-residual preview; no Leader force output',
                                     allow_abbrev=False)
    parser.add_argument('--preview-log', help='Create a NEW CSV (never overwrite)')
    args, remaining = parser.parse_known_args(argv)
    if any(x.startswith(('--feedback', '--collect', '--confirm-unloaded')) for x in remaining):
        parser.error('Preview does not accept feedback enable/config/capture flags')
    if args.preview_log and Path(args.preview_log).exists():
        parser.error('Preview log exists; choose a new file')
    module = importlib.import_module('lerobot.scripts.lerobot_teleoperate')
    original, saved_argv = module.teleop_loop, sys.argv
    try:
        install_runtime_wrapper(module, Config(), enabled=False, log_path=args.preview_log,
                                session_factory=PreviewSession)
        sys.argv = [saved_argv[0], *remaining]
        module.main()
    finally:
        module.teleop_loop = original
        sys.argv = saved_argv


if __name__ == '__main__':
    main()
