"""Offline ordinary-damping candidate. No hardware imports or execution path.

Fixed conservative thresholds for an initial 1 kg trial, NOT a calibration
profile. Only J2/J3/J4 participate. This does not simulate stall holding.
"""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path

from .config import NAMES
from .unloaded import digest, vector

DEADZONE_NM = [0., 1., 1., .6, 0., 0.]
EXCESS_FULL_SCALE_NM = [1., 3., 5., 3., 1., 1.]


class ShadowPolicy:
    def __init__(self):
        self.last = None
        self.since = [None]*6
        self.ratios = [0.]*6

    def update(self, residual, now, valid):
        vector(residual, 6)
        if not math.isfinite(now):
            raise ValueError('Invalid shadow timestamp')
        dt = None if self.last is None else now-self.last
        if not valid or dt is None or not 0 < dt <= .25:
            self.since = [None]*6
            self.ratios = [0.]*6
        self.last = now
        if not valid:
            return [0]*7
        alpha = (dt if dt is not None and 0 < dt <= .25 else .033)
        alpha /= .15+alpha
        for i in (1,2,3):
            excess = abs(residual[i])-DEADZONE_NM[i]
            target = 0.
            if excess > 0:
                if self.since[i] is None:
                    self.since[i] = now
                if now-self.since[i] >= .3:
                    target = min(1., excess/EXCESS_FULL_SCALE_NM[i])
            else:
                self.since[i] = None
            self.ratios[i] += alpha*(target-self.ratios[i])
        return [round(150*x) for x in self.ratios]+[0]


def replay(path):
    raw = Path(path).read_bytes()
    policy = ShadowPolicy()
    result = []
    first = last = None
    with Path(path).open(newline='', encoding='utf-8') as f:
        groups = itertools.groupby(csv.DictReader(f), lambda r:r['monotonic_s'])
        for ts, group in groups:
            rows = list(group)
            d = {r['joint']:r for r in rows}
            if len(rows)!=7 or set(d)!=set(NAMES):
                raise ValueError('Expected one complete seven-joint preview frame')
            t = float(ts)
            if not math.isfinite(t) or (last is not None and t<=last):
                raise ValueError('Invalid/nonmonotonic preview time')
            last = t
            if first is None:
                first = t
            if any(r['output_enabled']!='False' for r in rows):
                raise ValueError('Expected a preview with output disabled')
            status = d[NAMES[0]]['unloaded_status']
            valid = status=='PREVIEW: steady 1s signed mean (uncalibrated)'
            if any(r['unloaded_status']!=status for r in rows):
                raise ValueError('Inconsistent frame status')
            residual = [float(d[n]['residual_torque_nm']) for n in NAMES[:6]] if valid else [0.]*6
            powers = policy.update(residual,t,valid)
            result.append(dict(elapsed_s=t-first,valid=valid,residual_nm=residual if valid else None,
                               ordinary_power_mw=powers))
    if not result:
        raise ValueError('Empty preview')
    return dict(source=str(path),source_sha256=digest(raw),offline_only=True,
                policy=dict(deadzone_nm=DEADZONE_NM,excess_full_scale_nm=EXCESS_FULL_SCALE_NM,
                            max_ordinary_power_mw=150,confirmation_s=.3,smoothing_s=.15),
                limitation='Candidate only. No hardware output, calibration acceptance, or stall simulation.',
                frames=result)


def main():
    p=argparse.ArgumentParser(description='Offline residual-to-damping candidate replay; never controls hardware')
    p.add_argument('preview_csv')
    p.add_argument('--output',required=True)
    args=p.parse_args()
    try:
        output=Path(args.output)
        if output.exists():
            p.error('Output exists; choose a new file')
        report=replay(args.preview_csv)
        with output.open('x',encoding='utf-8') as f:
            json.dump(report,f,indent=2,allow_nan=False)
        print(f'Offline only: {len(report["frames"])} frames; saved {output}')
    except (ValueError,KeyError,OSError) as e:
        p.exit(1,f'Replay failed: {e}\n')


if __name__=='__main__':main()
