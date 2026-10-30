"""离线拟合与独立空载验收；此模块不打开任何硬件接口。"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from .config import Config, NAMES
from .gravity import GravityModel
from .unloaded import (ALGORITHM, RADIUS, SIGMA_MAX, Observer, Profile, canonical,
                       digest, features, vector, within)


def read_capture(path):
    path = Path(path)
    raw = path.read_bytes()
    meta = json.loads(path.with_suffix('.meta.json').read_text(encoding='utf-8'))
    if (meta.get('algorithm') != ALGORITHM or meta.get('confirmed_unloaded') is not True
            or meta.get('upright_stock_gripper') is not True or meta.get('complete') is not True
            or meta.get('csv_sha256') != digest(raw)
            or meta.get('model_sha256') != GravityModel().sha256):
        raise ValueError(f'{path}: incomplete, changed, or not explicitly unloaded capture')
    rows, last, smoothed = [], None, None
    with path.open(newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['t'])
            q, v, tau = ([float(row[f'{key}{i}']) for i in range(1, 8)] for key in ('q', 'v', 'tau'))
            vector(q, 7); vector(v, 7); vector(tau, 7)
            if not math.isfinite(t) or (last is not None and t <= last[0]):
                raise ValueError(f'{path}: invalid/non-monotonic timestamp')
            if last is None or t-last[0] > .25:
                smoothed = list(v)
                last = t, q
                continue
            dt = t-last[0]
            if dt < .05:
                raise ValueError(f'{path}: capture cadence too high')
            newv = [a+dt/(.1+dt)*(b-a) for a, b in zip(smoothed, v)]
            accel = max(abs(a-b)/dt for a, b in zip(newv[:6], smoothed[:6]))
            q_speed = max(abs(a-b)/dt for a, b in zip(q[:6], last[1][:6]))
            smoothed, last = newv, (t, q)
            limits = meta['joint_limits_deg']
            interior = all(math.radians(limits[n][0]+2) <= q[i] <= math.radians(limits[n][1]-2)
                           for i, n in enumerate(NAMES[:6]))
            if max(map(abs, v[:6])) <= .25 and q_speed <= .25 and accel <= .8 and interior:
                rows.append(dict(t=t, q=q, v=list(newv), v_raw=v, tau=tau))
        count = reader.line_num-1
    if count != meta['sample_count']:
        raise ValueError('Capture sample count mismatch')
    return meta, rows


def pose_centers(rows):
    poses, keys = [], set()
    for row in rows:
        q = row['q']
        key = tuple(round(x/math.radians(.5)) for x in q)
        if key not in keys:
            keys.add(key)
            poses.append(q)
    if len(poses) > 3000:
        raise ValueError('Too many calibration poses; split capture into smaller working regions')
    return poses


def fit(training_path, validation_path):
    # numpy 仅用于离线最小二乘；运行时仍为纯 Python。
    import numpy as np
    tm, train = read_capture(training_path)
    vm, check = read_capture(validation_path)
    if (tm['robot_signature'] != vm['robot_signature']
            or tm['run_id'] == vm['run_id'] or tm['csv_sha256'] == vm['csv_sha256']):
        raise ValueError('Need two independent captures of the same follower calibration/configuration')
    if len(train) < 120 or len(check) < 80:
        raise ValueError(f'Need >=120 train and >=80 check usable samples; got {len(train)} / {len(check)}')
    for title, rows in (('train', train), ('check', check)):
        for i in (1, 2, 3):
            span = max(x['q'][i] for x in rows)-min(x['q'][i] for x in rows)
            if span < math.radians(15):
                raise ValueError(f'{title}: J{i+1} needs >=15 degrees of unloaded pose variation')
    model = GravityModel()
    g_rows = [model.evaluate(row['q'])[0] for row in train]
    coeff, axes, noise = [], [], []
    for i in range(6):
        x = np.asarray([features(g[i], row['v'][i]) for row, g in zip(train, g_rows)])
        y = np.asarray([row['tau'][i] for row in train])
        cols = [0]
        if float(np.std(x[:, 1])) >= .08:
            axes.append(i)
            cols.append(1)
        for c, threshold in ((2, .008), (3, .2)):
            if float(np.std(x[:, c])) >= threshold:
                cols.append(c)
        xx = x[:, cols]
        scale = np.sqrt(np.mean(xx*xx, axis=0))
        scaled = xx/scale
        if float(np.linalg.cond(scaled)) > 1e4:
            raise ValueError(f'J{i+1}: poorly identified model; vary poses/speeds independently')
        solution = np.linalg.lstsq(scaled, y, rcond=None)[0]/scale
        beta = [0.]*4
        for c, b in zip(cols, solution):
            beta[c] = float(b)
        if i in axes and not .2 <= abs(beta[1]) <= 3:
            raise ValueError(f'J{i+1}: gravity sign/scale inconsistent; check zero, tool and model mapping')
        coeff.append(beta)
        residual = np.abs(y-x@np.asarray(beta))
        sigma = max(.03, float(np.quantile(residual, .95)))
        if sigma > SIGMA_MAX[i]:
            raise ValueError(f'J{i+1}: training residual noise {sigma:.3f}Nm too high; inspect data/model')
        noise.append(sigma)
    if not {1, 2, 3} <= set(axes):
        raise ValueError('Need gravity excitation at J2/J3/J4; sample more varied unloaded arm/wrist poses')
    train_poses = pose_centers(train)
    covered = [r for r in check if within(r['q'], train_poses)]
    if len(covered) < 80 or len(covered)/len(check) < .7:
        raise ValueError('Independent check has insufficient overlap with training pose neighborhoods')
    d = dict(algorithm=ALGORITHM, model_sha256=model.sha256,
             robot_signature=tm['robot_signature'], joint_limits_deg=tm['joint_limits_deg'],
             gravity_axes=axes, coefficients=coeff, sigma_nm=noise,
             validated_poses_rad=pose_centers(covered),
             training_samples=len(train), training_csv_sha256=tm['csv_sha256'],
             validation_csv_sha256=vm['csv_sha256'], training_run_id=tm['run_id'],
             validation_run_id=vm['run_id'], validated=False)
    candidate = Profile(data=d, require_validated=False)
    observer = Observer(candidate, Config().residual_reference_nm)
    false_frames, steady = 0, 0
    covered_times = {r['t'] for r in covered}
    for row in check:
        if row['t'] not in covered_times:
            observer.invalidate('outside training coverage')
            continue
        loads = observer.update(row['q'], row['v_raw'], row['tau'], row['t'])
        steady += observer.status == 'steady residual'
        false_frames += any(x > 0 for x in loads)
    d['validation'] = dict(usable_samples=len(check), covered_samples=len(covered),
                           coverage_fraction=len(covered)/len(check), steady_samples=steady,
                           false_activation_frames=false_frames,
                           noise_source='training_only_p95_absolute_residual',
                           evidence_sigma=3., confirmation_s=.2)
    d['validated'] = steady >= 40 and false_frames == 0
    d['integrity_sha256'] = digest(canonical(d))
    return d


def main(argv=None):
    parser = argparse.ArgumentParser(description='Offline RS unloaded model fitting; never opens hardware')
    parser.add_argument('--train', required=True)
    parser.add_argument('--check', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--feedback-config', default='feedback.unloaded.json')
    args = parser.parse_args(argv)
    output, config_path = Path(args.output), Path(args.feedback_config)
    rejected = output.with_suffix('.rejected.json')
    if output.exists() or config_path.exists() or rejected.exists():
        parser.error('Output already exists; choose new names instead of overwriting')
    data = fit(args.train, args.check)
    if not data['validated']:
        rejected.parent.mkdir(parents=True, exist_ok=True)
        with rejected.open('x', encoding='utf-8') as f:
            json.dump(data, f, indent=2, allow_nan=False)
        raise SystemExit(f'Unloaded validation FAILED; report: {rejected}; no feedback config generated')
    Profile(data=data)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as f:
        json.dump(data, f, indent=2, allow_nan=False)
    # 配置使用相对模型路径，可随交付目录整体移动。
    import os
    from dataclasses import asdict
    cfg = asdict(Config())
    cfg['unloaded_profile'] = os.path.relpath(output.resolve(), config_path.resolve().parent)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    with config_path.open('x', encoding='utf-8') as f:
        json.dump(cfg, f, indent=2, allow_nan=False)
    print(f'Independent unloaded validation PASSED: {data["validation"]}')
    print(f'Generated {config_path}; test monitor mode before enabling Leader feedback')


if __name__ == '__main__':
    main()
