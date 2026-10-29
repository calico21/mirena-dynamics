#!/usr/bin/env python3
"""Speed and steer->yaw-gain analysis for the real vs simulated run.

`compare_imu.py` answers "do the IMU channels match".  This answers the two
questions that explain *why*: how fast was each car, and does the yaw gain grow
with speed the way a bicycle model says it should.

Real speed comes from /sensors/imu/odom extracted from the bag; sim speed comes
from /debug/state/car (u) captured alongside the re-record.
"""
from __future__ import annotations

import argparse
import csv
import os
from typing import Dict, Optional

import numpy as np

BINS = [0.0, 2.0, 5.0, 8.0, 12.0, 16.0, 20.0, 25.0, 30.0]


def load(path: str) -> Optional[Dict[str, np.ndarray]]:
    if not os.path.exists(path):
        return None
    with open(path, newline='') as fh:
        r = csv.DictReader(fh)
        cols = {k: [] for k in r.fieldnames}
        for row in r:
            for k in cols:
                cols[k].append(row[k])
    out = {}
    for k, v in cols.items():
        try:
            out[k] = np.asarray(v, dtype=float)
        except (TypeError, ValueError):
            continue  # frame_id and other string columns
    return out


def interp(x: np.ndarray, y: np.ndarray, xq: np.ndarray) -> np.ndarray:
    keep = np.r_[True, np.diff(x) > 0]
    return np.interp(xq, x[keep], y[keep])


def first_onset(t: np.ndarray, gas: np.ndarray) -> float:
    idx = np.flatnonzero(gas > 0.1)
    return float(t[idx[0]]) if len(idx) else float(t[0])


def gain(steer: np.ndarray, gz: np.ndarray) -> tuple[float, float, int]:
    m = np.isfinite(steer) & np.isfinite(gz)
    n = int(m.sum())
    if n < 20 or np.ptp(steer[m]) < 1e-6:
        return float('nan'), float('nan'), n
    A = np.vstack([steer[m], np.ones(n)]).T
    coef, *_ = np.linalg.lstsq(A, gz[m], rcond=None)
    pred = A @ coef
    ss_res = float(((gz[m] - pred) ** 2).sum())
    ss_tot = float(((gz[m] - gz[m].mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return float(coef[0]), r2, n


def bin_table(name: str, tau: np.ndarray, steer: np.ndarray,
              gz: np.ndarray, speed: np.ndarray) -> list[str]:
    lines = [f'### {name}\n',
             '| speed bin [m/s] | samples | steer rms [rad] | gz std [rad/s] | gain [1/s] | R2 |',
             '|---|---:|---:|---:|---:|---:|']
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        m = (tau >= 0) & (speed >= lo) & (speed < hi)
        if m.sum() >= 20:
            g, r2, _ = gain(steer[m], gz[m])
            lines.append(f'| {lo:.0f}-{hi:.0f} | {int(m.sum())} | '
                         f'{np.sqrt(np.mean(steer[m] ** 2)):.3f} | '
                         f'{gz[m].std():.3f} | {g:.2f} | {r2:.2f} |')
    g, r2, n = gain(steer[tau >= 0], gz[tau >= 0])
    lines.append(f'\nOverall drive gain: **{g:.2f}** 1/s (R2 {r2:.2f}, n={n})\n')
    return lines


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--dir', default='.')
    a = p.parse_args()
    d = a.dir

    real_imu = load(os.path.join(d, 'real_imu.csv'))
    real_ctl = load(os.path.join(d, 'real_control.csv'))
    real_odom = load(os.path.join(d, 'real_odom.csv'))
    real_wheels = load(os.path.join(d, 'real_wheels.csv'))
    sim_imu = load(os.path.join(d, 'sim_imu.csv'))
    sim_ctl = load(os.path.join(d, 'sim_control.csv'))
    sim_state = load(os.path.join(d, 'sim_state.csv'))

    out = ['# Speed and yaw-gain analysis\n']

    if real_imu is not None and real_ctl is not None:
        on = first_onset(real_ctl['t_unix'], real_ctl['gas'])
        tau = real_imu['t_unix'] - on
        steer = interp(real_ctl['t_unix'] - on, real_ctl['steer_rad'], tau)
        spd = None
        source = ''
        if real_wheels is not None:
            # Rear wheel speed is RPM; driven wheels read ~7% high through slip,
            # so calibrate against odometry where the two overlap.
            vw = 0.5 * (real_wheels['rl'] + real_wheels['rr']) * 2 * np.pi / 60 * 0.2032
            k = 1.0
            if real_odom is not None:
                vo = np.hypot(real_odom['vx'], real_odom['vy'])
                vwi = interp(real_wheels['t'], vw, real_odom['t'])
                m = np.isfinite(vwi) & (vwi > 1.0)
                if m.sum() > 100:
                    k = float(np.dot(vwi[m], vo[m]) / np.dot(vwi[m], vwi[m]))
            # 't' is bag receive time, same epoch as the control header stamp.
            spd = interp(real_wheels['t'] - on, vw * k, tau)
            source = (f'rear wheel RPM x 2*pi/60 x 0.2032, '
                      f'calibrated k={k:.3f} against odom')
        elif real_odom is not None:
            tau_o = real_odom['t'] - on
            speed = np.hypot(real_odom['vx'], real_odom['vy'])
            spd = interp(tau_o, speed, tau)
            source = 'odometry twist magnitude'
        if spd is not None:
            out.append('## Real car\n')
            out.append(f'- drive onset (bag header): {on:.2f} s')
            out.append(f'- speed source: {source}')
            out.append(f'- speed p50/p95/max: {np.percentile(spd[tau>=0],50):.2f} / '
                       f'{np.percentile(spd[tau>=0],95):.2f} / {spd[tau>=0].max():.2f} m/s')
            for thr in (5, 10, 15, 20, 25):
                out.append(f'- time above {thr:2d} m/s: '
                           f'{np.mean(spd[tau>=0] > thr)*100:5.1f} %')
            out.append('')
            out.extend(bin_table('Real', tau, steer, real_imu['gz'], spd))
        else:
            out.append('## Real car\n\n(no real speed source; speed table skipped)\n')

    if sim_imu is not None and sim_ctl is not None and sim_state is not None:
        on = first_onset(sim_ctl['t_rel'], sim_ctl['gas'])
        tau = sim_imu['t_rel'] - on
        steer = interp(sim_ctl['t_rel'] - on, sim_ctl['steer_rad'], tau)
        # sim IMU t_unix is sim time; fit wall t_rel against it.
        a_fit, b_fit = np.polyfit(sim_imu['t_unix'], sim_imu['t_rel'], 1)
        tau_s = (a_fit * sim_state['t'] + b_fit) - on
        spd = interp(tau_s, np.abs(sim_state['u']), tau)
        out.append('## Simulator\n')
        out.append(f'- wall/sim time slope: {a_fit:.6f} (1.0 = real-time)')
        out.append(f'- drive onset (wall): {on:.2f} s')
        out.append(f'- speed p50/p95/max: {np.percentile(spd[tau>=0],50):.2f} / '
                   f'{np.percentile(spd[tau>=0],95):.2f} / {spd[tau>=0].max():.2f} m/s')
        for thr in (5, 10, 15, 20, 25):
            out.append(f'- time above {thr:2d} m/s: '
                       f'{np.mean(spd[tau>=0] > thr)*100:5.1f} %')
        out.append('')
        out.extend(bin_table('Simulator', tau, steer, sim_imu['gz'], spd))

    text = '\n'.join(out)
    print(text)
    with open(os.path.join(d, 'speed_analysis.md'), 'w') as fh:
        fh.write(text + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
