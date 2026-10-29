#!/usr/bin/env python3
"""Score a sim run on the active part of the drive, not the whole recording.

If the open-loop sim leaves the line and stops, the full 281 s window is mostly
stationary and the correlation collapses for reasons that have nothing to do
with the vehicle model.  This script reports the same metrics over the window
where the sim was actually moving, plus integrated yaw / longitudinal dv.
"""
from __future__ import annotations

import argparse
import os

import numpy as np

import compare_imu as ci

WINDOWS = [(20, 60), (20, 95), (20, 160), (20, 281)]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--sim-imu', default='sim_imu.csv')
    p.add_argument('--sim-control', default='sim_control.csv')
    p.add_argument('--tag', default='run')
    p.add_argument('--dir', default='.')
    a = p.parse_args()
    d = a.dir

    real = ci.load_imu(os.path.join(d, 'real_imu.csv'))
    rc = ci.load_control(os.path.join(d, 'real_control.csv'))
    on_r = ci.first_onset(rc)

    sim = ci.load_imu(os.path.join(d, a.sim_imu))
    sc = ci.load_control(os.path.join(d, a.sim_control))
    on_s = ci.first_onset(sc)

    end = min(rc['t_rel'][-1] - on_r, sc['t_rel'][-1] - on_s)
    grid = np.arange(-20.0, end, ci.DT)
    R = ci.interp_on_grid(real, on_r, grid)
    S = ci.interp_on_grid(sim, on_s, grid)
    st = (grid >= -15) & (grid < -1)
    for k in ci.ALL:
        R[k] -= R[k][st].mean()
        S[k] -= S[k][st].mean()

    lines = [f'# Active-window IMU comparison: {a.tag}\n',
             f'- real onset: {on_r:.2f} s, sim onset: {on_s:.2f} s',
             f'- compared to tau = {end:.1f} s\n']
    for lo, hi in WINDOWS:
        m = (grid >= lo) & (grid < hi)
        if m.sum() < 200:
            continue
        lines.append(f'## tau {lo}-{hi} s (n={int(m.sum())})\n')
        lines.append('| signal | real std | sim std | corr | gain sim/real | RMSE |')
        lines.append('|---|---:|---:|---:|---:|---:|')
        for k in ci.ALL:
            x = ci.axis_metrics(R[k][m], S[k][m])
            lines.append(f'| {k} | {x["real_std"]:.3f} | {x["sim_std"]:.3f} | '
                         f'{x["corr"]:+.3f} | {x["gain"]:+.3f} | {x["rmse"]:.3f} |')
        m0 = (grid >= 0) & (grid < hi)
        lines.append(f'\n- yaw change real {ci.trapz(R["gz"][m0], grid[m0]):+.3f} rad, '
                     f'sim {ci.trapz(S["gz"][m0], grid[m0]):+.3f} rad')
        lines.append(f'- longitudinal dv real {ci.trapz(R["ax"][m0], grid[m0]):+.2f} m/s, '
                     f'sim {ci.trapz(S["ax"][m0], grid[m0]):+.2f} m/s\n')

    text = '\n'.join(lines)
    print(text)
    with open(os.path.join(d, f'active_{a.tag}.md'), 'w') as fh:
        fh.write(text + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
