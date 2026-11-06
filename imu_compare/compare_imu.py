#!/usr/bin/env python3
"""Compare the real-bag IMU CSV with the simulator IMU CSV.

Alignment:
  * tau = t_rel - first control sample with gas > 0.1
  * both runs resampled onto a common 100 Hz tau grid
  * static bias removed before dynamic metrics

Outputs (in --out):
  metrics.csv, summary.md, plots/*.png
"""
from __future__ import annotations

import argparse
import csv
import math
import os
from typing import Dict, Tuple

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

G0 = 9.80665
FS = 100.0
DT = 1.0 / FS
AX = ('ax', 'ay', 'az')
GY = ('gx', 'gy', 'gz')
ALL = AX + GY
COLORS = {'real': '#2a78d6', 'sim': '#eb6834'}


def load_imu(path: str) -> Dict[str, np.ndarray]:
    cols = {}
    with open(path, newline='') as fh:
        r = csv.DictReader(fh)
        for name in ('t_unix', 't_rel') + ALL + ('qx', 'qy', 'qz', 'qw'):
            cols[name] = []
        frames = []
        for row in r:
            for name in cols:
                cols[name].append(float(row[name]))
            frames.append(row.get('frame_id', ''))
    out = {k: np.asarray(v, dtype=float) for k, v in cols.items()}
    out['frame_id'] = frames[0] if frames else ''
    return out


def load_control(path: str) -> Dict[str, np.ndarray]:
    cols = {'t_unix': [], 't_rel': [], 'gas': [], 'steer_raw': [], 'steer_rad': []}
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            for name in cols:
                cols[name].append(float(row[name]))
    return {k: np.asarray(v, dtype=float) for k, v in cols.items()}


def first_onset(ctrl: Dict[str, np.ndarray]) -> float:
    mask = ctrl['gas'] > 0.1
    idx = np.flatnonzero(mask)
    if len(idx):
        return float(ctrl['t_rel'][idx[0]])
    mask = np.abs(ctrl['steer_rad']) > 0.05
    idx = np.flatnonzero(mask)
    if len(idx):
        return float(ctrl['t_rel'][idx[0]])
    return float(ctrl['t_rel'][0])


def interp_on_grid(imu: Dict[str, np.ndarray], onset: float,
                   grid: np.ndarray) -> Dict[str, np.ndarray]:
    x = imu['t_rel'] - onset
    # keep a strictly increasing axis
    keep = np.r_[True, np.diff(x) > 0]
    x = x[keep]
    out = {}
    for name in ALL + ('qx', 'qy', 'qz', 'qw'):
        out[name] = np.interp(grid, x, imu[name][keep])
    return out


def corr_lag(ref: np.ndarray, sim: np.ndarray) -> Tuple[float, float]:
    if ref.size < 10:
        return float('nan'), 0.0
    a = ref - ref.mean()
    b = sim - sim.mean()
    denom = math.sqrt(float(np.dot(a, a) * np.dot(b, b)))
    if denom <= 0:
        return float('nan'), 0.0
    c = signal.correlate(b, a, mode='full', method='fft') / denom
    lags = signal.correlation_lags(len(b), len(a), mode='full')
    i = int(np.argmax(np.abs(c)))
    return float(lags[i] * DT), float(c[i])


def axis_metrics(ref: np.ndarray, sim: np.ndarray) -> Dict[str, float]:
    ref = ref - ref.mean()
    sim = sim - sim.mean()
    out = {
        'real_std': float(ref.std()),
        'sim_std': float(sim.std()),
        'rmse': float(np.sqrt(np.mean((sim - ref) ** 2))),
        'corr': float(np.corrcoef(ref, sim)[0, 1]) if ref.size > 2 else float('nan'),
        'gain': float(np.dot(sim, ref) / np.dot(ref, ref)) if np.dot(ref, ref) > 0 else float('nan'),
    }
    lag, _ = corr_lag(ref, sim)
    out['lag_s'] = lag
    return out


def trapz(y: np.ndarray, x: np.ndarray) -> float:
    if y.size < 2:
        return 0.0
    return float(np.sum(0.5 * (y[1:] + y[:-1]) * np.diff(x)))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--real-imu', required=True)
    p.add_argument('--sim-imu', required=True)
    p.add_argument('--real-control', required=True)
    p.add_argument('--sim-control', required=True)
    p.add_argument('--out', default='.')
    args = p.parse_args()

    os.makedirs(os.path.join(args.out, 'plots'), exist_ok=True)
    real = load_imu(args.real_imu)
    sim = load_imu(args.sim_imu)
    rc = load_control(args.real_control)
    sc = load_control(args.sim_control)

    onset_r = first_onset(rc)
    onset_s = first_onset(sc)
    end = min(rc['t_rel'][-1] - onset_r, sc['t_rel'][-1] - onset_s)
    grid = np.arange(-20.0, end, DT)

    R = interp_on_grid(real, onset_r, grid)
    S = interp_on_grid(sim, onset_s, grid)

    # static bias from the pre-onset window
    static = (grid >= -15.0) & (grid < -1.0)
    bias_r = {k: float(R[k][static].mean()) for k in ALL}
    bias_s = {k: float(S[k][static].mean()) for k in ALL}
    for k in ALL:
        R[k] = R[k] - bias_r[k]
        S[k] = S[k] - bias_s[k]

    drive = grid >= 0.0
    launch = (grid >= 0.0) & (grid < 20.0)
    main_phase = grid >= 20.0
    phases = {'static': static, 'launch': launch, 'drive': main_phase,
              'all_drive': drive}

    rows = []
    for phase, mask in phases.items():
        for name in ALL:
            if mask.sum() < 5:
                continue
            m = axis_metrics(R[name][mask], S[name][mask])
            rows.append({
                'phase': phase, 'signal': name,
                'real_mean': float(R[name][mask].mean()),
                'sim_mean': float(S[name][mask].mean()),
                'real_std': m['real_std'], 'sim_std': m['sim_std'],
                'rmse': m['rmse'], 'corr': m['corr'],
                'gain_sim_over_real': m['gain'], 'lag_s': m['lag_s'],
            })

    with open(os.path.join(args.out, 'metrics.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Rate / count diagnostics
    def rate(d):
        dt = np.diff(d['t_rel'])
        dt = dt[dt > 0]
        return 1.0 / float(np.median(dt)) if dt.size else float('nan')

    yaw_r = trapz(R['gz'][drive], grid[drive])
    yaw_s = trapz(S['gz'][drive], grid[drive])
    dv_r = trapz(R['ax'][drive], grid[drive])
    dv_s = trapz(S['ax'][drive], grid[drive])

    # Plots -----------------------------------------------------------------
    plt.rcParams.update({'font.size': 9, 'axes.grid': True,
                         'grid.alpha': 0.25, 'figure.dpi': 140})
    t = grid

    def time_plot(names, ylabel, fname, title):
        fig, axes = plt.subplots(len(names), 1, figsize=(11, 2.2 * len(names)),
                                 sharex=True)
        axes = np.atleast_1d(axes)
        for ax, name in zip(axes, names):
            ax.plot(t, R[name], color=COLORS['real'], lw=0.9, label='real bag')
            ax.plot(t, S[name], color=COLORS['sim'], lw=0.9, alpha=0.85,
                    label='sim')
            ax.axvspan(-20, 0, color='0.85', alpha=0.35, lw=0)
            ax.axvline(0, color='0.3', lw=0.8, ls='--')
            ax.set_ylabel(f'{name} [{ylabel}]')
        axes[0].legend(loc='upper right', ncol=2)
        axes[-1].set_xlabel('maneuver time since first throttle [s]')
        axes[0].set_title(title, loc='left')
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, 'plots', fname))
        plt.close(fig)

    time_plot(AX, 'm/s²', 'accel_vs_time.png',
              'Linear acceleration, bias removed (grey = static window)')
    time_plot(GY, 'rad/s', 'gyro_vs_time.png',
              'Angular velocity, bias removed (grey = static window)')

    def psd_plot(names, fname, title):
        fig, axes = plt.subplots(len(names), 1, figsize=(11, 2.2 * len(names)),
                                 sharex=True)
        axes = np.atleast_1d(axes)
        for ax, name in zip(axes, names):
            for label, d in (('real bag', R), ('sim', S)):
                f, pxx = signal.welch(d[name][drive], fs=FS, nperseg=2048)
                ax.semilogy(f, pxx, color=COLORS['real' if label == 'real bag' else 'sim'],
                            lw=0.9, label=label)
            ax.set_ylabel(f'{name} [unit²/Hz]')
        axes[0].legend(loc='upper right', ncol=2)
        axes[-1].set_xlabel('frequency [Hz]')
        axes[0].set_title(title, loc='left')
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, 'plots', fname))
        plt.close(fig)

    psd_plot(AX, 'accel_psd.png', 'Acceleration PSD')
    psd_plot(GY, 'gyro_psd.png', 'Angular velocity PSD')

    yaw_cum_r = np.cumsum(np.r_[0.0, 0.5 * (R['gz'][1:] + R['gz'][:-1]) * np.diff(t)])
    yaw_cum_s = np.cumsum(np.r_[0.0, 0.5 * (S['gz'][1:] + S['gz'][:-1]) * np.diff(t)])
    fig, axes = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
    axes[0].plot(t, R['gz'], color=COLORS['real'], lw=0.8, label='real bag')
    axes[0].plot(t, S['gz'], color=COLORS['sim'], lw=0.8, label='sim')
    axes[0].set_ylabel('yaw rate [rad/s]')
    axes[0].legend()
    axes[1].plot(t, yaw_cum_r, color=COLORS['real'], lw=1.2, label='real bag')
    axes[1].plot(t, yaw_cum_s, color=COLORS['sim'], lw=1.2, label='sim')
    axes[1].set_ylabel('integrated yaw [rad]')
    axes[1].set_xlabel('maneuver time [s]')
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'plots', 'yaw_integral.png'))
    plt.close(fig)

    # Summary ---------------------------------------------------------------
    drive_rows = {(r['phase'], r['signal']): r for r in rows}
    lines = []
    lines.append('# IMU comparison: real bag vs simulator\n')
    lines.append(f'- real IMU: `{os.path.basename(args.real_imu)}` '
                 f'({len(real["t_rel"])} rows, {rate(real):.2f} Hz, '
                 f'frame `{real["frame_id"]}`)')
    lines.append(f'- sim IMU: `{os.path.basename(args.sim_imu)}` '
                 f'({len(sim["t_rel"])} rows, {rate(sim):.2f} Hz, '
                 f'frame `{sim["frame_id"]}`)')
    lines.append(f'- real control: {len(rc["t_rel"])} rows, onset at '
                 f't_rel={onset_r:.3f}s')
    lines.append(f'- sim control: {len(sc["t_rel"])} rows, onset at '
                 f't_rel={onset_s:.3f}s')
    lines.append(f'- compared window: tau = 0..{end:.1f}s, '
                 f'{int(drive.sum())} samples at {FS:.0f} Hz\n')

    lines.append('## Static bias (pre-onset mean)\n')
    lines.append('| axis | real | sim | difference |')
    lines.append('|---|---:|---:|---:|')
    for k in ALL:
        lines.append(f'| {k} | {bias_r[k]:+.4f} | {bias_s[k]:+.4f} | '
                     f'{bias_s[k] - bias_r[k]:+.4f} |')
    lines.append('')

    lines.append('## Dynamic metrics (main drive, tau >= 20 s)\n')
    lines.append('| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |')
    lines.append('|---|---:|---:|---:|---:|---:|---:|')
    for k in ALL:
        m = drive_rows.get(('drive', k))
        if not m:
            continue
        lines.append(f'| {k} | {m["real_std"]:.4f} | {m["sim_std"]:.4f} | '
                     f'{m["rmse"]:.4f} | {m["corr"]:.3f} | '
                     f'{m["gain_sim_over_real"]:.3f} | {m["lag_s"]:+.3f} |')
    lines.append('')
    lines.append('## Integrated quantities, tau >= 0\n')
    lines.append(f'- yaw change: real {yaw_r:+.3f} rad, sim {yaw_s:+.3f} rad '
                 f'(ratio {yaw_s / yaw_r:.3f})' if abs(yaw_r) > 1e-6 else
                 f'- yaw change: real {yaw_r:+.3f} rad, sim {yaw_s:+.3f} rad')
    lines.append(f'- longitudinal Δv: real {dv_r:+.3f} m/s, sim {dv_s:+.3f} m/s '
                 f'(ratio {dv_s / dv_r:.3f})' if abs(dv_r) > 1e-6 else
                 f'- longitudinal Δv: real {dv_r:+.3f} m/s, sim {dv_s:+.3f} m/s')
    lines.append('')
    lines.append('Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, '
                 '`plots/accel_psd.png`, `plots/gyro_psd.png`, '
                 '`plots/yaw_integral.png`\n')
    with open(os.path.join(args.out, 'summary.md'), 'w') as fh:
        fh.write('\n'.join(lines))

    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
