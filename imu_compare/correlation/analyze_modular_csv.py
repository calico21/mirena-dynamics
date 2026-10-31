#!/usr/bin/env python3
"""Forensic diagnosis of a MirenaSim modular `DynamicsLogger` CSV.

Usage:
  python3 analyze_modular_csv.py imu_compare/incoming/modular_vehicle_dynamics_log.csv \
      --out imu_compare/reports/modular_csv_diagnosis

Produces `diagnosis.md`, `diagnosis.json` and plots in `--out`.

Every number in the report is computed from the CSV itself; the script embeds
the static vehicle constants that were active in the modular baseline
configuration (Config/baselines/modular_baseline/*.tres) only to evaluate
force/moment balance residuals.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

CORNERS = ('fl', 'fr', 'rl', 'rr')
# Modular baseline constants (vehicle_default.tres @ baseline snapshot)
M = 300.0
G = 9.81
CDA = 2.50 * 1.10
CLA = 4.14 * 1.10
RHO = 1.225
TTC_FZ = (150.0, 1600.0)          # TeR_Tire/config/ttc_dataset.yaml filters
TTC_ALPHA_DEG = 15.0
TTC_MIN_SPEED = 30.0 / 3.6        # min_velocity_kmh
R_WHEEL = 0.20


def load(path):
    df = pd.read_csv(path)
    return df


def contiguous_events(mask, t, min_len=1):
    ev = []
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    for a, b in zip(idx[0::2], idx[1::2]):
        if b - a >= min_len:
            ev.append((float(t[a]), float(t[b - 1]), int(b - a)))
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--out', default='imu_compare/reports/modular_csv_diagnosis')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    df = load(a.csv)
    t = df.t_s.values
    dt = np.diff(t)
    R = {}
    R['file'] = os.path.abspath(a.csv)
    R['rows'] = int(len(df))
    R['columns'] = list(df.columns)
    R['n_columns'] = int(len(df.columns))
    R['t_first'] = float(t[0]); R['t_last'] = float(t[-1])
    R['dt_median'] = float(np.median(dt)); R['dt_min'] = float(dt.min()); R['dt_max'] = float(dt.max())
    R['fs_hz'] = float(1.0 / np.median(dt))
    R['buffer_cap_hit'] = bool(len(df) == 40000)

    fz = np.stack([df[f'fz_{c}_n'].values for c in CORNERS], 1)
    fx = np.stack([df[f'fx_{c}_n'].values for c in CORNERS], 1)
    fy = np.stack([df[f'fy_{c}_n'].values for c in CORNERS], 1)
    kap = np.stack([df[f'kappa_{c}'].values for c in CORNERS], 1)
    alp = np.stack([df[f'alpha_{c}_rad'].values for c in CORNERS], 1)
    u = df.u_mps.values; v = df.v_mps.values; r = df.yaw_rate_rad_s.values
    ax = df.ax_mps2.values; ay = df.ay_mps2.values; az = df.az_mps2.values
    speed = np.hypot(u, v)

    # ---- extrema
    R['max_u'] = float(u.max()); R['max_abs_ay'] = float(np.abs(ay).max())
    R['t_max_abs_ay'] = float(t[np.argmax(np.abs(ay))])
    R['max_abs_r'] = float(np.abs(r).max()); R['t_max_abs_r'] = float(t[np.argmax(np.abs(r))])
    R['max_abs_alpha_deg'] = float(np.degrees(np.abs(alp).max()))
    R['max_abs_kappa'] = float(np.abs(kap).max())
    R['max_fz'] = float(fz.max())
    R['max_abs_fy_single'] = float(np.abs(fy).max())
    R['max_abs_fx_single'] = float(np.abs(fx).max())

    # ---- unloading
    zero = fz <= 1e-6
    R['frac_any_zero_fz'] = float(zero.any(1).mean())
    R['frac_zero_fz_per_wheel'] = {c: float(zero[:, i].mean()) for i, c in enumerate(CORNERS)}
    left_both = zero[:, 0] & zero[:, 2]
    right_both = zero[:, 1] & zero[:, 3]
    R['frac_same_side_left_zero'] = float(left_both.mean())
    R['frac_same_side_right_zero'] = float(right_both.mean())
    R['events_right_side_zero'] = contiguous_events(right_both, t)[:40]
    R['events_left_side_zero'] = contiguous_events(left_both, t)[:40]
    R['frac_three_or_more_zero'] = float((zero.sum(1) >= 3).mean())

    # ---- force balance: tire forces in the steered wheel frame are not
    # rotatable without the road-wheel angle (not logged), so we use the
    # body-frame acceleration identity on the vertical axis only, and on the
    # planar axes use |sum F| vs m|a| as an envelope check.
    q = 0.5 * RHO * speed ** 2
    down = q * CLA
    sum_fz = fz.sum(1)
    R['vertical_balance'] = {}
    mov = speed > 3.0
    if mov.any():
        resid = sum_fz - (M * G + down)
        R['vertical_balance']['mean_sumFz_minus_mg_minus_downforce_moving'] = float(resid[mov].mean())
        R['vertical_balance']['mean_downforce_moving'] = float(down[mov].mean())
        # regress residual on downforce -> slope 1.0 means downforce is double counted
        A = np.c_[down[mov], np.ones(mov.sum())]
        coef, *_ = np.linalg.lstsq(A, sum_fz[mov] - M * G, rcond=None)
        R['vertical_balance']['regression_sumFz_minus_mg_vs_downforce_slope'] = float(coef[0])
        R['vertical_balance']['regression_intercept'] = float(coef[1])

    # planar: |sum of tire forces| (frame invariant magnitude only if all in same frame)
    fplan = np.hypot(fx.sum(1), fy.sum(1))
    R['planar_force_vs_mass_accel_ratio_p99'] = float(np.percentile(fplan[mov] / np.maximum(M * np.hypot(ax, ay)[mov], 1.0), 99)) if mov.any() else None

    # ---- tire operating domain (TTC filters)
    ood = {}
    for i, c in enumerate(CORNERS):
        loaded = fz[:, i] > 0
        ood[c] = {
            'fz_below_150N_loaded': float(((fz[:, i] < TTC_FZ[0]) & loaded).mean()),
            'fz_zero': float((~loaded).mean()),
            'fz_above_1600N': float((fz[:, i] > TTC_FZ[1]).mean()),
            'alpha_above_15deg_loaded': float(((np.degrees(np.abs(alp[:, i])) > TTC_ALPHA_DEG) & loaded).mean()),
            'alpha_above_31p5deg_clamp_loaded': float(((np.abs(alp[:, i]) > 0.55) & loaded).mean()),
            'abs_kappa_above_0p2_loaded': float(((np.abs(kap[:, i]) > 0.2) & loaded).mean()),
            'speed_below_ttc_8p3mps_loaded': float(((speed < TTC_MIN_SPEED) & loaded).mean()),
        }
        dom = loaded & (fz[:, i] >= TTC_FZ[0]) & (fz[:, i] <= TTC_FZ[1]) & (np.degrees(np.abs(alp[:, i])) <= TTC_ALPHA_DEG)
        ood[c]['inside_ttc_fz_alpha_domain'] = float(dom.mean())
        ood[c]['inside_domain_while_moving_gt3mps'] = float(dom[mov].mean()) if mov.any() else None
    R['tire_domain'] = ood

    # ---- stationary tail
    tail = t >= 60.0
    st = {}
    st['mean_fz'] = {c: float(fz[tail, i].mean()) for i, c in enumerate(CORNERS)}
    st['sum_fz'] = float(fz[tail].sum(1).mean())
    st['front_fraction'] = float(fz[tail, :2].sum(1).mean() / fz[tail].sum(1).mean())
    st['mean_u'] = float(u[tail].mean()); st['std_u'] = float(u[tail].std())
    st['mean_rpm_rl'] = float(df.rpm_rl.values[tail].mean()); st['mean_rpm_rr'] = float(df.rpm_rr.values[tail].mean())
    st['std_rpm_rl'] = float(df.rpm_rl.values[tail].std())
    st['median_kappa'] = {c: float(np.median(kap[tail, i])) for i, c in enumerate(CORNERS)}
    st['std_fx'] = {c: float(fx[tail, i].std()) for i, c in enumerate(CORNERS)}
    st['mean_fx'] = {c: float(fx[tail, i].mean()) for i, c in enumerate(CORNERS)}
    st['sum_fx_mean'] = float(fx[tail].sum(1).mean()); st['sum_fx_std'] = float(fx[tail].sum(1).std())
    st['std_ay'] = float(ay[tail].std()); st['std_ax'] = float(ax[tail].std()); st['std_r'] = float(r[tail].std())
    # sample-to-sample sign alternation of Fx (limit cycle signature)
    alt = {}
    for i, c in enumerate(CORNERS):
        s = np.sign(fx[tail, i]); alt[c] = float(np.mean(s[1:] * s[:-1] < 0))
    st['fx_sign_flip_rate_per_sample'] = alt
    # dominant frequency of RL fx in tail
    x = fx[tail, 2] - fx[tail, 2].mean()
    if len(x) > 64:
        f = np.fft.rfftfreq(len(x), R['dt_median']); P = np.abs(np.fft.rfft(x)) ** 2
        st['fx_rl_dominant_freq_hz'] = float(f[1:][np.argmax(P[1:])])
    R['stationary_tail_60_100s'] = st

    # Explicit-Euler stability number of the wheel spin ODE at standstill
    # dω/dt = -(R^2/Iw) * dFx/dκ * dκ/d(ωR) ; dFx/dκ ≈ Kx = 18 Fz ; dκ/d(ωR) ≈ 1/ε, ε=0.25
    Iw = 0.32; eps = 0.25; kx = 18.0
    R['wheel_spin_explicit_euler_number_at_rest'] = {c: float(R['dt_median'] * kx * st['mean_fz'][c] * R_WHEEL ** 2 / (Iw * eps)) for c in CORNERS}
    R['wheel_spin_explicit_euler_stable_limit'] = 2.0

    # ---- event deep dives
    def snap(tq):
        k = int(np.argmin(np.abs(t - tq)))
        rows = df.iloc[max(0, k - 2):k + 3]
        return rows.to_dict(orient='records')
    R['snap_31p225'] = snap(31.225)
    R['snap_33p75'] = snap(33.75)

    # kinematic consistency: ay vs u*r (steady part); large mismatch -> sideslip transient
    R['ay_minus_u_r_p99_moving'] = float(np.percentile(np.abs(ay - u * r)[mov], 99)) if mov.any() else None
    # derivative consistency: dv/dt + u r ≈ ay
    dv = np.gradient(v, t); du = np.gradient(u, t)
    R['ay_identity_resid_rms_moving'] = float(np.sqrt(np.mean((dv + u * r - ay)[mov] ** 2)))
    R['ax_identity_resid_rms_moving'] = float(np.sqrt(np.mean((du - v * r - ax)[mov] ** 2)))
    R['max_sideslip_deg_moving'] = float(np.degrees(np.abs(np.arctan2(v, np.maximum(np.abs(u), 0.5)))[mov]).max())
    R['time_sideslip_gt_10deg_moving_s'] = float(((np.degrees(np.abs(np.arctan2(v, np.maximum(np.abs(u), 0.5)))) > 10) & mov).sum() * R['dt_median'])
    R['fault_flags_unique'] = sorted(set(int(x) for x in df.fault_flags.unique()))

    # ---- plots
    fig, axs = plt.subplots(6, 1, figsize=(14, 18), sharex=True)
    axs[0].plot(t, u, label='u'); axs[0].plot(t, v, label='v'); axs[0].set_ylabel('m/s'); axs[0].legend()
    axs[1].plot(t, ay, label='ay'); axs[1].plot(t, u * r, label='u*r', alpha=.6); axs[1].plot(t, ax, label='ax', alpha=.6); axs[1].set_ylabel('m/s²'); axs[1].legend()
    axs[2].plot(t, r); axs[2].set_ylabel('r rad/s')
    for i, c in enumerate(CORNERS): axs[3].plot(t, fz[:, i], label=c)
    axs[3].plot(t, sum_fz - M * G, 'k', lw=.6, label='ΣFz-mg'); axs[3].plot(t, down, 'k--', lw=.6, label='q·ClA'); axs[3].legend(); axs[3].set_ylabel('Fz N')
    for i, c in enumerate(CORNERS): axs[4].plot(t, np.degrees(alp[:, i]), label=c)
    axs[4].axhline(15, color='r', ls=':'); axs[4].axhline(-15, color='r', ls=':'); axs[4].set_ylabel('alpha deg'); axs[4].legend()
    for i, c in enumerate(CORNERS): axs[5].plot(t, kap[:, i], label=c)
    axs[5].set_ylabel('kappa'); axs[5].legend(); axs[5].set_xlabel('t [s]')
    fig.tight_layout(); fig.savefig(os.path.join(a.out, 'overview.png'), dpi=90); plt.close(fig)

    for name, (t0, t1) in {'event_31s': (29.5, 34.5), 'tail_stationary': (60, 61)}.items():
        m = (t >= t0) & (t <= t1)
        fig, axs = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
        axs[0].plot(t[m], u[m], label='u'); axs[0].plot(t[m], v[m], label='v'); axs[0].legend()
        axs[1].plot(t[m], ay[m], label='ay'); axs[1].plot(t[m], r[m] * 10, label='10 r'); axs[1].legend()
        for i, c in enumerate(CORNERS):
            axs[2].plot(t[m], fz[m, i], label=c); axs[3].plot(t[m], fx[m, i], label=c); axs[4].plot(t[m], fy[m, i], label=c)
        axs[2].set_ylabel('Fz'); axs[3].set_ylabel('Fx'); axs[4].set_ylabel('Fy')
        for x_ in axs[2:]: x_.legend()
        fig.tight_layout(); fig.savefig(os.path.join(a.out, f'{name}.png'), dpi=90); plt.close(fig)

    with open(os.path.join(a.out, 'diagnosis.json'), 'w') as fh:
        json.dump(R, fh, indent=1, default=float)
    print(json.dumps({k: v for k, v in R.items() if not k.startswith('snap') and not k.startswith('events')}, indent=1, default=float))
    print('right-side events', R['events_right_side_zero'][:15])
    print('left-side events', R['events_left_side_zero'][:15])


if __name__ == '__main__':
    main()
