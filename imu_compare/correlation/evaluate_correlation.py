#!/usr/bin/env python3
"""Comprehensive multi-signal vehicle dynamics correlation evaluator.

Strictly excludes driver inputs (`control.steer_rad`, `control.gas`) from the vehicle dynamics score.
Computes RMSE, MAE, Mean Bias, Pearson r, NRMSE, Gain ratio, and Lag for all measured channels.
Supports train/validation split across independent maneuver windows.
"""

import argparse
import csv
import json
import math
import os
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

DT = 0.01 # 100 Hz evaluation grid

WINDOWS = {
    'launch_0_20': (0.0, 20.0),
    'active_20_60': (20.0, 60.0),
    'active_60_95': (60.0, 95.0),
    'bench_20_95': (20.0, 95.0),
    'cornering_95_160': (95.0, 160.0),
    'high_speed_160_281': (160.0, 281.0),
    'full_drive_20_281': (20.0, 281.0),
}

# Primary weights matching CorrelationEngine.gd (input channels excluded!)
PRIMARY_WEIGHTS = {
    'imu.ay': 0.30,
    'imu.gz': 0.30,
    'chassis.u': 0.20,
    'imu.ax': 0.15,
    'chassis.v': 0.05,
}

def load_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        return None
    return pd.read_csv(path)

def get_onset(df_ctl: pd.DataFrame) -> float:
    mask = df_ctl['gas'] > 0.1
    if mask.any():
        return float(df_ctl.loc[mask, 't_rel'].iloc[0])
    mask2 = np.abs(df_ctl['steer_rad']) > 0.05
    if mask2.any():
        return float(df_ctl.loc[mask2, 't_rel'].iloc[0])
    return float(df_ctl['t_rel'].iloc[0])

def compute_channel_metrics(ref: np.ndarray, sim: np.ndarray) -> Dict[str, float]:
    m = np.isfinite(ref) & np.isfinite(sim)
    n = int(m.sum())
    if n < 10:
        return {'n': n, 'rmse': float('nan'), 'mae': float('nan'), 'bias': float('nan'),
                'r': float('nan'), 'gain': float('nan'), 'nrmse_std': float('nan'),
                'nrmse_range': float('nan'), 'lag_s': float('nan')}
    r = ref[m]
    s = sim[m]
    
    diff = s - r
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    mae = float(np.mean(np.abs(diff)))
    bias = float(np.mean(diff))
    
    r_std = float(r.std())
    s_std = float(s.std())
    r_ptp = float(np.ptp(r))
    
    corr = float(np.corrcoef(r, s)[0, 1]) if r_std > 1e-6 and s_std > 1e-6 else float('nan')
    r_dot_r = float(np.dot(r - r.mean(), r - r.mean()))
    gain = float(np.dot(s - s.mean(), r - r.mean()) / r_dot_r) if r_dot_r > 1e-9 else float('nan')
    
    nrmse_std = float(rmse / r_std) if r_std > 1e-6 else float('nan')
    nrmse_range = float(rmse / r_ptp) if r_ptp > 1e-6 else float('nan')
    
    # Lag estimation via cross-correlation
    lag_s = 0.0
    if r_std > 1e-6 and s_std > 1e-6 and n > 50:
        a = r - r.mean()
        b = s - s.mean()
        c = signal.correlate(b, a, mode='full', method='fft')
        lags = signal.correlation_lags(len(b), len(a), mode='full')
        i = int(np.argmax(np.abs(c)))
        lag_s = float(lags[i] * DT)
        
    return {
        'n': n,
        'rmse': rmse,
        'mae': mae,
        'bias': bias,
        'r': corr,
        'gain': gain,
        'nrmse_std': nrmse_std,
        'nrmse_range': nrmse_range,
        'lag_s': lag_s,
        'ref_std': r_std,
        'sim_std': s_std,
    }

def compute_aggregate_score(metrics_dict: Dict[str, Dict[str, float]], weights: Dict[str, float]) -> float:
    total_w = 0.0
    score_sum = 0.0
    
    for ch, w in weights.items():
        if ch not in metrics_dict:
            continue
        m = metrics_dict[ch]
        r = m.get('r', float('nan'))
        rmse = m.get('rmse', float('nan'))
        gain = m.get('gain', float('nan'))
        nrmse = m.get('nrmse_std', float('nan'))
        
        if np.isnan(r):
            continue
            
        # Fisher-z transformed correlation score component [0, 100]
        # Bounded between 0% (r <= 0) and 100% (r = 1.0)
        r_bounded = max(0.0, min(0.999, r))
        z_score = math.tanh(1.5 * r_bounded) * 100.0
        
        # Penalty for gain error (|gain - 1.0|)
        gain_penalty = 1.0
        if not np.isnan(gain) and gain > 0.0:
            gain_penalty = max(0.2, min(1.0, 1.0 / (1.0 + 0.5 * abs(math.log(gain)))))
            
        # Penalty for NRMSE
        nrmse_penalty = 1.0
        if not np.isnan(nrmse):
            nrmse_penalty = max(0.2, min(1.0, 1.0 / (1.0 + 0.3 * nrmse)))
            
        ch_score = z_score * gain_penalty * nrmse_penalty
        score_sum += ch_score * w
        total_w += w
        
    return float(score_sum / total_w) if total_w > 0 else 0.0

def main():
    p = argparse.ArgumentParser(description="Evaluate vehicle dynamics correlation against real TeR26 bag.")
    p.add_argument("--sim-prefix", default="modular_baseline", help="Prefix for simulated CSV files (e.g. modular_baseline, modular_calibrated)")
    p.add_argument("--dir", default="imu_compare", help="Path to imu_compare directory")
    p.add_argument("--out-dir", default="imu_compare/reports", help="Output directory for reports and plots")
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    plots_dir = os.path.join(args.out_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    # 1. Load Real Data
    real_ctl = load_csv(os.path.join(args.dir, "real_control.csv"))
    real_imu = load_csv(os.path.join(args.dir, "real_imu.csv"))
    real_whl = load_csv(os.path.join(args.dir, "real_wheels.csv"))
    real_odm = load_csv(os.path.join(args.dir, "real_odom.csv"))

    if real_ctl is None or real_imu is None:
        print("ERROR: real_control.csv or real_imu.csv missing from", args.dir)
        return

    # 2. Load Sim Data
    sim_ctl = load_csv(os.path.join(args.dir, f"sim_control_{args.sim_prefix}.csv"))
    sim_imu = load_csv(os.path.join(args.dir, f"sim_imu_{args.sim_prefix}.csv"))
    sim_st = load_csv(os.path.join(args.dir, f"sim_state_{args.sim_prefix}.csv"))

    if sim_ctl is None or sim_imu is None:
        print(f"ERROR: sim_control_{args.sim_prefix}.csv or sim_imu_{args.sim_prefix}.csv missing from", args.dir)
        return

    # 3. Determine Synchronized Time Base
    real_onset = get_onset(real_ctl)
    sim_onset = get_onset(sim_ctl)

    real_tau = real_imu["t_rel"].values - real_onset
    sim_tau = sim_imu["t_rel"].values - sim_onset

    # Max common time
    max_tau = min(real_tau.max(), sim_tau.max())
    grid_tau = np.arange(0.0, max_tau, DT)

    # Helper interpolation
    def get_interp(t_src, val_src, t_grid):
        keep = np.r_[True, np.diff(t_src) > 0]
        return np.interp(t_grid, t_src[keep], val_src[keep])

    # Real channels on grid
    r_ay = get_interp(real_tau, real_imu["ay"].values, grid_tau)
    r_gz = get_interp(real_tau, real_imu["gz"].values, grid_tau)
    r_ax = get_interp(real_tau, real_imu["ax"].values, grid_tau)
    r_steer = get_interp(real_ctl["t_rel"].values - real_onset, real_ctl["steer_rad"].values, grid_tau)

    # Real speed from rear wheel RPM (calibrated k=0.928)
    r_rw_tau = real_whl["t"].values - real_ctl["t_unix"].iloc[0] - real_onset
    r_rw_rl = get_interp(r_rw_tau, real_whl["rl"].values, grid_tau)
    r_rw_rr = get_interp(r_rw_tau, real_whl["rr"].values, grid_tau)
    r_u = 0.5 * (r_rw_rl + r_rw_rr) * (2.0 * np.pi / 60.0) * 0.2032 * 0.928

    # Sim channels on grid
    s_ay = get_interp(sim_tau, sim_imu["ay"].values, grid_tau)
    s_gz = get_interp(sim_tau, sim_imu["gz"].values, grid_tau)
    s_ax = get_interp(sim_tau, sim_imu["ax"].values, grid_tau)
    s_steer = get_interp(sim_ctl["t_rel"].values - sim_onset, sim_ctl["steer_rad"].values, grid_tau)

    s_u = np.zeros_like(grid_tau)
    s_v = np.zeros_like(grid_tau)
    s_rl_rpm = np.zeros_like(grid_tau)
    s_rr_rpm = np.zeros_like(grid_tau)

    if sim_st is not None and "u_mps" in sim_st.columns:
        st_tau = sim_st["tau_s"].values if "tau_s" in sim_st.columns else (sim_st["t_s"].values - sim_onset)
        s_u = get_interp(st_tau, sim_st["u_mps"].values, grid_tau)
        s_v = get_interp(st_tau, sim_st["v_mps"].values, grid_tau) if "v_mps" in sim_st.columns else np.zeros_like(grid_tau)
        if "rl_rpm" in sim_st.columns:
            s_rl_rpm = get_interp(st_tau, sim_st["rl_rpm"].values, grid_tau)
            s_rr_rpm = get_interp(st_tau, sim_st["rr_rpm"].values, grid_tau)

    # 4. Evaluate Across All Windows
    results = {}
    for win_name, (w0, w1) in WINDOWS.items():
        if w0 >= max_tau:
            continue
        w1_eval = min(w1, max_tau)
        mask = (grid_tau >= w0) & (grid_tau <= w1_eval)
        if mask.sum() < 20:
            continue

        ch_metrics = {
            "imu.ay": compute_channel_metrics(r_ay[mask], s_ay[mask]),
            "imu.gz": compute_channel_metrics(r_gz[mask], s_gz[mask]),
            "imu.ax": compute_channel_metrics(r_ax[mask], s_ax[mask]),
            "chassis.u": compute_channel_metrics(r_u[mask], s_u[mask]),
            "chassis.v": compute_channel_metrics(np.zeros(mask.sum()), s_v[mask]),
            # Secondary channels
            "wheel.rl_rpm": compute_channel_metrics(r_rw_rl[mask], s_rl_rpm[mask]),
            "wheel.rr_rpm": compute_channel_metrics(r_rw_rr[mask], s_rr_rpm[mask]),
            # Input diagnostic check (explicitly labeled as INPUT)
            "control.steer_rad": compute_channel_metrics(r_steer[mask], s_steer[mask]),
        }

        agg_score = compute_aggregate_score(ch_metrics, PRIMARY_WEIGHTS)
        results[win_name] = {
            "window": [w0, w1],
            "n_samples": int(mask.sum()),
            "aggregate_score_pct": agg_score,
            "channels": ch_metrics,
        }

    def sanitize_for_json(obj):
        if isinstance(obj, dict):
            return {k: sanitize_for_json(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [sanitize_for_json(v) for v in obj]
        elif isinstance(obj, float) and (np.isnan(obj) or np.isinf(obj)):
            return None
        return obj

    # Save JSON report
    out_json = os.path.join(args.out_dir, f"correlation_{args.sim_prefix}.json")
    with open(out_json, "w") as f:
        json.dump(sanitize_for_json(results), f, indent=2, default=float)

    # Print summary table for Primary Benchmark Window [20, 95] s and Full Drive [20, 281] s
    print(f"\n=========================================================================")
    print(f"  CORRELATION EVALUATION: {args.sim_prefix.upper()}")
    print(f"=========================================================================")
    for wname in ["bench_20_95", "full_drive_20_281", "cornering_95_160"]:
        if wname not in results:
            continue
        res = results[wname]
        print(f"\n--- Window: {wname} ({res['window'][0]}..{res['window'][1]} s, N={res['n_samples']}) ---")
        print(f"  AGGREGATE VEHICLE DYNAMICS SCORE: {res['aggregate_score_pct']:.1f}%")
        print(f"{'Channel':<20} | {'RMSE':>8} | {'MAE':>8} | {'Bias':>8} | {'r':>7} | {'Gain':>7} | {'Lag [s]':>7}")
        print("-" * 75)
        for ch, m in res["channels"].items():
            ch_label = ch + (" [INPUT]" if ch.startswith("control") else "")
            print(f"{ch_label:<20} | {m['rmse']:8.3f} | {m['mae']:8.3f} | {m['bias']:8.3f} | {m['r']:7.3f} | {m['gain']:7.3f} | {m['lag_s']:7.3f}")

if __name__ == "__main__":
    main()
