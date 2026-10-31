#!/usr/bin/env python3
"""
Extended Correlation Evaluator with Contiguous Train/Val/Test Splits & Matched-Bandwidth Diagnostics.
Preserves official metrics and PRIMARY_WEIGHTS from evaluate_correlation.py.
"""

import os
import sys
import math
import json
import argparse
from typing import Dict, Any
import numpy as np
import pandas as pd
from scipy import signal

# Add parent directory to path to import evaluate_correlation
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

import evaluate_correlation as ec

DT = ec.DT
PRIMARY_WEIGHTS = ec.PRIMARY_WEIGHTS

SPLIT_WINDOWS = {
    'train_20_160': (20.0, 160.0),
    'val_160_220': (160.0, 220.0),
    'test_220_281': (220.0, 281.0),
    'bench_20_95': (20.0, 95.0),
    'cornering_95_160': (95.0, 160.0),
    'high_speed_160_281': (160.0, 281.0),
    'full_drive_20_281': (20.0, 281.0),
}

def butter_lowpass_filter(data: np.ndarray, cutoff: float = 5.0, fs: float = 100.0, order: int = 2) -> np.ndarray:
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
    return signal.filtfilt(b, a, data)

def evaluate_splits(sim_prefix: str, data_dir: str = "imu_compare") -> Dict[str, Any]:
    # 1. Load Real Data
    real_ctl = ec.load_csv(os.path.join(data_dir, "real_control.csv"))
    real_imu = ec.load_csv(os.path.join(data_dir, "real_imu.csv"))
    real_whl = ec.load_csv(os.path.join(data_dir, "real_wheels.csv"))
    if real_ctl is None or real_imu is None:
        raise FileNotFoundError(f"Missing real files in {data_dir}")

    # 2. Load Sim Data
    sim_ctl = ec.load_csv(os.path.join(data_dir, f"sim_control_{sim_prefix}.csv"))
    sim_imu = ec.load_csv(os.path.join(data_dir, f"sim_imu_{sim_prefix}.csv"))
    sim_st = ec.load_csv(os.path.join(data_dir, f"sim_state_{sim_prefix}.csv"))
    if sim_ctl is None or sim_imu is None:
        raise FileNotFoundError(f"Missing sim files for prefix {sim_prefix} in {data_dir}")

    # 3. Synchronize
    real_onset = ec.get_onset(real_ctl)
    sim_onset = ec.get_onset(sim_ctl)

    real_tau = real_imu["t_rel"].values - real_onset
    sim_tau = sim_imu["t_rel"].values - sim_onset

    max_tau = min(real_tau.max(), sim_tau.max())
    grid_tau = np.arange(0.0, max_tau, DT)

    def get_interp(t_src, val_src, t_grid):
        keep = np.r_[True, np.diff(t_src) > 0]
        return np.interp(t_grid, t_src[keep], val_src[keep])

    r_ay = get_interp(real_tau, real_imu["ay"].values, grid_tau)
    r_gz = get_interp(real_tau, real_imu["gz"].values, grid_tau)
    r_ax = get_interp(real_tau, real_imu["ax"].values, grid_tau)
    r_steer = get_interp(real_ctl["t_rel"].values - real_onset, real_ctl["steer_rad"].values, grid_tau)

    r_rw_tau = real_whl["t"].values - real_ctl["t_unix"].iloc[0] - real_onset
    r_rw_rl = get_interp(r_rw_tau, real_whl["rl"].values, grid_tau)
    r_rw_rr = get_interp(r_rw_tau, real_whl["rr"].values, grid_tau)
    r_u = 0.5 * (r_rw_rl + r_rw_rr) * (2.0 * np.pi / 60.0) * 0.2032 * 0.928

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

    # 4. Evaluate Across Splits
    raw_results = {}
    filtered_results = {}

    r_ay_filt = butter_lowpass_filter(r_ay, 5.0, 1.0 / DT)
    s_ay_filt = butter_lowpass_filter(s_ay, 5.0, 1.0 / DT)
    r_ax_filt = butter_lowpass_filter(r_ax, 5.0, 1.0 / DT)
    s_ax_filt = butter_lowpass_filter(s_ax, 5.0, 1.0 / DT)
    r_gz_filt = butter_lowpass_filter(r_gz, 5.0, 1.0 / DT)
    s_gz_filt = butter_lowpass_filter(s_gz, 5.0, 1.0 / DT)

    for win_name, (w0, w1) in SPLIT_WINDOWS.items():
        if w0 >= max_tau:
            continue
        w1_eval = min(w1, max_tau)
        mask = (grid_tau >= w0) & (grid_tau <= w1_eval)
        if mask.sum() < 20:
            continue

        # Raw official channels
        ch_raw = {
            "imu.ay": ec.compute_channel_metrics(r_ay[mask], s_ay[mask]),
            "imu.gz": ec.compute_channel_metrics(r_gz[mask], s_gz[mask]),
            "imu.ax": ec.compute_channel_metrics(r_ax[mask], s_ax[mask]),
            "chassis.u": ec.compute_channel_metrics(r_u[mask], s_u[mask]),
            "chassis.v": ec.compute_channel_metrics(np.zeros(mask.sum()), s_v[mask]),
            "wheel.rl_rpm": ec.compute_channel_metrics(r_rw_rl[mask], s_rl_rpm[mask]),
            "wheel.rr_rpm": ec.compute_channel_metrics(r_rw_rr[mask], s_rr_rpm[mask]),
            "control.steer_rad": ec.compute_channel_metrics(r_steer[mask], s_steer[mask]),
        }
        raw_score = ec.compute_aggregate_score(ch_raw, PRIMARY_WEIGHTS)
        raw_results[win_name] = {
            "window": [w0, w1_eval],
            "n_samples": int(mask.sum()),
            "aggregate_score_pct": raw_score,
            "channels": ch_raw,
        }

        # Matched-bandwidth channels (5 Hz low-pass)
        ch_filt = {
            "imu.ay": ec.compute_channel_metrics(r_ay_filt[mask], s_ay_filt[mask]),
            "imu.gz": ec.compute_channel_metrics(r_gz_filt[mask], s_gz_filt[mask]),
            "imu.ax": ec.compute_channel_metrics(r_ax_filt[mask], s_ax_filt[mask]),
            "chassis.u": ec.compute_channel_metrics(r_u[mask], s_u[mask]),
            "chassis.v": ec.compute_channel_metrics(np.zeros(mask.sum()), s_v[mask]),
            "wheel.rl_rpm": ec.compute_channel_metrics(r_rw_rl[mask], s_rl_rpm[mask]),
            "wheel.rr_rpm": ec.compute_channel_metrics(r_rw_rr[mask], s_rr_rpm[mask]),
        }
        filt_score = ec.compute_aggregate_score(ch_filt, PRIMARY_WEIGHTS)
        filtered_results[win_name] = {
            "window": [w0, w1_eval],
            "n_samples": int(mask.sum()),
            "aggregate_score_pct": filt_score,
            "channels": ch_filt,
        }

    return {
        "sim_prefix": sim_prefix,
        "raw": raw_results,
        "filtered_5hz": filtered_results,
    }

def print_summary_table(res: Dict[str, Any]):
    print(f"\n=========================================================================")
    print(f"  EVALUATION SUMMARY: {res['sim_prefix'].upper()}")
    print(f"=========================================================================")
    print(f"{'Split / Window':<20} | {'Raw Score':<10} | {'5Hz Bandwidth Score':<20} | {'Duration [s]'}")
    print("-" * 75)
    for win, data in res["raw"].items():
        raw_sc = data["aggregate_score_pct"]
        filt_sc = res["filtered_5hz"][win]["aggregate_score_pct"]
        w = data["window"]
        dur = w[1] - w[0]
        print(f"{win:<20} | {raw_sc:>8.1f}% | {filt_sc:>18.1f}% | {dur:>10.1f} s")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sim-prefix", required=True, help="Simulated CSV prefix")
    p.add_argument("--dir", default="imu_compare", help="Path to imu_compare directory")
    args = p.parse_args()

    results = evaluate_splits(args.sim_prefix, args.dir)
    print_summary_table(results)
