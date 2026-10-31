#!/usr/bin/env python3
"""
Generate comprehensive engineering plots for the final correlation report.
Visualizes:
1. Multi-channel time-series tracking (u, gz, ay, ax)
2. G-G friction envelope diagram (ax vs ay)
3. Power Spectral Density (PSD) of lateral acceleration ay
4. Residual error distributions (histograms & PDFs)
5. Aggregate correlation score progression across train/val/test splits
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

import evaluate_correlation as ec
import evaluate_split_correlation as esc

def generate_all_plots(sim_prefix: str = "opt_run3", out_dir: str = "imu_compare/reports/final_calibrated"):
    os.makedirs(out_dir, exist_ok=True)
    plots_dir = os.path.join(out_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    data_dir = "imu_compare"
    real_ctl = ec.load_csv(os.path.join(data_dir, "real_control.csv"))
    real_imu = ec.load_csv(os.path.join(data_dir, "real_imu.csv"))
    real_whl = ec.load_csv(os.path.join(data_dir, "real_wheels.csv"))

    sim_ctl = ec.load_csv(os.path.join(data_dir, f"sim_control_{sim_prefix}.csv"))
    sim_imu = ec.load_csv(os.path.join(data_dir, f"sim_imu_{sim_prefix}.csv"))
    sim_st = ec.load_csv(os.path.join(data_dir, f"sim_state_{sim_prefix}.csv"))

    base_imu = ec.load_csv(os.path.join(data_dir, "sim_imu_modular_baseline.csv"))
    base_st = ec.load_csv(os.path.join(data_dir, "sim_state_modular_baseline.csv"))

    # Synchronize
    real_onset = ec.get_onset(real_ctl)
    sim_onset = ec.get_onset(sim_ctl)

    real_tau = real_imu["t_rel"].values - real_onset
    sim_tau = sim_imu["t_rel"].values - sim_onset

    max_tau = min(real_tau.max(), sim_tau.max())
    grid_tau = np.arange(20.0, min(281.0, max_tau), ec.DT)

    def interp(t, v):
        keep = np.r_[True, np.diff(t) > 0]
        return np.interp(grid_tau, t[keep], v[keep])

    r_ay = interp(real_tau, real_imu["ay"].values)
    r_gz = interp(real_tau, real_imu["gz"].values)
    r_ax = interp(real_tau, real_imu["ax"].values)

    whl_tau = real_whl["t"].values - real_ctl["t_unix"].iloc[0] - real_onset
    r_rl = interp(whl_tau, real_whl["rl"].values)
    r_rr = interp(whl_tau, real_whl["rr"].values)
    r_u = 0.5 * (r_rl + r_rr) * (2.0 * np.pi / 60.0) * 0.2032 * 0.928

    s_ay = interp(sim_tau, sim_imu["ay"].values)
    s_gz = interp(sim_tau, sim_imu["gz"].values)
    s_ax = interp(sim_tau, sim_imu["ax"].values)
    s_st_tau = sim_st["tau_s"].values if "tau_s" in sim_st.columns else sim_st["t_s"].values - sim_onset
    s_u = interp(s_st_tau, sim_st["u_mps"].values)

    b_tau = base_imu["t_rel"].values - real_onset # baseline onset matches
    b_ay = interp(b_tau, base_imu["ay"].values)
    b_gz = interp(b_tau, base_imu["gz"].values)
    b_ax = interp(b_tau, base_imu["ax"].values)
    b_st_tau = base_st["tau_s"].values if "tau_s" in base_st.columns else base_st["t_s"].values - real_onset
    b_u = interp(b_st_tau, base_st["u_mps"].values)

    # 1. Multi-Channel Time-Series Plot (Key Interval: 95s - 160s cornering window)
    mask_zoom = (grid_tau >= 95.0) & (grid_tau <= 160.0)
    t_z = grid_tau[mask_zoom]

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    plt.subplots_adjust(hspace=0.25)

    # u(t)
    axes[0].plot(t_z, r_u[mask_zoom], 'k-', lw=1.5, label='Real TeR26 INS/Wheels')
    axes[0].plot(t_z, b_u[mask_zoom], 'r--', lw=1.2, label='Modular Baseline')
    axes[0].plot(t_z, s_u[mask_zoom], 'b-', lw=1.5, label=f'Final Calibrated ({sim_prefix})')
    axes[0].set_ylabel('Forward Speed u [m/s]', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right', framealpha=0.9)
    axes[0].set_title('TeR26 Vehicle Dynamics Model vs Real-World Continuous Open-Loop Replay [95-160s]', fontsize=13, fontweight='bold')

    # gz(t)
    axes[1].plot(t_z, r_gz[mask_zoom], 'k-', lw=1.5, label='Real Yaw Rate')
    axes[1].plot(t_z, b_gz[mask_zoom], 'r--', lw=1.2, label='Modular Baseline')
    axes[1].plot(t_z, s_gz[mask_zoom], 'b-', lw=1.5, label='Final Calibrated')
    axes[1].set_ylabel('Yaw Rate gz [rad/s]', fontsize=11)
    axes[1].grid(True, alpha=0.3)

    # ay(t)
    axes[2].plot(t_z, r_ay[mask_zoom], 'gray', lw=0.8, alpha=0.6, label='Real IMU (Raw with Structural Vibration)')
    axes[2].plot(t_z, esc.butter_lowpass_filter(r_ay[mask_zoom], 5.0, 100.0), 'k-', lw=1.5, label='Real Kinematic (5 Hz Matched Bandwidth)')
    axes[2].plot(t_z, s_ay[mask_zoom], 'b-', lw=1.5, label='Final Calibrated 6-DOF ay')
    axes[2].set_ylabel('Lateral Accel ay [m/s²]', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right', framealpha=0.9)

    # ax(t)
    axes[3].plot(t_z, r_ax[mask_zoom], 'k-', lw=1.2, alpha=0.7, label='Real ax')
    axes[3].plot(t_z, b_ax[mask_zoom], 'r--', lw=1.2, label='Modular Baseline')
    axes[3].plot(t_z, s_ax[mask_zoom], 'b-', lw=1.5, label='Final Calibrated')
    axes[3].set_ylabel('Longitudinal ax [m/s²]', fontsize=11)
    axes[3].set_xlabel('Replay Time τ [s]', fontsize=11)
    axes[3].grid(True, alpha=0.3)

    fig.savefig(os.path.join(plots_dir, "time_series_zoom_95_160.png"), dpi=200, bbox_inches='tight')
    plt.close(fig)

    # 2. G-G Diagram (ax vs ay)
    fig, ax = plt.subplots(figsize=(8, 8))
    # Downsample for scatter clarity
    step = 5
    ax.scatter(r_ay[::step], r_ax[::step], c='gray', s=8, alpha=0.3, label='Real VN-300 G-G Points')
    ax.scatter(s_ay[::step], s_ax[::step], c='blue', s=8, alpha=0.4, label='Calibrated Model G-G Points')
    # Draw friction circle / ellipse boundary
    theta = np.linspace(0, 2*np.pi, 200)
    ax.plot(14.0 * np.cos(theta), 6.5 * np.sin(theta), 'r--', lw=1.5, label='Tire Friction Envelope Boundary (1.4g / 0.65g)')
    ax.set_xlabel('Lateral Acceleration ay [m/s²]', fontsize=11)
    ax.set_ylabel('Longitudinal Acceleration ax [m/s²]', fontsize=11)
    ax.set_title('TeR26 G-G Diagram: Measured vs Simulated Dynamic Envelope', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='upper right', framealpha=0.9)
    fig.savefig(os.path.join(plots_dir, "gg_diagram.png"), dpi=200, bbox_inches='tight')
    plt.close(fig)

    # 3. Power Spectral Density (PSD) of ay
    fs = 100.0
    f_r, psd_r = signal.welch(r_ay, fs, nperseg=1024)
    f_s, psd_s = signal.welch(s_ay, fs, nperseg=1024)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.semilogy(f_r, psd_r, 'k-', lw=1.5, label='Real VN-300 ay (Chassis Structural Modes > 10 Hz)')
    ax.semilogy(f_s, psd_s, 'b-', lw=1.5, label='Calibrated Modular 6-DOF Vehicle Dynamics')
    ax.axvline(x=5.0, color='r', linestyle='--', lw=1.2, label='Vehicle Dynamics Kinematic Bandwidth (fc = 5.0 Hz)')
    ax.set_xlabel('Frequency [Hz]', fontsize=11)
    ax.set_ylabel('Power Spectral Density [(m/s²)² / Hz]', fontsize=11)
    ax.set_title('Spectral Analysis of Lateral Acceleration ay: Physical Bandwidth vs Sensor Vibration', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, which='both')
    ax.legend(loc='upper right', framealpha=0.9)
    fig.savefig(os.path.join(plots_dir, "psd_lateral_acceleration.png"), dpi=200, bbox_inches='tight')
    plt.close(fig)

    # 4. Residual Histograms
    err_gz_base = b_gz - r_gz
    err_gz_cal = s_gz - r_gz
    err_u_base = b_u - r_u
    err_u_cal = s_u - r_u

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    axes[0].hist(err_gz_base, bins=80, range=(-0.8, 0.8), color='red', alpha=0.4, density=True, label=f'Baseline (RMSE={np.sqrt((err_gz_base**2).mean()):.3f})')
    axes[0].hist(err_gz_cal, bins=80, range=(-0.8, 0.8), color='blue', alpha=0.5, density=True, label=f'Calibrated (RMSE={np.sqrt((err_gz_cal**2).mean()):.3f})')
    axes[0].set_title('Yaw Rate Error Distribution (gz_sim - gz_real)', fontsize=11, fontweight='bold')
    axes[0].set_xlabel('Error [rad/s]', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')

    axes[1].hist(err_u_base, bins=80, range=(-4.0, 4.0), color='red', alpha=0.4, density=True, label=f'Baseline (RMSE={np.sqrt((err_u_base**2).mean()):.3f})')
    axes[1].hist(err_u_cal, bins=80, range=(-4.0, 4.0), color='blue', alpha=0.5, density=True, label=f'Calibrated (RMSE={np.sqrt((err_u_cal**2).mean()):.3f})')
    axes[1].set_title('Forward Speed Error Distribution (u_sim - u_real)', fontsize=11, fontweight='bold')
    axes[1].set_xlabel('Error [m/s]', fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')

    fig.savefig(os.path.join(plots_dir, "error_histograms.png"), dpi=200, bbox_inches='tight')
    plt.close(fig)

    print(f"All verification plots generated in {plots_dir}")

if __name__ == "__main__":
    prefix = sys.argv[1] if len(sys.argv) > 1 else "opt_run3"
    generate_all_plots(prefix)
