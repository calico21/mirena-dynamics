#!/usr/bin/env python3
"""Phase 3 Correlation Evaluator: Signal Reconstruction & Sensitivity Analysis.

Produces 4 rigorous evaluation scorecards on the frozen opt_run4 configuration:
  Case A: Original Raw Experimental Telemetry (Historical Official Reference)
  Case B: Reconstructed CoG Acceleration (Rigid-body lever-arm correction)
  Case C: Matched 5 Hz Bandwidth (Vehicle Dynamics True Physical Bandwidth)
  Case D: Matched 3 Hz Bandwidth (Sensitivity Analysis)
  
Also evaluates the Valid-Control-Coverage Subset (excluding the 47s uncommanded standstill [27, 74]s).

Uses exact channel scoring and weights from evaluate_correlation.py without alterations.
Filter: 4th-order Butterworth, zero-phase (scipy.signal.filtfilt), fs = 100 Hz.
"""
import math
import os
import numpy as np
import pandas as pd
from scipy import signal

DT = 0.01 # 100 Hz evaluation grid
PRIMARY_WEIGHTS = {
    'imu.ay': 0.30,
    'imu.gz': 0.30,
    'chassis.u': 0.20,
    'imu.ax': 0.15,
    'chassis.v': 0.05,
}

def load_data():
    base_dir = "imu_compare"
    ctl = pd.read_csv(os.path.join(base_dir, "real_control.csv"))
    imu = pd.read_csv(os.path.join(base_dir, "real_imu.csv"))
    whl = pd.read_csv(os.path.join(base_dir, "real_wheels.csv"))
    sim_imu = pd.read_csv(os.path.join(base_dir, "sim_imu_opt_run4.csv"))
    sim_st = pd.read_csv(os.path.join(base_dir, "sim_state_opt_run4.csv"))
    
    real_onset = 123.35
    sim_onset = 123.345
    
    real_tau = imu['t_rel'].values - real_onset
    sim_tau = sim_imu['t_rel'].values - sim_onset
    
    # 100 Hz evaluation grid across full mission [20, 281] s
    grid_tau = np.arange(20.0, 281.0, DT)
    
    def get_interp(t_src, x_src):
        return np.interp(grid_tau, t_src, x_src, left=x_src[0], right=x_src[-1])
        
    r_ay = get_interp(real_tau, imu['ay'].values)
    r_gz = get_interp(real_tau, imu['gz'].values)
    r_ax = get_interp(real_tau, imu['ax'].values)
    r_gx = get_interp(real_tau, imu['gx'].values)
    
    r_rw_tau = whl['t'].values - ctl['t_unix'].iloc[0] - real_onset
    r_rw_rl = get_interp(r_rw_tau, whl['rl'].values)
    r_rw_rr = get_interp(r_rw_tau, whl['rr'].values)
    r_u = 0.5 * (r_rw_rl + r_rw_rr) * (2.0 * np.pi / 60.0) * 0.2032 * 0.928
    
    s_ay = get_interp(sim_tau, sim_imu['ay'].values)
    s_gz = get_interp(sim_tau, sim_imu['gz'].values)
    s_ax = get_interp(sim_tau, sim_imu['ax'].values)
    
    st_tau = sim_st['tau_s'].values if 'tau_s' in sim_st.columns else (sim_st['t_s'].values - sim_onset)
    s_u = get_interp(st_tau, sim_st['u_mps'].values)
    s_v = get_interp(st_tau, sim_st['v_mps'].values) if 'v_mps' in sim_st.columns else np.zeros_like(grid_tau)
    
    return {
        'grid_tau': grid_tau,
        'r_ay': r_ay, 'r_gz': r_gz, 'r_ax': r_ax, 'r_gx': r_gx, 'r_u': r_u,
        's_ay': s_ay, 's_gz': s_gz, 's_ax': s_ax, 's_u': s_u, 's_v': s_v,
    }

def compute_channel_metrics(r: np.ndarray, s: np.ndarray):
    m = np.isfinite(r) & np.isfinite(s)
    r_clean = r[m]
    s_clean = s[m]
    n = len(r_clean)
    if n < 10:
        return {'r': np.nan, 'gain': np.nan, 'rmse': np.nan, 'nrmse_std': np.nan, 'score': 0.0}
        
    diff = s_clean - r_clean
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    r_std = float(r_clean.std())
    s_std = float(s_clean.std())
    
    corr = float(np.corrcoef(r_clean, s_clean)[0, 1]) if r_std > 1e-6 and s_std > 1e-6 else np.nan
    r_dot_r = float(np.dot(r_clean - r_clean.mean(), r_clean - r_clean.mean()))
    gain = float(np.dot(s_clean - s_clean.mean(), r_clean - r_clean.mean()) / r_dot_r) if r_dot_r > 1e-9 else np.nan
    nrmse_std = float(rmse / r_std) if r_std > 1e-6 else np.nan
    
    if np.isnan(corr):
        return {'r': np.nan, 'gain': gain, 'rmse': rmse, 'nrmse_std': nrmse_std, 'score': 0.0}
        
    r_bounded = max(0.0, min(0.999, corr))
    z_score = math.tanh(1.5 * r_bounded) * 100.0
    gain_penalty = max(0.2, min(1.0, 1.0 / (1.0 + 0.5 * abs(math.log(gain))))) if (gain and gain > 0) else 1.0
    nrmse_penalty = max(0.2, min(1.0, 1.0 / (1.0 + 0.3 * nrmse_std))) if nrmse_std else 1.0
    ch_score = z_score * gain_penalty * nrmse_penalty
    
    return {
        'r': corr,
        'gain': gain,
        'rmse': rmse,
        'nrmse_std': nrmse_std,
        'score': ch_score,
    }

def compute_aggregate(metrics_dict):
    score_sum = 0.0
    total_w = 0.0
    for ch, w in PRIMARY_WEIGHTS.items():
        if ch in metrics_dict and not np.isnan(metrics_dict[ch]['r']):
            score_sum += metrics_dict[ch]['score'] * w
            total_w += w
    return float(score_sum / total_w) if total_w > 0 else 0.0

def apply_lpf(data, cutoff_hz, fs=100.0, order=4):
    b, a = signal.butter(order, cutoff_hz / (0.5 * fs), btype='low')
    return signal.filtfilt(b, a, data)

def run_evaluation():
    d = load_data()
    t = d['grid_tau']
    
    # -------------------------------------------------------------------------
    # Reconstructed CoG acceleration (lever arm tangential compensation)
    # -------------------------------------------------------------------------
    # r = [-0.150, 0.075, 1.000]
    # In 5Hz rigid-body band:
    b5, a5 = signal.butter(4, 5.0 / (0.5 * 100.0), btype='low')
    gx_5hz = signal.filtfilt(b5, a5, d['r_gx'])
    gz_5hz = signal.filtfilt(b5, a5, d['r_gz'])
    alpha_x_5hz = np.gradient(gx_5hz, DT)
    alpha_z_5hz = np.gradient(gz_5hz, DT)
    
    # Lever arm tangential term: (alpha x r)_y = alpha_z * rx - alpha_x * rz
    tangential_ay = alpha_z_5hz * (-0.150) - alpha_x_5hz * (1.000)
    r_ay_cog_recon = d['r_ay'] - tangential_ay
    
    # Filtered signals
    r_ay_5hz = apply_lpf(d['r_ay'], 5.0)
    s_ay_5hz = apply_lpf(d['s_ay'], 5.0)
    r_gz_5hz = apply_lpf(d['r_gz'], 5.0)
    s_gz_5hz = apply_lpf(d['s_gz'], 5.0)
    r_ax_5hz = apply_lpf(d['r_ax'], 5.0)
    s_ax_5hz = apply_lpf(d['s_ax'], 5.0)
    r_u_5hz = apply_lpf(d['r_u'], 5.0)
    s_u_5hz = apply_lpf(d['s_u'], 5.0)
    
    r_ay_3hz = apply_lpf(d['r_ay'], 3.0)
    s_ay_3hz = apply_lpf(d['s_ay'], 3.0)
    r_gz_3hz = apply_lpf(d['r_gz'], 3.0)
    s_gz_3hz = apply_lpf(d['s_gz'], 3.0)
    r_ax_3hz = apply_lpf(d['r_ax'], 3.0)
    s_ax_3hz = apply_lpf(d['s_ax'], 3.0)
    r_u_3hz = apply_lpf(d['r_u'], 3.0)
    s_u_3hz = apply_lpf(d['s_u'], 3.0)
    
    # Masks
    full_mask = np.ones(len(t), dtype=bool)
    valid_ctrl_mask = (t < 27.0) | (t > 74.0)
    
    cases = {
        'Case A: Original Raw Official (Historical Reference)': {
            'mask': full_mask,
            'r_ay': d['r_ay'], 's_ay': d['s_ay'],
            'r_gz': d['r_gz'], 's_gz': d['s_gz'],
            'r_ax': d['r_ax'], 's_ax': d['s_ax'],
            'r_u': d['r_u'],   's_u': d['s_u'],
        },
        'Case B: Reconstructed CoG Acceleration (Rigid Lever-Arm)': {
            'mask': full_mask,
            'r_ay': r_ay_cog_recon, 's_ay': d['s_ay'],
            'r_gz': d['r_gz'],      's_gz': d['s_gz'],
            'r_ax': d['r_ax'],      's_ax': d['s_ax'],
            'r_u': d['r_u'],        's_u': d['s_u'],
        },
        'Case C: Matched 5 Hz Bandwidth (True Vehicle Dynamics)': {
            'mask': full_mask,
            'r_ay': r_ay_5hz, 's_ay': s_ay_5hz,
            'r_gz': r_gz_5hz, 's_gz': s_gz_5hz,
            'r_ax': r_ax_5hz, 's_ax': s_ax_5hz,
            'r_u': r_u_5hz,   's_u': s_u_5hz,
        },
        'Case D: Matched 3 Hz Bandwidth (Sensitivity Analysis)': {
            'mask': full_mask,
            'r_ay': r_ay_3hz, 's_ay': s_ay_3hz,
            'r_gz': r_gz_3hz, 's_gz': s_gz_3hz,
            'r_ax': r_ax_3hz, 's_ax': s_ax_3hz,
            'r_u': r_u_3hz,   's_u': s_u_3hz,
        },
        'Valid-Control Coverage (Raw, Excl. 47s Standstill)': {
            'mask': valid_ctrl_mask,
            'r_ay': d['r_ay'], 's_ay': d['s_ay'],
            'r_gz': d['r_gz'], 's_gz': d['s_gz'],
            'r_ax': d['r_ax'], 's_ax': d['s_ax'],
            'r_u': d['r_u'],   's_u': d['s_u'],
        },
        'Valid-Control Coverage (Matched 5 Hz, Excl. 47s Standstill)': {
            'mask': valid_ctrl_mask,
            'r_ay': r_ay_5hz, 's_ay': s_ay_5hz,
            'r_gz': r_gz_5hz, 's_gz': s_gz_5hz,
            'r_ax': r_ax_5hz, 's_ax': s_ax_5hz,
            'r_u': r_u_5hz,   's_u': s_u_5hz,
        },
    }
    
    print("====================================================================================================")
    print("  MIRENASIM / TER26 — PHASE 3 SCORECARD COMPARISON (FROZEN opt_run4 CONFIGURATION)")
    print("====================================================================================================")
    print(f"{'Evaluation Case':<55} | {'Agg. Score':>10} | {'ay Score':>8} | {'gz Score':>8} | {'u Score':>8} | {'ax Score':>8}")
    print("-" * 108)
    
    for cname, cdata in cases.items():
        m = cdata['mask']
        ch_m = {
            'imu.ay': compute_channel_metrics(cdata['r_ay'][m], cdata['s_ay'][m]),
            'imu.gz': compute_channel_metrics(cdata['r_gz'][m], cdata['s_gz'][m]),
            'chassis.u': compute_channel_metrics(cdata['r_u'][m], cdata['s_u'][m]),
            'imu.ax': compute_channel_metrics(cdata['r_ax'][m], cdata['s_ax'][m]),
        }
        agg = compute_aggregate(ch_m)
        print(f"{cname:<55} | {agg:9.1f}% | {ch_m['imu.ay']['score']:7.1f}% | {ch_m['imu.gz']['score']:7.1f}% | {ch_m['chassis.u']['score']:7.1f}% | {ch_m['imu.ax']['score']:7.1f}%")
        
    print("\nDetailed Per-Channel Metrics for Case A, Case C, and Valid-Control Coverage:")
    for cname in ['Case A: Original Raw Official (Historical Reference)',
                  'Case C: Matched 5 Hz Bandwidth (True Vehicle Dynamics)',
                  'Valid-Control Coverage (Matched 5 Hz, Excl. 47s Standstill)']:
        cdata = cases[cname]
        m = cdata['mask']
        print(f"\n--- {cname} (N = {m.sum()}, Duration = {m.sum()*DT:.1f} s) ---")
        print(f"{'Channel':<15} | {'r':>7} | {'Gain':>7} | {'RMSE':>8} | {'NRMSE':>7} | {'Score':>7}")
        print("-" * 62)
        for ch in ['imu.ay', 'imu.gz', 'chassis.u', 'imu.ax']:
            r_sig = cdata[ch.replace('.', '_').replace('chassis', 'r').replace('imu', 'r') if 'chassis' in ch else ch.replace('.', '_').replace('imu', 'r')][m]
            s_sig = cdata[ch.replace('.', '_').replace('chassis', 's').replace('imu', 's') if 'chassis' in ch else ch.replace('.', '_').replace('imu', 's')][m]
            met = compute_channel_metrics(r_sig, s_sig)
            print(f"{ch:<15} | {met['r']:7.3f} | {met['gain']:7.3f} | {met['rmse']:8.3f} | {met['nrmse_std']:7.3f} | {met['score']:6.1f}%")

if __name__ == "__main__":
    run_evaluation()
