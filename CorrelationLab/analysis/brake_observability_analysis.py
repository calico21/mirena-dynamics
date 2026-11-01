#!/usr/bin/env python3
"""
Brake Observability & Forward-Speed Residual Forensic Analysis Tool (Phase IV Priority 5)
Author: MirenaSim Vehicle Dynamics Team
Date: November 2026

Investigates real telemetry signals (control, imu, wheels, odom) to classify all braking
and deceleration events, quantify missing brake input uncertainty, and identify
observable evaluation intervals.
"""

import os
import sys
sys.path.insert(0, os.path.abspath("."))
import numpy as np
import pandas as pd
from typing import Dict, Any, List

from CorrelationLab.core.data_loader import DataLoader
from CorrelationLab.core.signal_registry import SignalRegistry
from CorrelationLab.core.synchronization import TimeSynchronizer
from CorrelationLab.core.resampling import Resampler
from CorrelationLab.core.filters import FilterEngine
from CorrelationLab.analysis.longitudinal_analysis import LongitudinalAnalyzer


def analyze_brake_observability():
    print("Executing brake observability and event classification analysis...")
    out_dir = "CorrelationLab/reports/phase_iv_model_integrity"
    os.makedirs(out_dir, exist_ok=True)

    loader = DataLoader("CorrelationLab/data/real", "CorrelationLab/data/simulated", SignalRegistry())
    loader.load_real_data()

    real_ctl = loader.real_raw["control"]
    real_imu = loader.real_raw["imu"]
    real_whl = loader.real_raw["wheels"]

    r_onset, _ = TimeSynchronizer.get_standard_onset(real_ctl)
    t_ctl_0 = float(real_ctl["t_unix"].iloc[0])

    dt = 0.01
    grid_tau = np.arange(20.0, 281.0, dt)

    # Resample signals
    ax_raw = Resampler.interpolate_to_grid(real_imu["t_rel"].values - r_onset, real_imu["ax"].values, grid_tau)
    ay_raw = Resampler.interpolate_to_grid(real_imu["t_rel"].values - r_onset, real_imu["ay"].values, grid_tau)
    gz_raw = Resampler.interpolate_to_grid(real_imu["t_rel"].values - r_onset, real_imu["gz"].values, grid_tau)

    gas_raw = Resampler.interpolate_to_grid(real_ctl["t_rel"].values - r_onset, real_ctl["gas"].values, grid_tau)
    steer_raw = Resampler.interpolate_to_grid(real_ctl["t_rel"].values - r_onset, real_ctl["steer_rad"].values, grid_tau)

    r_u_tau = real_whl["t"].values - t_ctl_0 - r_onset
    rl_rpm = Resampler.interpolate_to_grid(r_u_tau, real_whl["rl"].values, grid_tau)
    rr_rpm = Resampler.interpolate_to_grid(r_u_tau, real_whl["rr"].values, grid_tau)
    u_mps = LongitudinalAnalyzer.reconstruct_speed_from_wheels(rl_rpm, rr_rpm)

    # Apply 5 Hz lowpass filter to ax and ay for stable event boundary detection
    spec = FilterEngine.create_filter("butterworth", cutoff_hz=5.0, order=4, fs_hz=100.0, causal=False)
    ax_5hz = FilterEngine.apply_filter(ax_raw, spec)
    ay_5hz = FilterEngine.apply_filter(ay_raw, spec)

    # Threshold for deceleration event: ax < -0.5 m/s^2
    is_decel = (ax_5hz < -0.5)

    events = []
    in_event = False
    start_idx = 0

    for i in range(len(grid_tau)):
        if is_decel[i] and not in_event:
            in_event = True
            start_idx = i
        elif not is_decel[i] and in_event:
            in_event = False
            end_idx = i
            # Minimum duration: 0.2 s (20 samples) to reject noise spikes
            if (end_idx - start_idx) >= 20:
                events.append((start_idx, end_idx))

    if in_event and (len(grid_tau) - start_idx) >= 20:
        events.append((start_idx, len(grid_tau)))

    classified_events = []
    total_delta_u_unlogged = 0.0
    total_delta_u_logged = 0.0
    total_delta_u_coast = 0.0
    total_delta_u_corner = 0.0

    for idx, (s_idx, e_idx) in enumerate(events):
        t_s = round(float(grid_tau[s_idx]), 2)
        t_e = round(float(grid_tau[e_idx - 1]), 2)
        dur = round(t_e - t_s, 2)

        ev_ax = ax_5hz[s_idx:e_idx]
        ev_ay = ay_5hz[s_idx:e_idx]
        ev_gas = gas_raw[s_idx:e_idx]
        ev_u = u_mps[s_idx:e_idx]

        min_ax = round(float(np.min(ev_ax)), 2)
        mean_ax = round(float(np.mean(ev_ax)), 2)
        mean_ay = round(float(np.mean(np.abs(ev_ay))), 2)
        mean_gas = round(float(np.mean(ev_gas)), 3)
        min_gas = round(float(np.min(ev_gas)), 3)
        u_init = round(float(ev_u[0]), 2)
        u_final = round(float(ev_u[-1]), 2)
        delta_u = round(float(u_final - u_init), 2)

        frac_neg_gas = float(np.mean(ev_gas < -0.05))
        frac_zero_gas = float(np.mean(np.abs(ev_gas) <= 0.05))
        frac_pos_gas = float(np.mean(ev_gas > 0.05))

        # Classification Logic
        if u_init < 0.5 and u_final < 0.5 and t_e <= 74.0:
            cls_name = "STATIONARY_BRAKE_HOLD"
            certainty = "HIGH"
            cause = "Vehicle stationary on grid/paddock; mechanical brake applied to prevent roll."
        elif frac_neg_gas > 0.3:
            cls_name = "LOGGED_CONTROL_BRAKING"
            certainty = "HIGH"
            cause = "Active negative control gas/brake command logged in telemetry."
            total_delta_u_logged += abs(delta_u)
        elif frac_pos_gas > 0.5 and mean_ay > 4.0:
            cls_name = "CORNERING_SCRUB_DECELERATION"
            certainty = "HIGH"
            cause = "Driver maintaining throttle but severe lateral tire scrub drag induces deceleration."
            total_delta_u_corner += abs(delta_u)
        elif min_ax < -1.8 and frac_zero_gas > 0.5:
            cls_name = "UNLOGGED_HYDRAULIC_BRAKING"
            certainty = "HIGH"
            cause = "Heavy deceleration exceeding aero/rolling drag while gas=0; unlogged mechanical hydraulic brake pedal pressed."
            total_delta_u_unlogged += abs(delta_u)
        elif -1.8 <= min_ax <= -0.5 and frac_zero_gas > 0.6:
            cls_name = "NATURAL_COASTDOWN"
            certainty = "MEDIUM"
            cause = "Moderate deceleration consistent with aerodynamic drag and rolling resistance with throttle released."
            total_delta_u_coast += abs(delta_u)
        else:
            cls_name = "AMBIGUOUS_TRANSIENT_DECEL"
            certainty = "LOW"
            cause = "Mixed throttle release, tire scrub, and partial brake drag."

        classified_events.append({
            "event_id": f"DECEL_{idx+1:03d}",
            "t_start_s": t_s,
            "t_end_s": t_e,
            "duration_s": dur,
            "min_ax_mps2": min_ax,
            "mean_ax_mps2": mean_ax,
            "mean_abs_ay_mps2": mean_ay,
            "mean_gas": mean_gas,
            "min_gas": min_gas,
            "speed_start_mps": u_init,
            "speed_end_mps": u_final,
            "delta_speed_mps": delta_u,
            "event_classification": cls_name,
            "classification_confidence": certainty,
            "physical_mechanism": cause
        })

    df_events = pd.DataFrame(classified_events)
    events_csv = os.path.join(out_dir, "brake_event_classification.csv")
    df_events.to_csv(events_csv, index=False)
    print(f"Saved {len(df_events)} classified deceleration events to {events_csv}")

    # Summary Statistics
    counts = df_events["event_classification"].value_counts()
    print("\n=== Deceleration Event Breakdown ===")
    for k, v in counts.items():
        sub = df_events[df_events["event_classification"] == k]
        tot_dur = sub["duration_s"].sum()
        print(f"  {k:30s}: {v:2d} events ({tot_dur:5.1f} s total)")

    print(f"\nTotal Speed Loss Attributable to Unlogged Hydraulic Braking: {total_delta_u_unlogged:.1f} m/s")
    print(f"Total Speed Loss Attributable to Logged Control Braking:     {total_delta_u_logged:.1f} m/s")
    print(f"Total Speed Loss Attributable to Natural Coastdown:          {total_delta_u_coast:.1f} m/s")
    print(f"Total Speed Loss Attributable to Cornering Scrub Drag:       {total_delta_u_corner:.1f} m/s")


if __name__ == "__main__":
    analyze_brake_observability()
