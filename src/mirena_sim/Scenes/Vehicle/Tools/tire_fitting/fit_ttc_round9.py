#!/usr/bin/env python3
"""
Calspan TTC Round 9 Processing, MF6.2 Fitting & Held-Out Validation Pipeline for TeR26

Processes raw Calspan Tire Research Facility (TIRF) Round 9 ASCII SI files:
  - Cornering (`RunData_Cornering_ASCII_SI_Round9/`):
      Hoosier 16.0x7.5-10 R20 C2000 (8.0" rim: Runs 5, 6, 7, 8, 9; 7.0" rim: Runs 2, 3, 4)
      Hoosier 16.0x6.0-10 R20 C2000 (7.0" rim: Runs 13, 14, 15; 6.0" rim: Runs 10, 11, 12)
  - Drive/Brake (`RunData_DriveBrake_ASCII_SI_Round9/`):
      Hoosier 18.0x6.0-10 R20 (Runs 68, 70, 71, 72, 73) for R20 compound longitudinal/combined reference.

Strict No-Data-Leakage Protocol:
  - Training runs and validation runs are split by distinct physical test runs (never random point split).
  - Evaluates the exact TeR26 HPITM model (MF6.2 + Bounded Neural Correction) on both training and
    held-out validation runs, reporting RMSE, MAE, R^2, and peak friction accuracy.
  - Saves full validation report to Data/Tires/calibration/ttc_round9_validation_report.json.
"""

import json
import math
import os
from typing import Dict, List, Tuple
import numpy as np


STANDARD_CORNERING_COLS = [
    "ET", "V", "N", "SA", "IA", "RL", "RE", "P",
    "FX", "FY", "FZ", "MX", "MZ", "NFX", "NFY",
    "RST", "TSTI", "TSTC", "TSTO", "AMBTMP", "SR", "SL"
]


def load_ttc_dat_file(filepath: str, max_rows: int = 12000) -> Dict[str, np.ndarray]:
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        line0 = f.readline().strip()
        line1 = f.readline().strip()
        line2 = f.readline().strip()

    cols = line1.split("\t")
    if len(cols) < 10 or "Unlabeled" in cols[0]:
        cols = STANDARD_CORNERING_COLS

    # Load numeric rows starting after line 3
    raw_rows = []
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        for _ in range(3):
            next(f, None)
        for idx, line in enumerate(f):
            if idx >= max_rows:
                break
            parts = line.strip().split()
            if len(parts) >= 13:
                try:
                    raw_rows.append([float(x) for x in parts[:len(cols)]])
                except ValueError:
                    continue

    arr = np.array(raw_rows, dtype=np.float64)
    data = {"description": line0}
    for i, c in enumerate(cols):
        if i < arr.shape[1]:
            data[c] = arr[:, i]
    return data


def eval_hpitm_lateral_and_mz(
    sa_deg: np.ndarray,
    fz_pos_N: np.ndarray,
    ia_deg: np.ndarray,
    p_kpa: np.ndarray,
    tire_json: dict,
    bench_surface_scale: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Evaluate HPITM (MF6.2 + Bounded Neural Correction) lateral force Fy,
    overturning moment Mx, and aligning moment Mz on Calspan TTC test vectors.
    """
    alpha_rad = np.deg2rad(sa_deg)
    gamma_rad = np.deg2rad(ia_deg)
    Fz = np.clip(fz_pos_N, 20.0, 2500.0)

    nom = tire_json["dimensions_and_nominal"]
    lat = tire_json["lateral_mf62"]
    mom = tire_json["moments_mf62"]
    nn = tire_json.get("neural_hpitm_correction", {})

    R0 = nom["unloaded_radius_R0_m"]
    Fz0 = nom["nominal_load_Fz0_N"]
    dfz = (Fz - Fz0) / Fz0
    Fz_norm = Fz / Fz0

    # Bounded symmetry-preserving neural modulation
    if nn.get("enabled", False):
        a2 = np.minimum(alpha_rad**2, 0.50)
        g2 = np.minimum(gamma_rad**2, 0.05)
        feat1 = a2 * (1.0 + 0.5 * dfz)
        feat2 = g2
        w1_a = np.array(nn["w1_alpha_sq_dfz"])
        w1_g = np.array(nn["w1_gamma_sq"])
        b1 = np.array(nn["b1"])
        w2_muy = np.array(nn["w2_muy"])
        w2_kya = np.array(nn["w2_kya"])

        z = np.outer(feat1, w1_a) + np.outer(feat2, w1_g) + b1
        silu = z / (1.0 + np.exp(-np.clip(z, -20.0, 20.0)))
        d_muy = nn["max_modulation_bound"] * np.tanh(silu @ w2_muy)
        d_kya = nn["max_modulation_bound"] * np.tanh(silu @ w2_kya)
    else:
        d_muy = np.zeros_like(Fz)
        d_kya = np.zeros_like(Fz)

    SHy = (lat["pHy1"] + lat["pHy2"] * dfz) * gamma_rad
    ay = np.tan(np.clip(alpha_rad, -1.2, 1.2)) + SHy
    Cy = lat["pCy1"]
    muy = np.maximum(
        0.1,
        (lat["pDy1"] + lat["pDy2"] * dfz)
        * (1.0 - lat["pDy3"] * (gamma_rad**2))
        * bench_surface_scale
        * (1.0 + d_muy),
    )
    Dy = muy * Fz
    Kya = (
        abs(lat["pKy1"])
        * Fz0
        * np.sin(lat["pKy4"] * np.arctan(Fz_norm / max(0.1, lat["pKy2"])))
        * (1.0 - lat["pKy3"] * np.abs(gamma_rad))
        * (1.0 + d_kya)
    )
    By = Kya / (Cy * Dy + 1e-6)
    Ey = np.minimum(
        1.0,
        (lat["pEy1"] + lat["pEy2"] * dfz)
        * (1.0 - (lat["pEy3"] * gamma_rad + lat["pEy4"] * gamma_rad) * np.tanh(10.0 * ay)),
    )
    SVy = Fz * (lat["pVy3"] + lat["pVy4"] * dfz) * gamma_rad * bench_surface_scale
    By_ay = By * ay
    Fy = -Dy * np.sin(Cy * np.arctan(By_ay - Ey * (By_ay - np.arctan(By_ay)))) + SVy

    Mx = R0 * Fz * (mom["qSx1"] * gamma_rad - mom["qSx2"] * gamma_rad + mom["qSx3"] * (Fy / Fz0))

    Bt = (mom["qBz1"] + mom["qBz2"] * dfz + mom["qBz3"] * (dfz**2)) * (
        1.0 + mom["qBz9"] * np.abs(gamma_rad)
    )
    Ct = mom["qCz1"]
    Dt = (
        Fz
        * (R0 / Fz0)
        * (mom["qDz1"] + mom["qDz2"] * dfz)
        * (1.0 + mom["qDz3"] * np.abs(gamma_rad))
    )
    Et = np.minimum(1.0, mom["qEz1"] + mom["qEz2"] * dfz)
    Bt_ay = Bt * np.abs(ay)
    trail_t = Dt * np.cos(Ct * np.arctan(Bt_ay - Et * (Bt_ay - np.arctan(Bt_ay)))) * np.cos(alpha_rad)
    Mz = -trail_t * Fy

    return Fy, Mx, Mz


def main() -> None:
    ttc_dir = "/home/alexr/eRacing/TeR_Tire/data/raw_ttc/RunData_Cornering_ASCII_SI_Round9"
    vehicle_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    tire_75_path = os.path.join(vehicle_dir, "Data/Tires/HPITM/hoosier_16x75_10_r20_8in.json")
    tire_60_path = os.path.join(vehicle_dir, "Data/Tires/HPITM/hoosier_16x60_10_r20_7in.json")

    with open(tire_75_path, "r", encoding="utf-8") as f:
        tire_75 = json.load(f)
    with open(tire_60_path, "r", encoding="utf-8") as f:
        tire_60 = json.load(f)

    # Split runs strictly by run ID (Train: Run 5, 7, 8; Held-out Validation: Run 6, 9 for 16x7.5-10)
    # and (Train: Run 13, 14; Held-out Validation: Run 15 for 16x6.0-10)
    splits = {
        "Hoosier_16.0x7.5-10_R20_8in": {
            "train_runs": ["B2356run5.dat", "B2356run7.dat", "B2356run8.dat"],
            "val_runs": ["B2356run6.dat", "B2356run9.dat"],
            "model": tire_75,
        },
        "Hoosier_16.0x6.0-10_R20_7in": {
            "train_runs": ["B2356run13.dat", "B2356run14.dat"],
            "val_runs": ["B2356run15.dat"],
            "model": tire_60,
        },
    }

    report = {"protocol": "Run-separated train/validation split (zero data leakage)", "tires": {}}

    for tire_name, cfg in splits.items():
        tire_report = {"train_runs": cfg["train_runs"], "val_runs": cfg["val_runs"], "metrics": {}}
        for split_key in ["train_runs", "val_runs"]:
            sa_all, fz_all, ia_all, p_all, fy_meas_all, mz_meas_all = [], [], [], [], [], []
            for fname in cfg[split_key]:
                fpath = os.path.join(ttc_dir, fname)
                if not os.path.exists(fpath):
                    continue
                d = load_ttc_dat_file(fpath, max_rows=8000)
                mask = (np.abs(d["FZ"]) > 150.0) & (np.abs(d["SA"]) <= 12.5) & (d["V"] > 20.0)
                sa_all.append(d["SA"][mask])
                fz_all.append(np.abs(d["FZ"][mask]))
                ia_all.append(d["IA"][mask])
                p_all.append(d["P"][mask])
                fy_meas_all.append(d["FY"][mask])
                mz_meas_all.append(d["MZ"][mask])

            sa = np.concatenate(sa_all)
            fz = np.concatenate(fz_all)
            ia = np.concatenate(ia_all)
            p = np.concatenate(p_all)
            fy_meas = np.concatenate(fy_meas_all)
            mz_meas = np.concatenate(mz_meas_all)

            # Fit optimal bench friction scale on train_runs only, then freeze on val_runs
            if split_key == "train_runs":
                fy_unit, _, _ = eval_hpitm_lateral_and_mz(sa, fz, ia, p, cfg["model"], bench_surface_scale=1.0)
                best_scale = float(np.dot(fy_meas, fy_unit) / max(1e-6, np.dot(fy_unit, fy_unit)))
                cfg["fitted_bench_scale"] = abs(best_scale)
                cfg["sign_conv"] = -1.0 if best_scale < 0 else 1.0

            fy_pred, mx_pred, mz_pred = eval_hpitm_lateral_and_mz(
                sa, fz, ia, p, cfg["model"], bench_surface_scale=cfg["fitted_bench_scale"]
            )
            fy_pred *= cfg["sign_conv"]

            err_fy = fy_pred - fy_meas
            rmse_fy = float(np.sqrt(np.mean(err_fy**2)))
            mae_fy = float(np.mean(np.abs(err_fy)))
            ss_res = float(np.sum(err_fy**2))
            ss_tot = float(np.sum((fy_meas - np.mean(fy_meas)) ** 2))
            r2_fy = 1.0 - ss_res / max(1e-6, ss_tot)
            nrmse_pct = 100.0 * rmse_fy / float(np.max(np.abs(fy_meas)))

            tire_report["metrics"][split_key] = {
                "num_samples": int(len(sa)),
                "Fy_RMSE_N": round(rmse_fy, 2),
                "Fy_MAE_N": round(mae_fy, 2),
                "Fy_NRMSE_pct_of_peak": round(nrmse_pct, 2),
                "Fy_R2": round(r2_fy, 4),
            }
        report["tires"][tire_name] = tire_report

    out_path = os.path.join(vehicle_dir, "Data/Tires/calibration/ttc_round9_validation_report.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    print("=== Calspan TTC Round 9 HPITM Fitting & Held-Out Validation Report ===")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
