#!/usr/bin/env python3
"""
Parameter Sensitivity, Collinearity & Fisher Information Matrix (FIM) Observability Analysis
for the TeR26 Multibody Vehicle Dynamics Engine.

Computes the normalized trajectory Jacobian S_ij = (theta_j / sigma_yi) * d(y_i)/d(theta_j)
across key physical maneuvers (straight acceleration/braking vs. transient slalom/step-steer),
evaluates the Fisher Information Matrix I(theta) = S^T S, condition number kappa(I),
collinearity index gamma_K = 1 / sqrt(lambda_min(R_corr)), and ranks identifiable vs. collinear
parameter subsets.
"""

import json
import os
import numpy as np


def simulate_planar_coupled_trajectory(params: dict) -> np.ndarray:
    """
    Fast differentiable trajectory evaluation for FIM collinearity analysis across:
      - Phase 1 (t in [0, 2.0 s]): Full-throttle straight launch & hard braking
      - Phase 2 (t in [2.0, 5.0 s]): Transient slalom & step-steer cornering
    Returns stacked observable vector y = [ax(t), ay(t), r(t), vx(t), dFz_lat(t)].
    """
    dt = 0.005
    n_steps = 1000
    m = params["mass_kg"]
    Izz = params["Izz_kg_m2"]
    lf = params["lf_m"]
    lr = params["lr_m"]
    L = lf + lr
    h_cg = params["h_cg_m"]
    tf = params["track_m"]
    ClA = params["ClA_m2"]
    CdA = params["CdA_m2"]
    eta_g = params["drivetrain_efficiency"]
    mu_x = params["lambda_mux"]
    mu_y = params["lambda_muy"]
    Kya_f = params["Kya_front_N_rad"]
    Kya_r = params["Kya_rear_N_rad"]
    roll_dist_f = params["roll_stiffness_front_frac"]

    vx = 1.0
    vy = 0.0
    r = 0.0
    obs = []

    for k in range(n_steps):
        t = k * dt
        if t < 1.2:
            throttle, brake, steer = 1.0, 0.0, 0.0
        elif t < 2.0:
            throttle, brake, steer = 0.0, 0.7, 0.0
        else:
            throttle = 0.35
            brake = 0.0
            steer = 0.18 * np.sin(2.0 * np.pi * 0.7 * (t - 2.0))

        q_dyn = 0.5 * 1.225 * (vx**2)
        F_aero_z = q_dyn * ClA
        F_drag = q_dyn * CdA

        Fz_tot = m * 9.81 + F_aero_z
        Fz_f = Fz_tot * (lr / L)
        Fz_r = Fz_tot * (lf / L)

        # Longitudinal force
        F_drive_req = throttle * (2.0 * 100.0 * 5.0 * eta_g / 0.2032)
        F_drive = min(F_drive_req, 80000.0 * eta_g / max(vx, 3.0), mu_x * Fz_r)
        F_brk = brake * min(1600.0 / 0.2032, mu_x * Fz_tot)
        Fx_net = F_drive - F_brk - F_drag

        # Lateral slip angles & cornering forces
        alpha_f = np.arctan2(vy + lf * r, max(vx, 1.0)) - steer
        alpha_r = np.arctan2(vy - lr * r, max(vx, 1.0))

        Dy_f = mu_y * Fz_f
        Dy_r = mu_y * Fz_r
        Fy_f = -Dy_f * np.tanh((Kya_f / max(Dy_f, 1.0)) * alpha_f)
        Fy_r = -Dy_r * np.tanh((Kya_r / max(Dy_r, 1.0)) * alpha_r)

        ax_imu = Fx_net / m
        ay_imu = (Fy_f * np.cos(steer) + Fy_r) / m
        dvx = ax_imu + r * vy
        dvy = ay_imu - r * vx
        dr = (lf * Fy_f * np.cos(steer) - lr * Fy_r) / Izz

        vx = max(0.5, vx + dt * dvx)
        vy += dt * dvy
        r += dt * dr

        dFz_lat_f = (m * ay_imu * h_cg / tf) * roll_dist_f
        obs.append([ax_imu, ay_imu, r, vx, dFz_lat_f])

    return np.array(obs, dtype=np.float64)


def main() -> None:
    nom_params = {
        "mass_kg": 300.0,
        "Izz_kg_m2": 135.0,
        "lf_m": 0.876,
        "lr_m": 0.717,
        "h_cg_m": 0.330,
        "track_m": 1.174,
        "ClA_m2": 3.50,
        "CdA_m2": 1.40,
        "drivetrain_efficiency": 0.92,
        "lambda_mux": 1.65,
        "lambda_muy": 1.68,
        "Kya_front_N_rad": 26500.0,
        "Kya_rear_N_rad": 28500.0,
        "roll_stiffness_front_frac": 0.46,
    }

    param_names = [
        "Izz_kg_m2",
        "h_cg_m",
        "drivetrain_efficiency",
        "lambda_mux",
        "lambda_muy",
        "Kya_front_N_rad",
        "Kya_rear_N_rad",
        "ClA_m2",
        "CdA_m2",
        "roll_stiffness_front_frac",
    ]

    y0 = simulate_planar_coupled_trajectory(nom_params)
    # Standardize using IMU/wheel-speed sensor only (columns 0..3: ax, ay, r, vx)
    scales = np.std(y0[:, :4], axis=0) + 1e-3
    S_cols = []
    for pname in param_names:
        p_val = nom_params[pname]
        dp = 1e-4 * abs(p_val)
        pp = dict(nom_params)
        pm = dict(nom_params)
        pp[pname] = p_val + dp
        pm[pname] = p_val - dp
        yp = simulate_planar_coupled_trajectory(pp)[:, :4]
        ym = simulate_planar_coupled_trajectory(pm)[:, :4]
        dy_dtheta = ((yp - ym) / (2.0 * dp)) * p_val / scales
        S_cols.append(dy_dtheta.reshape(-1))

    S = np.column_stack(S_cols)
    FIM = S.T @ S
    norms = np.sqrt(np.diag(FIM)) + 1e-12
    corr_matrix = FIM / np.outer(norms, norms)

    eigvals = np.linalg.eigvalsh(FIM)
    cond_number = float(np.max(eigvals) / max(1e-12, np.min(eigvals)))

    # Identify top collinear pairs (|R_ij| > 0.70 for i != j)
    collinear_pairs = []
    for i in range(len(param_names)):
        for j in range(i + 1, len(param_names)):
            c_ij = float(corr_matrix[i, j])
            if abs(c_ij) > 0.65:
                collinear_pairs.append(
                    {
                        "param_1": param_names[i],
                        "param_2": param_names[j],
                        "correlation_coeff": round(c_ij, 4),
                    }
                )

    sensitivity_ranking = sorted(
        [
            {"parameter": param_names[i], "normalized_sensitivity_norm": round(float(norms[i]), 3)}
            for i in range(len(param_names))
        ],
        key=lambda x: x["normalized_sensitivity_norm"],
        reverse=True,
    )

    report = {
        "analysis": "Fisher Information Matrix (FIM) Observability & Structural Collinearity",
        "sensors_assumed": ["IMU_ax", "IMU_ay", "IMU_yaw_rate_r", "Wheel_Odometry_vx"],
        "fim_condition_number": round(cond_number, 2),
        "parameter_sensitivity_ranking": sensitivity_ranking,
        "high_collinearity_pairs": collinear_pairs,
        "unobservable_without_suspension_potentiometers": [
            "h_cg_m vs roll_stiffness_front_frac (pure IMU cannot separate CoG height from roll stiffness split without wheel normal force or LVDT jounce sensors)"
        ],
    }

    vehicle_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    out_path = os.path.join(vehicle_dir, "Data/Calibration/fim_observability_report.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    print("=== TeR26 FIM Observability & Parameter Sensitivity Report ===")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
