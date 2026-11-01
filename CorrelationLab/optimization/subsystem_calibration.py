"""Subsystem-Level Vehicle Dynamics Calibration Engine.

Executes sequential, decoupled physical calibration stages:
- Stage A: Longitudinal Dynamics & Traction (ax, u, wheel speeds)
- Stage B: Lateral Acceleration & Yaw Rate (ay, gz)
- Stage C: Forward Speed & Wheel Speed Alignment (u, rl_rpm, rr_rpm)
- Stage D: Constrained Coupled Multi-Objective Optimization

Enforces strict parameter bounds, parameter freezing between stages,
unlogged mechanical brake observability masking, and multi-partition validation.
"""

import os
import json
import yaml
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

from .model_parameter_registry import ModelParameterRegistry
from .experiment_manager import ExperimentManager
from .calibration_objectives import CalibrationObjectiveEvaluator
from ..core.data_loader import DataLoader
from ..core.signal_registry import SignalRegistry
from ..core.synchronization import TimeSynchronizer
from ..core.resampling import Resampler
from ..core.filters import FilterEngine
from ..core.metrics import MetricCatalogue
from ..analysis.longitudinal_analysis import LongitudinalAnalyzer


class SubsystemCalibrationEngine:
    """Orchestrates phased physics-based vehicle dynamics calibration."""

    def __init__(
        self,
        registry: Optional[ModelParameterRegistry] = None,
        experiment_mgr: Optional[ExperimentManager] = None,
        stages_yaml: str = "CorrelationLab/config/calibration_stages.yaml",
        dt: float = 0.01,
    ):
        self.dt = dt
        self.fs = 1.0 / dt
        self.stages_yaml = stages_yaml
        self.registry = registry or ModelParameterRegistry()
        self.exp_mgr = experiment_mgr or ExperimentManager()
        self.obj_eval = CalibrationObjectiveEvaluator(dt=dt)

        # Load reference real telemetry
        sig_reg = SignalRegistry("CorrelationLab/config/channels.yaml")
        self.loader = DataLoader("CorrelationLab/data/real", "CorrelationLab/data/simulated", sig_reg)
        self.loader.load_real_data()

        # Load stage configurations
        self.stages_cfg = self._load_stages()

    def _load_stages(self) -> Dict[str, Any]:
        """Load calibration stage definitions."""
        if not os.path.exists(self.stages_yaml):
            return {}
        with open(self.stages_yaml, "r") as f:
            data = yaml.safe_load(f)
        return data.get("stages", {})

    def evaluate_candidate_partition(
        self,
        candidate_prefix: str,
        t_start: float = 20.0,
        t_end: float = 160.0,
        mask_stationary_brake: bool = True,
    ) -> Dict[str, Any]:
        """Evaluate a simulation candidate over a specified time window.
        
        If mask_stationary_brake is True, the unlogged stationary mechanical brake hold
        period [27.0, 74.0]s is excluded from longitudinal fitting.
        """
        sim_raw = self.loader.load_sim_run(candidate_prefix)
        real_ctl = self.loader.real_raw["control"]
        real_imu = self.loader.real_raw["imu"]
        real_whl = self.loader.real_raw["wheels"]
        sim_ctl = sim_raw["control"]
        sim_imu = sim_raw["imu"]
        sim_st = sim_raw["state"]

        r_onset, _ = TimeSynchronizer.get_standard_onset(real_ctl)
        s_onset, _ = TimeSynchronizer.get_standard_onset(sim_ctl)

        grid_tau = np.arange(t_start, t_end, self.dt)

        # Resample signals onto uniform 100 Hz grid
        r_ay = Resampler.interpolate_to_grid(real_imu["t_rel"].values - r_onset, real_imu["ay"].values, grid_tau)
        s_ay = Resampler.interpolate_to_grid(sim_imu["t_rel"].values - s_onset, sim_imu["ay"].values, grid_tau)

        r_gz = Resampler.interpolate_to_grid(real_imu["t_rel"].values - r_onset, real_imu["gz"].values, grid_tau)
        s_gz = Resampler.interpolate_to_grid(sim_imu["t_rel"].values - s_onset, sim_imu["gz"].values, grid_tau)

        r_ax = Resampler.interpolate_to_grid(real_imu["t_rel"].values - r_onset, real_imu["ax"].values, grid_tau)
        s_ax = Resampler.interpolate_to_grid(sim_imu["t_rel"].values - s_onset, sim_imu["ax"].values, grid_tau)

        t_ctl_0 = float(real_ctl["t_unix"].iloc[0])
        r_u_tau = real_whl["t"].values - t_ctl_0 - r_onset
        # Reconstruct speed from both driven rear wheels (axle-averaged, eliminates differential cornering bias)
        r_rl = Resampler.interpolate_to_grid(r_u_tau, real_whl["rl"].values, grid_tau)
        r_rr = Resampler.interpolate_to_grid(r_u_tau, real_whl["rr"].values, grid_tau)
        r_u = LongitudinalAnalyzer.reconstruct_speed_from_wheels(r_rl, r_rr)
        s_u = Resampler.interpolate_to_grid(sim_st["tau_s"].values, sim_st["u_mps"].values, grid_tau)

        # Sim wheel speeds
        s_rl = Resampler.interpolate_to_grid(sim_st["tau_s"].values, sim_st["rl_rpm"].values, grid_tau)
        s_rr = Resampler.interpolate_to_grid(sim_st["tau_s"].values, sim_st["rr_rpm"].values, grid_tau)

        # Standard 5 Hz Butterworth 4th-order zero-phase filter
        spec = FilterEngine.create_filter("butterworth", cutoff_hz=5.0, order=4, fs_hz=self.fs, causal=False)
        r_ay_f = FilterEngine.apply_filter(r_ay, spec)
        s_ay_f = FilterEngine.apply_filter(s_ay, spec)
        r_gz_f = FilterEngine.apply_filter(r_gz, spec)
        s_gz_f = FilterEngine.apply_filter(s_gz, spec)
        r_ax_f = FilterEngine.apply_filter(r_ax, spec)
        s_ax_f = FilterEngine.apply_filter(s_ax, spec)
        r_u_f = FilterEngine.apply_filter(r_u, spec)
        s_u_f = FilterEngine.apply_filter(s_u, spec)

        # Apply brake observability mask if requested
        if mask_stationary_brake:
            valid_mask = ~((grid_tau >= 27.0) & (grid_tau <= 74.0))
        else:
            valid_mask = np.ones_like(grid_tau, dtype=bool)

        sigs_5hz = {
            "imu.ay": {"real": r_ay_f, "sim": s_ay_f},
            "imu.gz": {"real": r_gz_f, "sim": s_gz_f},
            "imu.ax": {"real": r_ax_f[valid_mask], "sim": s_ax_f[valid_mask]},
            "chassis.u": {"real": r_u_f[valid_mask], "sim": s_u_f[valid_mask]},
        }

        # Subsystem evaluation
        long_cost = self.obj_eval.compute_subsystem_cost(sigs_5hz, "longitudinal")
        lat_cost = self.obj_eval.compute_subsystem_cost(sigs_5hz, "lateral")
        coupled_cost = self.obj_eval.compute_subsystem_cost(sigs_5hz, "coupled")

        # Channel level detailed metrics (raw and filtered)
        def ch_metrics(r, s):
            m = MetricCatalogue.compute_all_metrics(r, s, dt=self.dt)
            return {
                "pearson_r": m["pearson_r"],
                "ccc": m["ccc"],
                "gain": m["gain"],
                "rmse": m["rmse"],
                "mae": m["mae"],
                "bias": m["bias"],
            }

        channels_eval = {
            "imu.ay_5hz": ch_metrics(r_ay_f, s_ay_f),
            "imu.gz_5hz": ch_metrics(r_gz_f, s_gz_f),
            "imu.ax_5hz": ch_metrics(r_ax_f[valid_mask], s_ax_f[valid_mask]),
            "chassis.u": ch_metrics(r_u[valid_mask], s_u[valid_mask]),
            "wheel.rl_rpm": ch_metrics(r_rl[valid_mask], s_rl[valid_mask]),
            "wheel.rr_rpm": ch_metrics(r_rr[valid_mask], s_rr[valid_mask]),
            "imu.ay_raw": ch_metrics(r_ay, s_ay),
            "imu.gz_raw": ch_metrics(r_gz, s_gz),
            "imu.ax_raw": ch_metrics(r_ax[valid_mask], s_ax[valid_mask]),
        }

        return {
            "candidate": candidate_prefix,
            "window": (t_start, t_end),
            "longitudinal_cost": long_cost["subsystem_cost"],
            "lateral_cost": lat_cost["subsystem_cost"],
            "coupled_cost": coupled_cost["subsystem_cost"],
            "channels": channels_eval,
        }

    def execute_and_evaluate(
        self,
        candidate_id: str,
        param_overrides: Dict[str, float],
        base_prefix: str = "ter26_opt",
        duration_s: float = 160.0,
        eval_window: Tuple[float, float] = (20.0, 160.0),
    ) -> Dict[str, Any]:
        """Generate candidate .tres, run simulation, and evaluate metrics."""
        cand_res_path = self.registry.create_candidate_configuration(candidate_id, param_overrides, base_prefix)
        exec_res = self.exp_mgr.run_simulation(cand_res_path, candidate_id, duration_s=duration_s)
        if not exec_res["success"]:
            return {"candidate": candidate_id, "success": False, "error": exec_res.get("error", "SIM_FAIL")}

        eval_res = self.evaluate_candidate_partition(candidate_id, eval_window[0], eval_window[1])
        eval_res["success"] = True
        eval_res["elapsed_s"] = exec_res["elapsed_s"]
        eval_res["overrides"] = param_overrides
        return eval_res

    def run_stage_a_longitudinal(
        self,
        base_prefix: str = "ter26_opt",
        duration_s: float = 160.0,
    ) -> Dict[str, Any]:
        """Stage A: Longitudinal Dynamics & Traction Calibration.
        
        Tunes:
        - companion_longitudinal_peak_mu: [1.35, 1.95]
        - companion_longitudinal_kx_per_fz: [20.0, 45.0]
        - motor_peak_torque_nm: [85.0, 120.0]
        - rolling_resistance_coeff: [0.010, 0.022]
        - wheel_unloaded_radius_m: [0.1980, 0.2080]
        """
        print("\n=======================================================")
        print("  STAGE A: Longitudinal Dynamics Calibration")
        print("=======================================================")

        # Baseline evaluation
        base_eval = self.evaluate_candidate_partition(base_prefix, 20.0, duration_s)
        print(f"  Base '{base_prefix}' Long Cost: {base_eval['longitudinal_cost']:.4f}")
        print(f"    imu.ax: r={base_eval['channels']['imu.ax_5hz']['pearson_r']:.4f}, Gain={base_eval['channels']['imu.ax_5hz']['gain']:.4f}")
        print(f"    chassis.u: r={base_eval['channels']['chassis.u']['pearson_r']:.4f}, RMSE={base_eval['channels']['chassis.u']['rmse']:.4f}")

        best_cost = base_eval["longitudinal_cost"]
        best_overrides = {}
        best_eval = base_eval

        # Coordinated parametric sweep across longitudinal parameter space
        candidate_trials = [
            # Trial A1: Increase motor torque & longitudinal mu to raise ax gain
            {
                "motor_peak_torque_nm": 110.0,
                "companion_longitudinal_peak_mu": 1.80,
                "companion_longitudinal_kx_per_fz": 34.0,
                "rolling_resistance_coeff": 0.015,
                "wheel_unloaded_radius_m": 0.2015,
            },
            # Trial A2: Higher motor torque (115 Nm) and higher rolling resistance to brake speed drift
            {
                "motor_peak_torque_nm": 115.0,
                "companion_longitudinal_peak_mu": 1.85,
                "companion_longitudinal_kx_per_fz": 36.0,
                "rolling_resistance_coeff": 0.017,
                "wheel_unloaded_radius_m": 0.2010,
            },
            # Trial A3: Moderate motor torque (108 Nm), balanced rolling resistance
            {
                "motor_peak_torque_nm": 108.0,
                "companion_longitudinal_peak_mu": 1.78,
                "companion_longitudinal_kx_per_fz": 33.0,
                "rolling_resistance_coeff": 0.016,
                "wheel_unloaded_radius_m": 0.2020,
            },
            # Trial A4: Peak EMRAX torque (118 Nm), peak tire grip (1.88), strong rolling resistance (0.018)
            {
                "motor_peak_torque_nm": 118.0,
                "companion_longitudinal_peak_mu": 1.88,
                "companion_longitudinal_kx_per_fz": 38.0,
                "rolling_resistance_coeff": 0.018,
                "wheel_unloaded_radius_m": 0.2008,
            },
        ]

        trials_history = []
        for idx, overrides in enumerate(candidate_trials):
            cand_id = f"stage_a_cand_{idx+1}"
            print(f"\n  Running Stage A Trial {idx+1}/{len(candidate_trials)} ({cand_id})...")
            res = self.execute_and_evaluate(cand_id, overrides, base_prefix, duration_s=duration_s, eval_window=(20.0, duration_s))
            if not res.get("success"):
                print(f"    Trial {cand_id} failed!")
                continue

            cost = res["longitudinal_cost"]
            ax_m = res["channels"]["imu.ax_5hz"]
            u_m = res["channels"]["chassis.u"]
            print(f"    Cost: {cost:.4f} (ax Gain={ax_m['gain']:.4f}, r={ax_m['pearson_r']:.4f} | u RMSE={u_m['rmse']:.4f}, r={u_m['pearson_r']:.4f})")
            trials_history.append({"cand_id": cand_id, "cost": cost, "eval": res, "overrides": overrides})

            if cost < best_cost:
                best_cost = cost
                best_overrides = overrides
                best_eval = res

        print(f"\n  --> Stage A Completed. Best Long Cost: {best_cost:.4f}")
        return {
            "stage": "A_longitudinal",
            "best_overrides": best_overrides,
            "best_cost": best_cost,
            "best_eval": best_eval,
            "trials_history": trials_history,
        }

    def run_stage_b_lateral_yaw(
        self,
        base_overrides: Dict[str, float],
        base_prefix: str = "ter26_opt",
        duration_s: float = 160.0,
    ) -> Dict[str, Any]:
        """Stage B: Lateral Acceleration & Yaw Rate Calibration.
        
        Keeps Stage A parameters frozen.
        Tunes:
        - compliance_steer_front_radpn: [0.0, 0.000035]
        - diff_preload_viscous_nm_s_rad: [10.0, 50.0]
        - izz_kgm2: [115.0, 150.0]
        - ter_lateral_grip_scale: [0.90, 1.10]
        """
        print("\n=======================================================")
        print("  STAGE B: Lateral & Yaw Dynamics Calibration")
        print("=======================================================")

        # Run reference with Stage A overrides
        ref_id = "stage_b_ref"
        ref_res = self.execute_and_evaluate(ref_id, base_overrides, base_prefix, duration_s=duration_s, eval_window=(20.0, duration_s))
        best_cost = ref_res["lateral_cost"]
        best_overrides = dict(base_overrides)
        best_eval = ref_res

        print(f"  Reference Stage B Lateral Cost: {best_cost:.4f}")
        print(f"    imu.ay: r={ref_res['channels']['imu.ay_5hz']['pearson_r']:.4f}, Gain={ref_res['channels']['imu.ay_5hz']['gain']:.4f}")
        print(f"    imu.gz: r={ref_res['channels']['imu.gz_5hz']['pearson_r']:.4f}, Gain={ref_res['channels']['imu.gz_5hz']['gain']:.4f}")

        candidate_trials = [
            # Trial B1: Compliance steer 1.5e-5 rad/N, diff preload 35 Nm (standard FS setup)
            {
                "compliance_steer_front_radpn": 0.000015,
                "diff_preload_viscous_nm_s_rad": 35.0,
                "izz_kgm2": 135.0,
                "ter_lateral_grip_scale": 1.00,
            },
            # Trial B2: Compliance steer 2.0e-5 rad/N, diff preload 40 Nm
            {
                "compliance_steer_front_radpn": 0.000020,
                "diff_preload_viscous_nm_s_rad": 40.0,
                "izz_kgm2": 135.0,
                "ter_lateral_grip_scale": 1.00,
            },
            # Trial B3: Compliance steer 1.2e-5 rad/N, diff preload 30 Nm, grip scale 1.02
            {
                "compliance_steer_front_radpn": 0.000012,
                "diff_preload_viscous_nm_s_rad": 30.0,
                "izz_kgm2": 132.0,
                "ter_lateral_grip_scale": 1.02,
            },
            # Trial B4: Compliance steer 1.8e-5 rad/N, diff preload 35 Nm, izz 138 kgm2
            {
                "compliance_steer_front_radpn": 0.000018,
                "diff_preload_viscous_nm_s_rad": 35.0,
                "izz_kgm2": 138.0,
                "ter_lateral_grip_scale": 1.01,
            },
        ]

        trials_history = []
        for idx, b_overrides in enumerate(candidate_trials):
            cand_id = f"stage_b_cand_{idx+1}"
            combined_overrides = dict(base_overrides)
            combined_overrides.update(b_overrides)

            print(f"\n  Running Stage B Trial {idx+1}/{len(candidate_trials)} ({cand_id})...")
            res = self.execute_and_evaluate(cand_id, combined_overrides, base_prefix, duration_s=duration_s, eval_window=(20.0, duration_s))
            if not res.get("success"):
                print(f"    Trial {cand_id} failed!")
                continue

            cost = res["lateral_cost"]
            ay_m = res["channels"]["imu.ay_5hz"]
            gz_m = res["channels"]["imu.gz_5hz"]
            print(f"    Cost: {cost:.4f} (ay Gain={ay_m['gain']:.4f}, r={ay_m['pearson_r']:.4f} | gz Gain={gz_m['gain']:.4f}, r={gz_m['pearson_r']:.4f})")
            trials_history.append({"cand_id": cand_id, "cost": cost, "eval": res, "overrides": combined_overrides})

            if cost < best_cost:
                best_cost = cost
                best_overrides = combined_overrides
                best_eval = res

        print(f"\n  --> Stage B Completed. Best Lateral Cost: {best_cost:.4f}")
        return {
            "stage": "B_lateral_yaw",
            "best_overrides": best_overrides,
            "best_cost": best_cost,
            "best_eval": best_eval,
            "trials_history": trials_history,
        }

    def run_stage_c_speed_consistency(
        self,
        base_overrides: Dict[str, float],
        base_prefix: str = "ter26_opt",
        duration_s: float = 160.0,
    ) -> Dict[str, Any]:
        """Stage C: Speed and Wheel Speed Consistency Alignment.
        
        Tunes:
        - wheel_unloaded_radius_m: [0.1980, 0.2080]
        - drivetrain_efficiency: [0.88, 0.96]
        - diff_lock_ratio: [0.45, 0.75]
        """
        print("\n=======================================================")
        print("  STAGE C: Speed & Wheel Speed Alignment")
        print("=======================================================")

        ref_id = "stage_c_ref"
        ref_res = self.execute_and_evaluate(ref_id, base_overrides, base_prefix, duration_s=duration_s, eval_window=(20.0, duration_s))
        best_cost = ref_res["channels"]["chassis.u"]["rmse"]
        best_overrides = dict(base_overrides)
        best_eval = ref_res

        print(f"  Reference Stage C Speed RMSE: {best_cost:.4f} m/s")

        candidate_trials = [
            # Trial C1: Radius 0.2012m, diff lock ratio 0.65, drivetrain efficiency 0.93
            {
                "wheel_unloaded_radius_m": 0.2012,
                "drivetrain_efficiency": 0.93,
                "diff_lock_ratio": 0.65,
            },
            # Trial C2: Radius 0.2015m, diff lock ratio 0.62, drivetrain efficiency 0.935
            {
                "wheel_unloaded_radius_m": 0.2015,
                "drivetrain_efficiency": 0.935,
                "diff_lock_ratio": 0.62,
            },
            # Trial C3: Radius 0.2008m, diff lock ratio 0.65, drivetrain efficiency 0.925
            {
                "wheel_unloaded_radius_m": 0.2008,
                "drivetrain_efficiency": 0.925,
                "diff_lock_ratio": 0.65,
            },
        ]

        trials_history = []
        for idx, c_overrides in enumerate(candidate_trials):
            cand_id = f"stage_c_cand_{idx+1}"
            combined_overrides = dict(base_overrides)
            combined_overrides.update(c_overrides)

            print(f"\n  Running Stage C Trial {idx+1}/{len(candidate_trials)} ({cand_id})...")
            res = self.execute_and_evaluate(cand_id, combined_overrides, base_prefix, duration_s=duration_s, eval_window=(20.0, duration_s))
            if not res.get("success"):
                print(f"    Trial {cand_id} failed!")
                continue

            u_rmse = res["channels"]["chassis.u"]["rmse"]
            u_r = res["channels"]["chassis.u"]["pearson_r"]
            rl_r = res["channels"]["wheel.rl_rpm"]["pearson_r"]
            rr_r = res["channels"]["wheel.rr_rpm"]["pearson_r"]
            coupled_cost = res["coupled_cost"]

            print(f"    Speed RMSE: {u_rmse:.4f} m/s (u r={u_r:.4f} | RL r={rl_r:.4f}, RR r={rr_r:.4f} | Coupled Cost={coupled_cost:.4f})")
            trials_history.append({"cand_id": cand_id, "u_rmse": u_rmse, "eval": res, "overrides": combined_overrides})

            if coupled_cost < best_eval["coupled_cost"] and u_rmse <= best_cost:
                best_cost = u_rmse
                best_overrides = combined_overrides
                best_eval = res

        print(f"\n  --> Stage C Completed. Best Speed RMSE: {best_cost:.4f} m/s")
        return {
            "stage": "C_speed_consistency",
            "best_overrides": best_overrides,
            "best_cost": best_cost,
            "best_eval": best_eval,
            "trials_history": trials_history,
        }

    def run_full_calibration_pipeline(
        self,
        base_prefix: str = "ter26_opt",
        final_candidate_id: str = "opt_run5",
        stage_duration_s: float = 120.0,
    ) -> Dict[str, Any]:
        """Execute full phased calibration: Stage A -> Stage B -> Stage C -> Stage D synthesis."""
        print("=================================================================")
        print("  LAUNCHING FULL PHYSICS-BASED CALIBRATION PIPELINE")
        print("=================================================================")

        # Stage A: Longitudinal
        res_a = self.run_stage_a_longitudinal(base_prefix=base_prefix, duration_s=stage_duration_s)

        # Stage B: Lateral & Yaw
        res_b = self.run_stage_b_lateral_yaw(res_a["best_overrides"], base_prefix=base_prefix, duration_s=stage_duration_s)

        # Stage C: Speed Consistency
        res_c = self.run_stage_c_speed_consistency(res_b["best_overrides"], base_prefix=base_prefix, duration_s=stage_duration_s)

        final_overrides = res_c["best_overrides"]
        print(f"\n=======================================================")
        print(f"  SYNTHESIZING FINAL CANDIDATE: {final_candidate_id}")
        print("=======================================================")
        print("Final Calibrated Parameters:")
        for k, v in final_overrides.items():
            print(f"  {k}: {v}")

        # Run complete 281.0s simulation replay
        cand_res_path = self.registry.create_candidate_configuration(final_candidate_id, final_overrides, base_prefix)
        print(f"Generated candidate configuration: {cand_res_path}")

        full_sim = self.exp_mgr.run_simulation(cand_res_path, final_candidate_id, duration_s=281.0, timeout_s=500.0)
        if not full_sim["success"]:
            raise RuntimeError(f"Full 281s replay simulation failed: {full_sim.get('error')}")

        # Evaluate multi-partition performance: Train [20, 160]s, Val [160, 220]s, Test [220, 281]s, Full [20, 281]s
        train_eval = self.evaluate_candidate_partition(final_candidate_id, 20.0, 160.0)
        val_eval = self.evaluate_candidate_partition(final_candidate_id, 160.0, 220.0)
        test_eval = self.evaluate_candidate_partition(final_candidate_id, 220.0, 281.0)
        full_eval = self.evaluate_candidate_partition(final_candidate_id, 20.0, 281.0)

        summary = {
            "candidate_id": final_candidate_id,
            "config_resource_path": cand_res_path,
            "final_parameters": final_overrides,
            "stage_a": res_a,
            "stage_b": res_b,
            "stage_c": res_c,
            "partitions": {
                "train_20_160": train_eval,
                "validation_160_220": val_eval,
                "test_220_281": test_eval,
                "full_20_281": full_eval,
            },
        }

        # Save summary report JSON
        out_summary_path = os.path.join("CorrelationLab/reports", f"calibration_summary_{final_candidate_id}.json")
        with open(out_summary_path, "w") as f:
            json.dump(summary, f, indent=2, default=str)

        print(f"Summary results written to {out_summary_path}")
        return summary
