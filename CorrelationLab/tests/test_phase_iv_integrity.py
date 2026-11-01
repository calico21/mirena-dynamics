"""
Phase IV Model Integrity & Audit Closure Automated Test Suite.
Author: MirenaSim Vehicle Dynamics Team
Date: November 2026
"""

import unittest
import numpy as np
import pandas as pd
import os
import sys

from CorrelationLab.analysis.global_correlation import GlobalCorrelationEvaluator
from CorrelationLab.analysis.longitudinal_analysis import LongitudinalAnalyzer
from CorrelationLab.core.data_loader import DataLoader
from CorrelationLab.core.signal_registry import SignalRegistry
from CorrelationLab.core.synchronization import TimeSynchronizer
from CorrelationLab.core.resampling import Resampler
from CorrelationLab.core.metrics import MetricCatalogue


class TestPhaseIVModelIntegrity(unittest.TestCase):
    """Test suite verifying Phase IV integrity fixes and reconciliation."""

    def setUp(self):
        self.loader = DataLoader("CorrelationLab/data/real", "CorrelationLab/data/simulated", SignalRegistry())
        self.loader.load_real_data()

    def test_01_speed_reconstruction_axle_averaging(self):
        """Verify that axle-averaged wheel speed eliminates differential cornering bias."""
        # Simulated right turn: outside (left) wheel turns faster, inside (right) wheel turns slower
        omega_rl = np.array([55.0, 56.0, 54.0]) # rad/s
        omega_rr = np.array([45.0, 44.0, 46.0]) # rad/s
        
        # Method A: Axle average
        u_avg = LongitudinalAnalyzer.reconstruct_speed_from_wheels(omega_rl * (60.0 / (2.0 * np.pi)), 
                                                                  omega_rr * (60.0 / (2.0 * np.pi)))
        # Expected: 0.5 * (55 + 45) * 0.2032 * 0.928 = 50.0 * 0.1885696 = 9.42848 m/s
        expected_u = 50.0 * 0.2032 * 0.928
        self.assertAlmostEqual(u_avg[0], expected_u, places=4)
        
        # Method B: Single RL wheel only
        u_rl_only = omega_rl[0] * 0.2032 * 0.928
        # Discrepancy is 10% on corner entry
        self.assertGreater(abs(u_rl_only - expected_u), 0.9)

    def test_02_reconciled_opt_run5_metrics(self):
        """Verify that GlobalCorrelationEvaluator produces exact Phase II numbers for opt_run5."""
        evaluator = GlobalCorrelationEvaluator()
        res = evaluator.evaluate_run("opt_run5")
        full = res["windows"]["full_drive_20_281"]
        
        # chassis.u 5Hz: RMSE = 1.8579, gain = 1.0081, r = 0.9482
        u_5hz = full["channels_5hz"]["chassis.u"]
        self.assertAlmostEqual(u_5hz["rmse"], 1.8579, places=3)
        self.assertAlmostEqual(u_5hz["gain"], 1.0081, places=3)
        self.assertAlmostEqual(u_5hz["pearson_r"], 0.9482, places=3)
        
        # imu.ax 5Hz: RMSE = 0.9962, gain = 0.7215, r = 0.8471
        ax_5hz = full["channels_5hz"]["imu.ax"]
        self.assertAlmostEqual(ax_5hz["rmse"], 0.9962, places=3)
        self.assertAlmostEqual(ax_5hz["gain"], 0.7215, places=3)
        self.assertAlmostEqual(ax_5hz["pearson_r"], 0.8471, places=3)

        # imu.ay 5Hz: RMSE = 2.8858, gain = 1.0966, r = 0.8477
        ay_5hz = full["channels_5hz"]["imu.ay"]
        self.assertAlmostEqual(ay_5hz["rmse"], 2.8858, places=3)
        self.assertAlmostEqual(ay_5hz["gain"], 1.0966, places=3)
        self.assertAlmostEqual(ay_5hz["pearson_r"], 0.8477, places=3)

        # imu.gz 5Hz: RMSE = 0.1687, gain = 1.0918, r = 0.9630
        gz_5hz = full["channels_5hz"]["imu.gz"]
        self.assertAlmostEqual(gz_5hz["rmse"], 0.1687, places=3)
        self.assertAlmostEqual(gz_5hz["gain"], 1.0918, places=3)
        self.assertAlmostEqual(gz_5hz["pearson_r"], 0.9630, places=3)

    def test_03_brake_event_classification_file_exists(self):
        """Verify that brake event classification deliverable exists and has correct columns."""
        csv_path = "CorrelationLab/reports/phase_iv_model_integrity/brake_event_classification.csv"
        self.assertTrue(os.path.exists(csv_path))
        df = pd.read_csv(csv_path)
        self.assertGreater(len(df), 20)
        self.assertIn("event_classification", df.columns)
        self.assertIn("min_ax_mps2", df.columns)
        self.assertIn("delta_speed_mps", df.columns)

    def test_04_parameter_dependency_matrix_completeness(self):
        """Verify that parameter dependency matrix covers primary vehicle subsystems."""
        csv_path = "CorrelationLab/reports/phase_iv_model_integrity/parameter_dependency_matrix.csv"
        self.assertTrue(os.path.exists(csv_path))
        df = pd.read_csv(csv_path)
        params = df["parameter_name"].values
        self.assertIn("wheel_unloaded_radius_m", params)
        self.assertIn("unloaded_radius_m", params)
        self.assertIn("diff_viscous_damping_nm_s_per_rad", params)
        self.assertIn("compliance_steer_front_radpn", params)


if __name__ == "__main__":
    unittest.main()
