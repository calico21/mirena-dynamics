#!/usr/bin/env python3
"""Synthetic regression test for sensor-to-CoG lever arm acceleration transfer.

Verifies:
1. Zero angular velocity (pure translation): a_CoG == a_IMU
2. Constant angular velocity (pure centripetal): a_CoG == a_IMU - omega x (omega x r)
3. Non-zero angular acceleration (transient angular motion):
   a_CoG == a_IMU - alpha x r - omega x (omega x r)
4. Check that no double compensation occurs.
"""
import numpy as np

def transfer_acceleration(accel_sensor, omega, alpha, r_arm):
    """Rigid-body acceleration transfer from sensor to target frame (CoG).
    
    Formula: a_CoG = a_sensor - alpha x r - omega x (omega x r)
    """
    centripetal = np.cross(omega, np.cross(omega, r_arm))
    tangential = np.cross(alpha, r_arm)
    return accel_sensor - tangential - centripetal

def run_tests():
    print("=================================================================")
    print("  SYNTHETIC REGRESSION TEST: SENSOR LEVER-ARM ACCELERATION TRANSFER")
    print("=================================================================")
    
    # Sensor arm in body frame (CoG to IMU mounting)
    r_arm = np.array([-0.150, 0.075, 1.000])
    
    # -------------------------------------------------------------------------
    # TEST 1: Pure Translation (omega = 0, alpha = 0)
    # -------------------------------------------------------------------------
    a_cog_true = np.array([3.5, -2.1, 0.0])
    omega_1 = np.zeros(3)
    alpha_1 = np.zeros(3)
    
    # Forward kinematic relation: a_sensor = a_CoG + alpha x r + omega x (omega x r)
    a_sensor_1 = a_cog_true + np.cross(alpha_1, r_arm) + np.cross(omega_1, np.cross(omega_1, r_arm))
    a_cog_recon_1 = transfer_acceleration(a_sensor_1, omega_1, alpha_1, r_arm)
    
    err_1 = np.linalg.norm(a_cog_recon_1 - a_cog_true)
    assert err_1 < 1e-12, f"Test 1 failed with error {err_1}"
    print(f"  [PASS] Test 1 (Pure Translation): max error = {err_1:.2e} m/s²")
    
    # -------------------------------------------------------------------------
    # TEST 2: Constant Angular Velocity (Steady-State Cornering, alpha = 0)
    # -------------------------------------------------------------------------
    # Yaw rate = 1.2 rad/s, forward speed 10 m/s -> centripetal accel = u * r = 12.0 m/s^2
    a_cog_true = np.array([0.0, 12.0, 0.0])
    omega_2 = np.array([0.0, 0.0, 1.2]) # gz = 1.2 rad/s
    alpha_2 = np.zeros(3)
    
    # Forward kinematic:
    a_sensor_2 = a_cog_true + np.cross(alpha_2, r_arm) + np.cross(omega_2, np.cross(omega_2, r_arm))
    # omega x (omega x r) = [0,0,w] x [-w*ry, w*rx, 0] = [-w^2*rx, -w^2*ry, 0]
    # For rx=-0.15, ry=0.075, w=1.2:
    # ax_cent = -1.44 * (-0.15) = +0.216
    # ay_cent = -1.44 * (0.075) = -0.108
    # a_sensor = [0.216, 12.0 - 0.108, 0] = [0.216, 11.892, 0]
    a_cog_recon_2 = transfer_acceleration(a_sensor_2, omega_2, alpha_2, r_arm)
    err_2 = np.linalg.norm(a_cog_recon_2 - a_cog_true)
    assert err_2 < 1e-12, f"Test 2 failed with error {err_2}"
    print(f"  [PASS] Test 2 (Constant Yaw Rotation): max error = {err_2:.2e} m/s²")
    print(f"         Centripetal arm correction: Delta_ax = {a_sensor_2[0] - a_cog_true[0]:+.4f} m/s², Delta_ay = {a_sensor_2[1] - a_cog_true[1]:+.4f} m/s²")
    
    # -------------------------------------------------------------------------
    # TEST 3: Full 3D Transient Motion (Non-Zero Angular Acceleration)
    # -------------------------------------------------------------------------
    # Roll transient: alpha_x = 5.0 rad/s^2, yaw angular accel: alpha_z = 3.0 rad/s^2
    a_cog_true = np.array([2.0, 8.0, -1.0])
    omega_3 = np.array([0.2, -0.1, 0.8])
    alpha_3 = np.array([5.0, -1.5, 3.0])
    
    # Forward kinematic:
    a_sensor_3 = a_cog_true + np.cross(alpha_3, r_arm) + np.cross(omega_3, np.cross(omega_3, r_arm))
    a_cog_recon_3 = transfer_acceleration(a_sensor_3, omega_3, alpha_3, r_arm)
    err_3 = np.linalg.norm(a_cog_recon_3 - a_cog_true)
    assert err_3 < 1e-12, f"Test 3 failed with error {err_3}"
    print(f"  [PASS] Test 3 (Full 3D Angular Accel): max error = {err_3:.2e} m/s²")
    print(f"         Tangential arm correction: Delta_ax = {np.cross(alpha_3, r_arm)[0]:+.4f} m/s², Delta_ay = {np.cross(alpha_3, r_arm)[1]:+.4f} m/s²")
    
    # -------------------------------------------------------------------------
    # TEST 4: Double-Compensation Guard Check
    # -------------------------------------------------------------------------
    # If compensation is applied twice, error should equal the compensation term:
    a_double_comp = transfer_acceleration(a_cog_recon_3, omega_3, alpha_3, r_arm)
    diff_double = np.linalg.norm(a_double_comp - a_cog_true)
    expected_diff = np.linalg.norm(np.cross(alpha_3, r_arm) + np.cross(omega_3, np.cross(omega_3, r_arm)))
    assert abs(diff_double - expected_diff) < 1e-12, "Double compensation guard check failed"
    print(f"  [PASS] Test 4 (Double-Compensation Guard): detectable offset = {diff_double:.4f} m/s²")
    
    print("=================================================================")
    print("  ALL 4 SYNTHETIC REGRESSION TESTS PASSED (100%)")
    print("=================================================================")

if __name__ == "__main__":
    run_tests()
