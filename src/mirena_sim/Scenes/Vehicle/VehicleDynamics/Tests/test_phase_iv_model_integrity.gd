extends SceneTree
## Unit test suite for Phase IV Model Integrity:
## 1. Steering compliance Option B audit status (unsupported / inactive).
## 2. Steering geometry sign conventions and Ackermann symmetry.
## 3. Differential viscous damping equation, dimensional consistency, and legacy alias.
## 4. Wheel radius parameter provenance separation.

func _init() -> void:
	var result := run_suite()
	if result["failed"] > 0:
		quit(1)
	else:
		quit(0)

static func run_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("========================================================================")
	print("  MirenaSim Vehicle Dynamics: Phase IV Model Integrity Test Suite")
	print("========================================================================")

	# 1. Steering Compliance Option B status
	var steer_cfg := SteeringConfiguration.new()
	if not steer_cfg.is_compliance_steer_supported():
		passed += 1
		print("  [PASS] SteeringConfiguration.is_compliance_steer_supported() returns false (Option B verified)")
	else:
		failed += 1
		print("  [FAIL] is_compliance_steer_supported() returned true")

	# 2. Steering Geometry Left/Right Symmetry
	var params := VehicleParameters.new()
	steer_cfg.ackermann_factor = 0.60
	steer_cfg.static_toe_front_rad = 0.0015
	var angles_left := SteeringGeometry.compute_wheel_steering_angles(0.20, params, steer_cfg)
	var angles_right := SteeringGeometry.compute_wheel_steering_angles(-0.20, params, steer_cfg)

	# Ackermann inside wheel turns more:
	# Left turn (+0.20): inside wheel is FL -> |angles_left[FL]| > |angles_left[FR]|
	# Right turn (-0.20): inside wheel is FR -> |angles_right[FR]| > |angles_right[FL]|
	var fl_mag_left: float = absf(angles_left[VehicleDynamicsTypes.WheelCorner.FL])
	var fr_mag_left: float = absf(angles_left[VehicleDynamicsTypes.WheelCorner.FR])
	var fl_mag_right: float = absf(angles_right[VehicleDynamicsTypes.WheelCorner.FL])
	var fr_mag_right: float = absf(angles_right[VehicleDynamicsTypes.WheelCorner.FR])

	if fl_mag_left > fr_mag_left and fr_mag_right > fl_mag_right and absf(fl_mag_left - fr_mag_right) < 0.005:
		passed += 1
		print("  [PASS] Steering geometry Ackermann asymmetry and left/right mirror symmetry verified")
	else:
		failed += 1
		print("  [FAIL] Steering geometry symmetry check failed: L=[%f, %f], R=[%f, %f]" % [fl_mag_left, fr_mag_left, fl_mag_right, fr_mag_right])

	# 3. Differential Viscous Damping Equation & Legacy Alias
	var pt_cfg := PowertrainConfiguration.new()
	pt_cfg.diff_lock_ratio = 0.65
	pt_cfg.diff_viscous_damping_nm_s_per_rad = 40.0
	
	# Check legacy alias read and write
	if absf(pt_cfg.diff_preload_viscous_nm_s_rad - 40.0) < 0.001:
		passed += 1
		print("  [PASS] Legacy alias diff_preload_viscous_nm_s_rad correctly reads diff_viscous_damping_nm_s_per_rad")
	else:
		failed += 1
		print("  [FAIL] Legacy alias read failed")

	pt_cfg.diff_preload_viscous_nm_s_rad = 50.0
	if absf(pt_cfg.diff_viscous_damping_nm_s_per_rad - 50.0) < 0.001:
		passed += 1
		print("  [PASS] Legacy alias diff_preload_viscous_nm_s_rad setter correctly updates diff_viscous_damping_nm_s_per_rad")
	else:
		failed += 1
		print("  [FAIL] Legacy alias setter failed")

	# Reset to 40.0
	pt_cfg.diff_viscous_damping_nm_s_per_rad = 40.0

	# 4. Differential Torque Distribution with Wheel Speed Differential
	# Equal wheel speeds -> zero differential transfer
	var omegas_equal: Array[float] = [0.0, 0.0, 50.0, 50.0]
	var trq_equal := DrivetrainModel.distribute_wheel_torques_nm(100.0, omegas_equal, pt_cfg)
	var expected_equal: float = 0.5 * (100.0 * 5.0 * 0.92) # 230 Nm
	if absf(trq_equal[2] - expected_equal) < 0.01 and absf(trq_equal[3] - expected_equal) < 0.01:
		passed += 1
		print("  [PASS] Symmetric rear wheel speeds yield zero differential transfer (T_RL=T_RR=%.1f Nm)" % trq_equal[2])
	else:
		failed += 1
		print("  [FAIL] Equal wheel speed torque split failed: %s" % str(trq_equal))

	# RL spinning faster: delta_omega = 52.0 - 48.0 = +4.0 rad/s
	# T_lock = 0.65 * 40.0 * 4.0 = 104.0 Nm
	# T_RL should decrease by 0.5 * 104.0 = 52.0 Nm -> 230 - 52 = 178 Nm
	# T_RR should increase by 0.5 * 104.0 = 52.0 Nm -> 230 + 52 = 282 Nm
	var omegas_diff: Array[float] = [0.0, 0.0, 52.0, 48.0]
	var trq_diff := DrivetrainModel.distribute_wheel_torques_nm(100.0, omegas_diff, pt_cfg)
	if absf(trq_diff[2] - 178.0) < 0.01 and absf(trq_diff[3] - 282.0) < 0.01:
		passed += 1
		print("  [PASS] Viscous differential transfer verified (d_omega=+4 rad/s -> T_RL=178 Nm, T_RR=282 Nm, Delta=104 Nm)")
	else:
		failed += 1
		print("  [FAIL] Viscous differential transfer calculation failed: RL=%f (exp 178), RR=%f (exp 282)" % [trq_diff[2], trq_diff[3]])

	# 5. Wheel Radius Parameter Provenance Separation
	var tire_cfg := TireConfiguration.new()
	var r_primary: float = params.wheel_unloaded_radius_m
	var r_nominal_tire: float = tire_cfg.unloaded_radius_m
	if absf(r_primary - 0.20) < 0.005 and absf(r_nominal_tire - 0.20) < 0.005:
		passed += 1
		print("  [PASS] Default wheel radii initialized correctly (primary=%f, tire=%f)" % [r_primary, r_nominal_tire])
	else:
		failed += 1
		print("  [FAIL] Default radii check failed")

	print("========================================================================")
	print("  PHASE IV TEST SUMMARY: %d PASSED, %d FAILED" % [passed, failed])
	print("========================================================================")

	return {"passed": passed, "failed": failed}
