extends SceneTree
## Unit test suite for VehicleCoordinates, VehicleUnits, and WheelKinematics.


func _init() -> void:
	var result := run_suite()
	if result["failed"] > 0:
		quit(1)
	else:
		quit(0)


static func run_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_wheel_kinematics] Starting Coordinate & Wheel Kinematics Tests ===")

	# 1. Godot <-> Vehicle (ISO 8855 / ROS) vector round-trip and axis mapping
	var godot_vec := Vector3(-2.0, 5.0, -10.0)
	var veh_vec := VehicleCoordinates.godot_to_vehicle_vec(godot_vec)
	var back_to_godot := VehicleCoordinates.vehicle_to_godot_vec(veh_vec)
	if veh_vec.is_equal_approx(Vector3(10.0, 2.0, 5.0)) and back_to_godot.is_equal_approx(godot_vec):
		passed += 1
		print("  [PASS] VehicleCoordinates Godot <-> ISO 8855/ROS linear vector mapping verified")
	else:
		failed += 1
		print("  [FAIL] VehicleCoordinates vector mapping failed: got %s" % str(veh_vec))

	# 2. Angular velocity / torque mapping (Right-handed parity preservation)
	var godot_omega := Vector3(-0.2, 0.5, -0.1) # pitch=+0.2, yaw=+0.5, roll=+0.1 in Vehicle
	var veh_omega := VehicleCoordinates.godot_to_vehicle_angular(godot_omega)
	var back_omega := VehicleCoordinates.vehicle_to_godot_angular(veh_omega)
	if veh_omega.is_equal_approx(Vector3(0.1, 0.2, 0.5)) and back_omega.is_equal_approx(godot_omega):
		passed += 1
		print("  [PASS] VehicleCoordinates angular velocity/moment mapping verified")
	else:
		failed += 1
		print("  [FAIL] VehicleCoordinates angular mapping failed: got %s" % str(veh_omega))

	# 3. Body <-> Steered Wheel frame round-trip
	var v_body := Vector3(12.0, 1.5, 0.0)
	var delta_rad := 0.25
	var v_wheel := VehicleCoordinates.body_to_wheel_frame(v_body, delta_rad)
	var v_body_recovered := VehicleCoordinates.wheel_to_body_frame(v_wheel, delta_rad)
	if v_body_recovered.is_equal_approx(v_body):
		passed += 1
		print("  [PASS] Body <-> Steered Wheel frame rotation is orthogonal and invertible")
	else:
		failed += 1
		print("  [FAIL] Body <-> Steered Wheel rotation failed")

	# 4. Wheel hub velocity under chassis yaw rate
	# For yaw rate r > 0 (CCW, turning left), FL wheel (y = +0.587 m) moves slower longitudinally:
	# u_hub = u - r * y_i
	var ws_fl := WheelState.new(VehicleDynamicsTypes.WheelCorner.FL)
	ws_fl.hub_offset_from_com_vehicle = Vector3(0.876, 0.587, 0.0)
	ws_fl.omega_rad_s = 50.0
	var ws_fr := WheelState.new(VehicleDynamicsTypes.WheelCorner.FR)
	ws_fr.hub_offset_from_com_vehicle = Vector3(0.876, -0.587, 0.0)
	ws_fr.omega_rad_s = 50.0
	WheelKinematics.update_wheel_kinematics(ws_fl, Vector3(10.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5), 0.25)
	WheelKinematics.update_wheel_kinematics(ws_fr, Vector3(10.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5), 0.25)
	var expected_u_fl: float = 10.0 - 0.5 * 0.587
	var expected_u_fr: float = 10.0 - 0.5 * (-0.587)
	if absf(ws_fl.hub_velocity_vehicle.x - expected_u_fl) < 1e-5 and absf(ws_fr.hub_velocity_vehicle.x - expected_u_fr) < 1e-5:
		passed += 1
		print("  [PASS] Yaw rate r > 0 decreases inside (FL) hub longitudinal speed and increases outside (FR)")
	else:
		failed += 1
		print("  [FAIL] Yaw rate hub velocity transfer failed")

	# 5. Slip ratio sign: driven wheel (omega * r_eff > v_wx) => kappa > 0; braked => kappa < 0
	var ws_test := WheelState.new(VehicleDynamicsTypes.WheelCorner.RL)
	ws_test.hub_offset_from_com_vehicle = Vector3(-0.717, 0.587, 0.0)
	ws_test.effective_radius_m = 0.20
	ws_test.omega_rad_s = 55.0 # r*omega = 11.0 m/s vs u = 10.0 m/s
	WheelKinematics.update_wheel_kinematics(ws_test, Vector3(10.0, 0.0, 0.0), Vector3.ZERO, 0.25)
	var kappa_drive: float = ws_test.slip_ratio_kappa
	ws_test.omega_rad_s = 45.0 # r*omega = 9.0 m/s vs u = 10.0 m/s
	WheelKinematics.update_wheel_kinematics(ws_test, Vector3(10.0, 0.0, 0.0), Vector3.ZERO, 0.25)
	var kappa_brake: float = ws_test.slip_ratio_kappa
	if absf(kappa_drive - 0.10) < 1e-3 and absf(kappa_brake - (-0.10)) < 1e-3:
		passed += 1
		print("  [PASS] Slip ratio kappa = +0.10 under drive and -0.10 under braking")
	else:
		failed += 1
		print("  [FAIL] Slip ratio calculation failed: drive=%f brake=%f" % [kappa_drive, kappa_brake])

	# 6. Standstill singularity protection (u = 0, omega = 0 => finite zero slip)
	ws_test.omega_rad_s = 0.0
	WheelKinematics.update_wheel_kinematics(ws_test, Vector3.ZERO, Vector3.ZERO, 0.25)
	if is_finite(ws_test.slip_ratio_kappa) and is_finite(ws_test.slip_angle_alpha_rad) and ws_test.slip_ratio_kappa == 0.0 and ws_test.slip_angle_alpha_rad == 0.0:
		passed += 1
		print("  [PASS] Standstill (u = 0, omega = 0) produces finite zero slip without division-by-zero")
	else:
		failed += 1
		print("  [FAIL] Standstill singularity check failed")

	# 7. RPM conversions and quantization
	var rpm: float = VehicleUnits.rad_s_to_rpm(PI) # 30 RPM
	var quant_rpm: float = VehicleUnits.quantize_rpm(30.4, 1.0)
	if absf(rpm - 30.0) < 1e-4 and absf(quant_rpm - 30.0) < 1e-5:
		passed += 1
		print("  [PASS] VehicleUnits rad/s <-> RPM conversion and encoder quantization verified")
	else:
		failed += 1
		print("  [FAIL] VehicleUnits conversion check failed")

	print("=== [test_wheel_kinematics] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}
