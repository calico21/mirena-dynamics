extends SceneTree
## Unit test suite for PowertrainModel, MotorModel, DrivetrainModel, BrakeSystem,
## and WheelRotationalDynamics (including passive brake lockup at standstill).


func _init() -> void:
	var result := run_suite()
	if result["failed"] > 0:
		quit(1)
	else:
		quit(0)


static func run_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_powertrain] Starting Powertrain, Brakes & Wheel Rotation Tests ===")

	var pt_cfg := PowertrainConfiguration.new()
	var pt_model := PowertrainModel.new()

	# 1. Low-speed peak torque limit (100 Nm motor * 5.0 ratio * 0.92 eff = 460 Nm total rear axle -> 230 Nm/wheel)
	var wheel_omegas_low: Array[float] = [10.0, 10.0, 10.0, 10.0] # motor omega = 50 rad/s (~477 RPM)
	var torques_low: Array[float] = pt_model.evaluate_drive_torques(1.0, 2.0, wheel_omegas_low, pt_cfg)
	var expected_per_rear_wheel: float = 0.5 * (100.0 * 5.0 * 0.92)
	if torques_low[0] == 0.0 and torques_low[1] == 0.0 and absf(torques_low[2] - expected_per_rear_wheel) < 0.5 and absf(torques_low[3] - expected_per_rear_wheel) < 0.5:
		passed += 1
		print("  [PASS] Low-speed peak motor torque (100 Nm -> 230 Nm/rear wheel) verified")
	else:
		failed += 1
		print("  [FAIL] Low-speed peak motor torque failed: %s" % str(torques_low))

	# 2. Formula Student 80 kW power ceiling at high speed
	var pt_cfg_high := PowertrainConfiguration.new()
	pt_cfg_high.motor_max_rpm = 12000.0
	var wheel_omegas_high: Array[float] = [200.0, 200.0, 200.0, 200.0] # u = 40 m/s, motor omega = 1000 rad/s -> T_motor = 80 Nm
	var torques_high: Array[float] = pt_model.evaluate_drive_torques(1.0, 40.0, wheel_omegas_high, pt_cfg_high)
	var expected_power_lim_wheel: float = 0.5 * (80.0 * 5.0 * 0.92)
	if absf(torques_high[2] - expected_power_lim_wheel) < 0.5:
		passed += 1
		print("  [PASS] 80 kW hyperbolic power limit enforced at high speed (T_RL=%.1f Nm)" % torques_high[2])
	else:
		failed += 1
		print("  [FAIL] 80 kW power ceiling check failed: T_RL=%f vs expected %f" % [torques_high[2], expected_power_lim_wheel])

	# 3. Reverse lockout (negative throttle command produces 0 drive torque)
	var torques_neg: Array[float] = pt_model.evaluate_drive_torques(-1.0, 2.0, wheel_omegas_low, pt_cfg)
	if torques_neg[2] == 0.0 and torques_neg[3] == 0.0:
		passed += 1
		print("  [PASS] Forward-only lockout rejects negative throttle commands")
	else:
		failed += 1
		print("  [FAIL] Reverse lockout check failed")

	# 4. BrakeSystem 60/40 front/rear bias and torque conversion
	var params := VehicleParameters.new()
	var brk_cfg := BrakeConfiguration.new()
	var brk_sys := BrakeSystem.new()
	var brk_state := BrakeState.new()
	var brk_torques: Array[float] = brk_sys.evaluate_brake_torques(1.0, false, brk_state, params, brk_cfg)
	var total_brk_torque: float = brk_torques[0] + brk_torques[1] + brk_torques[2] + brk_torques[3]
	var front_share: float = (brk_torques[0] + brk_torques[1]) / total_brk_torque
	if absf(total_brk_torque - 1177.2 * 0.20) < 1e-2 and absf(front_share - 0.60) < 1e-4:
		passed += 1
		print("  [PASS] BrakeSystem 60/40 front/rear bias and total braking force budget verified")
	else:
		failed += 1
		print("  [FAIL] BrakeSystem bias/budget check failed: total_T=%f front_share=%f" % [total_brk_torque, front_share])

	# 5. WheelRotationalDynamics passive brake lockup without spinning backward
	var ws := WheelState.new(VehicleDynamicsTypes.WheelCorner.FL)
	ws.omega_rad_s = 2.0 # slowly rotating forward
	ws.drive_torque_nm = 0.0
	ws.brake_torque_magnitude_nm = 100.0 # strong passive brake torque
	ws.contact_state.in_contact = false
	WheelRotationalDynamics.step_wheel_rotation(ws, 0.32, 0.05, 0.05)
	if ws.omega_rad_s == 0.0:
		passed += 1
		print("  [PASS] Passive brake torque locks wheel at omega = 0.0 without inducing reverse rotation")
	else:
		failed += 1
		print("  [FAIL] Passive brake lockup check failed: omega=%f" % ws.omega_rad_s)

	print("=== [test_powertrain] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}
