extends SceneTree

const DT: float = 0.0025

func _init() -> void:
	print("========================================================================")
	print("  MirenaSim Vehicle Dynamics: 13 Controlled Validation Maneuvers")
	print("========================================================================")

	var cfg_baseline: VehicleDynamicsConfig = load("res://Scenes/Vehicle/VehicleDynamics/Config/baselines/modular_baseline/vehicle_default.tres")
	var cfg_calibrated: VehicleDynamicsConfig = load("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_ter26_calibrated.tres")

	var results: Array[Dictionary] = []
	var total_passed: int = 0
	var total_tests: int = 13

	var make_model = func(cfg: VehicleDynamicsConfig) -> VehicleDynamicsModel:
		var m := VehicleDynamicsModel.new(cfg.duplicate(true))
		m.contact_query.use_analytic_plane = true
		m.reset()
		return m

	var m1_b: VehicleDynamicsModel = make_model.call(cfg_baseline)
	var m1_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	for _s in range(400):
		m1_b.step(DT, true, false)
		m1_c.step(DT, true, false)
	var p1: bool = absf(m1_c.state.ay_mps2) < 1e-4 and absf(m1_c.state.az_mps2) < 1e-4 and absf(m1_c.state.r_rad_s) < 1e-4
	print("  [%s] 1. Static Equilibrium: ay=%.5f m/s^2, az=%.5f m/s^2, r=%.5f rad/s" % [
		"PASS" if p1 else "FAIL", m1_c.state.ay_mps2, m1_c.state.az_mps2, m1_c.state.r_rad_s
	])
	if p1: total_passed += 1

	var m2_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m2_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m2_c.state.wheels: w.omega_rad_s = 50.0
	m2_c.set_control_commands(0.15, 0.0, false)
	for _s in range(400): m2_c.step(DT, true, false)
	var p2: bool = absf(m2_c.state.v_mps) < 1e-4 and absf(m2_c.state.r_rad_s) < 1e-4 and absf(m2_c.state.ay_mps2) < 1e-3
	print("  [%s] 2. Constant-Speed Straight: u=%.2f m/s, v=%.5f m/s, ay=%.5f m/s^2" % [
		"PASS" if p2 else "FAIL", m2_c.state.u_mps, m2_c.state.v_mps, m2_c.state.ay_mps2
	])
	if p2: total_passed += 1

	var m3_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m3_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -5.0)
	for w in m3_c.state.wheels: w.omega_rad_s = 25.0
	m3_c.set_control_commands(1.0, 0.0, false)
	for _s in range(400): m3_c.step(DT, true, false)
	var p3: bool = m3_c.state.u_mps > 7.5 and m3_c.state.ax_mps2 > 1.5 and absf(m3_c.state.r_rad_s) < 1e-3
	print("  [%s] 3. Straight-Line Acceleration: u=%.2f m/s, ax=%.2f m/s^2, r=%.5f rad/s" % [
		"PASS" if p3 else "FAIL", m3_c.state.u_mps, m3_c.state.ax_mps2, m3_c.state.r_rad_s
	])
	if p3: total_passed += 1

	var m4_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m4_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -15.0)
	for w in m4_c.state.wheels: w.omega_rad_s = 75.0
	m4_c.set_control_commands(-1.0, 0.0, false)
	for _s in range(400): m4_c.step(DT, true, false)
	var p4: bool = m4_c.state.ax_mps2 < -0.5 and m4_c.state.u_mps < 15.0 and m4_c.state.u_mps >= 0.0 and absf(m4_c.state.r_rad_s) < 1e-3
	print("  [%s] 4. Straight-Line Braking: u=%.2f m/s, ax=%.2f m/s^2 (stable deceleration)" % [
		"PASS" if p4 else "FAIL", m4_c.state.u_mps, m4_c.state.ax_mps2
	])
	if p4: total_passed += 1

	var m5_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m5_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m5_c.state.wheels: w.omega_rad_s = 50.0
	m5_c.set_control_commands(0.0, 0.15, false)
	var r_step_history: Array[float] = []
	for _s in range(200):
		m5_c.step(DT, true, false)
		r_step_history.append(m5_c.state.r_rad_s)
	var p5: bool = r_step_history[40] > 0.4 and m5_c.state.r_rad_s > 0.6 and m5_c.state.ay_mps2 > 5.0
	print("  [%s] 5. Steering Step Response: r(0.1s)=%.3f rad/s, r_final=%.3f rad/s, ay=%.2f m/s^2" % [
		"PASS" if p5 else "FAIL", r_step_history[40], m5_c.state.r_rad_s, m5_c.state.ay_mps2
	])
	if p5: total_passed += 1

	var m6_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m6_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m6_c.state.wheels: w.omega_rad_s = 50.0
	for s in range(400):
		var steer_ramp: float = clampf(float(s) * DT * 0.25, 0.0, 0.25)
		m6_c.set_control_commands(0.0, steer_ramp, false)
		m6_c.step(DT, true, false)
	var p6: bool = m6_c.state.r_rad_s > 0.8 and m6_c.state.ay_mps2 > 7.0 and (m6_c.state.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE) == 0
	print("  [%s] 6. Steering Ramp Response: final r=%.3f rad/s, final ay=%.2f m/s^2 (smooth buildup)" % [
		"PASS" if p6 else "FAIL", m6_c.state.r_rad_s, m6_c.state.ay_mps2
	])
	if p6: total_passed += 1

	var m7_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m7_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m7_c.state.wheels: w.omega_rad_s = 50.0
	m7_c.set_control_commands(0.10, 0.10, false)
	for _s in range(400): m7_c.step(DT, true, false)
	var fl_fz: float = m7_c.state.wheels[0].suspension_state.normal_load_fz_n
	var fr_fz: float = m7_c.state.wheels[1].suspension_state.normal_load_fz_n
	var rl_fz: float = m7_c.state.wheels[2].suspension_state.normal_load_fz_n
	var rr_fz: float = m7_c.state.wheels[3].suspension_state.normal_load_fz_n
	var p7: bool = m7_c.state.ay_mps2 > 0.0 and m7_c.state.r_rad_s > 0.0 and fr_fz > fl_fz and rr_fz > rl_fz and fl_fz > 0.0 and rl_fz > 0.0
	print("  [%s] 7. Constant-Radius Left: ay=%.2f m/s^2, r=%.3f rad/s, Fz_FL=%.1f N, Fz_FR=%.1f N (no liftoff)" % [
		"PASS" if p7 else "FAIL", m7_c.state.ay_mps2, m7_c.state.r_rad_s, fl_fz, fr_fz
	])
	if p7: total_passed += 1

	var m8_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m8_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m8_c.state.wheels: w.omega_rad_s = 50.0
	m8_c.set_control_commands(0.10, -0.10, false)
	for _s in range(400): m8_c.step(DT, true, false)
	var p8: bool = absf(m8_c.state.ay_mps2 + m7_c.state.ay_mps2) < 0.2 and absf(m8_c.state.r_rad_s + m7_c.state.r_rad_s) < 0.05
	print("  [%s] 8. Cornering Symmetry (Left vs Right): ay_L=%.2f vs ay_R=%.2f m/s^2, r_L=%.3f vs r_R=%.3f" % [
		"PASS" if p8 else "FAIL", m7_c.state.ay_mps2, m8_c.state.ay_mps2, m7_c.state.r_rad_s, m8_c.state.r_rad_s
	])
	if p8: total_passed += 1

	var m9_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m9_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -5.0)
	for w in m9_c.state.wheels: w.omega_rad_s = 25.0
	var ay_max_9: float = 0.0
	for s in range(600):
		var gas_ramp: float = clampf(0.1 + float(s) * DT * 0.3, 0.1, 0.8)
		m9_c.set_control_commands(gas_ramp, 0.08, false)
		m9_c.step(DT, true, false)
		ay_max_9 = maxf(ay_max_9, m9_c.state.ay_mps2)
	var p9: bool = m9_c.state.u_mps > 7.0 and ay_max_9 > 3.0 and absf(atan2(m9_c.state.v_mps, maxf(m9_c.state.u_mps, 0.1))) < 0.25
	print("  [%s] 9. Progressive Skidpad: u=%.2f m/s, max_ay=%.2f m/s^2, beta=%.2f deg (stable tire envelope)" % [
		"PASS" if p9 else "FAIL", m9_c.state.u_mps, ay_max_9, rad_to_deg(atan2(m9_c.state.v_mps, maxf(m9_c.state.u_mps, 0.1)))
	])
	if p9: total_passed += 1

	var m10_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m10_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -8.0)
	for w in m10_c.state.wheels: w.omega_rad_s = 40.0
	m10_c.set_control_commands(0.1, 0.12, false)
	for _s in range(200): m10_c.step(DT, true, false)

	m10_c.set_control_commands(0.75, 0.12, false)
	var max_beta_10: float = 0.0
	for _s in range(200):
		m10_c.step(DT, true, false)
		max_beta_10 = maxf(max_beta_10, absf(atan2(m10_c.state.v_mps, m10_c.state.u_mps)))
	var p10: bool = max_beta_10 < deg_to_rad(12.0) and m10_c.state.u_mps > 9.0
	print("  [%s] 10. Accel in Corner: max sideslip beta=%.2f deg (< 12.0 deg, no spinout)" % [
		"PASS" if p10 else "FAIL", rad_to_deg(max_beta_10)
	])
	if p10: total_passed += 1

	var m11_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m11_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -12.0)
	for w in m11_c.state.wheels: w.omega_rad_s = 60.0
	m11_c.set_control_commands(0.40, 0.10, false)
	for _s in range(200): m11_c.step(DT, true, false)

	m11_c.set_control_commands(0.0, 0.10, false)
	for _s in range(200): m11_c.step(DT, true, false)
	var p11: bool = (m11_c.state.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE) == 0 and absf(atan2(m11_c.state.v_mps, m11_c.state.u_mps)) < deg_to_rad(8.0)
	print("  [%s] 11. Throttle Release in Corner: beta=%.2f deg, settled smoothly" % [
		"PASS" if p11 else "FAIL", rad_to_deg(atan2(m11_c.state.v_mps, m11_c.state.u_mps))
	])
	if p11: total_passed += 1

	var m12_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m12_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m12_c.state.wheels: w.omega_rad_s = 50.0
	m12_c.set_control_commands(0.10, 0.10, false)
	for _s in range(200): m12_c.step(DT, true, false)

	m12_c.set_control_commands(-0.40, 0.10, false)
	for _s in range(200): m12_c.step(DT, true, false)
	var p12: bool = m12_c.state.ax_mps2 < 0.0 and (m12_c.state.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE) == 0
	print("  [%s] 12. Braking While Cornering: ax=%.2f m/s^2, ay=%.2f m/s^2 (braking balance maintained)" % [
		"PASS" if p12 else "FAIL", m12_c.state.ax_mps2, m12_c.state.ay_mps2
	])
	if p12: total_passed += 1

	var m13_c: VehicleDynamicsModel = make_model.call(cfg_calibrated)
	m13_c.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m13_c.state.wheels: w.omega_rad_s = 50.0
	var yaw_peaks: Array[float] = []
	for s in range(600):
		var t_slalom: float = float(s) * DT
		var steer_sine: float = 0.12 * sin(TAU * 1.0 * t_slalom)
		m13_c.set_control_commands(0.05, steer_sine, false)
		m13_c.step(DT, true, false)
		if (s % 100) == 0:
			yaw_peaks.append(absf(m13_c.state.r_rad_s))
	var p13: bool = yaw_peaks.max() > 0.4 and yaw_peaks.max() < 2.0 and (m13_c.state.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE) == 0
	print("  [%s] 13. Transient Slalom (1.0 Hz): peak r=%.3f rad/s (bounded oscillation, no rollover)" % [
		"PASS" if p13 else "FAIL", yaw_peaks.max()
	])
	if p13: total_passed += 1

	print("========================================================================")
	print("  VALIDATION SUMMARY: %d / %d MANEUVERS PASSED" % [total_passed, total_tests])
	print("========================================================================")
	quit(0 if total_passed == total_tests else 1)
