extends SceneTree

const TestTireModel := preload("res://Scenes/Vehicle/VehicleDynamics/Tests/test_tire_model.gd")
const TestWheelKinematics := preload("res://Scenes/Vehicle/VehicleDynamics/Tests/test_wheel_kinematics.gd")
const TestSuspensionModel := preload("res://Scenes/Vehicle/VehicleDynamics/Tests/test_suspension_model.gd")
const TestPowertrain := preload("res://Scenes/Vehicle/VehicleDynamics/Tests/test_powertrain.gd")
const FloatingGraphManager := preload("res://Scenes/Sim/UI/FloatingGraphManager.gd")
const FloatingGraphPanel := preload("res://Scenes/Sim/UI/FloatingGraphPanel.gd")

var _suite_executed: bool = false

func _process(_delta: float) -> bool:
	if _suite_executed:
		return true
	_suite_executed = true
	var total_passed: int = 0
	var total_failed: int = 0

	print("========================================================================")
	print("  MirenaSim VehicleDynamics Phase 1 — Automated Verification Suite")
	print("========================================================================\n")

	var suites := [
		TestTireModel.run_suite(),
		TestWheelKinematics.run_suite(),
		TestSuspensionModel.run_suite(),
		TestPowertrain.run_suite(),
		run_integration_suite(),
		run_physics_behavior_and_steering_suite(),
		run_telemetry_and_correlation_suite(),
		run_scene_and_dashboard_suite(root),
	]

	for res in suites:
		total_passed += int(res["passed"])
		total_failed += int(res["failed"])

	print("========================================================================")
	print("  TOTAL SUMMARY: %d PASSED, %d FAILED" % [total_passed, total_failed])
	print("========================================================================")

	if total_failed > 0:
		quit(1)
	else:
		quit(0)
	return true

static func run_integration_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_vehicle_dynamics] Starting Full-Vehicle Integration & Resource Tests ===")

	var default_cfg_res: Resource = load("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_default.tres")
	if default_cfg_res != null and default_cfg_res is VehicleDynamicsConfig:
		var cfg_valid: bool = default_cfg_res.is_valid()
		if cfg_valid:
			passed += 1
			print("  [PASS] Loaded and validated Config/vehicle_default.tres with zero errors")
		else:
			failed += 1
			print("  [FAIL] Config/vehicle_default.tres failed is_valid()")
	else:
		failed += 1
		print("  [FAIL] Could not load Config/vehicle_default.tres")

	var model := VehicleDynamicsModel.new(default_cfg_res)
	var static_loads: Array[float] = VehicleInertia.compute_static_corner_loads_n(model.config.vehicle_parameters)
	var static_sum: float = static_loads[0] + static_loads[1] + static_loads[2] + static_loads[3]
	var expected_weight_n: float = 300.0 * 9.81
	if absf(static_sum - expected_weight_n) < 1e-2 and static_loads[2] > static_loads[0]:
		passed += 1
		print("  [PASS] Static axle weight distribution sums to m*g (%.1f N) with rear bias (FL=%.1f N, RL=%.1f N)" % [
			static_sum, static_loads[0], static_loads[2]
		])
	else:
		failed += 1
		print("  [FAIL] Static wheel load distribution check failed: sum=%f" % static_sum)

	model.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in model.state.wheels:
		w.omega_rad_s = 50.0

	model.set_control_commands(0.8, 0.0, false)
	for _step in range(25):
		model.step(0.0025, true, true)

	if (
		model.state.total_force_vehicle.x > 0.0 and
		(model.state.aero_downforce_front_n + model.state.aero_downforce_rear_n) > 0.0 and
		model.state.aero_drag_force_n > 0.0 and
		model.state.wheels[VehicleDynamicsTypes.WheelCorner.RL].omega_rad_s > 50.0 and
		model.state.u_mps > 10.0 and
		(model.state.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE) == 0
	):
		passed += 1
		print("  [PASS] Straight-line traction step generates net +Fx (%.1f N), downforce (%.1f N), and speed increase (u=%.2f m/s)" % [
			model.state.total_force_vehicle.x,
			model.state.aero_downforce_front_n + model.state.aero_downforce_rear_n,
			model.state.u_mps,
		])
	else:
		failed += 1
		print("  [FAIL] Straight-line traction step check failed: Fx=%f u=%f" % [
			model.state.total_force_vehicle.x, model.state.u_mps
		])

	model.reset()
	model.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -12.0)
	for w in model.state.wheels:
		w.omega_rad_s = 60.0
	model.set_control_commands(0.0, 0.20, false)
	model.step(0.0025, false, true)
	if model.state.wheels[0].steering_angle_rad > 0.0 and model.state.total_moment_vehicle.z > 0.0:
		passed += 1
		print("  [PASS] Positive (left) steering command produces positive (CCW) yaw moment Mz (+%.1f Nm)" % model.state.total_moment_vehicle.z)
	else:
		failed += 1
		print("  [FAIL] Left steering yaw moment sign check failed: Mz=%f" % model.state.total_moment_vehicle.z)

	var ros_adapter := ROSVehicleAdapter.new()
	var enc_res: Dictionary = {}
	for _tick in range(4):
		enc_res = ros_adapter.step_wheel_speed_encoder(600.0, 600.0, 400.0)
	var car_dict: Dictionary = ros_adapter.build_car_state_payload(model.state)
	if enc_res.get("should_publish", false) and enc_res["rl_rpm"] > 0.0 and car_dict.has_all(["x", "y", "psi", "u", "v", "omega"]):
		passed += 1
		print("  [PASS] ROSVehicleAdapter produces conditioned WheelSpeeds RPM and Car state payload")
	else:
		failed += 1
		print("  [FAIL] ROSVehicleAdapter dictionary schema check failed: %s" % str(enc_res))

	var logger := DynamicsLogger.new()
	logger.logging_enabled = true
	logger.record_sample(model.diagnostics)
	var csv_cols: PackedStringArray = DynamicsLogger.get_csv_header().split(",")
	if csv_cols.size() == 30 and logger.get_sample_count() == 1:
		passed += 1
		print("  [PASS] DynamicsLogger 30-column CSV header and sample buffer verified")
	else:
		failed += 1
		print("  [FAIL] DynamicsLogger schema check failed: cols=%d count=%d" % [csv_cols.size(), logger.get_sample_count()])

	model.reset()
	if (
		model.state.sim_time_s == 0.0 and
		model.state.wheels[VehicleDynamicsTypes.WheelCorner.RL].omega_rad_s == 0.0 and
		model.state.u_mps == 0.0 and
		model.state.fault_flags == VehicleDynamicsTypes.FaultFlag.NONE
	):
		passed += 1
		print("  [PASS] VehicleDynamicsModel.reset() zeroes all dynamic states, wheel speeds, and fault flags")
	else:
		failed += 1
		print("  [FAIL] VehicleDynamicsModel.reset() check failed")

	var controller := VehicleDynamicsController.new()
	var dummy_body := RigidBody3D.new()
	var applied_in_passthrough: bool = GodotPhysicsAdapter.apply_forces_to_body(
		dummy_body, model.state, VehicleDynamicsTypes.IntegrationMode.INACTIVE_LEGACY_PASSTHROUGH
	)
	var applied_in_custom: bool = GodotPhysicsAdapter.apply_forces_to_body(
		dummy_body, model.state, VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D
	)
	dummy_body.free()
	controller.free()
	if not applied_in_passthrough and applied_in_custom:
		passed += 1
		print("  [PASS] GodotPhysicsAdapter blocks force application in INACTIVE_LEGACY_PASSTHROUGH and allows in CUSTOM_RIGID_BODY_3D")
	else:
		failed += 1
		print("  [FAIL] GodotPhysicsAdapter force-ownership guard check failed")

	print("=== [test_vehicle_dynamics] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}

static func run_physics_behavior_and_steering_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_physics_behavior_and_steering] Starting Static ay, Rear Stability & Steering Response Tests ===")
	var default_cfg_res: VehicleDynamicsConfig = load("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_default.tres")

	var m_stat := VehicleDynamicsModel.new(default_cfg_res.duplicate(true))
	m_stat.contact_query.use_analytic_plane = true
	m_stat.set_control_commands(0.0, 0.0, false)
	for _s in range(400):
		m_stat.step(0.0025, true, false)
	var fl_s: WheelState = m_stat.state.wheels[0]
	var fr_s: WheelState = m_stat.state.wheels[1]
	var rl_s: WheelState = m_stat.state.wheels[2]
	var rr_s: WheelState = m_stat.state.wheels[3]
	if (
		absf(m_stat.state.ay_mps2) < 1e-3 and
		absf(m_stat.state.total_force_vehicle.y) < 0.2 and
		absf(m_stat.state.total_moment_vehicle.z) < 0.2 and
		fl_s.tire_forces.fy_n < -50.0 and
		fr_s.tire_forces.fy_n > 50.0 and
		absf(fl_s.tire_forces.fy_n + fr_s.tire_forces.fy_n) < 0.1 and
		absf(rl_s.tire_forces.fy_n + rr_s.tire_forces.fy_n) < 0.1
	):
		passed += 1
		print("  [PASS] Test A (Static Equilibrium): ay=%.5f m/s^2, Fy_net=%.3f N, Mz_net=%.3f Nm (camber thrust FL=%.1f N, FR=+%.1f N cancels)" % [
			m_stat.state.ay_mps2, m_stat.state.total_force_vehicle.y, m_stat.state.total_moment_vehicle.z,
			fl_s.tire_forces.fy_n, fr_s.tire_forces.fy_n
		])
	else:
		failed += 1
		print("  [FAIL] Test A (Static Equilibrium) failed: ay=%f Fy_net=%f FL=%f FR=%f" % [
			m_stat.state.ay_mps2, m_stat.state.total_force_vehicle.y, fl_s.tire_forces.fy_n, fr_s.tire_forces.fy_n
		])

	var m_str := VehicleDynamicsModel.new(default_cfg_res.duplicate(true))
	m_str.contact_query.use_analytic_plane = true
	m_str.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m_str.state.wheels:
		w.omega_rad_s = 10.0 / 0.20
	m_str.set_control_commands(0.3, 0.0, false)
	for _s in range(200):
		m_str.step(0.0025, true, false)
	if absf(m_str.state.ay_mps2) < 1e-3 and absf(m_str.state.v_mps) < 1e-4 and absf(m_str.state.r_rad_s) < 1e-4:
		passed += 1
		print("  [PASS] Test B (Straight-Line Symmetry): ay=%.5f m/s^2, v=%.5f m/s, r=%.5f rad/s at u=%.2f m/s" % [
			m_str.state.ay_mps2, m_str.state.v_mps, m_str.state.r_rad_s, m_str.state.u_mps
		])
	else:
		failed += 1
		print("  [FAIL] Test B (Straight-Line Symmetry) failed: ay=%f v=%f r=%f" % [
			m_str.state.ay_mps2, m_str.state.v_mps, m_str.state.r_rad_s
		])

	var m_left := VehicleDynamicsModel.new(default_cfg_res.duplicate(true))
	m_left.contact_query.use_analytic_plane = true
	m_left.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m_left.state.wheels:
		w.omega_rad_s = 50.0
	m_left.set_control_commands(0.0, 0.20, false)
	var r_at_100ms: float = 0.0
	for s in range(400):
		m_left.step(0.0025, true, false)
		if s == 39:
			r_at_100ms = m_left.state.r_rad_s

	var m_right := VehicleDynamicsModel.new(default_cfg_res.duplicate(true))
	m_right.contact_query.use_analytic_plane = true
	m_right.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -10.0)
	for w in m_right.state.wheels:
		w.omega_rad_s = 50.0
	m_right.set_control_commands(0.0, -0.20, false)
	for _s in range(400):
		m_right.step(0.0025, true, false)

	var ack_ok: bool = m_left.state.wheels[0].steering_angle_rad > m_left.state.wheels[1].steering_angle_rad
	var sym_ok: bool = (
		absf(m_left.state.ay_mps2 + m_right.state.ay_mps2) < 0.05 and
		absf(m_left.state.r_rad_s + m_right.state.r_rad_s) < 0.01 and
		r_at_100ms > 0.80
	)
	if ack_ok and sym_ok:
		passed += 1
		print("  [PASS] Test C (Cornering Symmetry & Steering Response): r(0.1s)=+%.2f rad/s, ay_L=+%.2f / ay_R=%.2f m/s^2, Ackermann FL=%.3f > FR=%.3f rad" % [
			r_at_100ms, m_left.state.ay_mps2, m_right.state.ay_mps2,
			m_left.state.wheels[0].steering_angle_rad, m_left.state.wheels[1].steering_angle_rad
		])
	else:
		failed += 1
		print("  [FAIL] Test C failed: r_100ms=%f ay_L=%f ay_R=%f FL=%f FR=%f" % [
			r_at_100ms, m_left.state.ay_mps2, m_right.state.ay_mps2,
			m_left.state.wheels[0].steering_angle_rad, m_left.state.wheels[1].steering_angle_rad
		])

	var betas_deg: Dictionary = {}
	for gas_lvl in [0.2, 0.5, 0.8, 1.0]:
		var m_acc := VehicleDynamicsModel.new(default_cfg_res.duplicate(true))
		m_acc.contact_query.use_analytic_plane = true
		m_acc.state.linear_velocity_world_godot = Vector3(0.0, 0.0, -8.0)
		for w in m_acc.state.wheels:
			w.omega_rad_s = 40.0
		m_acc.set_control_commands(gas_lvl, 0.20, false)
		for _s in range(600):
			m_acc.step(0.0025, true, false)
		betas_deg[gas_lvl] = rad_to_deg(atan2(m_acc.state.v_mps, maxf(absf(m_acc.state.u_mps), 0.25)))

	if (
		absf(float(betas_deg[0.2])) < 6.0 and
		absf(float(betas_deg[0.5])) < 6.0 and
		absf(float(betas_deg[0.8])) < 6.0 and
		absf(float(betas_deg[1.0])) < 12.0
	):
		passed += 1
		print("  [PASS] Test D (Cornering Under Throttle Stability): beta(0.2)=%+.1f deg, beta(0.5)=%+.1f deg, beta(0.8)=%+.1f deg, beta(1.0)=%+.1f deg (no spinout)" % [
			float(betas_deg[0.2]), float(betas_deg[0.5]), float(betas_deg[0.8]), float(betas_deg[1.0])
		])
	else:
		failed += 1
		print("  [FAIL] Test D (Cornering Under Throttle) failed: betas=%s" % str(betas_deg))

	print("=== [test_physics_behavior_and_steering] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}

static func run_telemetry_and_correlation_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_telemetry_and_correlation] Starting Telemetry Ingestion & Correlation Engine Tests ===")

	var imu_dir: String = "/home/alexr/eRacing/mirena_ws-SIM/imu_compare"
	var real_rec: TelemetryRecording = TelemetryImporter.load_imu_compare_real_bundle(imu_dir, 20000)
	var sim_rec: TelemetryRecording = TelemetryImporter.load_imu_compare_sim_bundle(imu_dir, "front10_rear12", 20000)

	if (
		real_rec != null and sim_rec != null and
		real_rec.has_usable_channel("imu.ax") and
		real_rec.has_usable_channel("imu.gz") and
		real_rec.has_usable_channel("control.gas") and
		sim_rec.has_usable_channel("imu.ax") and
		sim_rec.has_usable_channel("chassis.u") and
		real_rec.maneuver_onset_time_s > 0.0 and
		sim_rec.maneuver_onset_time_s > 0.0
	):
		passed += 1
		print("  [PASS] TelemetryImporter loaded real_*.csv (onset=%.2fs) and sim_*_front10_rear12.csv (onset=%.2fs)" % [
			real_rec.maneuver_onset_time_s, sim_rec.maneuver_onset_time_s
		])
	else:
		failed += 1
		print("  [FAIL] TelemetryImporter failed to load imu_compare real/sim bundles")

	var all_metrics: Dictionary = CorrelationEngine.evaluate_all_channels(real_rec, sim_rec, 20.0, 95.0, 0.05, 0.0)
	var m_ax: Dictionary = all_metrics.get("imu.ax", {})
	var m_ay: Dictionary = all_metrics.get("imu.ay", {})
	var m_gz: Dictionary = all_metrics.get("imu.gz", {})
	var st_ax: int = int(m_ax.get("status", -1))
	var st_ay: int = int(m_ay.get("status", -1))
	var st_gz: int = int(m_gz.get("status", -1))

	if (
		(st_ax == CorrelationEngine.EvaluationStatus.EVALUATED or st_ax == CorrelationEngine.EvaluationStatus.VALIDATION_WARNING) and
		(st_ay == CorrelationEngine.EvaluationStatus.EVALUATED or st_ay == CorrelationEngine.EvaluationStatus.VALIDATION_WARNING) and
		(st_gz == CorrelationEngine.EvaluationStatus.EVALUATED or st_gz == CorrelationEngine.EvaluationStatus.VALIDATION_WARNING) and
		int(m_ax.get("sample_count", 0)) > 500 and
		is_finite(float(m_ax.get("rmse", NAN))) and float(m_ax.get("rmse", -1.0)) > 0.0 and
		is_finite(float(m_gz.get("rmse", NAN))) and float(m_gz.get("rmse", -1.0)) > 0.0 and
		is_finite(float(m_ax.get("pearson_r", NAN)))
	):
		passed += 1
		print("  [PASS] CorrelationEngine evaluated imu.ax (RMSE=%.3f m/s^2, r=%.2f), imu.ay (RMSE=%.3f m/s^2), imu.gz (RMSE=%.3f rad/s, N=%d)" % [
			float(m_ax["rmse"]), float(m_ax["pearson_r"]), float(m_ay["rmse"]), float(m_gz["rmse"]), int(m_ax["sample_count"])
		])
	else:
		failed += 1
		print("  [FAIL] CorrelationEngine metric check failed: ax=%s gz=%s" % [str(m_ax), str(m_gz)])

	var m_fy: Dictionary = all_metrics.get("tire.fy_fl", {})
	var m_susp: Dictionary = all_metrics.get("susp.heave_fl", {})
	var m_aero: Dictionary = all_metrics.get("aero.drag_n", {})
	if (
		int(m_fy.get("status", -1)) == CorrelationEngine.EvaluationStatus.AWAITING_REFERENCE_DATA and
		int(m_susp.get("status", -1)) == CorrelationEngine.EvaluationStatus.AWAITING_REFERENCE_DATA and
		int(m_aero.get("status", -1)) == CorrelationEngine.EvaluationStatus.AWAITING_REFERENCE_DATA
	):
		passed += 1
		print("  [PASS] Unmeasured channels (tire.fy_fl, susp.heave_fl, aero.drag_n) explicitly report AWAITING_REFERENCE_DATA")
	else:
		failed += 1
		print("  [FAIL] Unmeasured channel status check failed")

	var bag_info: Dictionary = TelemetryImporter.inspect_rosbag2_directory("/home/alexr/eRacing/mirena_ws-SIM/bags/bici")
	if bool(bag_info.get("valid", false)) and (bag_info.get("topics", []) as Array).size() >= 1:
		passed += 1
		print("  [PASS] TelemetryImporter.inspect_rosbag2_directory parsed metadata.yaml (%d topics)" % (bag_info["topics"] as Array).size())
	else:
		failed += 1
		print("  [FAIL] ROS 2 bag metadata inspection failed: %s" % str(bag_info))

	var agg_last: Dictionary = CorrelationEngine.compute_aggregate_vehicle_correlation(all_metrics, "LAST")
	var agg_na: Dictionary = CorrelationEngine.compute_aggregate_vehicle_correlation({}, "N/A")
	var agg_standby: Dictionary = CorrelationEngine.compute_aggregate_vehicle_correlation(
		{"imu.ay": {"status": CorrelationEngine.EvaluationStatus.INSUFFICIENT_DATA, "sample_count": 5}},
		"STANDBY"
	)
	if (
		str(agg_na.get("hud_text", "")) == "CORR: N/A" and
		str(agg_standby.get("hud_text", "")) == "CORR: STANDBY" and
		is_finite(float(agg_last.get("correlation_pct", NAN))) and
		str(agg_last.get("hud_text", "")).begins_with("CORR: ") and
		int(agg_last.get("valid_channel_count", 0)) >= 3
	):
		passed += 1
		print("  [PASS] CorrelationEngine.compute_aggregate_vehicle_correlation verified (%s, N/A='%s', STANDBY='%s')" % [
			str(agg_last["hud_text"]), str(agg_na["hud_text"]), str(agg_standby["hud_text"])
		])
	else:
		failed += 1
		print("  [FAIL] Aggregate correlation verification failed: last=%s na=%s standby=%s" % [
			str(agg_last), str(agg_na), str(agg_standby)
		])

	print("=== [test_telemetry_and_correlation] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}

static func run_scene_and_dashboard_suite(root_node: Window) -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_scene_and_dashboard] Starting MirenaCar Scene Integration & Dashboard Window Tests ===")

	var car_scene: PackedScene = load("res://Scenes/Vehicle/mirena_car.tscn")
	var car: MirenaCar = car_scene.instantiate() as MirenaCar
	car.enable_ros_node = false

	for ros_child_name in ["Lidar", "Camera", "INS", "MirenaTransform"]:
		var rc: Node = car.get_node_or_null(ros_child_name)
		if rc != null:
			car.remove_child(rc)
			rc.free()
	root_node.add_child(car)

	if (
		car != null and
		car.dynamics_controller != null and
		car.physics_engine_mode == MirenaCar.PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS and
		not car.has_legacy_direct_wheel_children() and
		car.dynamics_controller.config.integration_mode == VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D
	):
		passed += 1
		print("  [PASS] mirena_car.tscn instantiated in MODULAR_VEHICLE_DYNAMICS mode with 0 direct VehicleWheel3D children (sole force authority verified)")
	else:
		failed += 1
		print("  [FAIL] mirena_car.tscn modular initialization check failed")

	car.set_physics_engine_mode(MirenaCar.PhysicsEngineMode.LEGACY_GODOT_VEHICLE_BODY)
	var legacy_ok: bool = (
		car.physics_engine_mode == MirenaCar.PhysicsEngineMode.LEGACY_GODOT_VEHICLE_BODY and
		car.has_legacy_direct_wheel_children() and
		car.dynamics_controller.config.integration_mode == VehicleDynamicsTypes.IntegrationMode.INACTIVE_LEGACY_PASSTHROUGH
	)
	car.set_physics_engine_mode(MirenaCar.PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS)
	var modular_restored_ok: bool = (
		car.physics_engine_mode == MirenaCar.PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS and
		not car.has_legacy_direct_wheel_children() and
		car.dynamics_controller.config.integration_mode == VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D
	)
	if legacy_ok and modular_restored_ok:
		passed += 1
		print("  [PASS] Runtime switching between LEGACY_GODOT_VEHICLE_BODY and MODULAR_VEHICLE_DYNAMICS strictly enforces mutual exclusion")
	else:
		failed += 1
		print("  [FAIL] Mode switching mutual exclusion check failed: legacy_ok=%s modular_restored_ok=%s" % [str(legacy_ok), str(modular_restored_ok)])

	car.dynamics_controller.model.reset()
	car.dynamics_controller.model.contact_query.use_analytic_plane = true
	car.global_transform = Transform3D.IDENTITY
	car.linear_velocity = Vector3(0.0, 0.0, -10.0)
	for w in car.dynamics_controller.model.state.wheels:
		w.omega_rad_s = 10.0 * cos(0.15) / 0.20
	car.dynamics_controller.config.tire_config.ter_lateral_grip_scale = 1.0
	car.dynamics_controller.step_from_host(0.0025, 0.0, 0.15, false, Transform3D.IDENTITY)
	var snap_nominal: Dictionary = car.get_live_dynamics_snapshot()
	var fy_nominal: float = absf(float(snap_nominal.get("tire.fy_fl", 0.0)))

	car.dynamics_controller.model.reset()
	car.dynamics_controller.model.contact_query.use_analytic_plane = true
	car.global_transform = Transform3D.IDENTITY
	car.linear_velocity = Vector3(0.0, 0.0, -10.0)
	for w in car.dynamics_controller.model.state.wheels:
		w.omega_rad_s = 10.0 * cos(0.15) / 0.20
	car.dynamics_controller.config.tire_config.ter_lateral_grip_scale = 1.5
	car.dynamics_controller.step_from_host(0.0025, 0.0, 0.15, false, Transform3D.IDENTITY)
	var snap_boosted: Dictionary = car.get_live_dynamics_snapshot()
	var fy_boosted: float = absf(float(snap_boosted.get("tire.fy_fl", 0.0)))
	car.dynamics_controller.config.tire_config.ter_lateral_grip_scale = 1.0

	if fy_nominal > 100.0 and fy_boosted > fy_nominal * 1.25:
		passed += 1
		print("  [PASS] Live parameter change on MirenaCar observably modifies tire forces (Fy_FL: %.1f N -> %.1f N)" % [fy_nominal, fy_boosted])
	else:
		failed += 1
		print("  [FAIL] Live parameter sensitivity check failed: fy_nom=%f fy_boost=%f" % [fy_nominal, fy_boosted])

	var dash: VehicleDynamicsDashboardWindow = car.open_dynamics_dashboard_window()
	dash._physics_process(0.025)
	dash._on_mode_selected(1)
	var all_cats_ok: bool = true
	for cat_idx in range(VehicleDynamicsDashboardWindow.CATEGORY_SPECS.size()):
		dash._select_category(cat_idx)
		if dash.active_category_index != cat_idx:
			all_cats_ok = false
	dash._on_mode_selected(0)

	if (
		dash != null and
		all_cats_ok and
		dash.real_reference_recording != null and
		dash.sim_comparison_recording != null and
		dash.cached_correlation_metrics.size() >= 15
	):
		passed += 1
		print("  [PASS] VehicleDynamicsDashboardWindow initialized, preloaded imu_compare correlation bundle, and navigated all 8 categories")
	else:
		failed += 1
		print("  [FAIL] VehicleDynamicsDashboardWindow verification failed")

	var sim_ui_scene: PackedScene = load("res://Scenes/Sim/UI/SimUI.tscn")
	var sim_ui: Control = sim_ui_scene.instantiate() as Control
	root_node.add_child(sim_ui)
	var stats_panel: Node = sim_ui.get_node_or_null("Stats")
	var corr_lbl: Label = sim_ui.find_child("CorrelationLabel", true, false) as Label
	var fps_lbl: Label = sim_ui.find_child("FpsLabel", true, false) as Label
	var timer_lbl: Label = sim_ui.find_child("TimerLabel", true, false) as Label
	var cones_lbl: Label = sim_ui.find_child("FallenConesLabel", true, false) as Label
	var mode_lbl: Label = sim_ui.find_child("PhysicsModeLabel", true, false) as Label
	var ds_lbl: Label = sim_ui.find_child("DatasetLabel", true, false) as Label
	var gz_lbl: Label = sim_ui.find_child("GzMetric", true, false) as Label
	var ay_lbl: Label = sim_ui.find_child("AyMetric", true, false) as Label
	var ax_lbl: Label = sim_ui.find_child("AxMetric", true, false) as Label
	var u_lbl: Label = sim_ui.find_child("UMetric", true, false) as Label
	var v_lbl: Label = sim_ui.find_child("VMetric", true, false) as Label

	var legacy_corr: Label = sim_ui.get_node_or_null("Stats/MarginContainer/VBoxContainer/CorrelationLabel") as Label
	var legacy_fps: Label = sim_ui.get_node_or_null("Stats/MarginContainer/VBoxContainer/FpsLabel") as Label

	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.30)
	var hud_corr_text: String = corr_lbl.text if corr_lbl != null else ""
	var hud_mode_text: String = mode_lbl.text if mode_lbl != null else ""
	var hud_gz_text: String = gz_lbl.text if gz_lbl != null else ""

	car.set_physics_engine_mode(0)
	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.30)
	var legacy_mode_text: String = mode_lbl.text if mode_lbl != null else ""

	car.set_physics_engine_mode(1)
	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.30)
	var restored_mode_text: String = mode_lbl.text if mode_lbl != null else ""

	car.set_correlation_reference_data(null, null, "N/A")
	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.30)
	var hud_na_text: String = corr_lbl.text if corr_lbl != null else ""

	if (
		corr_lbl != null and
		fps_lbl != null and
		timer_lbl != null and
		cones_lbl != null and
		mode_lbl != null and
		ds_lbl != null and
		gz_lbl != null and
		ay_lbl != null and
		ax_lbl != null and
		u_lbl != null and
		v_lbl != null and
		legacy_corr != null and
		legacy_fps != null and
		hud_corr_text.begins_with("CORR: ") and
		hud_corr_text != "CORR: N/A" and
		hud_mode_text.contains("MODULAR") and
		legacy_mode_text.contains("LEGACY") and
		restored_mode_text.contains("MODULAR") and
		hud_gz_text.begins_with("GZ") and
		hud_na_text == "CORR: N/A"
	):
		passed += 1
		print("  [PASS] Main Simulator Persistent Horizontal Engineering Metrics Bar verified:")
		print("         - FPS/Timer/Cones: '%s' | '%s' | '%s'" % [fps_lbl.text, timer_lbl.text, cones_lbl.text])
		print("         - Mode Badge: '%s' -> (switch) '%s' -> (restore) '%s'" % [hud_mode_text, legacy_mode_text, restored_mode_text])
		print("         - Dataset & Window: '%s'" % ds_lbl.text)
		print("         - Correlation Badge: '%s' -> (cleared) '%s'" % [hud_corr_text, hud_na_text])
		print("         - Channel Chips: '%s' | '%s' | '%s' | '%s'" % [gz_lbl.text, ay_lbl.text, ax_lbl.text, u_lbl.text])
	else:
		failed += 1
		print("  [FAIL] Horizontal Engineering Metrics Bar verification failed:")
		print("         corr='%s' mode='%s' leg_mode='%s' gz='%s' na='%s'" % [
			hud_corr_text, hud_mode_text, legacy_mode_text, hud_gz_text, hud_na_text
		])

	var speed_metric: Label = sim_ui.find_child("SpeedMetric", true, false) as Label
	var ax_metric: Label = sim_ui.find_child("AxMetric", true, false) as Label
	var ay_metric: Label = sim_ui.find_child("AyMetric", true, false) as Label
	var az_metric: Label = sim_ui.find_child("AzMetric", true, false) as Label
	var yaw_metric: Label = sim_ui.find_child("YawMetric", true, false) as Label
	var beta_metric: Label = sim_ui.find_child("BetaMetric", true, false) as Label
	var roll_metric: Label = sim_ui.find_child("RollMetric", true, false) as Label
	var pitch_metric: Label = sim_ui.find_child("PitchMetric", true, false) as Label

	car.set_physics_engine_mode(1)
	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.15)

	var mod_speed: String = speed_metric.text if speed_metric != null else ""
	var mod_ax: String = ax_metric.text if ax_metric != null else ""
	var mod_ay: String = ay_metric.text if ay_metric != null else ""
	var mod_az: String = az_metric.text if az_metric != null else ""
	var mod_yaw: String = yaw_metric.text if yaw_metric != null else ""
	var mod_beta: String = beta_metric.text if beta_metric != null else ""
	var mod_roll: String = roll_metric.text if roll_metric != null else ""
	var mod_pitch: String = pitch_metric.text if pitch_metric != null else ""

	car.set_physics_engine_mode(0)
	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.15)

	var leg_ax: String = ax_metric.text if ax_metric != null else ""
	var leg_az: String = az_metric.text if az_metric != null else ""
	var leg_ay: String = ay_metric.text if ay_metric != null else ""
	var leg_beta: String = beta_metric.text if beta_metric != null else ""

	car.set_physics_engine_mode(1)
	if stats_panel != null and stats_panel.has_method("_process"):
		stats_panel.call("_process", 0.15)

	if (
		speed_metric != null and ax_metric != null and ay_metric != null and az_metric != null and
		yaw_metric != null and beta_metric != null and roll_metric != null and pitch_metric != null and
		mod_speed.begins_with("SPEED:") and
		mod_ax.begins_with("AX:") and
		mod_ay.begins_with("AY:") and
		mod_az.begins_with("AZ:") and
		mod_yaw.begins_with("YAW") and
		(mod_beta.contains("BETA") or mod_beta.contains("β")) and
		mod_roll.begins_with("ROLL:") and
		mod_pitch.begins_with("PITCH:") and
		leg_ax.begins_with("AX:") and
		leg_az.begins_with("AZ:") and
		leg_ay.begins_with("AY:")
	):
		passed += 1
		print("  [PASS] Row 2 Live Vehicle Telemetry Chips verified in Modular and Legacy modes:")
		print("         - Modular: '%s' | '%s' | '%s' | '%s' | '%s' | '%s' | '%s' | '%s'" % [
			mod_speed, mod_ax, mod_ay, mod_az, mod_yaw, mod_beta, mod_roll, mod_pitch
		])
		print("         - Legacy continuous kinematics: Ax='%s', Az='%s', Ay='%s', Slip='%s'" % [
			leg_ax, leg_az, leg_ay, leg_beta
		])
	else:
		failed += 1
		print("  [FAIL] Row 2 Live Vehicle Telemetry Chips verification failed:")
		print("         mod_speed='%s', mod_ax='%s', mod_ay='%s', leg_ax='%s', leg_ay='%s'" % [
			mod_speed, mod_ax, mod_ay, leg_ax, leg_ay
		])

	var app_launcher: Node = sim_ui.find_child("AppLauncher", true, false)
	var btn_vd: Button = app_launcher.get_node_or_null("HBoxContainer/BtnVD") as Button if app_launcher != null else null
	var btn_corr_btn: Button = app_launcher.get_node_or_null("HBoxContainer/BtnCorr") as Button if app_launcher != null else null
	var btn_log: Button = app_launcher.get_node_or_null("HBoxContainer/BtnLog") as Button if app_launcher != null else null
	var btn_tire: Button = app_launcher.get_node_or_null("HBoxContainer/BtnTire") as Button if app_launcher != null else null
	var btn_susp: Button = app_launcher.get_node_or_null("HBoxContainer/BtnSusp") as Button if app_launcher != null else null
	var btn_setup: Button = app_launcher.get_node_or_null("HBoxContainer/BtnSetup") as Button if app_launcher != null else null
	var btn_graphs: Button = app_launcher.get_node_or_null("HBoxContainer/BtnGraphs") as Button if app_launcher != null else null
	var btn_menu: Button = app_launcher.get_node_or_null("HBoxContainer/BtnMenu") as Button if app_launcher != null else null

	var app_launcher_structure_ok: bool = (
		app_launcher != null and
		btn_vd != null and btn_corr_btn != null and btn_log != null and
		btn_tire != null and btn_susp != null and btn_setup != null and
		btn_graphs != null and btn_menu != null
	)

	var app_signals_ok: bool = false
	if app_launcher_structure_ok:
		app_signals_ok = (
			btn_vd.pressed.is_connected(app_launcher._on_btn_vd_pressed) and
			btn_corr_btn.pressed.is_connected(app_launcher._on_btn_corr_pressed) and
			btn_log.pressed.is_connected(app_launcher._on_btn_log_pressed) and
			btn_tire.pressed.is_connected(app_launcher._on_btn_tire_pressed) and
			btn_susp.pressed.is_connected(app_launcher._on_btn_susp_pressed) and
			btn_setup.pressed.is_connected(app_launcher._on_btn_setup_pressed) and
			btn_graphs.pressed.is_connected(app_launcher._on_btn_graphs_pressed) and
			btn_menu.pressed.is_connected(app_launcher._on_btn_menu_pressed)
		)

	if app_launcher_structure_ok:
		app_launcher.set("car", car)
		app_launcher.set("main_menu", sim_ui.find_child("MainMenu", true, false))
		app_launcher._on_btn_vd_pressed()
		app_launcher._on_btn_corr_pressed()
		app_launcher._on_btn_tire_pressed()
		app_launcher._on_btn_susp_pressed()
		app_launcher._on_btn_log_pressed()
		app_launcher._on_btn_setup_pressed()
		app_launcher._on_btn_graphs_pressed()
		app_launcher._on_btn_menu_pressed()

	if app_launcher_structure_ok and app_signals_ok:
		passed += 1
		print("  [PASS] Motorsport Applications Launcher verified (8 shortcuts connected: VD, CORR, LOG, TIRE, SUSP, SETUP, GRAPHS, MENU)")
	else:
		failed += 1
		print("  [FAIL] Motorsport Applications Launcher verification failed: structure_ok=%s, signals_ok=%s" % [
			app_launcher_structure_ok, app_signals_ok
		])

	var graph_mgr: Node = sim_ui.find_child("FloatingGraphManager", true, false)
	var mgr_ok: bool = (graph_mgr != null)
	var preset_ok: bool = false
	var panel_types_ok: bool = false
	var serialize_ok: bool = false
	var toggle_ok: bool = false

	if mgr_ok:
		graph_mgr.set("car", car)
		graph_mgr.call("load_preset_vehicle_dynamics")
		var p_list: Array = graph_mgr.get("panels") as Array
		preset_ok = (p_list != null and p_list.size() >= 3)

		var has_ts: bool = false
		var has_xy: bool = false
		if p_list != null:
			for p in p_list:
				var gtype: int = int(p.get("graph_type"))
				if gtype == 0:
					has_ts = true
				elif gtype == 1:
					has_xy = true
		panel_types_ok = (has_ts and has_xy)

		if p_list != null and not p_list.is_empty():
			var p0: Node = p_list[0]
			p0.set("position", Vector2(150, 120))
			p0.set("size", Vector2(400, 250))
			p0.call("_toggle_collapse")
			var collapsed: bool = bool(p0.get("is_collapsed"))
			p0.call("_toggle_collapse")
			var uncollapsed: bool = not bool(p0.get("is_collapsed"))
			toggle_ok = (collapsed and uncollapsed)

		graph_mgr.call("save_layout")
		var loaded: bool = bool(graph_mgr.call("load_layout"))
		var reloaded_list: Array = graph_mgr.get("panels") as Array
		serialize_ok = (loaded and reloaded_list != null and not reloaded_list.is_empty())

		var snap: Dictionary = car.call("get_live_dynamics_snapshot")
		if reloaded_list != null:
			for p in reloaded_list:
				p.call("sample_snapshot", snap, 1.0)

	var final_panel_count: int = (graph_mgr.get("panels") as Array).size() if mgr_ok else 0
	if mgr_ok and preset_ok and panel_types_ok and toggle_ok and serialize_ok:
		passed += 1
		print("  [PASS] Configurable Floating Telemetry Graphs System verified (Panels=%d, Types, Drag/Resize/Collapse, Persistence)" % [
			final_panel_count
		])
	else:
		failed += 1
		print("  [FAIL] Floating Telemetry Graphs verification failed: mgr=%s, preset=%s, types=%s, toggle=%s, serialize=%s" % [
			mgr_ok, preset_ok, panel_types_ok, toggle_ok, serialize_ok
		])

	if sim_ui != null and is_instance_valid(sim_ui):
		sim_ui.free()
	if dash != null and is_instance_valid(dash):
		dash.free()
	if car != null and is_instance_valid(car):
		car.free()

	print("=== [test_scene_and_dashboard] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}
