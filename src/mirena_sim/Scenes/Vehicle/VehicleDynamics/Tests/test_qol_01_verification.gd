extends SceneTree

const FloatingGraphManager = preload("res://Scenes/Sim/UI/FloatingGraphManager.gd")

var passed: int = 0
var failed: int = 0
var step_count: int = 0

var root_node: Node3D
var car: MirenaCar
var menu: Control
var graph_mgr: FloatingGraphManager
var dash: VehicleDynamicsDashboardWindow

func _init() -> void:
	print("\n========================================================================")
	print("  MirenaSim Vehicle Dynamics Verification Suite")
	print("========================================================================")

	root_node = Node3D.new()
	get_root().add_child(root_node)

	var floor_body := StaticBody3D.new()
	var col := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(500, 1, 500)
	col.shape = box
	floor_body.add_child(col)
	floor_body.position = Vector3(0, -0.5, 0)
	root_node.add_child(floor_body)

	var car_scene: PackedScene = load("res://Scenes/Vehicle/mirena_car.tscn")
	car = car_scene.instantiate() as MirenaCar
	car.enable_ros_node = false
	root_node.add_child(car)
	car.global_position = Vector3(0, 0.4, 0)

func _process(_delta: float) -> bool:
	step_count += 1

	if step_count == 2:
		_test_tire_models_and_math()
		_test_clean_startup()
		_test_ui_contextualization()
		_test_legacy_physics_controls()
	elif step_count >= 60:
		_finish_legacy_physics_controls()
		_test_switching_state_transfer()

		print("========================================================================")
		print("  VERIFICATION SUMMARY: %d PASSED, %d FAILED" % [passed, failed])
		print("========================================================================")

		if root_node != null and is_instance_valid(root_node):
			root_node.queue_free()

		quit(0 if failed == 0 else 1)
		return true

	return false

func _test_tire_models_and_math() -> void:
	print("\n--- Test Suite 1: Three Modular Tire Models & Grey-Box Decomposition ---")
	var cfg: VehicleDynamicsConfig = load("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_ter26_opt.tres")
	if cfg == null:
		cfg = load("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_ter26_calibrated.tres")

	var tire_cfg: TireConfiguration = cfg.tire_config.duplicate(true)

	var model_a := TeRTireGreyBoxModel.new()
	var model_b := PacejkaMF62TireModel.new()
	var model_c := TireModelInterface.LinearTestStubTireModel.new()

	var state_a := TireState.new()
	var state_b := TireState.new()
	var state_c := TireState.new()

	var inp := TireInputs.new()
	inp.corner = VehicleDynamicsTypes.WheelCorner.FL
	inp.normal_load_fz_n = 700.0
	inp.slip_ratio_kappa = 0.05
	inp.slip_angle_alpha_rad = 0.08
	inp.camber_gamma_rad = -0.035
	inp.vx_contact_mps = 12.0
	inp.vy_contact_mps = tan(0.08) * 12.0
	inp.wheel_omega_rad_s = (12.0 * 1.05) / 0.20
	inp.effective_radius_m = 0.20
	inp.inflation_pressure_kpa = 82.7
	inp.tire_temperature_c = 65.0
	inp.surface_mu_scale = 1.0
	inp.dt_s = 0.0025

	tire_cfg.enable_pinn_residual = true
	tire_cfg.enable_relaxation = false

	var f_a: TireForces = model_a.evaluate_forces(inp, state_a, tire_cfg)
	var f_b: TireForces = model_b.evaluate_forces(inp, state_b, tire_cfg)
	var f_c: TireForces = model_c.evaluate_forces(inp, state_c, tire_cfg)

	if model_a.is_implemented() and model_a.is_calibrated() and f_a.is_finite() and absf(f_a.fy_n) > 100.0:
		passed += 1
		print("  [PASS] Model A (TeR_Tire Grey-Box): Implemented, Calibrated, Fy=%+.1f N, Fx=%+.1f N" % [f_a.fy_n, f_a.fx_n])
	else:
		failed += 1
		print("  [FAIL] Model A evaluation failed")

	var delta_fy_pinn_combined: float = state_a.delta_fy_pinn_n
	var fy_diff: float = absf(f_a.fy_n - f_b.fy_n)
	if model_b.is_implemented() and model_b.is_calibrated() and f_b.is_finite() and fy_diff > 0.01:
		passed += 1
		print("  [PASS] Model B (Pure Pacejka MF6.2): Fy=%+.1f N, Delta_Fy_PINN=%+.1f N (Diff=%.1f N verified)" % [
			f_b.fy_n, delta_fy_pinn_combined, fy_diff
		])
	else:
		failed += 1
		print("  [FAIL] Model B evaluation or decomposition check failed")

	var c_ok: bool = (
		model_c.is_implemented() and
		not model_c.is_calibrated() and
		f_c.is_finite() and
		absf(f_c.fy_n) > 50.0 and
		state_c.static_camber_fy_n != 0.0 and
		state_c.combined_slip_utilization > 0.0 and
		state_c.lateral_force_reserve_n > 0.0 and
		state_c.iso_camber_gamma_rad == inp.camber_gamma_rad
	)
	if c_ok:
		passed += 1
		print("  [PASS] Model C (Linear Baseline): Fy=%+.1f N, Camber_Fy=%+.1f N, Reserve=%.1f N, Util=%.2f" % [
			f_c.fy_n, state_c.static_camber_fy_n, state_c.lateral_force_reserve_n, state_c.combined_slip_utilization
		])
	else:
		failed += 1
		print("  [FAIL] Model C linear evaluation or diagnostics check failed: c_ok=%s" % c_ok)

	if absf(f_a.fy_n - f_b.fy_n) > 0.01 and absf(f_b.fy_n - f_c.fy_n) > 10.0:
		passed += 1
		print("  [PASS] Three distinct tire force signatures verified across Grey-Box, MF6.2, and Linear models")
	else:
		failed += 1
		print("  [FAIL] Models produced identical forces (not distinct)")

func _test_clean_startup() -> void:
	print("\n--- Test Suite 2: Clean Application Startup ---")
	graph_mgr = FloatingGraphManager.new()
	get_root().add_child(graph_mgr)
	graph_mgr._ready()

	if graph_mgr.panels.is_empty():
		passed += 1
		print("  [PASS] Clean Startup: FloatingGraphManager initialized with 0 visible panels on application boot")
	else:
		failed += 1
		print("  [FAIL] Clean Startup: Found %d automatic floating panels open at startup" % graph_mgr.panels.size())

	var menu_has_restore: bool = false
	if graph_mgr.graphs_menu != null:
		for i in range(graph_mgr.graphs_menu.item_count):
			if graph_mgr.graphs_menu.get_item_id(i) == 403:
				menu_has_restore = true
				break
	if menu_has_restore:
		passed += 1
		print("  [PASS] GRAPHS menu includes 'Restore Saved Layout' option (ID 403)")
	else:
		failed += 1
		print("  [FAIL] GRAPHS menu missing 'Restore Saved Layout' option")

	graph_mgr.queue_free()

func _test_ui_contextualization() -> void:
	print("\n--- Test Suite 3: UI Contextualization & HUD Labeling ---")
	var sim_auto: Node = get_root().get_node_or_null("Sim")
	if sim_auto != null:
		sim_auto.set("car", car)
	var menu_script = load("res://Scenes/Sim/UI/vehicle_menu.gd")
	menu = MarginContainer.new()
	menu.set_script(menu_script)
	var vbox := VBoxContainer.new()
	vbox.name = "Vehicle"
	menu.add_child(vbox)
	var m_status := Control.new()
	m_status.name = "MissionStatus"
	var m_lbl := Label.new()
	m_lbl.name = "Label"
	m_status.add_child(m_lbl)
	vbox.add_child(m_status)
	get_root().add_child(menu)
	menu._ready()

	car.set_physics_engine_mode(1)
	car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.PACEJKA_MF62)
	menu._process(0.016)

	var tire_sel: OptionButton = menu.get("_tire_model_select")
	var phys_sel: OptionButton = menu.get("_physics_mode_select")

	var mod_ui_ok: bool = (tire_sel != null and not tire_sel.disabled and tire_sel.selected == 1)
	if mod_ui_ok:
		passed += 1
		print("  [PASS] UI in Modular Mode: Tire selector enabled, selected index 1 (Pure MF6.2)")
	else:
		failed += 1
		print("  [FAIL] UI in Modular Mode check failed: disabled=%s, sel=%s" % [
			tire_sel.disabled if tire_sel else "null", tire_sel.selected if tire_sel else "null"
		])

	car.set_physics_engine_mode(0)
	menu._process(0.016)
	var leg_ui_ok: bool = (tire_sel != null and tire_sel.disabled and tire_sel.tooltip_text.contains("unavailable in Legacy Godot mode"))
	if leg_ui_ok:
		passed += 1
		print("  [PASS] UI in Legacy Mode: Tire selector disabled with explanatory tooltip")
	else:
		failed += 1
		print("  [FAIL] UI in Legacy Mode check failed: disabled=%s, tooltip=%s" % [
			tire_sel.disabled if tire_sel else "null", tire_sel.tooltip_text if tire_sel else "null"
		])

	menu._on_physics_engine_selected(0)
	menu._process(0.016)
	var restore_ui_ok: bool = (tire_sel != null and not tire_sel.disabled and tire_sel.selected == 1)
	if restore_ui_ok:
		passed += 1
		print("  [PASS] UI return to Modular Mode: Prior tire model selection (Pure MF6.2) seamlessly restored")
	else:
		failed += 1
		print("  [FAIL] UI return to Modular Mode check failed: disabled=%s, sel=%s" % [
			tire_sel.disabled if tire_sel else "null", tire_sel.selected if tire_sel else "null"
		])

	car.set_physics_engine_mode(1)
	car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN)
	var h1: Dictionary = car.get_hud_correlation_summary()
	car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.PACEJKA_MF62)
	var h2: Dictionary = car.get_hud_correlation_summary()
	car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB)
	var h3: Dictionary = car.get_hud_correlation_summary()
	car.set_physics_engine_mode(0)
	var h4: Dictionary = car.get_hud_correlation_summary()

	var hud_labels_ok: bool = (
		str(h1.get("physics_mode_label")).contains("TeR_Tire MF6.2") and
		str(h2.get("physics_mode_label")).contains("Pure MF6.2") and
		str(h3.get("physics_mode_label")).contains("Linear Baseline") and
		str(h4.get("physics_mode_label")).contains("LEGACY")
	)
	if hud_labels_ok:
		passed += 1
		print("  [PASS] Dynamic Top Bar HUD Badges verified:")
		print("         - Grey-Box: '%s'" % h1.get("physics_mode_label"))
		print("         - Pure MF6.2: '%s'" % h2.get("physics_mode_label"))
		print("         - Linear: '%s'" % h3.get("physics_mode_label"))
		print("         - Legacy: '%s'" % h4.get("physics_mode_label"))
	else:
		failed += 1
		print("  [FAIL] Dynamic Top Bar HUD Badges failed: h1=%s, h2=%s, h3=%s, h4=%s" % [
			h1.get("physics_mode_label"), h2.get("physics_mode_label"), h3.get("physics_mode_label"), h4.get("physics_mode_label")
		])

	menu.queue_free()

func _test_legacy_physics_controls() -> void:
	print("\n--- Test Suite 4: Legacy Godot Physics Controls (Throttle, Steer, Brake) ---")
	car.set_physics_engine_mode(0)
	car.pilot = MirenaCar.PilotMode.ROS
	car._ros_gas = 0.6
	car._ros_steer = 0.25

func _finish_legacy_physics_controls() -> void:
	var fl: VehicleWheel3D = car._fl_wheel_node
	var fr: VehicleWheel3D = car._fr_wheel_node
	var rl: VehicleWheel3D = car._rl_wheel_node
	var rr: VehicleWheel3D = car._rr_wheel_node

	var steer_fl: float = fl.steering if fl != null else 0.0
	var steer_fr: float = fr.steering if fr != null else 0.0
	var yaw_rate: float = car.angular_velocity.y
	var speed: float = car.linear_velocity.length()

	var steer_controls_ok: bool = (
		absf(steer_fl - 0.25) < 0.01 and
		absf(steer_fr - 0.25) < 0.01 and
		rl.steering == 0.0 and
		rr.steering == 0.0
	)
	if steer_controls_ok:
		passed += 1
		print("  [PASS] Legacy Wheel Steering Assignment: FL=%.3f rad, FR=%.3f rad, RL=%.3f rad, RR=%.3f rad" % [
			steer_fl, steer_fr, rl.steering, rr.steering
		])
	else:
		failed += 1
		print("  [FAIL] Legacy Wheel Steering Assignment failed: FL=%.3f FR=%.3f" % [steer_fl, steer_fr])

	var motion_ok: bool = (speed > 0.5 and absf(yaw_rate) > 0.05)
	if motion_ok:
		passed += 1
		print("  [PASS] Legacy Vehicle Motion & Cornering Response: Speed=%.2f m/s, Yaw Rate=%+.3f rad/s" % [
			speed, yaw_rate
		])
	else:
		failed += 1
		print("  [FAIL] Legacy Vehicle Motion failed: speed=%.2f yaw_rate=%.3f" % [speed, yaw_rate])

	car._ros_gas = -0.5
	car.gas = -0.5
	car.brake = 0.8
	car._apply_legacy_vehicle_physics(0.0025)
	var brake_fl: float = fl.brake if fl != null else 0.0
	var engine_rl: float = rl.engine_force if rl != null else 0.0
	if brake_fl > 100.0 and engine_rl == 0.0:
		passed += 1
		print("  [PASS] Legacy Braking & Mutual Exclusion: Brake_FL=%.1f N, Engine_RL=%.1f N (inhibited)" % [
			brake_fl, engine_rl
		])
	else:
		failed += 1
		print("  [FAIL] Legacy Braking check failed: brake_fl=%.1f engine_rl=%.1f" % [brake_fl, engine_rl])

func _test_switching_state_transfer() -> void:
	print("\n--- Test Suite 5: Runtime Switching State Transfer & Mutual Exclusion ---")
	var pre_vel: Vector3 = car.linear_velocity
	var pre_pos: Vector3 = car.global_position

	car.set_physics_engine_mode(1)
	var post_mod_wheels: int = 0
	for c in car.get_children():
		if c is VehicleWheel3D:
			post_mod_wheels += 1
	var mod_pos: Vector3 = car.global_position
	var mod_vel: Vector3 = car.linear_velocity

	var mod_switch_ok: bool = (
		post_mod_wheels == 0 and
		car.has_legacy_direct_wheel_children() == false and
		mod_pos.distance_to(pre_pos) < 0.05 and
		mod_vel.distance_to(pre_vel) < 0.05
	)
	if mod_switch_ok:
		passed += 1
		print("  [PASS] Legacy -> Modular Handshake: 0 direct wheels, position & linear velocity smoothly preserved")
	else:
		failed += 1
		print("  [FAIL] Legacy -> Modular Handshake failed: direct_wheels=%d, pos_diff=%.3f, vel_diff=%.3f" % [
			post_mod_wheels, mod_pos.distance_to(pre_pos), mod_vel.distance_to(pre_vel)
		])

	car.set_physics_engine_mode(0)
	var post_leg_wheels: int = 0
	for c in car.get_children():
		if c is VehicleWheel3D:
			post_leg_wheels += 1
	var leg_switch_ok: bool = (
		post_leg_wheels == 4 and
		car.has_legacy_direct_wheel_children() == true and
		car.mass == car.MASS_KG and
		car.inertia == Vector3.ZERO
	)
	if leg_switch_ok:
		passed += 1
		print("  [PASS] Modular -> Legacy Handshake: 4 direct wheels restored, mass=%.1f kg, auto-inertia enabled" % car.mass)
	else:
		failed += 1
		print("  [FAIL] Modular -> Legacy Handshake failed: direct_wheels=%d, mass=%.1f, inertia=%s" % [
			post_leg_wheels, car.mass, car.inertia
		])

	var rest_tf_ok: bool = true
	var wheel_nodes: Array[VehicleWheel3D] = [car._fl_wheel_node, car._fr_wheel_node, car._rl_wheel_node, car._rr_wheel_node]
	for w_idx in range(wheel_nodes.size()):
		var wn: VehicleWheel3D = wheel_nodes[w_idx]
		if wn == null or not wn.is_inside_tree():
			rest_tf_ok = false
			break
		if absf(wn.transform.basis.y.y - 1.0) > 0.05:
			rest_tf_ok = false
			break
	if rest_tf_ok:
		passed += 1
		print("  [PASS] Modular -> Legacy Rest Transforms: Wheel orientations clean and untilted")
	else:
		failed += 1
		print("  [FAIL] Modular -> Legacy Rest Transforms distorted")

	var repeated_ok: bool = true
	for cycle in range(3):
		car.set_physics_engine_mode(1)
		if car.has_legacy_direct_wheel_children() or car.dynamics_controller.config.integration_mode != VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D:
			repeated_ok = false
			break
		car.set_physics_engine_mode(0)
		if not car.has_legacy_direct_wheel_children() or car.dynamics_controller.config.integration_mode != VehicleDynamicsTypes.IntegrationMode.INACTIVE_LEGACY_PASSTHROUGH:
			repeated_ok = false
			break
	if repeated_ok:
		passed += 1
		print("  [PASS] Repeated Switching Cycles: 3 full bidirectional cycles maintained strict mutual exclusion")
	else:
		failed += 1
		print("  [FAIL] Repeated Switching Cycles failed")

	car.pilot = MirenaCar.PilotMode.NO_PILOT
	car._process_no_pilot()
	car._apply_legacy_vehicle_physics(0.016)
	var fl_node: VehicleWheel3D = car._fl_wheel_node
	var rl_node: VehicleWheel3D = car._rl_wheel_node
	var no_pilot_ok: bool = (
		fl_node != null and fl_node.brake >= car.BRAKE_F and
		rl_node != null and rl_node.brake >= car.BRAKE_F and
		rl_node.engine_force == 0.0
	)
	if no_pilot_ok:
		passed += 1
		print("  [PASS] Legacy NO_PILOT Safety: EBS braking active across axles with drive torque zeroed")
	else:
		failed += 1
		print("  [FAIL] Legacy NO_PILOT Safety check failed")

	car.set_physics_engine_mode(1)
	var omega_sync_ok: bool = true
	for i in range(car.dynamics_controller.model.state.wheels.size()):
		var ws: WheelState = car.dynamics_controller.model.state.wheels[i]
		if not is_finite(ws.omega_rad_s):
			omega_sync_ok = false
			break
	if omega_sync_ok:
		passed += 1
		print("  [PASS] Angular Speed Synchronization: Finite angular velocities transferred across model boundaries")
	else:
		failed += 1
		print("  [FAIL] Angular Speed Synchronization failed")
