extends SceneTree
## Headless Godot 4.7 integration test suite for the TeR26 14-DOF C++20 Multibody Engine
## inside `mirena_car.tscn` (`MirenaCar` / `vehicleMotion.gd`).

var tests_run: int = 0
var tests_passed: int = 0
var tests_failed: int = 0

func check(cond: bool, name: String, detail: String = "") -> void:
	tests_run += 1
	if cond:
		tests_passed += 1
		print("  [PASS] ", name, (" (" + detail + ")") if detail != "" else "")
	else:
		tests_failed += 1
		print("  [FAIL] ", name, (" (" + detail + ")") if detail != "" else "")

func _initialize() -> void:
	call_deferred("_run_suite")

func _run_suite() -> void:
	print("==========================================================================")
	print("  GODOT 4.7 HEADLESS INTEGRATION TEST SUITE — TeR26 MULTIBODY ENGINE")
	print("==========================================================================")

	# 1. Verify GDExtension registration
	var has_ext := ClassDB.class_exists("Ter26VehicleBridge")
	check(has_ext, "GDExtension class `Ter26VehicleBridge` registered in Godot 4.7")
	if not has_ext:
		quit(1)
		return

	# 2. Build a test arena with a flat track StaticBody3D on Layer 1
	var arena := Node3D.new()
	arena.name = "TestArena"
	root.add_child(arena)

	var ground := StaticBody3D.new()
	ground.name = "FlatTrack"
	ground.collision_layer = 1
	ground.collision_mask = 1
	var ground_col := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(500.0, 1.0, 500.0)
	ground_col.shape = box
	ground_col.position = Vector3(0.0, -0.5, 0.0) # Top surface at y = 0.0
	ground.add_child(ground_col)
	arena.add_child(ground)

	# 3. Instantiate `mirena_car.tscn` with ROS node disabled for standalone physics test
	var car_scene: PackedScene = load("res://Scenes/Vehicle/mirena_car.tscn")
	check(car_scene != null, "Load `res://Scenes/Vehicle/mirena_car.tscn`")
	var car = car_scene.instantiate()
	car.enable_ros_node = false
	car.pilot = car.PilotMode.ROS
	arena.add_child(car)

	await process_frame
	await physics_frame

	check(car._ter26_adapter != null and car._ter26_adapter.is_native_available,
		"MirenaCar connected to native C++20 Ter26VehicleBridge")
	check(car.custom_integrator == true and is_zero_approx(car.gravity_scale),
		"Single Physical Authority enforced (custom_integrator=true, gravity_scale=0)")
	check(is_zero_approx(car.get_node("FL_WHEEL").suspension_stiffness) and is_zero_approx(car.get_node("RL_WHEEL").wheel_friction_slip),
		"Godot built-in VehicleWheel3D forces neutralized to zero")

	# 4. Verify preserved child sensor nodes
	check(car.get_node_or_null("INS") != null and car.get_node_or_null("Lidar") != null and car.get_node_or_null("Camera") != null and car.get_node_or_null("MirenaTransform") != null,
		"All child sensor & transform nodes ($INS, $Lidar, $Camera, $MirenaTransform) intact")

	# 5. Step 100 ticks (0.25 s at 400 Hz) with zero input -> verify static equilibrium
	var dt := 1.0 / 400.0
	car._ros_gas = 0.0
	car._ros_steer = 0.0
	for _i in range(100):
		car._physics_process(dt)

	var diag: Dictionary = car.multibody_diagnostics
	var fz_sum: float = float(diag["FL"]["Fz"]) + float(diag["FR"]["Fz"]) + float(diag["RL"]["Fz"]) + float(diag["RR"]["Fz"])
	var v_eq: float = car.linear_velocity.length()
	check(absf(fz_sum - 2943.0) < 5.0 and v_eq < 0.01,
		"Static equilibrium in Godot scene",
		"sum(Fz)=%.2f N vs 2943.0 N, |v|=%.5f m/s" % [fz_sum, v_eq])

	# 6. Full-throttle acceleration (400 ticks = 1.00 s)
	car._ros_gas = 1.0
	car._ros_steer = 0.0
	for _i in range(400):
		car._physics_process(dt)

	var vx_launch: float = float(car.multibody_diagnostics["vx"])
	var rl_rpm: float = float(car.get_wheel_rpm_authoritative(2))
	var rr_rpm: float = float(car.get_wheel_rpm_authoritative(3))
	var fz_rear_launch: float = float(car.multibody_diagnostics["RL"]["Fz"]) + float(car.multibody_diagnostics["RR"]["Fz"])
	var fz_front_launch: float = float(car.multibody_diagnostics["FL"]["Fz"]) + float(car.multibody_diagnostics["FR"]["Fz"])
	check(vx_launch > 6.0 and rl_rpm > 300.0 and rr_rpm > 300.0 and fz_rear_launch > fz_front_launch,
		"Full-throttle launch in Godot scene (1.0 s)",
		"vx=%.2f m/s, RL_RPM=%.1f, Fz_R=%.1f N > Fz_F=%.1f N" % [vx_launch, rl_rpm, fz_rear_launch, fz_front_launch])

	# 7. Left step-steer (+0.15 rad for 120 ticks = 0.30 s)
	car._ros_gas = 0.25
	car._ros_steer = 0.15
	for _i in range(120):
		car._physics_process(dt)

	var yaw_rate: float = float(car.angular_velocity.y)
	var fz_fr: float = float(car.multibody_diagnostics["FR"]["Fz"])
	var fz_fl: float = float(car.multibody_diagnostics["FL"]["Fz"])
	var fz_rr: float = float(car.multibody_diagnostics["RR"]["Fz"])
	var fz_rl: float = float(car.multibody_diagnostics["RL"]["Fz"])
	check(yaw_rate > 0.3 and fz_fr > fz_fl and fz_rr > fz_rl,
		"Left step-steer cornering & lateral load transfer in Godot scene",
		"omega_y=%.3f rad/s, Fz_FR=%.1f > Fz_FL=%.1f, Fz_RR=%.1f > Fz_RL=%.1f" % [yaw_rate, fz_fr, fz_fl, fz_rr, fz_rl])

	# 8. Straighten steering, then trigger EBS braking & verify forward pitch load transfer (100 ticks = 0.25 s)
	car._ros_steer = 0.0
	car._ros_gas = 0.50
	for _i in range(80):
		car._physics_process(dt)
	var v_before_brake: float = float(car.multibody_diagnostics["vx"])
	car.pilot = car.PilotMode.NO_PILOT # activates EBS brake
	for _i in range(100):
		car._physics_process(dt)
	var v_after_brake: float = float(car.multibody_diagnostics["vx"])
	var fz_front_brake: float = float(car.multibody_diagnostics["FL"]["Fz"]) + float(car.multibody_diagnostics["FR"]["Fz"])
	check(v_after_brake < v_before_brake - 1.5 and fz_front_brake > 1350.0,
		"EBS braking deceleration & forward pitch load transfer",
		"v_before=%.2f -> v_after=%.2f m/s, Fz_front=%.1f N" % [v_before_brake, v_after_brake, fz_front_brake])

	# 9. External Cone Collision Impulse Exchange
	var cone := RigidBody3D.new()
	cone.name = "TestCone"
	cone.mass = 0.20
	cone.add_to_group("Cones")
	arena.add_child(cone)
	var fwd_dir: Vector3 = car.linear_velocity.normalized() if car.linear_velocity.length() > 0.1 else (-car.global_transform.basis.z)
	cone.global_position = car.global_position + fwd_dir * 1.2 + Vector3(0.0, 0.2, 0.0)

	var v_car_pre: float = float(car.linear_velocity.length())
	car._on_body_entered(cone)
	var v_car_post: float = float(car.linear_velocity.length())
	var v_cone_post: float = float(cone.linear_velocity.length())
	check(car.hit_cones.size() == 1 and v_cone_post > 1.0 and v_car_post < v_car_pre,
		"External cone collision impulse bridge",
		"hit_cones=%d, v_cone=%.2f m/s, dv_car=%.4f m/s" % [car.hit_cones.size(), v_cone_post, v_car_pre - v_car_post])

	print("==========================================================================")
	print("  GODOT INTEGRATION SUMMARY: %d / %d PASSED (%d FAILED)" % [tests_passed, tests_run, tests_failed])
	print("==========================================================================")

	quit(0 if tests_failed == 0 else 1)
