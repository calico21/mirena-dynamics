extends VehicleBody3D
class_name MirenaCar

signal cone_hit(count: int)
enum PilotMode { NO_PILOT, MANUAL, ROS, TRACK_RAIL }

# --- Configuration (Dynamic Config Bridge from Candidate Registry) ---
var POWER_LIM: float = 80000.0 # W
var MOTOR_PEAK_TRQ: float = 100.0 # Nm
var GEAR_RATIO: float = 5.0 # 1:5
var WHEEL_RADIUS: float = 0.20 # m
var BRAKE_F: float = 20.0
var BRAKE_MU: float = 0.40
var BRAKE_BIAS_F: float = 0.60
var MASS_KG: float = 298.0
var GRAVITY: float = 9.81
var MAX_STEER: float = 0.35
var RHO_AIR: float = 1.225 # kg/m^3
var CD_A: float = 2.75 # m^2, drag area
var CL_A: float = 4.554 # m^2, lift (downforce) area
var DRIVETRAIN_EFF: float = 0.92

var tire_integration_mode: String = "NATIVE_VEHICLE_WHEEL"
var active_candidate_id: String = "default"
var active_candidate_version: String = "1.0.0"
var active_config_hash: String = ""
var _pacejka_model = null
var tire_diagnostics: Dictionary = {}
const _Ter26AdapterScript = preload("res://Scenes/Vehicle/Godot/vehicle_adapter.gd")
const _Ter26WheelVisScript = preload("res://Scenes/Vehicle/Godot/wheel_visualizer.gd")
const _Ter26CollisionScript = preload("res://Scenes/Vehicle/Godot/vehicle_collision_bridge.gd")
@export var use_ter26_multibody: bool = true
var _ter26_adapter = null
var multibody_diagnostics: Dictionary = {}
## /control carries a [-1, 1] throttle where -1 is full braking, never reverse,
## so the car must not drive backwards on one. The friction brakes are what
## holds it at a standstill now, not a regen term.

# --- Pilot Settings ---
@export var pilot: PilotMode = PilotMode.NO_PILOT
@export var rail_speed: float = 10.0
@export var rail_look_ahead: float = 3.0
var _rail_progress: float = 0.0
var path : Path3D

# --- ROS Variables ---
@export var frame_id: String = "car/cog"
## Rate the rear wheel speeds go out at, in Hz. Mirrors the real bus, where
## WheelInfo is a cyclic frame polled by mirena_can's 5 ms rx timer and stamped
## on arrival -- the message carries no timestamp of its own.
@export var wheel_speed_rate: float = 100.0
## Low-pass filter time constant (s). tau = 0 passes the quantised encoder
## reading straight through, which is the most honest thing available: on the
## real car the filtering is inside the motor controller, not in this driver.
@export var wheel_speed_lpf_tau: float = 0.05
## Quantisation step of the reported speed in RPM. The bus value is a signed
## int16 in whole RPM (see mirena_can.cpp handle_wheel_info), so a real encoder
## can never report a fraction. At WHEEL_RADIUS that is a 0.021 m/s step, which
## the fusion absorbs as measurement noise and deserves to be exercised against.
@export var wheel_speed_quantise_rpm: float = 1.0
var _node: RosNode
# Publishers
var _state_pub: RosPublisher
var _as_info_pub : RosPublisher
var _perception_pub: RosPublisher
var _slam_pub: RosPublisher
var _inferred_control_pub: RosPublisher
var _wheel_speeds_pub: RosPublisher
var _lap_status_pub: RosPublisher
var _debug_tim : RosTimer
var _wheel_decim_n: int = 0
var _wheel_rpm_acc: Array = [0.0, 0.0] # [rl, rr] sum over the window
var _wheel_rpm_lpf: Array = [0.0, 0.0] # [rl, rr] filtered RPM
# Subscribers
var _control_sub: RosSubscriber
var _mission_info_sub: RosSubscriber
var _as_status_sub: RosSubscriber
var _ros_gas: float = 0.0
var _ros_steer: float = 0.0

# --- Internal State ---
var origin : Transform3D = Transform3D.IDENTITY
var gas: float = 0.0
var _steer_smoothed: float = 0.0
var slam_cones: Array = [] # Seen cones
var hit_cones: Array = [] # Unique struck cones
var _as_info : RosMirenaCommonAsInfo
var _mission_info

var initial_gate_crossings: int = 0
var lap_start_ticks: float = 0.0

@onready var car_state : RosMirenaCommonCar = RosMirenaCommonCar.new()
@onready var wheel_speeds : RosMirenaCommonWheelSpeeds = RosMirenaCommonWheelSpeeds.new()
@onready var lap_status_msg : RosMirenaCommonLapStatus = RosMirenaCommonLapStatus.new()
var enable_ros_node: bool = true

func _ready():
	print("[MirenaCar] _ready entered: ", get_path())

	# Ingest active canonical configuration from Candidate Registry
	var bridge_cls = preload("res://Scenes/Vehicle/vehicleConfigBridge.gd")
	if bridge_cls:
		var rec = bridge_cls.apply_configuration_to_car(self)
		if rec.has("candidate_id"):
			active_candidate_id = rec.get("candidate_id", "unknown")
			active_candidate_version = rec.get("candidate_version", "unknown")
			active_config_hash = rec.get("configuration_hash", "")
			tire_integration_mode = rec.get("tire_integration_mode", "NATIVE_VEHICLE_WHEEL")

	if tire_integration_mode in ["CUSTOM_TIRE_DIAGNOSTIC", "CUSTOM_PACEJKA_FORCES"]:
		var pacejka_cls = preload("res://Scenes/Vehicle/PacejkaTireModel.gd")
		if pacejka_cls:
			_pacejka_model = pacejka_cls.new()

	# Initialize C++20 14-DOF TeR26 Multibody Engine Adapter (Single Physical Authority)
	_ter26_adapter = _Ter26AdapterScript.new()
	if use_ter26_multibody and _ter26_adapter.is_native_available:
		gravity_scale = 0.0
		custom_integrator = true
		_Ter26WheelVisScript.neutralize_godot_wheel_physics([$FL_WHEEL, $FR_WHEEL, $RL_WHEEL, $RR_WHEEL])
		_ter26_adapter.reset_to_transform(global_transform)
		print("[MirenaCar] TeR26 14-DOF C++20 Multibody Engine active (Single Physical Authority)")
	else:
		print("[MirenaCar] Using legacy VehicleBody3D fallback")

	# Enable collision contact reporting
	
	cone_collision_set(true)

	# Add to vehicle group so gates can detect us
	add_to_group("vehicle")
	add_to_group("Car")
	contact_monitor = true
	max_contacts_reported = 10
	body_entered.connect(_on_body_entered)

	# Enable cone collisions by default
	

	# Connect to InitialGate's vehicle_passed signal for lap timing
	_connect_to_initial_gate()

	## ROS — no namespace: all topic names below are absolute
	var ros_bootstrap: Node = get_node_or_null("/root/rclgd_init")
	var ros_context_ok := false
	if ros_bootstrap != null:
		ros_context_ok = rclgd.ok()
	print("[MirenaROS] bootstrap=", ros_bootstrap != null, " context_ok=", ros_context_ok, " enabled=", enable_ros_node)
	if enable_ros_node and is_inside_tree() and ros_bootstrap != null and ros_context_ok:
		_node = RosNode.new()
		_node.init("mirena_car", "")
	
		## Publishers (absolute topic names from topic contract)
		_as_info_pub          = _node.create_publisher("/as_status",        "mirena_common/msg/AsInfo")
		_state_pub            = _node.create_publisher("/debug/state/car", "mirena_common/msg/Car")
		_perception_pub       = _node.create_publisher("/debug/perception",  "mirena_common/msg/EntityList")
		_slam_pub             = _node.create_publisher("/debug/slam",        "mirena_common/msg/EntityList")
		_inferred_control_pub = _node.create_publisher("/inferred_control",  "mirena_common/msg/CarControl")
		_lap_status_pub       = _node.create_publisher("/lap_status",        "mirena_common/msg/LapStatus")
	
		var wheel_qos := RosQoS.new()
		wheel_qos.history = RosQoS.KEEP_LAST
		# depth 5 / BEST_EFFORT mirrors rclcpp::SensorDataQoS(), which is what the
		# real CAN bridge publishes /sensors/wheel_speeds on. Deliberately not
		# widened: a sim that queues more than the car does hides its own hitches.
		wheel_qos.depth = 5
		wheel_qos.reliability = RosQoS.BEST_EFFORT
		wheel_qos.durability = RosQoS.VOLATILE
		_wheel_speeds_pub = _node.create_publisher(
			"/sensors/wheel_speeds", "mirena_common/msg/WheelSpeeds", wheel_qos)

		# State setup
		_as_info = RosMirenaCommonAsInfo.new()
		_as_info.status = 0
		_mission_info = RosMirenaCommonMissionInfo.new()
		_mission_info.status = 0
	
		## Publisher timers
		#
		# Wheel speeds are deliberately NOT on a RosTimer. RosTimer callbacks are
		# handed to the Godot main thread with call_deferred (see
		# rclgd/src/rclgd/ros_timer.cpp), so a "100 Hz" timer actually fires on a
		# render-frame boundary -- with vsync off that frame rate floats with load,
		# and the published intervals jitter even though the stamps come from
		# /clock. The encoder is driven off _physics_process instead, which lands
		# exactly on the requested tick.
		_debug_tim = _node.create_timer(0.1, _debug_publish)
	
		# Subscribers
		_control_sub = _node.create_subscription("/control", "mirena_common/msg/CarControl", _on_control)
		_mission_info_sub = _node.create_subscription("/system/mission_status", "mirena_common/msg/MissionInfo", _on_mission_info)
		_as_status_sub = _node.create_subscription("/as_status", "mirena_common/msg/AsInfo", _on_as_status)
		print("[MirenaROS] RosNode.init invoked for mirena_car; wheel publisher valid=", _wheel_speeds_pub != null)
	else:
		push_warning("[MirenaROS] Node not created: enable_ros_node, /root/rclgd_init or rclgd context is unavailable.")

	# Camera Registration
	Sim.register_camera($TPCam)
	Sim.register_camera($FPCam)

func _on_body_entered(body: Node) -> void:
	if use_ter26_multibody and _ter26_adapter != null and _ter26_adapter.is_native_available:
		_Ter26CollisionScript.handle_body_collision(self, _ter26_adapter.native_bridge, body)
		linear_velocity = _ter26_adapter.native_bridge.call("get_chassis_linear_velocity_godot")
		angular_velocity = _ter26_adapter.native_bridge.call("get_chassis_angular_velocity_godot")

	if body.is_in_group("Cones") or body.is_in_group("cone"):
		hit_cones.append(body)
		cone_hit.emit(hit_cones.size())
		# lap_start_ticks is 0.0 until the car first crosses the start gate, and it
		# is spawned START_OFFSET metres before it. A cone struck in that stretch
		# would otherwise report get_ticks_msec() - 0, which is engine uptime, not
		# lap time -- and the sweep reads it as a long survival. The count above
		# still stands, so a later post-gate hit reports the correct total.
		if lap_start_ticks == 0.0:
			return
		var elapsed_ms: float = Time.get_ticks_msec() - lap_start_ticks
		var lap_time: float = elapsed_ms / 1000.0
		var total_cones_hit: int = hit_cones.size()
		_publish_lap_status(total_cones_hit, lap_time)
		print("CONE Hit")
		

var _gate_connect_retries: int = 0

func _connect_to_initial_gate():
	if not is_inside_tree():
		return
	var gate = get_tree().get_first_node_in_group("InitialGate")
	if gate and gate.has_signal("vehicle_passed"):
		if not gate.vehicle_passed.is_connected(_on_initial_gate_passed):
			gate.vehicle_passed.connect(_on_initial_gate_passed)
	elif _gate_connect_retries < 30:
		_gate_connect_retries += 1
		get_tree().process_frame.connect(_connect_to_initial_gate, CONNECT_ONE_SHOT)

func _on_initial_gate_passed():
	initial_gate_crossings += 1

	if initial_gate_crossings == 1:
		# First crossing: start timing
		lap_start_ticks = Time.get_ticks_msec()
		print("Lap started!")

	elif initial_gate_crossings >= 2:
		# Finishing crossing: calculate lap time and publish
		var elapsed_ms: float = Time.get_ticks_msec() - lap_start_ticks
		var lap_time: float = elapsed_ms / 1000.0
		var total_cones_hit: int = hit_cones.size()

		_publish_lap_status(total_cones_hit, lap_time)
		print("Lap finished! Time: %.3f s | Cones hit: %d" % [lap_time, total_cones_hit])

		# Reset to 1 and update start ticks to time subsequent continuous laps
		initial_gate_crossings = 1
		lap_start_ticks = Time.get_ticks_msec()

func _publish_lap_status(cones_hit: int, lap_time: float) -> void:
	if _node == null or _lap_status_pub == null:
		return

	lap_status_msg.header.stamp = _node.now()
	lap_status_msg.header.frame_id = "map"
	lap_status_msg.cones_hit = cones_hit
	lap_status_msg.lap_time = lap_time

	_lap_status_pub.publish(lap_status_msg)

func _on_control(msg):
	_ros_gas = msg.gas
	_ros_steer = msg.steer_angle
	
func _on_mission_info(msg):
	_mission_info = msg
	
func _physics_process(delta: float) -> void:
	# Process driving commands
	match pilot:
		PilotMode.NO_PILOT:
			_process_no_pilot()
			_apply_vehicle_physics(delta)
		PilotMode.MANUAL:
			_process_manual_pilot(delta)
			_apply_vehicle_physics(delta)
		PilotMode.ROS:
			_process_ros_pilot()
			_apply_vehicle_physics(delta)
		PilotMode.TRACK_RAIL:
			_process_track_rail(delta)
	
	if global_position.y < -1:
		reset_position()

	_update_wheel_speeds()
	_publish_car_state()
		
# Publish Debug Info
func _debug_publish():
	if _node == null:
		return
	_publish_perception()
	_publish_slam()
	_publish_control()

func _on_as_status(msg):
	_as_info.status = msg.status
	_as_info.mission = msg.mission

func _publish_car_state():
	if _node == null or _state_pub == null:
		return
	var now = _node.now()
	car_state.header.stamp = now
	car_state.header.frame_id = "odom"
	car_state.child_frame_id = frame_id  # "car/cog"
	
	var odom_transform : Transform3D = origin.inverse() * global_transform
	# 1. Pose and Dynamics
	car_state.x = -odom_transform.origin.z
	car_state.y = -odom_transform.origin.x
	car_state.psi = odom_transform.basis.get_euler().y
	
	var local_vel = _body_velocity()
	car_state.u = -local_vel.z
	car_state.v = -local_vel.x
	car_state.omega = angular_velocity.y
	
	# 2. Covariance Setup
	var cov = []
	cov.resize(36)
	for i in range(36):
		cov[i] = 0.0
		
	cov[0]  = 0
	cov[7]  = 0
	cov[14] = 0
	cov[21] = 0
	cov[28] = 0
	cov[35] = 0
	
	car_state.covariance = cov
	_state_pub.publish(car_state)

## Accumulate wheel speed once per physics tick and publish at wheel_speed_rate.
##
## An encoder counts pulses over a measurement window, so what the bus carries is
## the *mean* over that window rather than the instantaneous rate. Godot
## integrates wheel rotation every physics tick, so the mean over
## (physics rate / output rate) ticks is the same quantity the real frame's
## period would average -- and averaging is what removes the per-tick solver
## jitter that made this topic unusable, instead of merely hiding it behind a
## filter. The previous code published one held sample per output period.
func get_wheel_rpm_authoritative(corner_idx: int) -> float:
	if use_ter26_multibody and _ter26_adapter != null and _ter26_adapter.is_native_available:
		return float(_ter26_adapter.native_bridge.call("get_wheel_rpm", corner_idx))
	var wheels: Array[VehicleWheel3D] = [$FL_WHEEL, $FR_WHEEL, $RL_WHEEL, $RR_WHEEL]
	if corner_idx >= 0 and corner_idx < wheels.size() and wheels[corner_idx] != null:
		return -wheels[corner_idx].get_rpm()
	return 0.0

func _update_wheel_speeds() -> void:
	if _node == null or _wheel_speeds_pub == null:
		return
	# Read authoritative wheel spin speeds from C++20 14-DOF multibody engine when active
	if use_ter26_multibody and _ter26_adapter != null and _ter26_adapter.is_native_available:
		_wheel_rpm_acc[0] += float(_ter26_adapter.native_bridge.call("get_wheel_rpm", 2))
		_wheel_rpm_acc[1] += float(_ter26_adapter.native_bridge.call("get_wheel_rpm", 3))
	else:
		# Godot's convention: forward drive makes get_rpm() negative.
		_wheel_rpm_acc[0] += -$RL_WHEEL.get_rpm()
		_wheel_rpm_acc[1] += -$RR_WHEEL.get_rpm()
	_wheel_decim_n += 1

	var window: int = maxi(1, int(round(Engine.physics_ticks_per_second / wheel_speed_rate)))
	if _wheel_decim_n < window:
		return

	# The interval actually accumulated, taken from the tick count rather than
	# assumed from wheel_speed_rate: a nominal dt silently retunes the filter
	# every time the loop hitches, which is precisely when it matters.
	var dt := float(_wheel_decim_n) / float(Engine.physics_ticks_per_second)
	var raw_rl: float = _wheel_rpm_acc[0] / float(_wheel_decim_n)
	var raw_rr: float = _wheel_rpm_acc[1] / float(_wheel_decim_n)
	_wheel_rpm_acc[0] = 0.0
	_wheel_rpm_acc[1] = 0.0
	_wheel_decim_n = 0

	wheel_speeds.header.stamp = _node.now()
	wheel_speeds.header.frame_id = frame_id  # "car/cog"
	# Low-pass filter: Godot vehicle physics produces jittery RPM values.
	# Proper exponential moving average: alpha = 1 - exp(-dt/tau).
	var alpha := 1.0 - exp(-dt / maxf(wheel_speed_lpf_tau, 1e-6))
	# Both rear wheels are reported. RL is the traction wheel and its reading
	# carries whatever slip the solver produced; RR free-rolls, so it reflects
	# the road. Both are real hardware (the bus instruments both), and
	# mirena_state averages them, so publishing only one would halve u.
	_wheel_rpm_lpf[0] = lerpf(_wheel_rpm_lpf[0], raw_rl, alpha)
	_wheel_rpm_lpf[1] = lerpf(_wheel_rpm_lpf[1], raw_rr, alpha)
	# Whole RPM, as int16 on the bus. This is what gives the topic its final
	# quantisation step; filtering above stands in for the motor controller's
	# internal filtering, not for anything the driver adds.
	wheel_speeds.rl = _quantise_rpm(_wheel_rpm_lpf[0])
	wheel_speeds.rr = _quantise_rpm(_wheel_rpm_lpf[1])
	_wheel_speeds_pub.publish(wheel_speeds)

func _quantise_rpm(rpm: float) -> float:
	if wheel_speed_quantise_rpm <= 0.0:
		return rpm
	return roundf(rpm / wheel_speed_quantise_rpm) * wheel_speed_quantise_rpm

func _publish_perception():
	if _node == null or _perception_pub == null:
		return
	var cones = get_cones_in_sight(12.0)
	for cone in cones:
		if not slam_cones.has(cone): slam_cones.append(cone)
	
	var msg = RosMirenaCommonEntityList.new()
	msg.header.stamp = _node.now()
	msg.header.frame_id = frame_id  # "car/cog"
	msg.entities = cones.map(func(c): return _to_ent(c))
	_perception_pub.publish(msg)

func _publish_slam():
	if _node == null or _slam_pub == null:
		return
	var msg = RosMirenaCommonEntityList.new()
	msg.header.stamp = _node.now()
	msg.header.frame_id = "map"
	slam_cones = slam_cones.filter(func(c): return is_instance_valid(c))
	msg.entities = slam_cones.map(func(c): return _to_ent(c, true))
	_slam_pub.publish(msg)

func _publish_control():
	if _node == null or _inferred_control_pub == null:
		return
	var msg = RosMirenaCommonCarControl.new()
	msg.header.stamp = _node.now()
	msg.header.frame_id = "map"
	msg.gas = self.gas
	msg.steer_angle = self.steering

	_inferred_control_pub.publish(msg)

# Conversion helper
func _to_ent(cone: Node3D, global : bool = false) -> RosMirenaCommonEntity:
	var ent = RosMirenaCommonEntity.new()
	var pos = cone.global_position if global else to_local(cone.global_position)
	ent.type = cone.get_type_as_string()
	
	# ROS Swizzle: Forward=Z, Left=-X, Up=Y
	ent.position.x = -pos.z
	ent.position.y = -pos.x
	ent.position.z = pos.y
	return ent

# --- Pilot Logic ---

func _process_no_pilot() -> void:
	self.gas = 0.0
	self.steering = 0.0
	self.brake = BRAKE_F

func _process_manual_pilot(delta: float) -> void:
	var steer_input = Input.get_action_strength("manual_steer_l") - Input.get_action_strength("manual_steer_r")
	_steer_smoothed = _smooth_steer(_steer_smoothed, steer_input, delta, 2.0)
	
	self.gas = Input.get_action_strength("manual_gas_pos") - Input.get_action_strength("manual_gas_neg")
	self.steering = _steer_smoothed * MAX_STEER
	self.brake = Input.get_action_strength("EBS") * BRAKE_F

func _process_ros_pilot() -> void:
	self.gas = _ros_gas
	# The real steering actuator saturates at max_steer_angle; clamp the ROS
	# command so the sim cannot steer beyond the real car's physical limit.
	self.steering = clampf(_ros_steer, -MAX_STEER, MAX_STEER)
	self.brake = 0.0

func _process_track_rail(delta: float) -> void:
	if not path or not path.curve: 
		return

	_rail_progress += rail_speed * delta
	var local_transform = path.curve.sample_baked_with_rotation(_rail_progress, true)
	var target_global_transform = path.global_transform * local_transform
	
	var motion = target_global_transform.origin - global_position
	var collision = move_and_collide(motion)
	if collision:
		var remainder = collision.get_remainder().slide(collision.get_normal())
		move_and_collide(remainder)

	var look_ahead_p = _rail_progress + rail_look_ahead
	var look_target_local = path.curve.sample_baked_with_rotation(look_ahead_p, true)
	var look_target_global = path.global_transform * look_target_local
	
	global_transform.basis = global_transform.basis.slerp(
		look_target_global.basis.orthonormalized(), 
		5.0 * delta
	).orthonormalized()

# --- Physics & Low Level Control ---

func _body_velocity() -> Vector3:
	return global_basis.inverse() * linear_velocity

func _apply_vehicle_physics(_delta: float) -> void:
	if use_ter26_multibody and _ter26_adapter != null and _ter26_adapter.is_native_available:
		var throttle_cmd := clampf(maxf(gas, 0.0), 0.0, 1.0)
		var brake_cmd := clampf(maxf(-gas, 0.0), 0.0, 1.0)
		var ebs_active := (brake > 0.5)
		multibody_diagnostics = _ter26_adapter.query_ground_planes_and_step(
			self,
			_delta,
			throttle_cmd,
			brake_cmd,
			steering,
			ebs_active
		)
		global_transform = _ter26_adapter.native_bridge.call("get_chassis_transform_godot")
		linear_velocity = _ter26_adapter.native_bridge.call("get_chassis_linear_velocity_godot")
		angular_velocity = _ter26_adapter.native_bridge.call("get_chassis_angular_velocity_godot")
		_Ter26WheelVisScript.sync_wheels_from_bridge(
			_ter26_adapter.native_bridge,
			[$FL_WHEEL, $FR_WHEEL, $RL_WHEEL, $RR_WHEEL]
		)
		for w_name in ["FL", "FR", "RL", "RR"]:
			if multibody_diagnostics.has(w_name):
				var cd: Dictionary = multibody_diagnostics[w_name]
				tire_diagnostics[w_name] = {
					"fz_n": cd.get("Fz", 0.0),
					"fx_n": cd.get("Fx", 0.0),
					"fy_pacejka_n": cd.get("Fy", 0.0),
					"mz_pacejka_nm": cd.get("Mz", 0.0),
					"alpha_rad": cd.get("alpha", 0.0),
					"kappa": cd.get("kappa", 0.0),
					"jounce_m": cd.get("jounce", 0.0),
				}
		return

	var u = -_body_velocity().z
	var max_fx_motor = MOTOR_PEAK_TRQ * GEAR_RATIO / WHEEL_RADIUS

	# Aerodynamic drag and downforce (real-car coefficients). Applied at the
	# centre of mass, which keeps the total load and the speed limit right even
	# though it loses the small aero pitch moment. q = 0.5 * rho * v^2.
	var speed := linear_velocity.length()
	if speed > 0.1:
		var q := 0.5 * RHO_AIR * speed * speed
		apply_central_force(-q * CD_A * linear_velocity / speed)
		apply_central_force(-q * CL_A * global_basis.y)

	var throttle = max(gas, 0.0)
	var braking = max(-gas, 0.0)

	# The whole braking budget, and how it is split between the axles.
	var brake_budget = BRAKE_MU * MASS_KG * GRAVITY
	var front_share = brake_budget * BRAKE_BIAS_F * 0.5
	var rear_share = brake_budget * (1.0 - BRAKE_BIAS_F)

	# No separate regen term. Scaling one by the motor's drive torque made it
	# saturate the rear axle's share at 6% of the brake command -- a light
	# pedal touch was shedding 1.3 m/s^2, and the recorded car shows a coast
	# of only 1.9 m/s^2 including gradient. The tyre-limited friction brake
	# carries the whole budget instead, and it also holds the car at rest,
	# which is what bounding the braking at zero needed the regen for.
	# Real drivetrain is power-limited (80 kW) as well as torque-limited. The
	# constant was declared but never used, so the car pulled full torque at any
	# speed. Keep the torque cap but also cap by P/v.
	var fx = minf(throttle * max_fx_motor,
		POWER_LIM * DRIVETRAIN_EFF / maxf(u, 1.0))

	# Rear left and right. max_fx_motor is the total tractive force, so each
	# driven wheel gets half of it -- otherwise the pair asks for twice the
	# motor's output.
	$RL_WHEEL.engine_force = fx / 2.0
	$RR_WHEEL.engine_force = fx / 2.0

	# The front axle carries 60% of the budget -- which the old code never
	# applied at all, it braked the rear wheels only. Both axles scale with
	# the brake command, so a light pedal touch stays a light brake.
	var ebs = min(brake, 1.0) * BRAKE_F
	$FL_WHEEL.brake = front_share * braking + ebs
	$FR_WHEEL.brake = front_share * braking + ebs
	$RL_WHEEL.brake = rear_share * 0.5 * braking + ebs
	$RR_WHEEL.brake = rear_share * 0.5 * braking + ebs

	# Custom Tire Model / Diagnostics
	if _pacejka_model != null:
		_process_custom_tire_forces()

func _process_custom_tire_forces() -> void:
	var wheels = {
		"FL": $FL_WHEEL,
		"FR": $FR_WHEEL,
		"RL": $RL_WHEEL,
		"RR": $RR_WHEEL,
	}

	var local_vel = _body_velocity()
	var u = maxf(abs(local_vel.z), 0.1)
	var ay = angular_velocity.y * u
	var total_w = MASS_KG * GRAVITY
	var a = 0.876
	var b = 0.717
	var L = 1.593
	var tf = 1.174
	var tr = 1.174
	var hcg = 0.33

	var fz_f = total_w * (b / L)
	var fz_r = total_w * (a / L)
	var d_fz_lat_f = (MASS_KG * ay * hcg * (b / L)) / tf
	var d_fz_lat_r = (MASS_KG * ay * hcg * (a / L)) / tr

	var fz_dict = {
		"FL": maxf(10.0, 0.5 * fz_f + d_fz_lat_f),
		"FR": maxf(10.0, 0.5 * fz_f - d_fz_lat_f),
		"RL": maxf(10.0, 0.5 * fz_r + d_fz_lat_r),
		"RR": maxf(10.0, 0.5 * fz_r - d_fz_lat_r),
	}

	for w_name in wheels.keys():
		var w: VehicleWheel3D = wheels[w_name]
		if not w:
			continue

		var fz = fz_dict[w_name]
		var wheel_transform = w.global_transform
		var wheel_fwd = -wheel_transform.basis.z.normalized()
		var wheel_lat = wheel_transform.basis.x.normalized()
		var v_wheel = linear_velocity + angular_velocity.cross(w.global_position - global_position)
		var v_lat = v_wheel.dot(wheel_lat)
		var v_fwd = v_wheel.dot(wheel_fwd)

		var alpha = -atan2(v_lat, maxf(abs(v_fwd), 0.5))
		var forces = _pacejka_model.evaluate_pure_lateral(fz, alpha)
		var fy = forces.x
		var mz = forces.y

		tire_diagnostics[w_name] = {
			"fz_n": fz,
			"alpha_rad": alpha,
			"fy_pacejka_n": fy,
			"mz_pacejka_nm": mz,
		}

		if tire_integration_mode == "CUSTOM_PACEJKA_FORCES" and w.is_in_contact():
			var contact_pt = w.get_contact_point()
			var force_world = wheel_lat * fy
			apply_force(force_world, contact_pt - global_position)
			apply_torque(Vector3(0.0, mz, 0.0))

func _smooth_steer(current: float, target: float, delta: float, speed: float) -> float:
	var diff = target - current
	var t = clamp(abs(diff), 0.0, 1.0)
	var _ease = t * t * (3.0 - 2.0 * t) 
	current += sign(diff) * _ease * speed * delta
	if sign(target - current) != sign(diff): current = target
	return clamp(current, -1.0, 1.0)

# --- Interface & Utility ---

func set_origin(transform, reset_vel: bool = false) -> void:
	origin = transform
	if reset_vel:
		linear_velocity = Vector3.ZERO
		angular_velocity = Vector3.ZERO
	if use_ter26_multibody and _ter26_adapter != null and _ter26_adapter.is_native_available:
		_ter26_adapter.reset_to_transform(origin)
		global_transform = _ter26_adapter.native_bridge.call("get_chassis_transform_godot")
		return
	await get_tree().process_frame
	set_deferred("global_transform", origin)

func reset_position() -> void:
	set_origin(Sim.track.origin, true)
	self.gas = 0
	self.steering = 0
	self.brake = 0
	_rail_progress = 0
	slam_cones.clear()

	# Reset gate crossing and cone timing tracking state
	hit_cones.clear()
	initial_gate_crossings = 0
	lap_start_ticks = 0.0
	cone_hit.emit(0)

	# Sensor state has to be wiped with the pose. Otherwise the first wheel-speed
	# window after a respawn spans the teleport, and the INS finite-differences
	# its way across one too.
	_wheel_rpm_acc[0] = 0.0
	_wheel_rpm_acc[1] = 0.0
	_wheel_rpm_lpf[0] = 0.0
	_wheel_rpm_lpf[1] = 0.0
	_wheel_decim_n = 0
	# Guarded, because a parse error in ros_ins.gd leaves $INS as a bare Node3D
	# with no script -- and an unguarded call here took the whole car down with
	# it, stopping reset_position before it ever reached the wheel speeds above.
	# The INS carrying state across a teleport is bad; the car not respawning at
	# all is worse.
	if $INS.has_method("reset"):
		$INS.reset()

func cone_collision_set(enable: bool) -> void:
	self.collision_layer = (self.collision_layer & ~2) | (2 * int(enable))
	self.collision_mask = (self.collision_mask & ~2) | (2 * int(enable))
	
func get_cones_in_sight(max_dist: float = 10.0) -> Array:
	var visible_cones: Array = []
	var camera = $Camera._camera
	if not camera: return []
	for cone in get_tree().get_nodes_in_group("Cones"):
		if global_position.distance_to(cone.global_position) < max_dist:
			if camera.is_position_in_frustum(cone.global_position):
				visible_cones.append(cone)
	return visible_cones
