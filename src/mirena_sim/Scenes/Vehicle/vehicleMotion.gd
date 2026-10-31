extends VehicleBody3D
class_name MirenaCar

signal cone_hit(count: int)
enum PilotMode { NO_PILOT, MANUAL, ROS, TRACK_RAIL }
enum PhysicsEngineMode {
	LEGACY_GODOT_VEHICLE_BODY = 0,
	MODULAR_VEHICLE_DYNAMICS = 1,
}

# --- Physics Engine Authority Selection ---
@export var physics_engine_mode: PhysicsEngineMode = PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS
var dynamics_controller: VehicleDynamicsController = null
var dashboard_window: VehicleDynamicsDashboardWindow = null
var _modular_wheel_holder: Node3D = null
var _fl_wheel_node: VehicleWheel3D = null
var _fr_wheel_node: VehicleWheel3D = null
var _rl_wheel_node: VehicleWheel3D = null
var _rr_wheel_node: VehicleWheel3D = null
var _wheel_rest_transforms: Array[Transform3D] = []
var _hud_layer: CanvasLayer = null
var _hud_status_panel: PanelContainer = null
var _hud_status_label: Label = null
var _hud_refresh_accum: float = 0.0

# --- Open-Loop Telemetry Control Replay & Persistent Correlation Cache ---
var is_replaying_controls: bool = false
var _replay_recording: TelemetryRecording = null
var _replay_cursor_s: float = 0.0
var _replay_end_s: float = 0.0
var _hud_corr_ref_rec: TelemetryRecording = null
var _hud_corr_sim_rec: TelemetryRecording = null
var _hud_corr_loaded: bool = false
var _hud_corr_eval_accum: float = 0.0
var _hud_corr_summary: Dictionary = {}
var _hud_corr_active_mode: int = -1

# --- Configuration ---
const POWER_LIM = 80000 # W
const MOTOR_PEAK_TRQ = 100 # Nm
const GEAR_RATIO = 5 # 1:5
## Rolling radius of the tyre. Must agree with wheel_radius on every
## VehicleWheel3D in mirena_car.tscn *and* with mirena_state's `wheel_radius`
## parameter, which is what turns the RPM published below into m/s. All three
## were different once: 0.23 here, 0.26 in the scene, 0.20 in the fusion.
const WHEEL_RADIUS = 0.20 # m
const BRAKE_F = 20
## --- Brakes (real-car spec) ---
## Godot applies a wheel's brake force against that wheel's own friction
## coefficient, so with wheel_friction at 1.4 this car would brake with 4120 N.
## The real car's brake_mu is 0.40, and the recorded bag shows it decelerating
## at 3.93 m/s^2 -- 0.40*g to three significant figures -- so its whole
## braking budget is 1177 N, split 60/40 front/rear.
const BRAKE_MU = 0.40
const BRAKE_BIAS_F = 0.60
## Keep in step with `mass` on the MirenaCar node in mirena_car.tscn.
const MASS_KG = 300.0
const GRAVITY = 9.81
## Real-car max_steer_angle is 0.35 rad (20 deg).  The recorded CAN stream
## occasionally asks for more, so clamp to the physical limit in ROS mode too.
const MAX_STEER = 0.35
## --- Aero (real-car spec) ---
## Cl_ref = 4.14, Cd_ref = 2.50, A_ref = 1.10 m^2, rho = 1.225 kg/m^3.
## Godot's VehicleBody3D models neither drag nor downforce, so without these the
## car coasts forever and reaches 20-35 m/s instead of the real car's ~7 m/s.
## That excess speed is what saturates the tyres and collapses the yaw response,
## so this is the single most important term for matching the recorded IMU.
const RHO_AIR = 1.225 # kg/m^3
const CD_A = 2.50 * 1.10 # m^2, drag area
const CL_A = 4.14 * 1.10 # m^2, lift (downforce) area
const DRIVETRAIN_EFF = 0.92
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

# Continuous kinematic body acceleration tracking (gravity-free body frame: +X fwd, +Y left, +Z up)
var _prev_linear_vel: Vector3 = Vector3.ZERO
var _has_prev_linear_vel: bool = false
var _kin_accel_body_lpf: Vector3 = Vector3.ZERO

var initial_gate_crossings: int = 0
var lap_start_ticks: float = 0.0

@onready var car_state : RosMirenaCommonCar = RosMirenaCommonCar.new()
@onready var wheel_speeds : RosMirenaCommonWheelSpeeds = RosMirenaCommonWheelSpeeds.new()
@onready var lap_status_msg : RosMirenaCommonLapStatus = RosMirenaCommonLapStatus.new()
var enable_ros_node: bool = true

func _ready():
	# Enable collision contact reporting
	
	cone_collision_set(true)

	# Add to vehicle and Car groups so gates and HUD Stats panel can detect us
	add_to_group("vehicle")
	add_to_group("Car")
	contact_monitor = true
	max_contacts_reported = 10
	if not body_entered.is_connected(_on_body_entered):
		body_entered.connect(_on_body_entered)

	# Enable cone collisions by default
	

	# Connect to InitialGate's vehicle_passed signal for lap timing
	_connect_to_initial_gate()

	# State setup
	_as_info = RosMirenaCommonAsInfo.new()
	_as_info.status = 0
	_mission_info = RosMirenaCommonMissionInfo.new()
	_mission_info.status = 0

	## ROS — no namespace: all topic names below are absolute
	if enable_ros_node and is_inside_tree() and get_node_or_null("/root/rclgd_init") != null:
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
		wheel_qos.depth = 5
		wheel_qos.reliability = RosQoS.BEST_EFFORT
		wheel_qos.durability = RosQoS.VOLATILE
		_wheel_speeds_pub = _node.create_publisher(
			"/sensors/wheel_speeds", "mirena_common/msg/WheelSpeeds", wheel_qos)
		
		_debug_tim = _node.create_timer(0.1, _debug_publish)
		
		# Subscribers
		_control_sub = _node.create_subscription("/control", "mirena_common/msg/CarControl", _on_control)
		_mission_info_sub = _node.create_subscription("/system/mission_status", "mirena_common/msg/MissionInfo", _on_mission_info)
		_as_status_sub = _node.create_subscription("/as_status", "mirena_common/msg/AsInfo", _on_as_status)

	# Camera Registration
	var sim_autoload: Node = get_node_or_null("/root/Sim") if is_inside_tree() else null
	if has_node("TPCam") and sim_autoload != null and sim_autoload.has_method("register_camera"):
		sim_autoload.call("register_camera", $TPCam)
	if has_node("FPCam") and sim_autoload != null and sim_autoload.has_method("register_camera"):
		sim_autoload.call("register_camera", $FPCam)

	# Initialize Modular VehicleDynamics Framework
	_setup_modular_vehicle_dynamics()

func _on_body_entered(body: Node) -> void:
	if body.is_in_group("Cones"):
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
		

var _gate_connected: bool = false

func _connect_to_initial_gate() -> void:
	if _gate_connected or not is_inside_tree() or get_tree() == null:
		return
	var gate = get_tree().get_first_node_in_group("InitialGate")
	if gate != null and not gate.vehicle_passed.is_connected(_on_initial_gate_passed):
		gate.vehicle_passed.connect(_on_initial_gate_passed)
		_gate_connected = true

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
	if _lap_status_pub == null:
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
	if not _gate_connected:
		_connect_to_initial_gate()
	# Process driving commands (or open-loop telemetry control replay)
	if is_replaying_controls and _replay_recording != null:
		_process_telemetry_replay_pilot(delta)
		_apply_vehicle_physics(delta)
	else:
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

	# Continuous kinematic body acceleration tracking (gravity-free body frame: +X fwd, +Y left, +Z up)
	if delta > 0.0:
		if not _has_prev_linear_vel:
			_prev_linear_vel = linear_velocity
			_has_prev_linear_vel = true
		else:
			var accel_world: Vector3 = (linear_velocity - _prev_linear_vel) / delta
			_prev_linear_vel = linear_velocity
			var local_accel: Vector3 = global_basis.inverse() * accel_world
			var kin_ax: float = -local_accel.z
			var kin_ay: float = -local_accel.x
			var kin_az: float = local_accel.y
			var tau: float = 0.035
			var alpha: float = clampf(delta / (tau + delta), 0.0, 1.0)
			_kin_accel_body_lpf.x = lerpf(_kin_accel_body_lpf.x, kin_ax, alpha)
			_kin_accel_body_lpf.y = lerpf(_kin_accel_body_lpf.y, kin_ay, alpha)
			_kin_accel_body_lpf.z = lerpf(_kin_accel_body_lpf.z, kin_az, alpha)

	_update_wheel_speeds()
	_publish_car_state()

func _process(delta: float) -> void:
	_hud_refresh_accum += delta
	_hud_corr_eval_accum += delta
	if _hud_refresh_accum >= 0.08:
		_hud_refresh_accum = 0.0
		_refresh_dynamics_hud_overlay()

func _unhandled_key_input(event: InputEvent) -> void:
	if not (event is InputEventKey):
		return
	var kev := event as InputEventKey
	if not kev.pressed or kev.echo:
		return
	if kev.keycode == KEY_F6 or kev.physical_keycode == KEY_F6:
		var next_mode: int = PhysicsEngineMode.LEGACY_GODOT_VEHICLE_BODY \
			if physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS \
			else PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS
		set_physics_engine_mode(next_mode)
		get_viewport().set_input_as_handled()
	elif kev.keycode == KEY_F7 or kev.physical_keycode == KEY_F7:
		toggle_dynamics_dashboard_window()
		get_viewport().set_input_as_handled()
		
# Publish Debug Info
func _debug_publish():
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
func _update_wheel_speeds() -> void:
	if physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS and dynamics_controller != null and dynamics_controller.model != null:
		var st: VehicleState = dynamics_controller.model.state
		_wheel_rpm_acc[0] += st.wheels[VehicleDynamicsTypes.WheelCorner.RL].get_rpm()
		_wheel_rpm_acc[1] += st.wheels[VehicleDynamicsTypes.WheelCorner.RR].get_rpm()
	else:
		# Godot's convention: forward drive makes get_rpm() negative.
		var rl_node: VehicleWheel3D = _rl_wheel_node if _rl_wheel_node != null else get_node_or_null("RL_WHEEL") as VehicleWheel3D
		var rr_node: VehicleWheel3D = _rr_wheel_node if _rr_wheel_node != null else get_node_or_null("RR_WHEEL") as VehicleWheel3D
		if rl_node != null and rr_node != null:
			_wheel_rpm_acc[0] += -rl_node.get_rpm()
			_wheel_rpm_acc[1] += -rr_node.get_rpm()
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

	if _node == null or _wheel_speeds_pub == null:
		return
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
	var cones = get_cones_in_sight(12.0)
	for cone in cones:
		if not slam_cones.has(cone): slam_cones.append(cone)
	
	var msg = RosMirenaCommonEntityList.new()
	msg.header.stamp = _node.now()
	msg.header.frame_id = frame_id  # "car/cog"
	msg.entities = cones.map(func(c): return _to_ent(c))
	_perception_pub.publish(msg)

func _publish_slam():
	var msg = RosMirenaCommonEntityList.new()
	msg.header.stamp = _node.now()
	msg.header.frame_id = "map"
	slam_cones = slam_cones.filter(func(c): return is_instance_valid(c))
	msg.entities = slam_cones.map(func(c): return _to_ent(c, true))
	_slam_pub.publish(msg)

func _publish_control():
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
	var smooth_spd: float = 7.5
	if dynamics_controller != null and dynamics_controller.config != null and dynamics_controller.config.steering_config != null:
		smooth_spd = dynamics_controller.config.steering_config.manual_steer_smooth_speed
	_steer_smoothed = _smooth_steer(_steer_smoothed, steer_input, delta, smooth_spd)
	
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

func _apply_vehicle_physics(delta: float) -> void:
	if physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS and dynamics_controller != null:
		var ebs_triggered: bool = (pilot == PilotMode.NO_PILOT) or (self.brake > 0.5)
		var steer_cmd_rad: float = self.steering
		# Ensure VehicleBody3D native properties are zeroed while Modular VehicleDynamics is sole force authority
		self.engine_force = 0.0
		dynamics_controller.step_from_host(delta, self.gas, steer_cmd_rad, ebs_triggered, self.origin)
		return

	_apply_legacy_vehicle_physics(delta)

func _apply_legacy_vehicle_physics(_delta: float) -> void:
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

	var fx = minf(throttle * max_fx_motor,
		POWER_LIM * DRIVETRAIN_EFF / maxf(u, 1.0))

	var fl_w: VehicleWheel3D = _fl_wheel_node if _fl_wheel_node != null else get_node_or_null("FL_WHEEL") as VehicleWheel3D
	var fr_w: VehicleWheel3D = _fr_wheel_node if _fr_wheel_node != null else get_node_or_null("FR_WHEEL") as VehicleWheel3D
	var rl_w: VehicleWheel3D = _rl_wheel_node if _rl_wheel_node != null else get_node_or_null("RL_WHEEL") as VehicleWheel3D
	var rr_w: VehicleWheel3D = _rr_wheel_node if _rr_wheel_node != null else get_node_or_null("RR_WHEEL") as VehicleWheel3D

	if rl_w != null:
		rl_w.engine_force = fx / 2.0
	if rr_w != null:
		rr_w.engine_force = fx / 2.0

	var ebs = min(brake, 1.0) * BRAKE_F
	if fl_w != null:
		fl_w.brake = front_share * braking + ebs
	if fr_w != null:
		fr_w.brake = front_share * braking + ebs
	if rl_w != null:
		rl_w.brake = rear_share * 0.5 * braking + ebs
	if rr_w != null:
		rr_w.brake = rear_share * 0.5 * braking + ebs

func _smooth_steer(current: float, target: float, delta: float, speed: float) -> float:
	var diff = target - current
	if absf(diff) < 1e-4:
		return clampf(target, -1.0, 1.0)
	var t = clampf(absf(diff), 0.0, 1.0)
	var _ease = maxf(t * t * (3.0 - 2.0 * t), 0.35)
	current += signf(diff) * _ease * speed * delta
	if signf(target - current) != signf(diff):
		current = target
	return clampf(current, -1.0, 1.0)

# --- Interface & Utility ---

func set_origin(transform, reset_vel: bool = false) -> void:
	origin = transform
	if reset_vel:
		linear_velocity = Vector3.ZERO
		angular_velocity = Vector3.ZERO
		_has_prev_linear_vel = false
		_kin_accel_body_lpf = Vector3.ZERO
	await get_tree().process_frame
	set_deferred("global_transform", origin)

func reset_position() -> void:
	var sim_autoload: Node = get_node_or_null("/root/Sim")
	if sim_autoload != null and sim_autoload.get("track") != null:
		set_origin(sim_autoload.get("track").origin, true)
	else:
		set_origin(origin, true)
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
	if has_node("INS") and $INS.has_method("reset"):
		$INS.reset()
	if dynamics_controller != null:
		dynamics_controller.reset_vehicle(origin)

func cone_collision_set(enable: bool) -> void:
	self.collision_layer = (self.collision_layer & ~2) | (2 * int(enable))
	self.collision_mask = (self.collision_mask & ~2) | (2 * int(enable))
	
func get_cones_in_sight(max_dist: float = 10.0) -> Array:
	var visible_cones: Array = []
	if not has_node("Camera"):
		return []
	var camera = $Camera._camera
	if not camera: return []
	for cone in get_tree().get_nodes_in_group("Cones"):
		if global_position.distance_to(cone.global_position) < max_dist:
			if camera.is_position_in_frustum(cone.global_position):
				visible_cones.append(cone)
	return visible_cones

# --- Modular VehicleDynamics Integration & Single-Authority Mode Switching ---

func _setup_modular_vehicle_dynamics() -> void:
	_fl_wheel_node = get_node_or_null("FL_WHEEL") as VehicleWheel3D
	_fr_wheel_node = get_node_or_null("FR_WHEEL") as VehicleWheel3D
	_rl_wheel_node = get_node_or_null("RL_WHEEL") as VehicleWheel3D
	_rr_wheel_node = get_node_or_null("RR_WHEEL") as VehicleWheel3D

	_wheel_rest_transforms.clear()
	for w in [_fl_wheel_node, _fr_wheel_node, _rl_wheel_node, _rr_wheel_node]:
		_wheel_rest_transforms.append(w.transform if w != null else Transform3D.IDENTITY)

	_modular_wheel_holder = Node3D.new()
	_modular_wheel_holder.name = "ModularWheelHolder"
	add_child(_modular_wheel_holder)

	dynamics_controller = VehicleDynamicsController.new()
	dynamics_controller.name = "VehicleDynamicsController"
	dynamics_controller.managed_by_host_script = true
	add_child(dynamics_controller)
	if dynamics_controller.model == null:
		dynamics_controller._ready()

	var explicit_wheels: Array[Node3D] = [
		_fl_wheel_node as Node3D,
		_fr_wheel_node as Node3D,
		_rl_wheel_node as Node3D,
		_rr_wheel_node as Node3D,
	]
	dynamics_controller.bind_to_host_vehicle(self, explicit_wheels)
	dynamics_controller.logger.logging_enabled = true

	# Apply configured initial physics engine mode with strict single-authority enforcement
	set_physics_engine_mode(physics_engine_mode)

## Switch between `LEGACY_GODOT_VEHICLE_BODY` (0) and `MODULAR_VEHICLE_DYNAMICS` (1) at runtime.
## Guarantees strict mutual exclusion:
## - In `MODULAR_VEHICLE_DYNAMICS`, the 4 `VehicleWheel3D` nodes are reparented under `ModularWheelHolder`
##   so `VehicleBody3D` has 0 direct `VehicleWheel3D` children and applies ZERO native wheel/suspension forces.
## - In `LEGACY_GODOT_VEHICLE_BODY`, the 4 `VehicleWheel3D` nodes are reparented back as direct children of `self`
##   at their exact rest transforms, and `VehicleDynamicsController` is set to `INACTIVE_LEGACY_PASSTHROUGH`.
func set_physics_engine_mode(mode: int) -> void:
	physics_engine_mode = PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS if mode == 1 else PhysicsEngineMode.LEGACY_GODOT_VEHICLE_BODY
	var wheel_list: Array[VehicleWheel3D] = [_fl_wheel_node, _fr_wheel_node, _rl_wheel_node, _rr_wheel_node]

	if physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS:
		self.engine_force = 0.0
		self.brake = 0.0
		for i in range(wheel_list.size()):
			var w: VehicleWheel3D = wheel_list[i]
			if w == null:
				continue
			w.engine_force = 0.0
			w.brake = 0.0
			w.steering = 0.0
			if w.get_parent() != _modular_wheel_holder and _modular_wheel_holder != null:
				w.owner = null
				w.reparent(_modular_wheel_holder, false)
		if dynamics_controller != null and dynamics_controller.config != null:
			dynamics_controller.config.integration_mode = VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D
			GodotPhysicsAdapter.configure_rigid_body(self, dynamics_controller.config.vehicle_parameters)
			if dynamics_controller.model != null and is_inside_tree():
				GodotPhysicsAdapter.sync_state_from_body(self, dynamics_controller.model.state, dynamics_controller.model.contact_query)
	else:
		if dynamics_controller != null and dynamics_controller.config != null:
			dynamics_controller.config.integration_mode = VehicleDynamicsTypes.IntegrationMode.INACTIVE_LEGACY_PASSTHROUGH
		self.mass = MASS_KG
		for i in range(wheel_list.size()):
			var w: VehicleWheel3D = wheel_list[i]
			if w == null:
				continue
			if w.get_parent() != self:
				w.owner = null
				w.reparent(self, false)
			if i < _wheel_rest_transforms.size():
				w.transform = _wheel_rest_transforms[i]

	_hud_corr_loaded = false
	_hud_corr_summary.clear()
	_refresh_dynamics_hud_overlay()

## Return true if any `VehicleWheel3D` is currently a direct child of `self` (`VehicleBody3D`).
## Used by tests and diagnostics to verify single-authority mutual exclusion.
func has_legacy_direct_wheel_children() -> bool:
	for child in get_children():
		if child is VehicleWheel3D:
			return true
	return false

## Select the active tire model implementation inside `VehicleDynamicsController`.
func set_active_tire_model_type(model_type: int) -> void:
	if dynamics_controller == null or dynamics_controller.config == null or dynamics_controller.model == null:
		return
	dynamics_controller.config.tire_config.model_type = model_type as VehicleDynamicsTypes.TireModelType
	dynamics_controller.model.tire_model.set_model_type(dynamics_controller.config.tire_config.model_type)
	dynamics_controller.model.tire_model.reset_all(dynamics_controller.model.state.wheels, dynamics_controller.config.tire_config)
	_refresh_dynamics_hud_overlay()

## Open or toggle the dedicated resizable `VehicleDynamicsDashboardWindow`.
func toggle_dynamics_dashboard_window() -> VehicleDynamicsDashboardWindow:
	if dashboard_window == null or not is_instance_valid(dashboard_window):
		return open_dynamics_dashboard_window()
	if dashboard_window.visible:
		dashboard_window.hide()
	else:
		dashboard_window.show()
		dashboard_window.grab_focus()
	return dashboard_window

func open_dynamics_dashboard_window() -> VehicleDynamicsDashboardWindow:
	if dashboard_window == null or not is_instance_valid(dashboard_window):
		dashboard_window = VehicleDynamicsDashboardWindow.new()
		dashboard_window.name = "VehicleDynamicsDashboardWindow"
		dashboard_window.target_car = self
		get_tree().root.add_child(dashboard_window)
		if dashboard_window._mode_select == null:
			dashboard_window._ready()
	dashboard_window.target_car = self
	dashboard_window.show()
	dashboard_window.grab_focus()
	return dashboard_window

## Start open-loop control replay from a loaded `TelemetryRecording` (`control.gas`, `control.steer_rad`).
func start_telemetry_control_replay(rec: TelemetryRecording, start_at_onset: bool = true) -> void:
	if rec == null or not rec.has_usable_channel("control.gas"):
		return
	_replay_recording = rec
	var trng: Vector2 = rec.get_time_range_s()
	_replay_cursor_s = rec.maneuver_onset_time_s if start_at_onset else trng.x
	_replay_end_s = trng.y
	is_replaying_controls = true

func stop_telemetry_control_replay() -> void:
	is_replaying_controls = false
	_replay_recording = null

func _process_telemetry_replay_pilot(delta: float) -> void:
	if _replay_recording == null or _replay_cursor_s > _replay_end_s:
		stop_telemetry_control_replay()
		_process_no_pilot()
		return
	var gas_ch: TelemetryChannel = _replay_recording.get_channel("control.gas")
	var steer_ch: TelemetryChannel = _replay_recording.get_channel("control.steer_rad")
	self.gas = clampf(gas_ch.sample_linear(_replay_cursor_s) if gas_ch != null else 0.0, -1.0, 1.0)
	var st_val: float = steer_ch.sample_linear(_replay_cursor_s) if steer_ch != null else 0.0
	self.steering = clampf(st_val, -MAX_STEER, MAX_STEER)
	self.brake = 0.0
	_replay_cursor_s += delta

## Export buffered 30-column `DynamicsLogger` CSV to `path`.
func save_dynamics_csv_log(path: String) -> bool:
	if dynamics_controller == null or dynamics_controller.logger == null:
		return false
	if dynamics_controller.logger.get_sample_count() == 0:
		return false
	return dynamics_controller.logger.export_csv(path)

## Return a comprehensive dictionary of live vehicle-dynamics quantities for `VehicleDynamicsDashboardWindow`.
func get_live_dynamics_snapshot() -> Dictionary:
	var is_mod: bool = (physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS)
	var mode_name: String = "MODULAR 6-DOF (VehicleDynamics)" if is_mod else "LEGACY (Godot VehicleBody3D)"
	var tire_name: String = "Legacy VehicleWheel3D Raycast"
	if is_mod and dynamics_controller != null and dynamics_controller.model != null:
		tire_name = dynamics_controller.model.tire_model.get_active_model_name()

	if is_mod and dynamics_controller != null and dynamics_controller.model != null:
		var st: VehicleState = dynamics_controller.model.state
		var fl: WheelState = st.wheels[VehicleDynamicsTypes.WheelCorner.FL]
		var fr: WheelState = st.wheels[VehicleDynamicsTypes.WheelCorner.FR]
		var rl: WheelState = st.wheels[VehicleDynamicsTypes.WheelCorner.RL]
		var rr: WheelState = st.wheels[VehicleDynamicsTypes.WheelCorner.RR]
		var contacts: int = 0
		for w in st.wheels:
			if w.contact_state.in_contact:
				contacts += 1
		var beta_rad: float = atan2(st.v_mps, maxf(absf(st.u_mps), 0.25)) if (absf(st.u_mps) > 0.05 or absf(st.v_mps) > 0.05) else 0.0
		var fy_front_veh: float = fl.force_vehicle.y + fr.force_vehicle.y
		var fy_rear_veh: float = rl.force_vehicle.y + rr.force_vehicle.y
		var rear_fy_reserve: float = rl.tire_state.lateral_force_reserve_n + rr.tire_state.lateral_force_reserve_n
		var ack_diff_deg: float = rad_to_deg(absf(fl.steering_angle_rad) - absf(fr.steering_angle_rad))
		return {
			"physics_mode": int(physics_engine_mode),
			"physics_mode_name": mode_name,
			"tire_model_name": tire_name,
			"modular_sole_authority": not has_legacy_direct_wheel_children(),
			"is_legacy": false,
			"is_ay_estimated": false,
			"is_ax_available": true,
			"is_az_available": true,
			"imu.ax": st.ax_mps2,
			"imu.ay": st.ay_mps2,
			"imu.az": st.az_mps2,
			"imu.gx": st.p_rad_s,
			"imu.gy": st.q_rad_s,
			"imu.gz": st.r_rad_s,
			"chassis.u": st.u_mps,
			"chassis.v": st.v_mps,
			"chassis.r": st.r_rad_s,
			"chassis.x": st.x_odom_m,
			"chassis.y": st.y_odom_m,
			"chassis.psi": st.psi_odom_rad,
			"chassis.roll_deg": rad_to_deg(st.roll_rad),
			"chassis.pitch_deg": rad_to_deg(st.pitch_rad),
			"chassis.beta_rad": beta_rad,
			"chassis.beta_deg": rad_to_deg(beta_rad),
			"chassis.fy_net_n": st.total_force_vehicle.y,
			"chassis.fx_net_n": st.total_force_vehicle.x,
			"chassis.mz_net_nm": st.total_moment_vehicle.z,
			"axle.fy_front_n": fy_front_veh,
			"axle.fy_rear_n": fy_rear_veh,
			"axle.fy_reserve_rear_n": rear_fy_reserve,
			"control.gas": self.gas,
			"control.steer_rad": self.steering,
			"steer.fl_rad": fl.steering_angle_rad,
			"steer.fr_rad": fr.steering_angle_rad,
			"steer.rl_rad": rl.steering_angle_rad,
			"steer.rr_rad": rr.steering_angle_rad,
			"steer.ackermann_diff_deg": ack_diff_deg,
			"wheel.fl_rpm": fl.get_rpm(),
			"wheel.fr_rpm": fr.get_rpm(),
			"wheel.rl_rpm": rl.get_rpm(),
			"wheel.rr_rpm": rr.get_rpm(),
			"tire.alpha_fl": fl.slip_angle_alpha_rad,
			"tire.alpha_fr": fr.slip_angle_alpha_rad,
			"tire.alpha_rl": rl.slip_angle_alpha_rad,
			"tire.alpha_rr": rr.slip_angle_alpha_rad,
			"tire.alpha_dot_fl": fl.tire_state.slip_angle_rate_rad_s,
			"tire.alpha_dot_fr": fr.tire_state.slip_angle_rate_rad_s,
			"tire.alpha_dot_rl": rl.tire_state.slip_angle_rate_rad_s,
			"tire.alpha_dot_rr": rr.tire_state.slip_angle_rate_rad_s,
			"tire.camber_setup_fl": fl.camber_angle_rad,
			"tire.camber_setup_fr": fr.camber_angle_rad,
			"tire.camber_setup_rl": rl.camber_angle_rad,
			"tire.camber_setup_rr": rr.camber_angle_rad,
			"tire.gamma_fl": fl.tire_state.iso_camber_gamma_rad,
			"tire.gamma_fr": fr.tire_state.iso_camber_gamma_rad,
			"tire.gamma_rl": rl.tire_state.iso_camber_gamma_rad,
			"tire.gamma_rr": rr.tire_state.iso_camber_gamma_rad,
			"tire.kappa_fl": fl.slip_ratio_kappa,
			"tire.kappa_fr": fr.slip_ratio_kappa,
			"tire.kappa_rl": rl.slip_ratio_kappa,
			"tire.kappa_rr": rr.slip_ratio_kappa,
			"tire.static_fy_fl": fl.tire_state.static_camber_fy_n,
			"tire.static_fy_fr": fr.tire_state.static_camber_fy_n,
			"tire.static_fy_rl": rl.tire_state.static_camber_fy_n,
			"tire.static_fy_rr": rr.tire_state.static_camber_fy_n,
			"tire.fy0_fl": fl.tire_state.fy_pure_mf62_n,
			"tire.fy0_fr": fr.tire_state.fy_pure_mf62_n,
			"tire.fy0_rl": rl.tire_state.fy_pure_mf62_n,
			"tire.fy0_rr": rr.tire_state.fy_pure_mf62_n,
			"tire.fy_fl": fl.tire_forces.fy_n,
			"tire.fy_fr": fr.tire_forces.fy_n,
			"tire.fy_rl": rl.tire_forces.fy_n,
			"tire.fy_rr": rr.tire_forces.fy_n,
			"tire.fx_fl": fl.tire_forces.fx_n,
			"tire.fx_fr": fr.tire_forces.fx_n,
			"tire.fx_rl": rl.tire_forces.fx_n,
			"tire.fx_rr": rr.tire_forces.fx_n,
			"tire.mz_fl": fl.tire_forces.mz_nm,
			"tire.mz_fr": fr.tire_forces.mz_nm,
			"tire.mz_rl": rl.tire_forces.mz_nm,
			"tire.mz_rr": rr.tire_forces.mz_nm,
			"tire.pinn_dfy_fl": fl.tire_state.delta_fy_pinn_n,
			"tire.pinn_dfy_fr": fr.tire_state.delta_fy_pinn_n,
			"tire.pinn_dfy_rl": rl.tire_state.delta_fy_pinn_n,
			"tire.pinn_dfy_rr": rr.tire_state.delta_fy_pinn_n,
			"tire.util_fl": fl.tire_state.combined_slip_utilization,
			"tire.util_fr": fr.tire_state.combined_slip_utilization,
			"tire.util_rl": rl.tire_state.combined_slip_utilization,
			"tire.util_rr": rr.tire_state.combined_slip_utilization,
			"tire.gyx_fl": fl.tire_state.combined_slip_factor_gyx,
			"tire.gyx_fr": fr.tire_state.combined_slip_factor_gyx,
			"tire.gyx_rl": rl.tire_state.combined_slip_factor_gyx,
			"tire.gyx_rr": rr.tire_state.combined_slip_factor_gyx,
			"tire.reserve_fl": fl.tire_state.lateral_force_reserve_n,
			"tire.reserve_fr": fr.tire_state.lateral_force_reserve_n,
			"tire.reserve_rl": rl.tire_state.lateral_force_reserve_n,
			"tire.reserve_rr": rr.tire_state.lateral_force_reserve_n,
			"tire.fz_fl": fl.suspension_state.normal_load_fz_n,
			"tire.fz_fr": fr.suspension_state.normal_load_fz_n,
			"tire.fz_rl": rl.suspension_state.normal_load_fz_n,
			"tire.fz_rr": rr.suspension_state.normal_load_fz_n,
			"susp.heave_fl": fl.suspension_state.compression_m,
			"susp.heave_fr": fr.suspension_state.compression_m,
			"susp.heave_rl": rl.suspension_state.compression_m,
			"susp.heave_rr": rr.suspension_state.compression_m,
			"susp.f_spring_fl": fl.suspension_state.spring_force_n,
			"susp.f_damper_fl": fl.suspension_state.damper_force_n,
			"susp.f_arb_fl": fl.suspension_state.arb_force_n,
			"powertrain.drive_torque_rl": rl.drive_torque_nm,
			"powertrain.drive_torque_rr": rr.drive_torque_nm,
			"aero.drag_n": st.aero_drag_force_n,
			"aero.downforce_front_n": st.aero_downforce_front_n,
			"aero.downforce_rear_n": st.aero_downforce_rear_n,
			"health.dt_ms": 1000.0 / float(maxi(1, Engine.physics_ticks_per_second)),
			"health.contact_count": contacts,
			"health.fault_flags": st.fault_flags,
		}

	# Legacy fallback snapshot when running LEGACY_GODOT_VEHICLE_BODY
	var bvel: Vector3 = _body_velocity()
	var u_leg: float = -bvel.z
	var v_leg: float = -bvel.x
	var r_leg: float = angular_velocity.y
	var odom_tf: Transform3D = origin.inverse() * global_transform
	var rl_rpm_leg: float = -_rl_wheel_node.get_rpm() if _rl_wheel_node != null else 0.0
	var rr_rpm_leg: float = -_rr_wheel_node.get_rpm() if _rr_wheel_node != null else 0.0
	var beta_leg_deg: float = rad_to_deg(atan2(v_leg, maxf(absf(u_leg), 0.25))) if (absf(u_leg) > 0.05 or absf(v_leg) > 0.05) else 0.0
	return {
		"physics_mode": int(physics_engine_mode),
		"physics_mode_name": mode_name,
		"tire_model_name": tire_name,
		"modular_sole_authority": false,
		"is_legacy": true,
		"is_ay_estimated": false,
		"is_ax_available": true,
		"is_az_available": true,
		"imu.ax": _kin_accel_body_lpf.x,
		"imu.ay": _kin_accel_body_lpf.y,
		"imu.az": _kin_accel_body_lpf.z,
		"kinematic.ay_steady_state": u_leg * r_leg,
		"imu.gx": angular_velocity.z,
		"imu.gy": angular_velocity.x,
		"imu.gz": r_leg,
		"chassis.u": u_leg,
		"chassis.v": v_leg,
		"chassis.r": r_leg,
		"chassis.x": -odom_tf.origin.z,
		"chassis.y": -odom_tf.origin.x,
		"chassis.psi": odom_tf.basis.get_euler().y,
		"chassis.roll_deg": rad_to_deg(odom_tf.basis.get_euler().z),
		"chassis.pitch_deg": rad_to_deg(odom_tf.basis.get_euler().x),
		"chassis.beta_deg": beta_leg_deg,
		"chassis.beta_rad": deg_to_rad(beta_leg_deg),
		"control.gas": self.gas,
		"control.steer_rad": self.steering,
		"wheel.rl_rpm": rl_rpm_leg,
		"wheel.rr_rpm": rr_rpm_leg,
		"health.dt_ms": 1000.0 / float(maxi(1, Engine.physics_ticks_per_second)),
		"health.contact_count": 4,
		"health.fault_flags": 0,
	}

func _setup_dynamics_hud_overlay() -> void:
	_hud_layer = CanvasLayer.new()
	_hud_layer.name = "VehicleDynamicsHudLayer"
	_hud_layer.layer = 5
	add_child(_hud_layer)

	_hud_status_panel = PanelContainer.new()
	_hud_status_panel.anchor_left = 0.5
	_hud_status_panel.anchor_right = 0.5
	_hud_status_panel.offset_left = -380
	_hud_status_panel.offset_right = 380
	_hud_status_panel.offset_top = 8
	_hud_status_panel.offset_bottom = 62

	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.06, 0.14, 0.11, 0.88)
	style.border_color = Color(0.24, 0.85, 0.52, 0.95)
	style.set_border_width_all(1)
	style.set_corner_radius_all(6)
	style.content_margin_left = 12
	style.content_margin_right = 12
	style.content_margin_top = 6
	style.content_margin_bottom = 6
	_hud_status_panel.add_theme_stylebox_override("panel", style)
	_hud_layer.add_child(_hud_status_panel)

	var hbox := HBoxContainer.new()
	hbox.add_theme_constant_override("separation", 12)
	_hud_status_panel.add_child(hbox)

	_hud_status_label = Label.new()
	_hud_status_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_hud_status_label.add_theme_font_size_override("font_size", 12)
	_hud_status_label.add_theme_color_override("font_color", Color(0.92, 0.97, 1.0))
	hbox.add_child(_hud_status_label)

	var switch_btn := Button.new()
	switch_btn.text = "Switch Mode (F6)"
	switch_btn.pressed.connect(func():
		var next_m: int = PhysicsEngineMode.LEGACY_GODOT_VEHICLE_BODY \
			if physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS \
			else PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS
		set_physics_engine_mode(next_m)
	)
	hbox.add_child(switch_btn)

	var dash_btn := Button.new()
	dash_btn.text = "Diagnostics & Correlation (F7)"
	dash_btn.pressed.connect(func(): toggle_dynamics_dashboard_window())
	hbox.add_child(dash_btn)

	_refresh_dynamics_hud_overlay()

func _refresh_dynamics_hud_overlay() -> void:
	if _hud_status_label == null or _hud_status_panel == null:
		return
	var snap: Dictionary = get_live_dynamics_snapshot()
	var is_mod: bool = (physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS)
	var style := _hud_status_panel.get_theme_stylebox("panel") as StyleBoxFlat
	if style != null:
		if is_mod:
			style.bg_color = Color(0.06, 0.16, 0.12, 0.88)
			style.border_color = Color(0.25, 0.88, 0.55, 0.95)
		else:
			style.bg_color = Color(0.18, 0.12, 0.06, 0.88)
			style.border_color = Color(0.95, 0.65, 0.22, 0.95)

	var u_mps: float = float(snap.get("chassis.u", 0.0))
	var ay_mps2: float = float(snap.get("imu.ay", 0.0))
	var r_rads: float = float(snap.get("chassis.r", 0.0))
	var corr_info: Dictionary = get_hud_correlation_summary()
	var corr_str: String = str(corr_info.get("hud_text", "CORR: N/A"))
	if is_mod:
		var fz_rl: float = float(snap.get("tire.fz_rl", 0.0))
		var fy_fl: float = float(snap.get("tire.fy_fl", 0.0))
		var beta_deg: float = float(snap.get("chassis.beta_deg", 0.0))
		_hud_status_label.text = "PHYSICS: MODULAR 6-DOF [%s] | %s\nu=%+.1f m/s (%.0f km/h) | ay=%+.1f m/s² | r=%+.2f rad/s | beta=%+.1f° | Fy_FL=%+.0f N | Fz_RL=%.0f N" % [
			str(snap.get("tire_model_name", "TeR_Tire")),
			corr_str,
			u_mps, u_mps * 3.6, ay_mps2, r_rads, beta_deg, fy_fl, fz_rl
		]
	else:
		_hud_status_label.text = "PHYSICS: LEGACY GODOT VehicleBody3D [Raycast Wheels] | %s\nu=%+.1f m/s (%.0f km/h) | ay_kin=%+.1f m/s² | r=%+.2f rad/s | RL_RPM=%.0f" % [
			corr_str,
			u_mps, u_mps * 3.6, ay_mps2, r_rads, float(snap.get("wheel.rl_rpm", 0.0))
		]

## Explicitly configure or clear the reference/comparison telemetry recordings used by the HUD correlation indicator.
func set_correlation_reference_data(ref_rec: TelemetryRecording, sim_rec: TelemetryRecording, context_mode: String = "LAST") -> void:
	_hud_corr_loaded = true
	_hud_corr_ref_rec = ref_rec
	_hud_corr_sim_rec = sim_rec
	if dashboard_window != null and is_instance_valid(dashboard_window):
		dashboard_window.real_reference_recording = ref_rec
		dashboard_window.sim_comparison_recording = sim_rec
		dashboard_window._recompute_correlation()
	if ref_rec == null:
		_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation({}, "N/A")
		return
	if sim_rec == null:
		_hud_corr_summary = {
			"state": "STANDBY",
			"correlation_pct": NAN,
			"hud_text": "CORR: STANDBY",
			"hud_color": Color(0.45, 0.74, 0.96),
			"details": "Reference data loaded — awaiting simulation comparison run.",
		}
		return
	var metrics: Dictionary = CorrelationEngine.evaluate_all_channels(ref_rec, sim_rec, 20.0, 95.0, 0.05, 0.0)
	_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation(metrics, context_mode)

## Return the persistent correlation indicator dictionary (`hud_text`, `hud_color`, `state`, `correlation_pct`, `details`)
## for display in the main simulator HUD (`Stats` panel) and top overlay.
## Uses the exact same `CorrelationEngine` as the `F7` dashboard window and updates at low rate without per-frame overhead.
func get_hud_correlation_summary() -> Dictionary:
	var is_modular: bool = (physics_engine_mode == PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS)
	var mode_label: String = "MODULAR 6-DOF: TeR_Tire MF6.2" if is_modular else "LEGACY RAYCAST: Godot Wheel"
	var mode_color: Color = Color(0.20, 0.85, 0.45) if is_modular else Color(0.95, 0.72, 0.20)
	var tire_label: String = "TeR_Tire (MF6.2 + PINN)" if is_modular else "Pacejka Raycast"
	var ref_name: String = "TeR26 VN-300 Real"
	var sim_name: String = "modular_calibrated" if is_modular else "front10_rear12"
	var win_label: String = "tau: 20.0 - 281.0 s"

	# 1. If the F7 dashboard window is instantiated, synchronize directly with its active correlation state
	if dashboard_window != null and is_instance_valid(dashboard_window):
		if dashboard_window.real_reference_recording == null:
			var na_sum := CorrelationEngine.compute_aggregate_vehicle_correlation({}, "N/A")
			na_sum["physics_mode_label"] = mode_label
			na_sum["physics_mode_color"] = mode_color
			na_sum["tire_model_label"] = tire_label
			na_sum["ref_dataset_name"] = "None"
			na_sum["sim_dataset_name"] = "None"
			na_sum["eval_window_label"] = "N/A"
			na_sum["channel_metrics"] = {}
			return na_sum
		if (is_replaying_controls or dashboard_window.is_recording_live) and dashboard_window.live_session_recording != null:
			var live_rec: TelemetryRecording = dashboard_window.live_session_recording
			var ax_ch: TelemetryChannel = live_rec.get_channel("imu.ax")
			if ax_ch == null or ax_ch.get_sample_count() < CorrelationEngine.MIN_AGGREGATE_SAMPLES:
				return {
					"state": "STANDBY",
					"correlation_pct": NAN,
					"overall_score": NAN,
					"hud_text": "CORR: STANDBY",
					"hud_color": Color(0.45, 0.74, 0.96),
					"physics_mode_label": mode_label,
					"physics_mode_color": mode_color,
					"tire_model_label": tire_label,
					"ref_dataset_name": ref_name,
					"sim_dataset_name": "live_session",
					"eval_window_label": "acquiring...",
					"channel_metrics": {},
					"details": "Live recording/replay started — collecting initial samples (%d/%d)." % [
						ax_ch.get_sample_count() if ax_ch != null else 0,
						CorrelationEngine.MIN_AGGREGATE_SAMPLES
					],
				}
			if _hud_corr_eval_accum >= 0.50 or _hud_corr_summary.is_empty():
				_hud_corr_eval_accum = 0.0
				live_rec.detect_maneuver_onset()
				var trng: Vector2 = live_rec.get_time_range_s()
				var t_end: float = maxf(2.0, trng.y - live_rec.maneuver_onset_time_s)
				var live_metrics: Dictionary = CorrelationEngine.evaluate_all_channels(
					dashboard_window.real_reference_recording, live_rec, 0.0, t_end, 0.05, 0.0
				)
				_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation(live_metrics, "LIVE")
				_hud_corr_summary["physics_mode_label"] = mode_label
				_hud_corr_summary["physics_mode_color"] = mode_color
				_hud_corr_summary["tire_model_label"] = tire_label
				_hud_corr_summary["ref_dataset_name"] = ref_name
				_hud_corr_summary["sim_dataset_name"] = "live_session"
				_hud_corr_summary["eval_window_label"] = "tau: 0.0 - %.1f s" % t_end
				_hud_corr_summary["channel_metrics"] = live_metrics
				_hud_corr_summary["overall_score"] = _hud_corr_summary.get("correlation_pct", NAN)
			return _hud_corr_summary
		if not dashboard_window.cached_correlation_metrics.is_empty():
			var mode_tag: String = "LIVE" if is_replaying_controls else "LAST"
			var dash_summary: Dictionary = CorrelationEngine.compute_aggregate_vehicle_correlation(
				dashboard_window.cached_correlation_metrics, mode_tag
			)
			dash_summary["physics_mode_label"] = mode_label
			dash_summary["physics_mode_color"] = mode_color
			dash_summary["tire_model_label"] = tire_label
			dash_summary["ref_dataset_name"] = ref_name
			dash_summary["sim_dataset_name"] = sim_name
			dash_summary["eval_window_label"] = "tau: %.1f - %.1f s" % [dashboard_window.corr_tau_start_s, dashboard_window.corr_tau_end_s]
			dash_summary["channel_metrics"] = dashboard_window.cached_correlation_metrics
			dash_summary["overall_score"] = dash_summary.get("correlation_pct", NAN)
			return dash_summary

	# 2. Check if reference data was explicitly cleared
	if _hud_corr_loaded and _hud_corr_ref_rec == null and not _hud_corr_summary.is_empty() and _hud_corr_summary.get("state") == "N/A":
		var na_res: Dictionary = _hud_corr_summary.duplicate()
		na_res["physics_mode_label"] = mode_label
		na_res["physics_mode_color"] = mode_color
		na_res["tire_model_label"] = tire_label
		na_res["ref_dataset_name"] = "None"
		na_res["sim_dataset_name"] = "None"
		na_res["eval_window_label"] = "N/A"
		na_res["channel_metrics"] = {}
		na_res["overall_score"] = NAN
		return na_res

	# 3. Dynamic loading based on current physics mode
	if not _hud_corr_loaded or _hud_corr_summary.is_empty() or _hud_corr_active_mode != int(physics_engine_mode):
		_hud_corr_loaded = true
		_hud_corr_active_mode = int(physics_engine_mode)
		var imu_dir: String = ProjectSettings.globalize_path("res://").get_base_dir().get_base_dir().path_join("imu_compare")
		if not DirAccess.dir_exists_absolute(imu_dir):
			imu_dir = "/home/alexr/eRacing/mirena_ws-SIM/imu_compare"
		
		var json_prefix: String = "modular_calibrated" if is_modular else "grip10"
		var json_path: String = imu_dir.path_join("reports/correlation_%s.json" % json_prefix)
		if not FileAccess.file_exists(json_path) and is_modular:
			json_path = imu_dir.path_join("reports/correlation_modular_baseline.json")
			json_prefix = "modular_baseline"

		var loaded_from_json: bool = false
		if FileAccess.file_exists(json_path):
			var json_str: String = FileAccess.get_file_as_string(json_path)
			json_str = json_str.replace(": NaN", ": null").replace(": -NaN", ": null").replace(": Infinity", ": null").replace(": -Infinity", ": null")
			var parsed = JSON.parse_string(json_str)
			if parsed is Dictionary and parsed.has("full_drive_20_281"):
				var win_data: Dictionary = parsed["full_drive_20_281"]
				var score: float = float(win_data.get("aggregate_score_pct", 0.0))
				var chs: Dictionary = win_data.get("channels", {})
				_hud_corr_summary = {
					"state": "LAST",
					"correlation_pct": score,
					"overall_score": score,
					"hud_text": "CORR: %.1f%% (LAST)" % score,
					"hud_color": Color(0.42, 0.88, 0.62) if score >= 50.0 else Color(0.96, 0.68, 0.20),
					"details": "Full drive benchmark [20..281s] | %s" % json_prefix,
					"physics_mode_label": mode_label,
					"physics_mode_color": mode_color,
					"tire_model_label": tire_label,
					"ref_dataset_name": ref_name,
					"sim_dataset_name": json_prefix,
					"eval_window_label": "tau: 20.0 - 281.0 s",
					"channel_metrics": chs,
				}
				loaded_from_json = true

		if not loaded_from_json and DirAccess.dir_exists_absolute(imu_dir):
			_hud_corr_ref_rec = TelemetryImporter.load_imu_compare_real_bundle(imu_dir, 25000)
			var sim_tag: String = "modular_calibrated" if is_modular else "front10_rear12"
			_hud_corr_sim_rec = TelemetryImporter.load_imu_compare_sim_bundle(imu_dir, sim_tag, 25000)
			if _hud_corr_ref_rec != null and _hud_corr_sim_rec != null:
				var init_metrics: Dictionary = CorrelationEngine.evaluate_all_channels(
					_hud_corr_ref_rec, _hud_corr_sim_rec, 20.0, 95.0, 0.05, 0.0
				)
				_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation(init_metrics, "LAST")
				_hud_corr_summary["channel_metrics"] = init_metrics
				_hud_corr_summary["sim_dataset_name"] = sim_tag
			else:
				_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation({}, "N/A")
		elif not loaded_from_json:
			_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation({}, "N/A")

	if _hud_corr_summary.is_empty():
		_hud_corr_summary = CorrelationEngine.compute_aggregate_vehicle_correlation({}, "N/A")

	_hud_corr_summary["physics_mode_label"] = mode_label
	_hud_corr_summary["physics_mode_color"] = mode_color
	_hud_corr_summary["tire_model_label"] = tire_label
	if not _hud_corr_summary.has("ref_dataset_name"):
		_hud_corr_summary["ref_dataset_name"] = ref_name
	if not _hud_corr_summary.has("sim_dataset_name"):
		_hud_corr_summary["sim_dataset_name"] = sim_name
	if not _hud_corr_summary.has("eval_window_label"):
		_hud_corr_summary["eval_window_label"] = win_label
	if not _hud_corr_summary.has("channel_metrics"):
		_hud_corr_summary["channel_metrics"] = {}
	_hud_corr_summary["overall_score"] = _hud_corr_summary.get("correlation_pct", NAN)
	return _hud_corr_summary
