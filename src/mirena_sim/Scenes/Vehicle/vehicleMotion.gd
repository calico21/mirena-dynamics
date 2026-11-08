extends VehicleBody3D
class_name MirenaCar

signal cone_hit(count: int)
enum PilotMode { NO_PILOT, MANUAL, ROS, TRACK_RAIL }

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

var initial_gate_crossings: int = 0
var lap_start_ticks: float = 0.0

@onready var car_state : RosMirenaCommonCar = RosMirenaCommonCar.new()
@onready var wheel_speeds : RosMirenaCommonWheelSpeeds = RosMirenaCommonWheelSpeeds.new()
@onready var lap_status_msg : RosMirenaCommonLapStatus = RosMirenaCommonLapStatus.new()

func _ready():
	# Enable collision contact reporting
	
	cone_collision_set(true)

	# Add to vehicle group so gates can detect us
	add_to_group("vehicle")
	contact_monitor = true
	max_contacts_reported = 10
	body_entered.connect(_on_body_entered)

	# Enable cone collisions by default
	

	# Connect to InitialGate's vehicle_passed signal for lap timing
	_connect_to_initial_gate()

	## ROS — no namespace: all topic names below are absolute
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

	# Camera Registration
	Sim.register_camera($TPCam)
	Sim.register_camera($FPCam)

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
		

func _connect_to_initial_gate():
	var gate = get_tree().get_first_node_in_group("InitialGate")
	if gate:
		gate.vehicle_passed.connect(_on_initial_gate_passed)
	else:
		# Gate not ready yet, try again next frame
		call_deferred("_connect_to_initial_gate")

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
	_publish_perception()
	_publish_slam()
	_publish_control()

func _on_as_status(msg):
	_as_info.status = msg.status
	_as_info.mission = msg.mission

func _publish_car_state():
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
