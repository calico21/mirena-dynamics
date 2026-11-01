extends RefCounted
class_name VehicleState

var origin_godot: Transform3D = Transform3D.IDENTITY

var global_transform_godot: Transform3D = Transform3D.IDENTITY

var linear_velocity_world_godot: Vector3 = Vector3.ZERO

var angular_velocity_world_godot: Vector3 = Vector3.ZERO

var x_odom_m: float = 0.0
var y_odom_m: float = 0.0
var psi_odom_rad: float = 0.0

var roll_rad: float = 0.0
var pitch_rad: float = 0.0
var yaw_rad: float = 0.0

var u_mps: float = 0.0
var v_mps: float = 0.0
var w_mps: float = 0.0

var p_rad_s: float = 0.0
var q_rad_s: float = 0.0
var r_rad_s: float = 0.0

var ax_mps2: float = 0.0
var ay_mps2: float = 0.0
var az_mps2: float = 0.0

var gas_cmd: float = 0.0
var throttle_cmd: float = 0.0
var brake_cmd: float = 0.0
var steer_cmd_rad: float = 0.0
var ebs_active: bool = false

var aero_drag_force_n: float = 0.0
var aero_downforce_front_n: float = 0.0
var aero_downforce_rear_n: float = 0.0
var aero_force_vehicle: Vector3 = Vector3.ZERO
var aero_moment_vehicle: Vector3 = Vector3.ZERO

var total_force_vehicle: Vector3 = Vector3.ZERO
var total_moment_vehicle: Vector3 = Vector3.ZERO

var total_force_world_godot: Vector3 = Vector3.ZERO
var total_moment_world_godot: Vector3 = Vector3.ZERO

var wheels: Array[WheelState] = []

var brake_state: BrakeState = BrakeState.new()

var sim_time_s: float = 0.0

var fault_flags: int = VehicleDynamicsTypes.FaultFlag.NONE

func _init() -> void:
	wheels.clear()
	for corner in VehicleDynamicsTypes.ALL_CORNERS:
		wheels.append(WheelState.new(corner))
	reset()

func reset(new_origin_godot: Transform3D = Transform3D.IDENTITY, params: VehicleParameters = null) -> void:
	origin_godot = new_origin_godot
	global_transform_godot = new_origin_godot
	linear_velocity_world_godot = Vector3.ZERO
	angular_velocity_world_godot = Vector3.ZERO

	x_odom_m = 0.0
	y_odom_m = 0.0
	psi_odom_rad = 0.0

	roll_rad = 0.0
	pitch_rad = 0.0
	yaw_rad = 0.0

	u_mps = 0.0
	v_mps = 0.0
	w_mps = 0.0

	p_rad_s = 0.0
	q_rad_s = 0.0
	r_rad_s = 0.0

	ax_mps2 = 0.0
	ay_mps2 = 0.0
	az_mps2 = 0.0

	gas_cmd = 0.0
	throttle_cmd = 0.0
	brake_cmd = 0.0
	steer_cmd_rad = 0.0
	ebs_active = false

	aero_drag_force_n = 0.0
	aero_downforce_front_n = 0.0
	aero_downforce_rear_n = 0.0
	aero_force_vehicle = Vector3.ZERO
	aero_moment_vehicle = Vector3.ZERO

	total_force_vehicle = Vector3.ZERO
	total_moment_vehicle = Vector3.ZERO
	total_force_world_godot = Vector3.ZERO
	total_moment_world_godot = Vector3.ZERO

	var radius_m: float = params.wheel_unloaded_radius_m if params != null else 0.20
	var wb_m: float = params.get_wheelbase_m() if params != null else 1.593
	var total_weight_n: float = (params.total_mass_kg * params.gravity_mps2) if params != null else (300.0 * 9.81)
	var front_corner_fz: float = 0.5 * total_weight_n * ((params.lr_m / wb_m) if params != null else 0.45)
	var rear_corner_fz: float = 0.5 * total_weight_n * ((params.lf_m / wb_m) if params != null else 0.55)

	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.ALL_CORNERS[i]
		var static_fz: float = front_corner_fz if VehicleDynamicsTypes.is_front_corner(corner) else rear_corner_fz
		wheels[i].reset(radius_m, static_fz, 0.0)
		if params != null:
			wheels[i].hub_offset_from_com_vehicle = params.get_wheel_offset_from_com_vehicle(corner)
			wheels[i].contact_offset_from_com_vehicle = wheels[i].hub_offset_from_com_vehicle - Vector3(0.0, 0.0, radius_m)

	brake_state.reset()
	sim_time_s = 0.0
	fault_flags = VehicleDynamicsTypes.FaultFlag.NONE

func get_body_velocity_vehicle() -> Vector3:
	return Vector3(u_mps, v_mps, w_mps)

func get_body_angular_velocity_vehicle() -> Vector3:
	return Vector3(p_rad_s, q_rad_s, r_rad_s)

func is_valid() -> bool:
	if not VehicleUnits.is_finite_transform(global_transform_godot):
		return false
	if not VehicleUnits.is_finite_vec3(linear_velocity_world_godot):
		return false
	if not VehicleUnits.is_finite_vec3(angular_velocity_world_godot):
		return false
	if not VehicleUnits.is_finite_scalar(u_mps) or not VehicleUnits.is_finite_scalar(v_mps) or not VehicleUnits.is_finite_scalar(w_mps):
		return false
	if not VehicleUnits.is_finite_scalar(p_rad_s) or not VehicleUnits.is_finite_scalar(q_rad_s) or not VehicleUnits.is_finite_scalar(r_rad_s):
		return false
	if not VehicleUnits.is_finite_vec3(total_force_vehicle) or not VehicleUnits.is_finite_vec3(total_moment_vehicle):
		return false
	if not brake_state.is_valid():
		return false
	for w in wheels:
		if not w.is_valid():
			return false
	return true
