extends RefCounted
class_name VehicleInertia

## Utilities for computing and validating the vehicle's inertia tensor and mass distribution.
##
## Operates in the ISO 8855 / ROS 2 Vehicle Body Frame (+X forward, +Y left, +Z up)
## with conversions to the Godot Chassis-Local Frame (-Z forward, -X left, +Y up).

## Return the principal inertia diagonal (Ixx, Iyy, Izz) in the ISO/ROS Vehicle Body Frame (kg*m^2).
static func get_inertia_vehicle(params: VehicleParameters) -> Vector3:
	if params == null:
		return Vector3(45.0, 85.0, 150.0)
	return Vector3(params.ixx_kgm2, params.iyy_kgm2, params.izz_kgm2)

## Return the principal inertia diagonal in the Godot Chassis-Local Frame (kg*m^2).
## Since Godot X = -Vehicle Y (pitch axis), Godot Y = +Vehicle Z (yaw axis), Godot Z = -Vehicle X (roll axis):
##   I_godot = (Iyy_veh, Izz_veh, Ixx_veh)
static func get_inertia_godot(params: VehicleParameters) -> Vector3:
	if params == null:
		return Vector3(85.0, 150.0, 45.0)
	return Vector3(params.iyy_kgm2, params.izz_kgm2, params.ixx_kgm2)

## Compute static vertical normal loads [FL, FR, RL, RR] (N) at zero pitch/roll on level ground.
static func compute_static_corner_loads_n(params: VehicleParameters) -> Array[float]:
	if params == null or not params.is_valid():
		return [0.0, 0.0, 0.0, 0.0]
	var total_weight_n: float = params.total_mass_kg * params.gravity_mps2
	var wb_m: float = maxf(params.get_wheelbase_m(), VehicleUnits.EPSILON_SMALL)
	var front_axle_load_n: float = total_weight_n * (params.lr_m / wb_m)
	var rear_axle_load_n: float = total_weight_n * (params.lf_m / wb_m)
	var fl: float = 0.5 * front_axle_load_n
	var fr: float = 0.5 * front_axle_load_n
	var rl: float = 0.5 * rear_axle_load_n
	var rr: float = 0.5 * rear_axle_load_n
	return [fl, fr, rl, rr]

## Compute gyroscopic cross-term omega x (I * omega) in the Vehicle Body Frame (N*m).
static func compute_gyroscopic_moment_vehicle(omega_veh: Vector3, inertia_diag_veh: Vector3) -> Vector3:
	var i_omega: Vector3 = Vector3(
		inertia_diag_veh.x * omega_veh.x,
		inertia_diag_veh.y * omega_veh.y,
		inertia_diag_veh.z * omega_veh.z
	)
	return omega_veh.cross(i_omega)
