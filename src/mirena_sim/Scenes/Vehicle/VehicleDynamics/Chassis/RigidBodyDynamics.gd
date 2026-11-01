extends RefCounted
class_name RigidBodyDynamics

static func assemble_wheel_force_and_moment_vehicle(
	force_wheel: Vector3,
	moment_wheel: Vector3,
	steer_angle_rad: float,
	application_offset_from_com_vehicle: Vector3
) -> Dictionary:
	var force_body: Vector3 = VehicleCoordinates.wheel_to_body_frame(force_wheel, steer_angle_rad)
	var intrinsic_moment_body: Vector3 = VehicleCoordinates.wheel_to_body_frame(moment_wheel, steer_angle_rad)
	var lever_moment_body: Vector3 = application_offset_from_com_vehicle.cross(force_body)
	return {
		"force_vehicle": force_body,
		"moment_vehicle": lever_moment_body + intrinsic_moment_body,
	}

static func compute_linear_acceleration_vehicle(
	net_force_vehicle: Vector3,
	mass_kg: float
) -> Vector3:
	if mass_kg <= 0.0 or not VehicleUnits.is_finite_vec3(net_force_vehicle):
		return Vector3.ZERO
	return net_force_vehicle / mass_kg

static func compute_angular_acceleration_vehicle(
	net_moment_vehicle: Vector3,
	omega_vehicle: Vector3,
	inertia_diag_vehicle: Vector3
) -> Vector3:
	if not VehicleUnits.is_finite_vec3(net_moment_vehicle) or not VehicleUnits.is_finite_vec3(omega_vehicle):
		return Vector3.ZERO
	if inertia_diag_vehicle.x <= 0.0 or inertia_diag_vehicle.y <= 0.0 or inertia_diag_vehicle.z <= 0.0:
		return Vector3.ZERO
	var gyro: Vector3 = VehicleInertia.compute_gyroscopic_moment_vehicle(omega_vehicle, inertia_diag_vehicle)
	var rhs: Vector3 = net_moment_vehicle - gyro
	return Vector3(
		rhs.x / inertia_diag_vehicle.x,
		rhs.y / inertia_diag_vehicle.y,
		rhs.z / inertia_diag_vehicle.z
	)

static func integrate_standalone_step(
	state: VehicleState,
	params: VehicleParameters,
	dt_s: float,
	constrain_planar: bool = false
) -> void:
	if state == null or params == null or dt_s <= 0.0:
		return
	var m: float = maxf(params.total_mass_kg, VehicleUnits.EPSILON_SMALL)
	var gravity_world: Vector3 = Vector3(0.0, -params.gravity_mps2, 0.0)
	var accel_world: Vector3 = (state.total_force_world_godot / m) + gravity_world

	if constrain_planar:
		accel_world.y = 0.0

	state.linear_velocity_world_godot += accel_world * dt_s
	if constrain_planar:
		state.linear_velocity_world_godot.y = 0.0

	state.global_transform_godot.origin += state.linear_velocity_world_godot * dt_s

	var omega_veh: Vector3 = state.get_body_angular_velocity_vehicle()
	var inertia_veh: Vector3 = VehicleInertia.get_inertia_vehicle(params)
	var alpha_veh: Vector3 = compute_angular_acceleration_vehicle(
		state.total_moment_vehicle, omega_veh, inertia_veh
	)
	if constrain_planar:
		alpha_veh.x = 0.0
		alpha_veh.y = 0.0

	var next_omega_veh: Vector3 = omega_veh + alpha_veh * dt_s
	if constrain_planar:
		next_omega_veh.x = 0.0
		next_omega_veh.y = 0.0

	var next_omega_local_godot: Vector3 = VehicleCoordinates.vehicle_to_godot_angular(next_omega_veh)
	state.angular_velocity_world_godot = state.global_transform_godot.basis * next_omega_local_godot

	var omega_norm: float = state.angular_velocity_world_godot.length()
	if omega_norm > VehicleUnits.EPSILON_STRICT:
		var axis: Vector3 = state.angular_velocity_world_godot / omega_norm
		var d_rot: Basis = Basis(axis, omega_norm * dt_s)
		state.global_transform_godot.basis = (d_rot * state.global_transform_godot.basis).orthonormalized()
