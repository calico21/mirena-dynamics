extends RefCounted
class_name RigidBodyDynamics

## 6-DOF Newton-Euler rigid-body equations of motion and force/moment assembly for the vehicle chassis.
##
## Coordinate Frame:
## - Force and moment assembly is performed in the ISO 8855 / ROS 2 Vehicle Body Frame (`car/cog`:
##   +X forward, +Y left, +Z up) and then transformed consistently to the Godot World Frame
##   via `VehicleCoordinates`.
## - Never mixes body-frame and world-frame vectors.

## Transform a per-wheel force and moment from the Steered Wheel Frame into the Vehicle Body Frame
## and compute the induced moment about the Chassis Center of Mass (CoM):
##   F_body = R_z(delta) * F_wheel
##   M_body_about_com = r_application_from_com x F_body + R_z(delta) * M_wheel
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

## Compute linear acceleration (a_x, a_y, a_z) in the Vehicle Body Frame (m/s^2) from net external force:
##   a_inertial_in_body = F_total_vehicle / m
##   du/dt = F_x / m + v * r - w * q
##   dv/dt = F_y / m + w * p - u * r
##   dw/dt = F_z / m + u * q - v * p
static func compute_linear_acceleration_vehicle(
	net_force_vehicle: Vector3,
	mass_kg: float
) -> Vector3:
	if mass_kg <= 0.0 or not VehicleUnits.is_finite_vec3(net_force_vehicle):
		return Vector3.ZERO
	return net_force_vehicle / mass_kg

## Compute angular acceleration (dp/dt, dq/dt, dr/dt) in the Vehicle Body Frame (rad/s^2)
## using Euler's rigid-body equation:
##   I * d(omega)/dt = M_total_vehicle - omega x (I * omega)
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

## Advance the chassis state by `dt_s` in standalone benchmark/test mode (when not driven by Godot's RigidBody3D solver).
## Includes gravity (acting along -Y_godot in world frame) so vertical suspension equilibrium can be tested standalone.
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
