extends RefCounted
class_name VehicleCoordinates

static func godot_to_vehicle_vec(v_godot: Vector3) -> Vector3:
	return Vector3(-v_godot.z, -v_godot.x, v_godot.y)

static func vehicle_to_godot_vec(v_veh: Vector3) -> Vector3:
	return Vector3(-v_veh.y, v_veh.z, -v_veh.x)

static func godot_to_vehicle_angular(w_godot: Vector3) -> Vector3:
	return Vector3(-w_godot.z, -w_godot.x, w_godot.y)

static func vehicle_to_godot_angular(w_veh: Vector3) -> Vector3:
	return Vector3(-w_veh.y, w_veh.z, -w_veh.x)

static func world_to_chassis_godot(v_world: Vector3, chassis_global_basis: Basis) -> Vector3:
	return chassis_global_basis.inverse() * v_world

static func chassis_to_world_godot(v_local_godot: Vector3, chassis_global_basis: Basis) -> Vector3:
	return chassis_global_basis * v_local_godot

static func world_godot_to_vehicle_body(v_world: Vector3, chassis_global_basis: Basis) -> Vector3:
	var local_godot: Vector3 = chassis_global_basis.inverse() * v_world
	return godot_to_vehicle_vec(local_godot)

static func vehicle_body_to_world_godot(v_veh_body: Vector3, chassis_global_basis: Basis) -> Vector3:
	var local_godot: Vector3 = vehicle_to_godot_vec(v_veh_body)
	return chassis_global_basis * local_godot

static func basis_godot_to_vehicle_rpy(basis_godot: Basis) -> Vector3:
	var euler_yxz: Vector3 = basis_godot.get_euler(EulerOrder.EULER_ORDER_YXZ)
	var roll_rad: float = -euler_yxz.z
	var pitch_rad: float = -euler_yxz.x
	var yaw_rad: float = euler_yxz.y
	return Vector3(roll_rad, pitch_rad, yaw_rad)

static func vehicle_rpy_to_basis_godot(rpy_veh: Vector3) -> Basis:
	return Basis.from_euler(Vector3(-rpy_veh.y, rpy_veh.z, -rpy_veh.x), EulerOrder.EULER_ORDER_YXZ)

static func body_to_wheel_frame(v_body: Vector3, steer_angle_rad: float) -> Vector3:
	var c: float = cos(steer_angle_rad)
	var s: float = sin(steer_angle_rad)
	return Vector3(
		v_body.x * c + v_body.y * s,
		-v_body.x * s + v_body.y * c,
		v_body.z
	)

static func wheel_to_body_frame(v_wheel: Vector3, steer_angle_rad: float) -> Vector3:
	var c: float = cos(steer_angle_rad)
	var s: float = sin(steer_angle_rad)
	return Vector3(
		v_wheel.x * c - v_wheel.y * s,
		v_wheel.x * s + v_wheel.y * c,
		v_wheel.z
	)

static func compute_ros_odom_pose(origin_godot: Transform3D, global_transform_godot: Transform3D) -> Vector3:
	var odom_tf: Transform3D = origin_godot.affine_inverse() * global_transform_godot
	var x_odom: float = -odom_tf.origin.z
	var y_odom: float = -odom_tf.origin.x
	var psi_odom: float = odom_tf.basis.get_euler().y
	return Vector3(x_odom, y_odom, psi_odom)
