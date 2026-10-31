extends RefCounted
class_name VehicleCoordinates

## Coordinate frame definitions and exact transformations for MirenaSim VehicleDynamics.
##
## VERIFIED CONVENTIONS AGAINST ACTIVE CODE (vehicleMotion.gd, track.gd, ros_ins.gd):
##
## 1. Godot Chassis-Local Frame (Godot 3D right-handed Y-up):
##    - Forward: -Z  (Vector3(0, 0, -1)) -> Front wheels at z = -0.993 m, rear at z = +0.600 m
##    - Left:    -X  (Vector3(-1, 0, 0)) -> FL wheel at x = -0.587 m, FR wheel at x = +0.587 m
##    - Right:   +X  (Vector3(1, 0, 0))
##    - Up:      +Y  (Vector3(0, 1, 0))
##
## 2. Vehicle Physical / ISO 8855 / ROS 2 REP-103 Body Frame ("car/cog"):
##    - Forward (+x_veh): -Z_godot  (positive u_mps = forward motion)
##    - Left    (+y_veh): -X_godot  (positive v_mps = leftward motion)
##    - Up      (+z_veh): +Y_godot  (positive w_mps = upward motion)
##
## 3. Rotational Sign Conventions (Right-Hand Rule about Vehicle/ISO/ROS Axes):
##    - Roll  (phi, p):   About +x_veh (-Z_godot). Positive = right side moves down.
##                        p_veh = -omega_godot.z
##    - Pitch (theta, q): About +y_veh (-X_godot). Positive = nose moves down (rear up).
##                        q_veh = -omega_godot.x
##    - Yaw   (psi, r):   About +z_veh (+Y_godot). Positive = counter-clockwise (left turn) viewed from above.
##                        r_veh = +omega_godot.y
##    - Steering (delta): About +z_veh (+Y_godot). Positive = left turn (CCW).

## Transform a position or translational vector (velocity, acceleration, force)
## from Godot local/world axes to ISO 8855 / ROS 2 REP-103 axes:
##   v_ros = (-v_godot.z, -v_godot.x, +v_godot.y)
static func godot_to_vehicle_vec(v_godot: Vector3) -> Vector3:
	return Vector3(-v_godot.z, -v_godot.x, v_godot.y)

## Transform a position or translational vector (velocity, acceleration, force)
## from ISO 8855 / ROS 2 REP-103 axes to Godot local/world axes:
##   v_godot = (-v_ros.y, +v_ros.z, -v_ros.x)
static func vehicle_to_godot_vec(v_veh: Vector3) -> Vector3:
	return Vector3(-v_veh.y, v_veh.z, -v_veh.x)

## Transform an angular velocity or moment/torque vector (pseudovector)
## from Godot local axes to ISO 8855 / ROS 2 REP-103 axes.
## Since the coordinate mapping (-z, -x, +y) has determinant (-1)*(-1)*(+1) = +1
## (a proper right-handed rotation), axial vectors transform with the exact same rule:
##   omega_veh = (-omega_godot.z, -omega_godot.x, +omega_godot.y)
static func godot_to_vehicle_angular(w_godot: Vector3) -> Vector3:
	return Vector3(-w_godot.z, -w_godot.x, w_godot.y)

## Transform an angular velocity or moment/torque vector (pseudovector)
## from ISO 8855 / ROS 2 REP-103 axes to Godot local axes:
##   omega_godot = (-omega_veh.y, +omega_veh.z, -omega_veh.x)
static func vehicle_to_godot_angular(w_veh: Vector3) -> Vector3:
	return Vector3(-w_veh.y, w_veh.z, -w_veh.x)

## Transform a world-frame Godot vector into the chassis-local Godot frame.
static func world_to_chassis_godot(v_world: Vector3, chassis_global_basis: Basis) -> Vector3:
	return chassis_global_basis.inverse() * v_world

## Transform a chassis-local Godot vector into the world-frame Godot frame.
static func chassis_to_world_godot(v_local_godot: Vector3, chassis_global_basis: Basis) -> Vector3:
	return chassis_global_basis * v_local_godot

## Transform a world-frame Godot vector directly into the ISO 8855 / ROS 2 vehicle body frame.
static func world_godot_to_vehicle_body(v_world: Vector3, chassis_global_basis: Basis) -> Vector3:
	var local_godot: Vector3 = chassis_global_basis.inverse() * v_world
	return godot_to_vehicle_vec(local_godot)

## Transform an ISO 8855 / ROS 2 vehicle body vector directly into the Godot world frame.
static func vehicle_body_to_world_godot(v_veh_body: Vector3, chassis_global_basis: Basis) -> Vector3:
	var local_godot: Vector3 = vehicle_to_godot_vec(v_veh_body)
	return chassis_global_basis * local_godot

## Extract [roll, pitch, yaw] in radians in the ISO 8855 / ROS 2 convention
## from a relative Godot Transform3D/Basis (e.g. odom_transform.basis).
## Matches vehicleMotion.gd (psi = basis.get_euler().y) and ros_ins.gd (_noisy_attitude).
static func basis_godot_to_vehicle_rpy(basis_godot: Basis) -> Vector3:
	var euler_yxz: Vector3 = basis_godot.get_euler(EulerOrder.EULER_ORDER_YXZ)
	var roll_rad: float = -euler_yxz.z
	var pitch_rad: float = -euler_yxz.x
	var yaw_rad: float = euler_yxz.y
	return Vector3(roll_rad, pitch_rad, yaw_rad)

## Construct a Godot Basis from ISO 8855 / ROS 2 [roll, pitch, yaw] (in radians).
static func vehicle_rpy_to_basis_godot(rpy_veh: Vector3) -> Basis:
	return Basis.from_euler(Vector3(-rpy_veh.y, rpy_veh.z, -rpy_veh.x), EulerOrder.EULER_ORDER_YXZ)

## Transform a planar vector (x_veh, y_veh, z_veh) from the Vehicle Body Frame
## into the Steered Wheel Frame given steering angle delta_rad (positive = left/CCW about +z_veh).
##
## v_wheel.x = +v_body.x * cos(delta) + v_body.y * sin(delta)
## v_wheel.y = -v_body.x * sin(delta) + v_body.y * cos(delta)
## v_wheel.z =  v_body.z
static func body_to_wheel_frame(v_body: Vector3, steer_angle_rad: float) -> Vector3:
	var c: float = cos(steer_angle_rad)
	var s: float = sin(steer_angle_rad)
	return Vector3(
		v_body.x * c + v_body.y * s,
		-v_body.x * s + v_body.y * c,
		v_body.z
	)

## Transform a vector (fx_wheel, fy_wheel, fz_wheel) from the Steered Wheel Frame
## back into the Vehicle Body Frame given steering angle delta_rad (positive = left/CCW about +z_veh).
##
## v_body.x = v_wheel.x * cos(delta) - v_wheel.y * sin(delta)
## v_body.y = v_wheel.x * sin(delta) + v_wheel.y * cos(delta)
## v_body.z = v_wheel.z
static func wheel_to_body_frame(v_wheel: Vector3, steer_angle_rad: float) -> Vector3:
	var c: float = cos(steer_angle_rad)
	var s: float = sin(steer_angle_rad)
	return Vector3(
		v_wheel.x * c - v_wheel.y * s,
		v_wheel.x * s + v_wheel.y * c,
		v_wheel.z
	)

## Compute the relative odom Transform3D from origin Transform3D and current global Transform3D,
## and return ROS 2 odom pose Vector3(x_odom_m, y_odom_m, psi_odom_rad).
## Matches vehicleMotion.gd lines 271-275 exactly.
static func compute_ros_odom_pose(origin_godot: Transform3D, global_transform_godot: Transform3D) -> Vector3:
	var odom_tf: Transform3D = origin_godot.affine_inverse() * global_transform_godot
	var x_odom: float = -odom_tf.origin.z
	var y_odom: float = -odom_tf.origin.x
	var psi_odom: float = odom_tf.basis.get_euler().y
	return Vector3(x_odom, y_odom, psi_odom)
