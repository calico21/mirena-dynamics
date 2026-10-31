extends RefCounted
class_name WheelState

## Complete mutable state for an individual wheel corner (FL, FR, RL, RR).
##
## Sign Conventions (ISO 8855 / Vehicle Frame):
## - `steering_angle_rad`: Positive (> 0) for counter-clockwise (left) steer about vehicle +Z (+Y_godot).
## - `camber_angle_rad`: Negative (< 0) when top of wheel leans toward vehicle centerline.
## - `omega_rad_s`: Positive (> 0) for forward vehicle motion.
## - `drive_torque_nm`: Positive (> 0) propels the wheel in the forward direction.
## - `brake_torque_magnitude_nm`: Non-negative (>= 0) friction brake torque capacity available to oppose spin.
## - `signed_brake_torque_nm`: Signed resistive torque opposing current spin or holding against net drive/tire torque.

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

## Total road-wheel steering angle (rack steer + Ackermann + static toe + bump/compliance steer) (rad).
var steering_angle_rad: float = 0.0
## Instantaneous wheel camber angle (rad).
var camber_angle_rad: float = 0.0
## Wheel rotational angular velocity about its spin axis (rad/s, positive = forward roll).
var omega_rad_s: float = 0.0
## Wheel rotational angular acceleration d(omega)/dt (rad/s^2).
var angular_accel_rad_s2: float = 0.0
## Integrated wheel spin angle for visual rendering (rad).
var spin_angle_rad: float = 0.0
## Effective loaded rolling radius R_eff (m).
var effective_radius_m: float = 0.20

## Wheel hub position relative to Chassis CoM in ISO/ROS Vehicle Body Frame (m).
var hub_offset_from_com_vehicle: Vector3 = Vector3.ZERO
## Contact patch position relative to Chassis CoM in ISO/ROS Vehicle Body Frame (m).
var contact_offset_from_com_vehicle: Vector3 = Vector3.ZERO
## Wheel hub linear velocity in ISO/ROS Vehicle Body Frame (m/s).
var hub_velocity_vehicle: Vector3 = Vector3.ZERO
## Wheel contact center linear velocity in Steered Wheel Frame (vx, vy, vz) (m/s).
var contact_velocity_wheel: Vector3 = Vector3.ZERO

## Instantaneous longitudinal slip ratio kappa (dimensionless).
var slip_ratio_kappa: float = 0.0
## Instantaneous lateral slip angle alpha (rad).
var slip_angle_alpha_rad: float = 0.0

## Applied drivetrain shaft torque at this wheel (N*m).
var drive_torque_nm: float = 0.0
## Available friction brake torque magnitude at this wheel (N*m, >= 0).
var brake_torque_magnitude_nm: float = 0.0
## Signed resistive brake torque actually applied to oppose rotation (N*m).
var signed_brake_torque_nm: float = 0.0
## Net torque acting on wheel rotational inertia I_w (N*m).
var net_wheel_torque_nm: float = 0.0

## Tire force vector transformed into the ISO/ROS Vehicle Body Frame (N).
var force_vehicle: Vector3 = Vector3.ZERO
## Total moment contributed by this wheel about the Chassis CoM in the ISO/ROS Vehicle Body Frame (N*m).
var moment_about_com_vehicle: Vector3 = Vector3.ZERO

## Owned sub-states for this corner.
var contact_state: ContactState
var suspension_state: SuspensionState
var tire_state: TireState
var tire_forces: TireForces

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	contact_state = ContactState.new(p_corner)
	suspension_state = SuspensionState.new(p_corner)
	tire_state = TireState.new(p_corner)
	tire_forces = TireForces.new()
	reset()

## Reset all wheel kinematic, dynamic, and sub-component states.
func reset(nominal_radius_m: float = 0.20, static_fz_n: float = 0.0, static_camber_rad: float = 0.0) -> void:
	steering_angle_rad = 0.0
	camber_angle_rad = static_camber_rad
	omega_rad_s = 0.0
	angular_accel_rad_s2 = 0.0
	spin_angle_rad = 0.0
	effective_radius_m = nominal_radius_m
	hub_velocity_vehicle = Vector3.ZERO
	contact_velocity_wheel = Vector3.ZERO
	slip_ratio_kappa = 0.0
	slip_angle_alpha_rad = 0.0
	drive_torque_nm = 0.0
	brake_torque_magnitude_nm = 0.0
	signed_brake_torque_nm = 0.0
	net_wheel_torque_nm = 0.0
	force_vehicle = Vector3.ZERO
	moment_about_com_vehicle = Vector3.ZERO
	contact_state.reset(nominal_radius_m)
	suspension_state.reset(static_fz_n, static_camber_rad)
	tire_state.reset()
	tire_forces.zero()

## Return wheel rotational speed in RPM (positive = forward motion, matching Mirena bus convention).
func get_rpm() -> float:
	return VehicleUnits.rad_s_to_rpm(omega_rad_s)

## Verify that all primary state variables are finite.
func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_scalar(steering_angle_rad)
		and VehicleUnits.is_finite_scalar(camber_angle_rad)
		and VehicleUnits.is_finite_scalar(omega_rad_s)
		and VehicleUnits.is_finite_scalar(angular_accel_rad_s2)
		and VehicleUnits.is_finite_scalar(effective_radius_m)
		and effective_radius_m > 0.0
		and VehicleUnits.is_finite_vec3(hub_velocity_vehicle)
		and VehicleUnits.is_finite_vec3(contact_velocity_wheel)
		and VehicleUnits.is_finite_scalar(slip_ratio_kappa)
		and VehicleUnits.is_finite_scalar(slip_angle_alpha_rad)
		and VehicleUnits.is_finite_vec3(force_vehicle)
		and VehicleUnits.is_finite_vec3(moment_about_com_vehicle)
		and contact_state.is_valid()
		and suspension_state.is_valid()
		and tire_state.is_valid()
		and tire_forces.is_finite()
	)
