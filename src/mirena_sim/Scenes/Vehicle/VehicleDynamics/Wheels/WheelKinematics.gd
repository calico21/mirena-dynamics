extends RefCounted
class_name WheelKinematics

## Pure kinematic equations for wheel hub velocity, contact-patch velocity in the steered wheel frame,
## and low-speed regularized longitudinal slip ratio (`kappa`) and lateral slip angle (`alpha`).
##
## Coordinate Frames & Sign Conventions (ISO 8855 / ROS 2 Vehicle Body Frame):
## - Chassis velocity at CoM: `v_com_veh = (u, v, w)` (+X forward, +Y left, +Z up).
## - Chassis angular velocity: `omega_veh = (p, q, r)` (roll, pitch, yaw rate).
## - Hub velocity in Vehicle Body Frame:
##     `v_hub_veh = v_com_veh + omega_veh x r_hub_from_com_veh`
## - Contact velocity in Steered Wheel Frame (rotated by `steer_angle_rad` about +Z_veh):
##     `v_wheel = VehicleCoordinates.body_to_wheel_frame(v_hub_veh, steer_angle_rad)`
## - Regularized denominator `vx_reg = sqrt(vx^2 + vx_eps^2)` guarantees smooth, finite behavior at `vx = 0`:
##     `slip_ratio_kappa = (omega_w * R_eff - vx) / max(abs(vx), vx_eps)` (or smooth `sqrt(vx^2 + vx_eps^2)`)
##     `slip_angle_alpha_rad = atan2(vy, sqrt(vx^2 + vx_eps^2))`

## Compute the wheel hub velocity in the ISO/ROS Vehicle Body Frame (m/s).
static func compute_hub_velocity_vehicle(
	v_com_vehicle: Vector3,
	omega_vehicle: Vector3,
	hub_offset_from_com_vehicle: Vector3
) -> Vector3:
	return v_com_vehicle + omega_vehicle.cross(hub_offset_from_com_vehicle)

## Compute contact center velocity in the Steered Wheel Frame (vx, vy, vz) (m/s).
static func compute_contact_velocity_wheel(
	v_com_vehicle: Vector3,
	omega_vehicle: Vector3,
	hub_offset_from_com_vehicle: Vector3,
	steer_angle_rad: float
) -> Vector3:
	var v_hub: Vector3 = compute_hub_velocity_vehicle(v_com_vehicle, omega_vehicle, hub_offset_from_com_vehicle)
	return VehicleCoordinates.body_to_wheel_frame(v_hub, steer_angle_rad)

## Compute regularized longitudinal slip ratio `kappa` (dimensionless).
## Positive (> 0) when driving (`omega_w * R_eff > vx`), Negative (< 0) when braking (`omega_w * R_eff < vx`).
static func compute_slip_ratio_kappa(
	vx_wheel_mps: float,
	wheel_omega_rad_s: float,
	effective_radius_m: float,
	vx_eps_mps: float = 0.25
) -> float:
	if not VehicleUnits.is_finite_scalar(vx_wheel_mps) \
			or not VehicleUnits.is_finite_scalar(wheel_omega_rad_s) \
			or not VehicleUnits.is_finite_scalar(effective_radius_m) \
			or effective_radius_m <= 0.0:
		return 0.0
	var eps: float = maxf(vx_eps_mps, 1e-4)
	var v_roll: float = wheel_omega_rad_s * effective_radius_m
	var denom: float = sqrt(vx_wheel_mps * vx_wheel_mps + eps * eps)
	var raw_kappa: float = (v_roll - vx_wheel_mps) / denom
	var speed_ref: float = maxf(absf(vx_wheel_mps), absf(v_roll))
	var t_low: float = clampf(speed_ref / eps, 0.0, 1.0)
	var low_speed_envelope: float = t_low * t_low * (3.0 - 2.0 * t_low)
	return raw_kappa * low_speed_envelope

## Compute regularized lateral slip angle `alpha` (rad).
## Matches dbm_model.py regularization: `atan2(vy_wheel, sqrt(vx_wheel^2 + eps^2))`, with a smooth
## cubic Hermite low-speed regularization envelope below `vx_eps_mps` so standstill micro-velocities
## cannot generate unphysical slip-angle oscillations.
## When `vy_wheel > 0` (wheel sliding to the left), `alpha > 0`, which generates a restoring lateral force `Fy < 0`.
## When the wheel is steered left (`delta > 0`) on a forward-moving vehicle (`vx > 0, vy_body = 0`),
## `vy_wheel = -vx * sin(delta) < 0`, so `alpha < 0`, which generates a positive leftward cornering force `Fy > 0`.
static func compute_slip_angle_alpha_rad(
	vx_wheel_mps: float,
	vy_wheel_mps: float,
	vx_eps_mps: float = 0.25
) -> float:
	if not VehicleUnits.is_finite_scalar(vx_wheel_mps) or not VehicleUnits.is_finite_scalar(vy_wheel_mps):
		return 0.0
	var eps: float = maxf(vx_eps_mps, 1e-4)
	var denom: float = sqrt(vx_wheel_mps * vx_wheel_mps + eps * eps)
	var raw_alpha: float = atan2(vy_wheel_mps, denom)
	var planar_speed: float = sqrt(vx_wheel_mps * vx_wheel_mps + vy_wheel_mps * vy_wheel_mps)
	var t_low: float = clampf(planar_speed / eps, 0.0, 1.0)
	var low_speed_envelope: float = t_low * t_low * (3.0 - 2.0 * t_low)
	return raw_alpha * low_speed_envelope

## Update kinematic velocity and slip variables on a `WheelState` instance.
static func update_wheel_kinematics(
	wheel: WheelState,
	v_com_vehicle: Vector3,
	omega_vehicle: Vector3,
	vx_eps_mps: float = 0.25
) -> void:
	if wheel == null:
		return
	wheel.hub_velocity_vehicle = compute_hub_velocity_vehicle(
		v_com_vehicle, omega_vehicle, wheel.hub_offset_from_com_vehicle
	)
	wheel.contact_velocity_wheel = VehicleCoordinates.body_to_wheel_frame(
		wheel.hub_velocity_vehicle, wheel.steering_angle_rad
	)
	if wheel.contact_state.in_contact:
		wheel.slip_ratio_kappa = compute_slip_ratio_kappa(
			wheel.contact_velocity_wheel.x,
			wheel.omega_rad_s,
			wheel.effective_radius_m,
			vx_eps_mps
		)
		wheel.slip_angle_alpha_rad = compute_slip_angle_alpha_rad(
			wheel.contact_velocity_wheel.x,
			wheel.contact_velocity_wheel.y,
			vx_eps_mps
		)
	else:
		wheel.slip_ratio_kappa = 0.0
		wheel.slip_angle_alpha_rad = 0.0
