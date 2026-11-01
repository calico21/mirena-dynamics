extends RefCounted
class_name WheelKinematics

static func compute_hub_velocity_vehicle(
	v_com_vehicle: Vector3,
	omega_vehicle: Vector3,
	hub_offset_from_com_vehicle: Vector3
) -> Vector3:
	return v_com_vehicle + omega_vehicle.cross(hub_offset_from_com_vehicle)

static func compute_contact_velocity_wheel(
	v_com_vehicle: Vector3,
	omega_vehicle: Vector3,
	hub_offset_from_com_vehicle: Vector3,
	steer_angle_rad: float
) -> Vector3:
	var v_hub: Vector3 = compute_hub_velocity_vehicle(v_com_vehicle, omega_vehicle, hub_offset_from_com_vehicle)
	return VehicleCoordinates.body_to_wheel_frame(v_hub, steer_angle_rad)

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
