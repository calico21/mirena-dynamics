extends RefCounted
class_name MotorModel

static func compute_motor_torque_nm(
	throttle_cmd: float,
	u_mps: float,
	motor_omega_rad_s: float,
	config: PowertrainConfiguration
) -> float:
	if config == null or not VehicleUnits.is_finite_scalar(throttle_cmd) \
			or not VehicleUnits.is_finite_scalar(u_mps) \
			or not VehicleUnits.is_finite_scalar(motor_omega_rad_s):
		return 0.0

	var clamped_cmd: float = clampf(throttle_cmd, -1.0 if config.allow_reverse else 0.0, 1.0)
	if absf(clamped_cmd) <= VehicleUnits.EPSILON_STRICT:
		return 0.0

	var motor_rpm: float = absf(VehicleUnits.rad_s_to_rpm(motor_omega_rad_s))
	if motor_rpm >= config.motor_max_rpm and clamped_cmd > 0.0:
		return 0.0

	var requested_motor_trq_nm: float = clamped_cmd * config.motor_peak_torque_nm

	var eff_speed_mps: float = maxf(absf(u_mps), config.power_limit_min_speed_mps)
	var equiv_omega_from_u: float = (eff_speed_mps / 0.20) * config.gear_ratio
	var denom_omega: float = maxf(absf(motor_omega_rad_s), maxf(equiv_omega_from_u, 1.0))
	var max_power_torque_nm: float = config.power_limit_w / denom_omega

	if requested_motor_trq_nm >= 0.0:
		return minf(requested_motor_trq_nm, max_power_torque_nm)
	else:
		return maxf(requested_motor_trq_nm, -max_power_torque_nm)
