extends RefCounted
class_name DrivetrainModel

static func distribute_wheel_torques_nm(
	motor_torque_nm: float,
	wheel_omegas_rad_s: Array[float],
	config: PowertrainConfiguration
) -> Array[float]:
	if config == null or not VehicleUnits.is_finite_scalar(motor_torque_nm):
		return [0.0, 0.0, 0.0, 0.0]

	var total_wheel_torque_nm: float = motor_torque_nm * config.gear_ratio * config.drivetrain_efficiency

	var front_bias: float = 0.0
	if config.architecture == VehicleDynamicsTypes.DrivetrainArchitecture.AWD_HUB_MOTORS:
		front_bias = clampf(config.drive_bias_front, 0.0, 1.0)

	var front_axle_trq: float = total_wheel_torque_nm * front_bias
	var rear_axle_trq: float = total_wheel_torque_nm * (1.0 - front_bias)

	var fl_trq: float = 0.5 * front_axle_trq
	var fr_trq: float = 0.5 * front_axle_trq
	var rl_trq: float = 0.5 * rear_axle_trq
	var rr_trq: float = 0.5 * rear_axle_trq

	if config.diff_lock_ratio > 0.0 and wheel_omegas_rad_s.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		var d_omega_rear: float = wheel_omegas_rad_s[VehicleDynamicsTypes.WheelCorner.RL] - wheel_omegas_rad_s[VehicleDynamicsTypes.WheelCorner.RR]
		var k_visc: float = config.diff_viscous_damping_nm_s_per_rad
		var lock_trq: float = config.diff_lock_ratio * k_visc * d_omega_rear
		rl_trq -= 0.5 * lock_trq
		rr_trq += 0.5 * lock_trq

	return [fl_trq, fr_trq, rl_trq, rr_trq]
