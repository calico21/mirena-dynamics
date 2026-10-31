extends RefCounted
class_name PowertrainModel

## Subsystem facade coordinating `MotorModel` and `DrivetrainModel`.
##
## Calculates available per-wheel drive torque [FL, FR, RL, RR] (N*m) from driver/ROS throttle command
## and wheel angular velocities. Never applies forces directly to the chassis.

## Evaluate per-wheel drive torques [FL, FR, RL, RR] (N*m) and write them into `wheels[i].drive_torque_nm`
## if a valid `wheels` array is provided.
func evaluate_drive_torques(
	throttle_cmd: float,
	u_mps: float,
	wheel_omegas_rad_s: Array[float],
	config: PowertrainConfiguration,
	wheels: Array[WheelState] = []
) -> Array[float]:
	if config == null or not config.is_valid():
		return [0.0, 0.0, 0.0, 0.0]

	var mean_driven_omega: float = 0.0
	if wheel_omegas_rad_s.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		mean_driven_omega = 0.5 * (
			wheel_omegas_rad_s[VehicleDynamicsTypes.WheelCorner.RL]
			+ wheel_omegas_rad_s[VehicleDynamicsTypes.WheelCorner.RR]
		)
	var motor_omega_rad_s: float = mean_driven_omega * config.gear_ratio

	var motor_trq_nm: float = MotorModel.compute_motor_torque_nm(
		throttle_cmd, u_mps, motor_omega_rad_s, config
	)
	var wheel_torques: Array[float] = DrivetrainModel.distribute_wheel_torques_nm(
		motor_trq_nm, wheel_omegas_rad_s, config
	)

	# Regulate excess wheelspin torque on driven wheels when a valid `wheels` array is provided,
	# preventing unloaded/airborne wheel runaway spin while allowing full tire traction up to peak slip (~0.12..0.15).
	if wheels.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			var w: WheelState = wheels[i]
			if wheel_torques[i] > 0.0 and w.contact_state.in_contact:
				var vx_w: float = maxf(w.contact_velocity_wheel.x, 1.0)
				var v_roll: float = w.omega_rad_s * w.effective_radius_m
				# Allow natural tire traction up to peak slip ratio (~0.14) without throttling normal corner exits
				var target_slip_vel: float = maxf(0.14 * vx_w, 0.75)
				var slip_vel_excess: float = (v_roll - vx_w) - target_slip_vel
				if slip_vel_excess > 0.0:
					var tc_scale: float = 1.0 / (1.0 + 0.80 * slip_vel_excess)
					wheel_torques[i] *= clampf(tc_scale, 0.35, 1.0)
				# Smoothly reduce torque only if tire normal load drops near zero (wheel liftoff prevention)
				if w.suspension_state.normal_load_fz_n < 80.0:
					wheel_torques[i] *= clampf(w.suspension_state.normal_load_fz_n / 80.0, 0.0, 1.0)
			elif wheel_torques[i] > 0.0 and not w.contact_state.in_contact:
				wheel_torques[i] = 0.0
			w.drive_torque_nm = wheel_torques[i]

	return wheel_torques
