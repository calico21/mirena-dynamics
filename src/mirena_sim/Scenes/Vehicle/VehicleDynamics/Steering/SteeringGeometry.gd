extends RefCounted
class_name SteeringGeometry

static func compute_wheel_steering_angles(
	rack_steer_rad: float,
	params: VehicleParameters,
	steer_cfg: SteeringConfiguration,
	bump_steer_rad_per_wheel: Array[float] = [0.0, 0.0, 0.0, 0.0]
) -> Array[float]:
	if steer_cfg == null:
		return [rack_steer_rad, rack_steer_rad, 0.0, 0.0]

	var clamped_steer: float = clampf(rack_steer_rad, -steer_cfg.max_steer_angle_rad, steer_cfg.max_steer_angle_rad)
	var fl_steer: float = clamped_steer
	var fr_steer: float = clamped_steer

	if params != null and absf(steer_cfg.ackermann_factor) > VehicleUnits.EPSILON_SMALL and absf(clamped_steer) > VehicleUnits.EPSILON_SMALL:
		var l_m: float = maxf(params.get_wheelbase_m(), 0.1)
		var t_f: float = maxf(params.track_front_m, 0.1)
		var tan_delta: float = tan(clamped_steer)

		var ack_fl: float = atan(l_m * tan_delta / maxf(l_m - 0.5 * t_f * tan_delta, 0.05))
		var ack_fr: float = atan(l_m * tan_delta / maxf(l_m + 0.5 * t_f * tan_delta, 0.05))
		fl_steer = lerpf(clamped_steer, ack_fl, steer_cfg.ackermann_factor)
		fr_steer = lerpf(clamped_steer, ack_fr, steer_cfg.ackermann_factor)

	fl_steer += steer_cfg.static_toe_front_rad
	fr_steer -= steer_cfg.static_toe_front_rad
	var rl_steer: float = steer_cfg.static_toe_rear_rad
	var rr_steer: float = -steer_cfg.static_toe_rear_rad

	if bump_steer_rad_per_wheel.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		fl_steer += bump_steer_rad_per_wheel[VehicleDynamicsTypes.WheelCorner.FL]
		fr_steer += bump_steer_rad_per_wheel[VehicleDynamicsTypes.WheelCorner.FR]
		rl_steer += bump_steer_rad_per_wheel[VehicleDynamicsTypes.WheelCorner.RL]
		rr_steer += bump_steer_rad_per_wheel[VehicleDynamicsTypes.WheelCorner.RR]

	return [fl_steer, fr_steer, rl_steer, rr_steer]
