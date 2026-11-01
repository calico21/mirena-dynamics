extends RefCounted
class_name SteeringModel

var _current_rack_angle_rad: float = 0.0

func reset() -> void:
	_current_rack_angle_rad = 0.0

func update_steering(
	requested_steer_rad: float,
	dt_s: float,
	wheels: Array[WheelState],
	params: VehicleParameters,
	steer_cfg: SteeringConfiguration
) -> float:
	if steer_cfg == null or not VehicleUnits.is_finite_scalar(requested_steer_rad):
		return _current_rack_angle_rad

	var unoffset_steer: float = requested_steer_rad - steer_cfg.steering_zero_offset_rad
	var target_rad: float = clampf(unoffset_steer, -steer_cfg.max_steer_angle_rad, steer_cfg.max_steer_angle_rad)
	if steer_cfg.max_steer_rate_rad_s > 0.0 and dt_s > 0.0:
		var max_step: float = steer_cfg.max_steer_rate_rad_s * dt_s
		_current_rack_angle_rad = move_toward(_current_rack_angle_rad, target_rad, max_step)
	else:
		_current_rack_angle_rad = target_rad

	var bump_steers: Array[float] = [0.0, 0.0, 0.0, 0.0]
	if wheels.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			bump_steers[i] = wheels[i].suspension_state.bump_steer_rad

	var angles: Array[float] = SteeringGeometry.compute_wheel_steering_angles(
		_current_rack_angle_rad, params, steer_cfg, bump_steers
	)
	if wheels.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			wheels[i].steering_angle_rad = angles[i]

	return _current_rack_angle_rad
