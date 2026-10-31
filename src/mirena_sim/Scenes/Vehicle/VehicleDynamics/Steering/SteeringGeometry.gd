extends RefCounted
class_name SteeringGeometry

## Pure geometric calculations for individual wheel steering angles [FL, FR, RL, RR].
##
## Sign Convention (ISO 8855 / ROS 2 Vehicle Body Frame):
## - Positive `rack_steer_rad` (> 0): Left turn (counter-clockwise about +Z_vehicle / +Y_godot).
## - Static toe (`static_toe_rad`): Negative (< 0) = Toe-In (front of wheel points toward vehicle centerline).
##   Therefore:
##   - On Left wheels (FL, RL), pointing inward (to the right) is a negative yaw rotation (`+static_toe_rad`).
##   - On Right wheels (FR, RR), pointing inward (to the left) is a positive yaw rotation (`-static_toe_rad`).
## - Ackermann geometry (`ackermann_factor > 0`): In a left turn (`rack_steer_rad > 0`), the inside wheel (FL)
##   steers at a larger angle than the outside wheel (FR): `|delta_FL| > |delta_FR|`.

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

	# Blend between parallel steering (ackermann_factor = 0) and exact Ackermann geometry (ackermann_factor = 1)
	if params != null and absf(steer_cfg.ackermann_factor) > VehicleUnits.EPSILON_SMALL and absf(clamped_steer) > VehicleUnits.EPSILON_SMALL:
		var l_m: float = maxf(params.get_wheelbase_m(), 0.1)
		var t_f: float = maxf(params.track_front_m, 0.1)
		var tan_delta: float = tan(clamped_steer)
		# True Ackermann angles for left (+delta) or right (-delta) turn:
		# cot(delta_FL) = cot(delta) - t_f / (2 * L)
		# cot(delta_FR) = cot(delta) + t_f / (2 * L)
		var ack_fl: float = atan(l_m * tan_delta / maxf(l_m - 0.5 * t_f * tan_delta, 0.05))
		var ack_fr: float = atan(l_m * tan_delta / maxf(l_m + 0.5 * t_f * tan_delta, 0.05))
		fl_steer = lerpf(clamped_steer, ack_fl, steer_cfg.ackermann_factor)
		fr_steer = lerpf(clamped_steer, ack_fr, steer_cfg.ackermann_factor)

	# Static toe (negative static_toe = toe-in: FL rotates right (-), FR rotates left (+))
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
