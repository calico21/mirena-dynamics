extends Resource
class_name SteeringConfiguration

@export_group("Steering Limits & Actuator")

@export var max_steer_angle_rad: float = 0.35

@export var max_steer_rate_rad_s: float = 0.0

@export var manual_steer_smooth_speed: float = 7.5

@export var steering_zero_offset_rad: float = 0.0

@export_group("Steering Geometry (Ackermann & Toe)")

@export var ackermann_factor: float = 0.60

@export var static_toe_front_rad: float = 0.0015

@export var static_toe_rear_rad: float = -0.0020

@export_group("Bump & Compliance Steer")

@export var bump_steer_front_radpm: float = 0.0
@export var bump_steer_rear_radpm: float = 0.0

@export var compliance_steer_front_radpn: float = 0.0
@export var compliance_steer_rear_radpn: float = 0.0

func is_compliance_steer_supported() -> bool:
	return false

func is_valid() -> bool:
	if not VehicleUnits.is_finite_scalar(max_steer_angle_rad) or max_steer_angle_rad <= 0.0:
		return false
	if max_steer_rate_rad_s < 0.0:
		return false
	return true
