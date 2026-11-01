extends Resource
class_name SteeringConfiguration

## Configuration resource for the steering subsystem (rack limits, Ackermann geometry, toe, actuator dynamics).
##
## Positive steering angle is counter-clockwise (left turn) about the vehicle +Z (+Y_godot) axis,
## matching `mirena_common/msg/CarControl.msg` and `vehicleMotion.gd`.

@export_group("Steering Limits & Actuator")
## Maximum physical road-wheel steering angle magnitude (rad).
## Verified in vehicleMotion.gd: MAX_STEER = 0.35 rad (20.05 deg).
@export var max_steer_angle_rad: float = 0.35
## Maximum steering actuator rate (rad/s). 0.0 disables rate limiting (instantaneous ROS command tracking).
@export var max_steer_rate_rad_s: float = 0.0
## Manual pilot steering smoothing speed factor (responsive direct steering feel).
@export var manual_steer_smooth_speed: float = 7.5
## Physical steering sensor zero-point calibration offset (rad).
## Subtracts static sensor bias from CAN steer command: delta_phys = delta_cmd - steering_zero_offset_rad.
@export var steering_zero_offset_rad: float = 0.0

@export_group("Steering Geometry (Ackermann & Toe)")
## Ackermann geometry factor: 0.0 = parallel steering, 1.0 = 100% true Ackermann, -1.0 = reverse Ackermann.
## Default 0.60 (60% pro-Ackermann) increases inner front wheel turn-in angle in tight Formula Student corners.
@export var ackermann_factor: float = 0.60
## Front static toe angle per wheel (rad). Negative = toe-in (front of tires point inward), Positive = toe-out.
## Slight front toe-out (+0.0015 rad = +0.086 deg) sharpens initial turn-in response.
@export var static_toe_front_rad: float = 0.0015
## Rear static toe angle per wheel (rad). Negative = toe-in, Positive = toe-out.
## Slight rear toe-in (-0.0020 rad = -0.115 deg) stabilizes the rear axle under lateral load transfer.
@export var static_toe_rear_rad: float = -0.0020

@export_group("Bump & Compliance Steer")
## Linear bump steer coefficient (rad of steer per m of suspension compression).
@export var bump_steer_front_radpm: float = 0.0
@export var bump_steer_rear_radpm: float = 0.0
## [UNSUPPORTED / INACTIVE] Lateral force compliance steer coefficient (rad of steer per N of lateral force Fy).
## AUDIT STATUS (Phase IV Priority 2 - Option B): INACTIVE in physics solver.
## Physical investigation confirmed that no steering arm geometry, kingpin inclination, tie-rod axial stiffness,
## rack bushing deflection, or upright K&C bench measurements exist for TeR26.
## Retained strictly for .tres resource schema backward-compatibility; not consumed in SteeringModel.gd.
@export var compliance_steer_front_radpn: float = 0.0
@export var compliance_steer_rear_radpn: float = 0.0

## Check whether elastokinematic compliance steer is supported by available vehicle physics data.
func is_compliance_steer_supported() -> bool:
	return false

## Validate steering configuration.
func is_valid() -> bool:
	if not VehicleUnits.is_finite_scalar(max_steer_angle_rad) or max_steer_angle_rad <= 0.0:
		return false
	if max_steer_rate_rad_s < 0.0:
		return false
	return true
