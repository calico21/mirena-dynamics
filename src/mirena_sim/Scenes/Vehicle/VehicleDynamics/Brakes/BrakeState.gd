extends RefCounted
class_name BrakeState

## Mutable state for the 4-wheel hydraulic brake and Emergency Braking System (EBS).
##
## Sign convention:
## - `available_torque_nm`: Non-negative magnitude (>= 0, in N*m) of maximum friction brake torque
##   available at each wheel for the current brake command.
## - `applied_torque_nm`: Signed resistive brake torque (N*m) applied to the wheel rotational dynamics
##   after accounting for wheel spin direction and standstill stiction.

var brake_command_norm: float = 0.0
var ebs_active: bool = false
## Per-wheel available brake torque magnitude [FL, FR, RL, RR] (N*m, >= 0).
var available_torque_nm: Array[float] = [0.0, 0.0, 0.0, 0.0]
## Per-wheel signed resistive brake torque [FL, FR, RL, RR] (N*m).
var applied_torque_nm: Array[float] = [0.0, 0.0, 0.0, 0.0]

func _init() -> void:
	reset()

func reset() -> void:
	brake_command_norm = 0.0
	ebs_active = false
	available_torque_nm = [0.0, 0.0, 0.0, 0.0]
	applied_torque_nm = [0.0, 0.0, 0.0, 0.0]

func is_valid() -> bool:
	if not VehicleUnits.is_finite_scalar(brake_command_norm) or brake_command_norm < 0.0:
		return false
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		if not VehicleUnits.is_finite_scalar(available_torque_nm[i]) or available_torque_nm[i] < 0.0:
			return false
		if not VehicleUnits.is_finite_scalar(applied_torque_nm[i]):
			return false
	return true
