extends RefCounted
class_name BrakeState

var brake_command_norm: float = 0.0
var ebs_active: bool = false

var available_torque_nm: Array[float] = [0.0, 0.0, 0.0, 0.0]

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
