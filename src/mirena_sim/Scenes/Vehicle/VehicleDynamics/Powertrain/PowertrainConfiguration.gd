extends Resource
class_name PowertrainConfiguration

@export_group("Architecture")

@export var architecture: VehicleDynamicsTypes.DrivetrainArchitecture = VehicleDynamicsTypes.DrivetrainArchitecture.RWD_SINGLE_MOTOR

@export var allow_reverse: bool = false

@export var regen_enabled: bool = false

@export_group("Motor & Power Limits")

@export var motor_peak_torque_nm: float = 100.0

@export var power_limit_w: float = 80000.0

@export var motor_max_rpm: float = 6000.0

@export var power_limit_min_speed_mps: float = 1.0

@export_group("Transmission & Differential")

@export var gear_ratio: float = 5.0

@export var drivetrain_efficiency: float = 0.92

@export var drive_bias_front: float = 0.0

@export var diff_lock_ratio: float = 0.65

@export var diff_viscous_damping_nm_s_per_rad: float = 40.0

@export var diff_preload_viscous_nm_s_rad: float:
	get:
		return diff_viscous_damping_nm_s_per_rad
	set(val):
		diff_viscous_damping_nm_s_per_rad = val

func get_peak_wheel_torque_nm() -> float:
	return motor_peak_torque_nm * gear_ratio * drivetrain_efficiency

func is_valid() -> bool:
	if motor_peak_torque_nm < 0.0 or power_limit_w <= 0.0 or motor_max_rpm <= 0.0:
		return false
	if gear_ratio <= 0.0 or drivetrain_efficiency <= 0.0 or drivetrain_efficiency > 1.0:
		return false
	if drive_bias_front < 0.0 or drive_bias_front > 1.0:
		return false
	return true
