extends Resource
class_name PowertrainConfiguration

## Configuration resource for the motor and drivetrain subsystem.
##
## Default parameters match the verified values in `src/mirena_sim/Scenes/Vehicle/vehicleMotion.gd`:
## - POWER_LIM = 80000.0 W (FS EV 80 kW accumulator rule limit)
## - MOTOR_PEAK_TRQ = 100.0 N*m
## - GEAR_RATIO = 5.0 (1:5 reduction)
## - DRIVETRAIN_EFF = 0.92
## - Reverse driving disabled (`allow_reverse = false`), because `/control` uses [-1, 1] where
##   negative `gas` is friction braking, never reverse.

@export_group("Architecture")
## Drivetrain layout (RWD single motor vs AWD hub motors).
@export var architecture: VehicleDynamicsTypes.DrivetrainArchitecture = VehicleDynamicsTypes.DrivetrainArchitecture.RWD_SINGLE_MOTOR
## Allow negative motor torque to drive the vehicle in reverse. False on MirenaCar (/control < 0 is braking).
@export var allow_reverse: bool = false
## Enable regenerative braking torque through the drivetrain. False in current vehicleMotion.gd baseline.
@export var regen_enabled: bool = false

@export_group("Motor & Power Limits")
## Peak motor shaft torque (N*m). Verified in vehicleMotion.gd (MOTOR_PEAK_TRQ = 100.0 N*m).
@export var motor_peak_torque_nm: float = 100.0
## Maximum electrical/mechanical power limit (W). Verified in vehicleMotion.gd (POWER_LIM = 80000.0 W).
@export var power_limit_w: float = 80000.0
## Maximum motor shaft speed (RPM). Source: ter26.py (6000.0 RPM).
@export var motor_max_rpm: float = 6000.0
## Minimum effective forward speed (m/s) or angular speed denominator used in power limit calculation P / max(u, u_min).
## Verified in vehicleMotion.gd line 471: maxf(u, 1.0).
@export var power_limit_min_speed_mps: float = 1.0

@export_group("Transmission & Differential")
## Final drive gear reduction ratio (motor_omega / wheel_omega). Verified in vehicleMotion.gd (GEAR_RATIO = 5.0).
@export var gear_ratio: float = 5.0
## Mechanical drivetrain efficiency (0.0 to 1.0]. Verified in vehicleMotion.gd (DRIVETRAIN_EFF = 0.92).
@export var drivetrain_efficiency: float = 0.92
## Front axle drive torque fraction (0.0 = 100% RWD, 1.0 = 100% FWD, 0.35 = TeR27 AWD baseline).
@export var drive_bias_front: float = 0.0
## Differential locking coefficient on driven axle (0.0 = open equal-torque split, 1.0 = spool).
## Default 0.35 (35% LSD / viscous locking) prevents unloaded inside rear wheelspin during cornering.
@export var diff_lock_ratio: float = 0.65
## Viscous locking stiffness (N*m / (rad/s) wheel speed difference) when diff_lock_ratio > 0.
@export var diff_preload_viscous_nm_s_rad: float = 40.0

## Return maximum total tractive wheel torque (N*m) at zero speed before power limiting.
func get_peak_wheel_torque_nm() -> float:
	return motor_peak_torque_nm * gear_ratio * drivetrain_efficiency

## Validate powertrain configuration.
func is_valid() -> bool:
	if motor_peak_torque_nm < 0.0 or power_limit_w <= 0.0 or motor_max_rpm <= 0.0:
		return false
	if gear_ratio <= 0.0 or drivetrain_efficiency <= 0.0 or drivetrain_efficiency > 1.0:
		return false
	if drive_bias_front < 0.0 or drive_bias_front > 1.0:
		return false
	return true
