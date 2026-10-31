extends RefCounted
class_name BrakeSystem

## Evaluates front and rear hydraulic friction brake and EBS torque demand [FL, FR, RL, RR] (N*m).
##
## Physical Semantics & Sign Conventions:
## - `brake_cmd` is normalized in `[0.0, 1.0]` (extracted from `max(-gas, 0.0)` in `/control` or manual brake).
## - Available brake torque magnitudes are always non-negative (`>= 0.0 N*m`).
## - Front/rear distribution strictly obeys `config.brake_bias_front` (60% front / 40% rear in Mirena baseline),
##   split equally between left and right wheels on each axle (30% FL, 30% FR, 20% RL, 20% RR).
## - Actual signed resistive torque opposing wheel rotation is applied in `WheelRotationalDynamics`.

func evaluate_brake_torques(
	brake_cmd: float,
	ebs_active: bool,
	brake_state: BrakeState,
	params: VehicleParameters,
	config: BrakeConfiguration,
	wheels: Array[WheelState] = []
) -> Array[float]:
	if config == null or not config.is_valid() or not VehicleUnits.is_finite_scalar(brake_cmd):
		return [0.0, 0.0, 0.0, 0.0]

	var clamped_brake: float = clampf(brake_cmd, 0.0, 1.0)
	var mass_kg: float = params.total_mass_kg if params != null else 300.0
	var g_mps2: float = params.gravity_mps2 if params != null else 9.81
	var r_m: float = params.wheel_unloaded_radius_m if params != null else 0.20

	var total_budget_nm: float = config.get_total_brake_torque_budget_nm(mass_kg, g_mps2, r_m)
	var front_axle_nm: float = total_budget_nm * config.brake_bias_front * clamped_brake
	var rear_axle_nm: float = total_budget_nm * (1.0 - config.brake_bias_front) * clamped_brake

	var ebs_nm: float = config.ebs_per_wheel_torque_nm if ebs_active else 0.0

	var fl_nm: float = 0.5 * front_axle_nm + ebs_nm
	var fr_nm: float = 0.5 * front_axle_nm + ebs_nm
	var rl_nm: float = 0.5 * rear_axle_nm + ebs_nm
	var rr_nm: float = 0.5 * rear_axle_nm + ebs_nm

	var result: Array[float] = [fl_nm, fr_nm, rl_nm, rr_nm]

	if brake_state != null:
		brake_state.brake_command_norm = clamped_brake
		brake_state.ebs_active = ebs_active
		brake_state.available_torque_nm = result.duplicate()

	if wheels.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			wheels[i].brake_torque_magnitude_nm = result[i]

	return result
