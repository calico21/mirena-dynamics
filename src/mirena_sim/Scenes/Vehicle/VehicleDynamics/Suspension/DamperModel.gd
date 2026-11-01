extends RefCounted
class_name DamperModel

static func compute_damper_force_n(
	compression_velocity_mps: float,
	axle: VehicleDynamicsTypes.Axle,
	config: SuspensionConfiguration
) -> float:
	if config == null or not VehicleUnits.is_finite_scalar(compression_velocity_mps):
		return 0.0

	var is_front: bool = (axle == VehicleDynamicsTypes.Axle.FRONT)
	var mr: float = maxf(config.motion_ratio_front if is_front else config.motion_ratio_rear, 1e-4)
	var c_low: float = config.damper_c_low_front_nspm if is_front else config.damper_c_low_rear_nspm
	var c_high: float = config.damper_c_high_front_nspm if is_front else config.damper_c_high_rear_nspm
	var v_knee: float = maxf(config.damper_v_knee_front_mps if is_front else config.damper_v_knee_rear_mps, 1e-5)
	var reb_ratio: float = config.rebound_ratio_front if is_front else config.rebound_ratio_rear

	var v_damper: float = compression_velocity_mps / mr
	var abs_v: float = absf(v_damper)
	var base_force_at_damper: float = (c_low * v_damper) / (1.0 + abs_v / v_knee) + (c_high * v_damper)

	if v_damper < 0.0:
		base_force_at_damper *= reb_ratio

	return base_force_at_damper / mr
