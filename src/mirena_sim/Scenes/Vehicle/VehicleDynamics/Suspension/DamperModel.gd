extends RefCounted
class_name DamperModel

## Evaluates digressive (bilinear knee) asymmetric bump/rebound damping force for a wheel corner.
##
## Physical Equation (Horstman / Ohlins digressive damper model from TeR26/TeR27):
##   v_damper = v_wheel / MR
##   F_damper_bump(v_d) = c_low * v_d / (1 + |v_d| / v_knee) + c_high * v_d         for v_d >= 0 (bump)
##   F_damper_reb(v_d)  = rebound_ratio * [c_low * v_d / (1 + |v_d| / v_knee) + c_high * v_d]  for v_d < 0 (rebound)
##   F_wheel_damper     = F_damper / MR
##
## Sign Convention:
## - `compression_velocity_mps > 0` in bump (compressing) -> `damper_force_n > 0` (pushes chassis upward).
## - `compression_velocity_mps < 0` in rebound (extending) -> `damper_force_n < 0` (pulls chassis downward).

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
