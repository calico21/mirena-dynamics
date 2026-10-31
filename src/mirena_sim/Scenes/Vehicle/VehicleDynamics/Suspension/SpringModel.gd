extends RefCounted
class_name SpringModel

## Evaluates coil spring and bump-stop elastic forces at a single wheel corner.
##
## Sign Convention:
## - `compression_m > 0` in bump (suspension shortened relative to static design ride height).
## - Returned forces are positive upward (pushing the sprung mass along +Z_vehicle / +Y_godot).
## - At `compression_m == 0.0`, `compute_spring_force_n` returns `static_preload_force_n`.

## Compute elastic spring force at the wheel (N), including static preload and motion ratio:
##   k_wheel = k_spring / MR^2
##   F_spring = F_preload + k_wheel * compression_m
static func compute_spring_force_n(
	compression_m: float,
	static_preload_force_n: float,
	axle: VehicleDynamicsTypes.Axle,
	config: SuspensionConfiguration
) -> float:
	if config == null or not VehicleUnits.is_finite_scalar(compression_m):
		return 0.0
	var k_wheel_npm: float = config.get_effective_wheel_rate_npm(axle)
	return static_preload_force_n + k_wheel_npm * compression_m

## Compute bump-stop force at the wheel (N, >= 0) when compression exceeds `bump_stop_engage_m`.
static func compute_bump_stop_force_n(
	compression_m: float,
	config: SuspensionConfiguration
) -> float:
	if config == null or not VehicleUnits.is_finite_scalar(compression_m):
		return 0.0
	var excess_m: float = compression_m - config.bump_stop_engage_m
	if excess_m <= 0.0:
		return 0.0
	return config.bump_stop_rate_npm * excess_m
