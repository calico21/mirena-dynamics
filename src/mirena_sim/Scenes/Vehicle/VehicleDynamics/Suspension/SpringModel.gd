extends RefCounted
class_name SpringModel

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
