extends RefCounted
class_name AntiRollBarModel

static func compute_axle_arb_forces_n(
	compression_left_m: float,
	compression_right_m: float,
	axle: VehicleDynamicsTypes.Axle,
	config: SuspensionConfiguration
) -> Vector2:
	if config == null or not VehicleUnits.is_finite_scalar(compression_left_m) or not VehicleUnits.is_finite_scalar(compression_right_m):
		return Vector2.ZERO
	var k_arb_npm: float = config.arb_rate_front_npm if axle == VehicleDynamicsTypes.Axle.FRONT else config.arb_rate_rear_npm
	var diff_m: float = compression_left_m - compression_right_m
	var f_left: float = k_arb_npm * diff_m
	return Vector2(f_left, -f_left)
