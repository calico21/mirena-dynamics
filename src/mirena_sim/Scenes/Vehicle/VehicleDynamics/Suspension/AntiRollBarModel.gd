extends RefCounted
class_name AntiRollBarModel

## Evaluates front and rear Anti-Roll Bar (ARB) vertical force transfer between left and right wheels.
##
## Sign Convention:
## - When the left wheel is compressed more than the right wheel (`compression_left_m > compression_right_m`),
##   the ARB increases upward force on the left wheel (`+F_arb`) and decreases upward force on the right wheel (`-F_arb`).
## - Sum of left and right ARB forces on an axle is identically zero (`F_arb_left + F_arb_right == 0`).

## Compute [left_arb_force_n, right_arb_force_n] for a single axle.
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
