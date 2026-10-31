extends RefCounted
class_name SuspensionState

## Mutable per-wheel suspension state and vertical force breakdown.
##
## Sign conventions (strictly enforced):
## - `compression_m`: Positive (> 0) in bump/jounce (suspension compressed shorter than static equilibrium),
##   Negative (< 0) in rebound/droop (extended longer than static equilibrium).
## - `compression_velocity_mps`: Positive (> 0) when compressing (d(compression_m)/dt > 0).
## - `spring_force_n`: Upward vertical force exerted by the spring on the sprung mass (N).
## - `damper_force_n`: Upward vertical force exerted by the damper on the sprung mass (N, positive during bump).
## - `arb_force_n`: Upward vertical force transferred to this corner by the anti-roll bar (N).
## - `bump_stop_force_n`: Upward vertical force from bump stop engagement (N, >= 0).
## - `normal_load_fz_n`: Total downward/upward normal reaction force at the tire-ground contact patch (N, >= 0; 0 on liftoff).

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL
## Suspension displacement from static equilibrium (m). Positive = compression (bump).
var compression_m: float = 0.0
## Rate of suspension compression (m/s). Positive = compressing (bump velocity).
var compression_velocity_mps: float = 0.0
## Static spring preload force at the wheel (N).
var static_preload_force_n: float = 0.0
## Elastic spring force at the wheel including static preload (N).
var spring_force_n: float = 0.0
## Damping force at the wheel (N). Positive opposes compression (pushes chassis up).
var damper_force_n: float = 0.0
## Anti-roll bar force at the wheel (N). Positive when this wheel is compressed more than its axle mate.
var arb_force_n: float = 0.0
## Bump stop contact force at the wheel (N, >= 0).
var bump_stop_force_n: float = 0.0
## Total suspension vertical strut force acting upward on the chassis at this corner (N).
var total_suspension_force_n: float = 0.0
## Resulting vertical normal load Fz at the tire contact patch (N, clamped >= 0.0).
var normal_load_fz_n: float = 0.0
## Dynamic camber angle at current wheel travel and body roll (rad).
var dynamic_camber_rad: float = 0.0
## Induced bump-steer angle at current wheel travel (rad).
var bump_steer_rad: float = 0.0

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	reset()

## Reset suspension state to zero deflection and static load.
func reset(static_fz_n: float = 0.0, static_camber: float = 0.0) -> void:
	compression_m = 0.0
	compression_velocity_mps = 0.0
	static_preload_force_n = maxf(0.0, static_fz_n)
	spring_force_n = static_preload_force_n
	damper_force_n = 0.0
	arb_force_n = 0.0
	bump_stop_force_n = 0.0
	total_suspension_force_n = static_preload_force_n
	normal_load_fz_n = static_preload_force_n
	dynamic_camber_rad = static_camber
	bump_steer_rad = 0.0

## Verify all suspension quantities are finite.
func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_scalar(compression_m)
		and VehicleUnits.is_finite_scalar(compression_velocity_mps)
		and VehicleUnits.is_finite_scalar(spring_force_n)
		and VehicleUnits.is_finite_scalar(damper_force_n)
		and VehicleUnits.is_finite_scalar(arb_force_n)
		and VehicleUnits.is_finite_scalar(bump_stop_force_n)
		and VehicleUnits.is_finite_scalar(total_suspension_force_n)
		and VehicleUnits.is_finite_scalar(normal_load_fz_n)
		and normal_load_fz_n >= 0.0
	)
