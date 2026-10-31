extends Resource
class_name BrakeConfiguration

## Configuration resource for the hydraulic friction brake and Emergency Braking System (EBS) subsystem.
##
## Defaults match the verified parameters in `src/mirena_sim/Scenes/Vehicle/vehicleMotion.gd`:
## - BRAKE_MU = 0.40 (calibrated against recorded bag deceleration of 3.93 m/s^2 = 0.40 * g)
## - BRAKE_BIAS_F = 0.60 (60% front / 40% rear brake distribution)
## - Total braking force budget = BRAKE_MU * MASS_KG * GRAVITY = 0.40 * 300.0 * 9.81 = 1177.2 N

@export_group("Friction Brake Sizing & Balance")
## Fraction of total braking torque/force allocated to the front axle [0.0, 1.0].
## Verified in vehicleMotion.gd (BRAKE_BIAS_F = 0.60) and ter26.py (0.60).
@export var brake_bias_front: float = 0.60
## Effective vehicle deceleration friction factor used to size the total brake budget when derive_budget_from_mu is true.
## Verified in vehicleMotion.gd (BRAKE_MU = 0.40).
@export var brake_mu: float = 0.40
## If true, total max brake force budget is computed as `brake_mu * mass * g` (1177.2 N at 300 kg),
## converted to wheel torque via `wheel_radius`. If false, `max_total_brake_torque_nm` is used directly.
@export var derive_budget_from_mu: bool = true
## Total maximum hydraulic braking torque across all four wheels (N*m) when `derive_budget_from_mu` is false.
## At 1177.2 N total force and 0.20 m wheel radius, total brake torque is 235.44 N*m.
@export var max_total_brake_torque_nm: float = 235.44

@export_group("Emergency Braking System (EBS) & Standstill Hold")
## Additional per-wheel braking torque applied when EBS is triggered or in NO_PILOT mode (N*m).
@export var ebs_per_wheel_torque_nm: float = 100.0
## Wheel angular speed threshold (rad/s) below which brake torque is smoothly regularized or clamped
## to prevent numerical sign oscillation at standstill.
@export var standstill_omega_eps_rad_s: float = 0.05

## Compute total maximum hydraulic braking torque (N*m) across all four wheels.
func get_total_brake_torque_budget_nm(mass_kg: float, gravity_mps2: float, wheel_radius_m: float) -> float:
	if derive_budget_from_mu:
		var total_brake_force_n: float = brake_mu * mass_kg * gravity_mps2
		return total_brake_force_n * wheel_radius_m
	return max_total_brake_torque_nm

## Validate brake configuration.
func is_valid() -> bool:
	if brake_bias_front < 0.0 or brake_bias_front > 1.0:
		return false
	if brake_mu < 0.0 or max_total_brake_torque_nm < 0.0 or ebs_per_wheel_torque_nm < 0.0:
		return false
	if standstill_omega_eps_rad_s <= 0.0:
		return false
	return true
