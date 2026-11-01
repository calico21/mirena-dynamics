extends Resource
class_name BrakeConfiguration

@export_group("Friction Brake Sizing & Balance")

@export var brake_bias_front: float = 0.60

@export var brake_mu: float = 0.40

@export var derive_budget_from_mu: bool = true

@export var max_total_brake_torque_nm: float = 235.44

@export_group("Emergency Braking System (EBS) & Standstill Hold")

@export var ebs_per_wheel_torque_nm: float = 100.0

@export var standstill_omega_eps_rad_s: float = 0.05

func get_total_brake_torque_budget_nm(mass_kg: float, gravity_mps2: float, wheel_radius_m: float) -> float:
	if derive_budget_from_mu:
		var total_brake_force_n: float = brake_mu * mass_kg * gravity_mps2
		return total_brake_force_n * wheel_radius_m
	return max_total_brake_torque_nm

func is_valid() -> bool:
	if brake_bias_front < 0.0 or brake_bias_front > 1.0:
		return false
	if brake_mu < 0.0 or max_total_brake_torque_nm < 0.0 or ebs_per_wheel_torque_nm < 0.0:
		return false
	if standstill_omega_eps_rad_s <= 0.0:
		return false
	return true
