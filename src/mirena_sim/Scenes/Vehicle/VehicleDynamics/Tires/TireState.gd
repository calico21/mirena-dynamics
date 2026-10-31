extends RefCounted
class_name TireState

## Mutable per-wheel internal tire state (transient relaxation, carcass deflection, thermal state).
## Kept strictly separate from `TireConfiguration` (immutable parameters) and `TireInputs`.

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL
## Relaxed longitudinal slip ratio kappa_prime (dimensionless).
var relaxed_slip_ratio: float = 0.0
## Relaxed lateral slip angle alpha_prime (rad).
var relaxed_slip_angle_rad: float = 0.0
## Previous instantaneous slip angle (rad) for finite-difference alpha_dot computation.
var prev_slip_angle_rad: float = 0.0
## Longitudinal carcass bristle deflection (m).
var deflection_x_m: float = 0.0
## Lateral carcass bristle deflection (m).
var deflection_y_m: float = 0.0
## Surface tread temperature (C).
var tread_temp_c: float = 65.0
## Bulk carcass temperature (C).
var carcass_temp_c: float = 62.0
## Temperature spread across tread ribs Delta_T (C), used by TeR_Tire 7-input ResidualTireMLP.
var delta_temp_c: float = 1.5
## True if the combined tire force reached the friction envelope limit on the last step.
var is_saturated: bool = false
## Pure lateral force Fy0 from calibrated TeR_Tire Pacejka MF6.2 baseline (N).
var fy_pure_mf62_n: float = 0.0
## Pure self-aligning moment Mz0 from calibrated TeR_Tire Pacejka MF6.2 baseline (N*m).
var mz_pure_mf62_nm: float = 0.0
## PINN residual correction Delta_Fy from TeR_Tire ResidualTireMLP (N).
var delta_fy_pinn_n: float = 0.0
## PINN residual correction Delta_Mz from TeR_Tire ResidualTireMLP (N*m).
var delta_mz_pinn_nm: float = 0.0
## Uncalibrated companion longitudinal force Fx0 before combined-slip scaling (N).
var fx_companion_uncalibrated_n: float = 0.0
## Combined-slip lateral reduction factor Gyx in [0, 1].
var combined_slip_factor_gyx: float = 1.0
## Static/zero-slip camber thrust lateral force component Fy_camber0 (N) in ISO wheel frame.
var static_camber_fy_n: float = 0.0
## Combined-slip friction envelope utilization in [0, 1+].
var combined_slip_utilization: float = 0.0
## Remaining lateral force reserve before friction-ellipse saturation (N).
var lateral_force_reserve_n: float = 0.0
## Instantaneous tire inclination angle gamma in ISO wheel frame (rad).
var iso_camber_gamma_rad: float = 0.0
## Instantaneous slip-angle derivative alpha_dot in ISO wheel frame (rad/s).
var slip_angle_rate_rad_s: float = 0.0

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	reset()

## Reset transient and thermal states to equilibrium defaults.
func reset(ambient_temp_c: float = 62.0) -> void:
	relaxed_slip_ratio = 0.0
	relaxed_slip_angle_rad = 0.0
	prev_slip_angle_rad = 0.0
	deflection_x_m = 0.0
	deflection_y_m = 0.0
	tread_temp_c = ambient_temp_c
	carcass_temp_c = ambient_temp_c
	delta_temp_c = 1.5
	is_saturated = false
	fy_pure_mf62_n = 0.0
	mz_pure_mf62_nm = 0.0
	delta_fy_pinn_n = 0.0
	delta_mz_pinn_nm = 0.0
	fx_companion_uncalibrated_n = 0.0
	combined_slip_factor_gyx = 1.0
	static_camber_fy_n = 0.0
	combined_slip_utilization = 0.0
	lateral_force_reserve_n = 0.0
	iso_camber_gamma_rad = 0.0
	slip_angle_rate_rad_s = 0.0

## Verify all state variables are finite.
func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_scalar(relaxed_slip_ratio)
		and VehicleUnits.is_finite_scalar(relaxed_slip_angle_rad)
		and VehicleUnits.is_finite_scalar(prev_slip_angle_rad)
		and VehicleUnits.is_finite_scalar(deflection_x_m)
		and VehicleUnits.is_finite_scalar(deflection_y_m)
		and VehicleUnits.is_finite_scalar(tread_temp_c)
		and VehicleUnits.is_finite_scalar(carcass_temp_c)
	)
