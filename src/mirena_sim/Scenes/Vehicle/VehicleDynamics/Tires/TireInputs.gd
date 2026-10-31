extends RefCounted
class_name TireInputs

## Physical input contract supplied to any `TireModelInterface` evaluation.
##
## Coordinate Frame (ISO 8855 Tire / Contact Patch Frame):
## - +X_tire: Forward along the wheel intersection line with the ground plane.
## - +Y_tire: Leftward perpendicular to the wheel plane on the ground.
## - +Z_tire: Upward normal to the ground plane.
##
## Sign Conventions:
## - `slip_ratio_kappa`: Positive (> 0) under driving traction (omega * R_eff > vx_contact);
##   Negative (< 0) under braking (omega * R_eff < vx_contact).
## - `slip_angle_alpha_rad`: Defined as `atan2(vy_contact, |vx_reg|)` or `-atan2(vy_contact, |vx_reg|)`.
##   Here we follow the Mirena/ISO convention where a positive steering turn to the left (+delta)
##   with `vy_contact < 0` produces a positive lateral force `Fy > 0` when `alpha < 0` (or `Fy` opposes `vy_contact`).
##   Specifically: `slip_angle_alpha_rad = atan2(vy_contact_mps, vx_reg_mps)`, so `vy_contact > 0` (sliding left)
##   gives `alpha > 0` and generates a restoring rightward force `Fy < 0`, while `alpha < 0` (sliding right,
##   as in a left-hand corner) generates a leftward cornering force `Fy > 0`.
## - `normal_load_fz_n`: Positive upward reaction load at the contact patch (N, >= 0).
## - `camber_gamma_rad`: Inclination/camber angle (rad). Negative = top of tire tilted inward toward vehicle centerline.

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL
## Dimensionless longitudinal slip ratio kappa.
var slip_ratio_kappa: float = 0.0
## Lateral slip angle alpha (rad).
var slip_angle_alpha_rad: float = 0.0
## Time derivative of slip angle d(alpha)/dt (rad/s), used by grey-box/transient models.
var slip_angle_rate_rad_s: float = 0.0
## Vertical normal load Fz (N, >= 0).
var normal_load_fz_n: float = 0.0
## Camber / inclination angle gamma (rad).
var camber_gamma_rad: float = 0.0
## Longitudinal velocity of the wheel contact center in the wheel frame (m/s).
var vx_contact_mps: float = 0.0
## Lateral velocity of the wheel contact center in the wheel frame (m/s).
var vy_contact_mps: float = 0.0
## Wheel angular spin velocity omega_w (rad/s, positive = forward rolling).
var wheel_omega_rad_s: float = 0.0
## Effective rolling radius R_eff (m).
var effective_radius_m: float = 0.20
## Tire inflation pressure (kPa).
var inflation_pressure_kpa: float = 82.7
## Average tire tread temperature (C).
var tire_temperature_c: float = 65.0
## External road surface friction multiplier (1.0 = nominal dry tarmac).
var surface_mu_scale: float = 1.0
## Integration time step (s).
var dt_s: float = 0.0025

## Return true if all input variables are finite and physically within domain.
func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_scalar(slip_ratio_kappa)
		and VehicleUnits.is_finite_scalar(slip_angle_alpha_rad)
		and VehicleUnits.is_finite_scalar(slip_angle_rate_rad_s)
		and VehicleUnits.is_finite_scalar(normal_load_fz_n)
		and normal_load_fz_n >= 0.0
		and VehicleUnits.is_finite_scalar(camber_gamma_rad)
		and VehicleUnits.is_finite_scalar(vx_contact_mps)
		and VehicleUnits.is_finite_scalar(vy_contact_mps)
		and VehicleUnits.is_finite_scalar(wheel_omega_rad_s)
		and VehicleUnits.is_finite_scalar(effective_radius_m)
		and effective_radius_m > 0.0
		and VehicleUnits.is_finite_scalar(inflation_pressure_kpa)
		and inflation_pressure_kpa > 0.0
		and VehicleUnits.is_finite_scalar(tire_temperature_c)
		and VehicleUnits.is_finite_scalar(surface_mu_scale)
		and surface_mu_scale >= 0.0
		and VehicleUnits.is_finite_scalar(dt_s)
		and dt_s >= 0.0
	)
