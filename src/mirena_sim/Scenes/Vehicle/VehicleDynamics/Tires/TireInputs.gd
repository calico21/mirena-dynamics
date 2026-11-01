extends RefCounted
class_name TireInputs

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

var slip_ratio_kappa: float = 0.0

var slip_angle_alpha_rad: float = 0.0

var slip_angle_rate_rad_s: float = 0.0

var normal_load_fz_n: float = 0.0

var camber_gamma_rad: float = 0.0

var vx_contact_mps: float = 0.0

var vy_contact_mps: float = 0.0

var wheel_omega_rad_s: float = 0.0

var effective_radius_m: float = 0.20

var inflation_pressure_kpa: float = 82.7

var tire_temperature_c: float = 65.0

var surface_mu_scale: float = 1.0

var dt_s: float = 0.0025

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
