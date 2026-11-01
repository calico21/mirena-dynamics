extends RefCounted
class_name TireState

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

var relaxed_slip_ratio: float = 0.0

var relaxed_slip_angle_rad: float = 0.0

var prev_slip_angle_rad: float = 0.0

var deflection_x_m: float = 0.0

var deflection_y_m: float = 0.0

var tread_temp_c: float = 65.0

var carcass_temp_c: float = 62.0

var delta_temp_c: float = 1.5

var is_saturated: bool = false

var fy_pure_mf62_n: float = 0.0

var mz_pure_mf62_nm: float = 0.0

var delta_fy_pinn_n: float = 0.0

var delta_mz_pinn_nm: float = 0.0

var fx_companion_uncalibrated_n: float = 0.0

var combined_slip_factor_gyx: float = 1.0

var static_camber_fy_n: float = 0.0

var combined_slip_utilization: float = 0.0

var lateral_force_reserve_n: float = 0.0

var iso_camber_gamma_rad: float = 0.0

var slip_angle_rate_rad_s: float = 0.0

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	reset()

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
