extends RefCounted
class_name TireModelInterface

func get_model_name() -> String:
	return "TireModelInterface(Abstract)"

func is_calibrated() -> bool:
	return false

func is_implemented() -> bool:
	return false

func reset_state(state: TireState, config: TireConfiguration = null) -> void:
	if state == null:
		return
	var temp_c: float = config.nominal_temperature_c if config != null else 65.0
	state.reset(temp_c)

func evaluate_forces(
	inputs: TireInputs,
	state: TireState,
	config: TireConfiguration
) -> TireForces:
	var out := TireForces.new()
	if inputs == null or state == null or config == null or not inputs.is_valid() or not config.is_valid():
		out.valid = false
		out.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_INPUT
		return out
	if inputs.normal_load_fz_n <= 0.0:
		out.zero()
		return out
	out.fz_n = inputs.normal_load_fz_n
	out.fault_flags |= VehicleDynamicsTypes.FaultFlag.UNIMPLEMENTED_MODEL_CALLED
	return out

class LinearTestStubTireModel extends TireModelInterface:
	func get_model_name() -> String:
		return "LinearTestStubTireModel(Uncalibrated)"

	func is_calibrated() -> bool:
		return false

	func is_implemented() -> bool:
		return true

	func get_provenance_metadata() -> Dictionary:
		return {
			"tire_spec": "Hoosier 16.0x7.5-10 R20 (Linear Baseline)",
			"fy_provenance": "UNCALIBRATED_LINEAR_STIFFNESS (C_alpha=25 kN/rad, C_gamma=0.10*C_alpha)",
			"fx_provenance": "UNCALIBRATED_LINEAR_STIFFNESS (Kx=15.0/Fz)",
			"friction_envelope": "FRICTION_ELLIPSE_SATURATION (mu=1.20)",
			"calibrated": false,
			"status": "FUNCTIONAL_LINEAR_BASELINE",
		}

	func evaluate_forces(
		inputs: TireInputs,
		state: TireState,
		config: TireConfiguration
	) -> TireForces:
		var out := TireForces.new()
		if inputs == null or state == null or config == null or not inputs.is_valid() or not config.is_valid():
			out.valid = false
			out.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_INPUT
			return out
		if inputs.normal_load_fz_n <= 0.0:
			state.is_saturated = false
			state.combined_slip_utilization = 0.0
			state.lateral_force_reserve_n = 0.0
			state.static_camber_fy_n = 0.0
			state.iso_camber_gamma_rad = 0.0
			out.zero()
			return out
		var fz: float = minf(inputs.normal_load_fz_n, config.fz_max_n)
		out.fz_n = inputs.normal_load_fz_n

		var eff_kappa: float = inputs.slip_ratio_kappa
		var eff_alpha: float = inputs.slip_angle_alpha_rad
		if config.enable_relaxation and inputs.dt_s > 0.0:
			var vx_abs: float = maxf(absf(inputs.vx_contact_mps), config.vx_min_regularization_mps)
			var alpha_k: float = 1.0 - exp(-(vx_abs * inputs.dt_s) / maxf(config.relaxation_length_long_m, 1e-3))
			var alpha_a: float = 1.0 - exp(-(vx_abs * inputs.dt_s) / maxf(config.relaxation_length_lat_m, 1e-3))
			state.relaxed_slip_ratio = lerpf(state.relaxed_slip_ratio, inputs.slip_ratio_kappa, alpha_k)
			state.relaxed_slip_angle_rad = lerpf(state.relaxed_slip_angle_rad, inputs.slip_angle_alpha_rad, alpha_a)
			eff_kappa = state.relaxed_slip_ratio
			eff_alpha = state.relaxed_slip_angle_rad
		else:
			state.relaxed_slip_ratio = eff_kappa
			state.relaxed_slip_angle_rad = eff_alpha

		state.prev_slip_angle_rad = inputs.slip_angle_alpha_rad
		state.iso_camber_gamma_rad = inputs.camber_gamma_rad
		state.slip_angle_rate_rad_s = inputs.slip_angle_rate_rad_s

		var kx_n: float = config.stub_kx_per_fz * fz
		var fx_raw: float = kx_n * eff_kappa
		state.fx_companion_uncalibrated_n = fx_raw

		var fz_ratio: float = fz / maxf(config.fz_nominal_n, 1.0)
		var ky_n_rad: float = config.stub_cornering_stiffness_n_rad * fz_ratio
		var c_gamma_n_rad: float = 0.10 * ky_n_rad
		var fy_camber: float = c_gamma_n_rad * inputs.camber_gamma_rad
		state.static_camber_fy_n = fy_camber

		var fy_raw: float = -ky_n_rad * eff_alpha + fy_camber

		var f_max: float = maxf(0.0, config.stub_peak_mu * inputs.surface_mu_scale * fz)
		var f_mag: float = sqrt(fx_raw * fx_raw + fy_raw * fy_raw)

		if f_mag > f_max and f_mag > VehicleUnits.EPSILON_STRICT:
			var scale: float = f_max / f_mag
			out.fx_n = fx_raw * scale
			out.fy_n = fy_raw * scale
			state.is_saturated = true
			out.fault_flags |= VehicleDynamicsTypes.FaultFlag.TIRE_SATURATION
		else:
			out.fx_n = fx_raw
			out.fy_n = fy_raw
			state.is_saturated = false

		state.combined_slip_utilization = clampf(f_mag / maxf(f_max, 1e-4), 0.0, 1.5)
		state.lateral_force_reserve_n = maxf(0.0, f_max - absf(out.fy_n))

		out.mz_nm = -config.stub_pneumatic_trail_m * out.fy_n

		if absf(inputs.wheel_omega_rad_s) > VehicleUnits.EPSILON_SMALL:
			out.my_nm = -signf(inputs.wheel_omega_rad_s) * config.rolling_resistance_coeff * fz * inputs.effective_radius_m
		else:
			out.my_nm = 0.0
		out.mx_nm = 0.0

		return out
