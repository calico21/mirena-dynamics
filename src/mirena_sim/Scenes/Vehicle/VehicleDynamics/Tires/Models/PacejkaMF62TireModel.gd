extends TireModelInterface
class_name PacejkaMF62TireModel

## Calibrated Pacejka Magic Formula 6.2 Tire Model (`TireModelType.PACEJKA_MF62`).
##
## Directly integrates the analytical baseline from `TeR_Tire/physics/pacejka_baseline.py`
## and the 39 fitted parameters from `TeR_Tire/config/pacejka_mf62_params.yaml`
## (Hoosier 16.0x7.5-10 R20 on 8-inch rim, calibrated on TTC Round 9 Run 1 cornering data).
##
## HONEST PROVENANCE & SCOPE:
## - Pure Lateral Force (`Fy0`) and Self-Aligning Moment (`Mz0`) are experimentally calibrated
##   from `TeR_Tire` (`pacejka_mf62_params.yaml`).
## - Because `TeR_Tire` only calibrates pure lateral `Fy0` and `Mz0` (its `friction_envelope.py`,
##   `relaxation.py`, and `thermal_core.py` are empty 0-byte placeholders and no longitudinal `p_Cx/Dx/Kx`
##   coefficients exist in `pacejka_mf62_params.yaml`), longitudinal traction/braking force (`Fx`) and
##   combined-slip coupling (`Gyx`) are computed by an explicitly separated, clearly labeled companion
##   module (`_evaluate_uncalibrated_longitudinal_companion`) and recorded in `TireState.fx_companion_uncalibrated_n`.

const EPS: float = 1.0e-6

var params: Dictionary = TeRTireWeightsData.MF62_PARAMS.duplicate()

func get_model_name() -> String:
	return "TeR_Tire Pacejka MF6.2 (Calibrated Fy/Mz + Companion Fx)"

func is_calibrated() -> bool:
	return true

func is_implemented() -> bool:
	return true

## Return provenance metadata describing which channels are calibrated vs. companion estimates.
func get_provenance_metadata() -> Dictionary:
	return {
		"tire_spec": TeRTireWeightsData.TIRE_NAME,
		"source_module": "src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Tires/TeR_Tire",
		"fy_provenance": "CALIBRATED_TER_MF62 (TTC Round 9 Run 1)",
		"mz_provenance": "CALIBRATED_TER_MF62 (TTC Round 9 Run 1)",
		"fx_provenance": "UNCALIBRATED_COMPANION_SLIP_ELLIPSE (TeR_Tire has no longitudinal fit)",
		"combined_slip_provenance": "FRICTION_ELLIPSE_COMPANION",
	}

## Exact port of `TeR_Tire/physics/pacejka_baseline.py::PacejkaMF62.forward`.
## In `TeR_Tire`'s sign convention, `alpha_ter > 0` yields `Fy0 > 0` and `Mz0 < 0`.
## Set `include_ply_steer = true` for 1:1 match with `TeR_Tire` single-tire rig equations,
## or `false` to zero out single-tire conicity/ply-steer offsets (`p_Hy1`, `p_Hy2`, `p_Vy1`, `p_Vy2`, `q_Hz1`, `q_Dz6`)
## so a 4-wheel vehicle tracks straight at zero steer and zero camber.
func eval_ter_mf62_raw(
	alpha_ter_rad: float,
	gamma_rad: float,
	fz_n: float,
	pressure_kpa: float = TeRTireWeightsData.P0_KPA,
	include_ply_steer: bool = true
) -> Vector2:
	var fz0: float = TeRTireWeightsData.FZ0_N
	var p0: float = TeRTireWeightsData.P0_KPA
	var r0: float = TeRTireWeightsData.R0_M

	# Bound dfz in empirical dimensionless coefficients to TTC calibration envelope (Fz in [150, 1600] N)
	# to prevent polynomial/hyperbolic blow-up when vehicle encounters extreme dynamic normal loads
	var dfz: float = clampf((fz_n - fz0) / (fz0 + EPS), -0.786, 1.286)
	var dpi: float = (pressure_kpa - p0) / (p0 + EPS)

	var p_Cy1: float = float(params["p_Cy1"])
	var p_Dy1: float = float(params["p_Dy1"])
	var p_Dy2: float = float(params["p_Dy2"])
	var p_Dy3: float = float(params["p_Dy3"])
	var p_Ey1: float = float(params["p_Ey1"])
	var p_Ey2: float = float(params["p_Ey2"])
	var p_Ey3: float = float(params["p_Ey3"])
	var p_Ey4: float = float(params["p_Ey4"])
	var p_Hy1: float = float(params["p_Hy1"]) if include_ply_steer else 0.0
	var p_Hy2: float = float(params["p_Hy2"]) if include_ply_steer else 0.0
	var p_Hy3: float = float(params["p_Hy3"])
	var p_Ky1: float = float(params["p_Ky1"])
	var p_Ky2: float = float(params["p_Ky2"])
	var p_Vy1: float = float(params["p_Vy1"]) if include_ply_steer else 0.0
	var p_Vy2: float = float(params["p_Vy2"]) if include_ply_steer else 0.0
	var p_Vy3: float = float(params["p_Vy3"])
	var p_Vy4: float = float(params["p_Vy4"])
	var p_py1: float = float(params["p_py1"])
	var p_py2: float = float(params["p_py2"])

	# 1. Pure Lateral Force (Fy0)
	var mu_y: float = (p_Dy1 + p_Dy2 * dfz) * (1.0 + p_py1 * dpi) * (1.0 - p_Dy3 * gamma_rad * gamma_rad)
	var Dy: float = mu_y * fz_n
	var Cy: float = p_Cy1

	var Kya0: float = p_Ky1 * fz0 * sin(2.0 * atan(fz_n / (p_Ky2 * fz0 + EPS))) * (1.0 + p_py2 * dpi)
	var By: float = Kya0 / (Cy * Dy + EPS)

	var S_Hy: float = p_Hy1 + p_Hy2 * dfz + p_Hy3 * gamma_rad
	var S_Vy: float = fz_n * ((p_Vy1 + p_Vy2 * dfz) + (p_Vy3 + p_Vy4 * dfz) * gamma_rad)
	var alpha_y: float = alpha_ter_rad + S_Hy

	var sgn_ay: float = 1.0 if alpha_y > 0.0 else (-1.0 if alpha_y < 0.0 else 0.0)
	var Ey: float = tanh(p_Ey1 + p_Ey2 * dfz) * (1.0 - (p_Ey3 + p_Ey4 * gamma_rad) * sgn_ay)

	var B_ay: float = By * alpha_y
	var atan_B_ay: float = atan(B_ay)
	var inside_sin: float = Cy * atan(B_ay - Ey * (B_ay - atan_B_ay))
	var Fy0: float = Dy * sin(inside_sin) + S_Vy

	# 2. Pure Self-Aligning Moment (Mz0)
	var Fy0_prime: float = Fy0 - S_Vy

	var q_Bz1: float = float(params["q_Bz1"])
	var q_Bz2: float = float(params["q_Bz2"])
	var q_Bz3: float = float(params["q_Bz3"])
	var q_Bz5: float = float(params["q_Bz5"])
	var q_Cz1: float = float(params["q_Cz1"])
	var q_Dz1: float = float(params["q_Dz1"])
	var q_Dz2: float = float(params["q_Dz2"])
	var q_Dz3: float = float(params["q_Dz3"])
	var q_Dz6: float = float(params["q_Dz6"]) if include_ply_steer else 0.0
	var q_Dz7: float = float(params["q_Dz7"]) if include_ply_steer else 0.0
	var q_Dz8: float = float(params["q_Dz8"])
	var q_Dz9: float = float(params["q_Dz9"])
	var q_Ez1: float = float(params["q_Ez1"])
	var q_Ez2: float = float(params["q_Ez2"])
	var q_Ez4: float = float(params["q_Ez4"])
	var q_Ez5: float = float(params["q_Ez5"])
	var q_Hz1: float = float(params["q_Hz1"]) if include_ply_steer else 0.0
	var q_Hz2: float = float(params["q_Hz2"]) if include_ply_steer else 0.0
	var q_Hz3: float = float(params["q_Hz3"])
	var q_Hz4: float = float(params["q_Hz4"])
	var q_pz1: float = float(params["q_pz1"])

	var alpha_t: float = alpha_ter_rad + q_Hz1 + q_Hz2 * dfz + (q_Hz3 + q_Hz4 * dfz) * gamma_rad
	var Dt: float = fz_n * (r0 / fz0) * (q_Dz1 + q_Dz2 * dfz) * (1.0 + q_pz1 * dpi) * (1.0 - q_Dz3 * absf(gamma_rad))
	var Ct: float = q_Cz1

	var Bt: float = (q_Bz1 + q_Bz2 * dfz + q_Bz3 * dfz * dfz) * (1.0 + q_Bz5 * absf(gamma_rad)) * (r0 / (fz0 * Dt + EPS))
	var sgn_at: float = 1.0 if alpha_t > 0.0 else (-1.0 if alpha_t < 0.0 else 0.0)
	var Et: float = tanh(q_Ez1 + q_Ez2 * dfz) * (1.0 + (q_Ez4 + q_Ez5 * gamma_rad) * sgn_at)

	var B_at: float = Bt * alpha_t
	var trail_t: float = Dt * cos(Ct * atan(B_at - Et * (B_at - atan(B_at)))) * cos(alpha_ter_rad)

	var alpha_r: float = alpha_ter_rad + S_Hy + (S_Vy / (Kya0 + EPS))
	var Dr: float = fz_n * r0 * ((q_Dz6 + q_Dz7 * dfz) + (q_Dz8 + q_Dz9 * dfz) * gamma_rad)
	var Br: float = r0 / fz0
	var Mzr: float = Dr * cos(atan(Br * alpha_r))

	var Mz0: float = -trail_t * Fy0_prime + Mzr
	return Vector2(Fy0, Mz0)

## Evaluate tire forces in the ISO 8855 Steered Wheel Frame.
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
		state.fy_pure_mf62_n = 0.0
		state.mz_pure_mf62_nm = 0.0
		state.delta_fy_pinn_n = 0.0
		state.delta_mz_pinn_nm = 0.0
		state.fx_companion_uncalibrated_n = 0.0
		state.combined_slip_factor_gyx = 1.0
		state.static_camber_fy_n = 0.0
		state.combined_slip_utilization = 0.0
		state.lateral_force_reserve_n = 0.0
		state.iso_camber_gamma_rad = inputs.camber_gamma_rad
		state.slip_angle_rate_rad_s = inputs.slip_angle_rate_rad_s
		out.zero()
		return out

	var fz: float = clampf(inputs.normal_load_fz_n, 0.0, config.fz_max_n)
	out.fz_n = inputs.normal_load_fz_n
	state.iso_camber_gamma_rad = inputs.camber_gamma_rad
	state.slip_angle_rate_rad_s = inputs.slip_angle_rate_rad_s

	# Optional first-order slip relaxation
	var eff_kappa: float = inputs.slip_ratio_kappa
	var eff_alpha_iso: float = inputs.slip_angle_alpha_rad
	if config.enable_relaxation and inputs.dt_s > 0.0:
		var vx_abs: float = maxf(absf(inputs.vx_contact_mps), config.vx_min_regularization_mps)
		var alpha_k: float = 1.0 - exp(-vx_abs * inputs.dt_s / config.relaxation_length_long_m)
		var alpha_a: float = 1.0 - exp(-vx_abs * inputs.dt_s / config.relaxation_length_lat_m)
		state.relaxed_slip_ratio = lerpf(state.relaxed_slip_ratio, inputs.slip_ratio_kappa, alpha_k)
		state.relaxed_slip_angle_rad = lerpf(state.relaxed_slip_angle_rad, inputs.slip_angle_alpha_rad, alpha_a)
		eff_kappa = state.relaxed_slip_ratio
		eff_alpha_iso = state.relaxed_slip_angle_rad
	else:
		state.relaxed_slip_ratio = eff_kappa
		state.relaxed_slip_angle_rad = eff_alpha_iso

	# Convert ISO slip angle (alpha_iso = atan2(vy, |vx|)) to TeR_Tire convention (alpha_ter = -alpha_iso)
	# so left steer (+delta -> vy < 0 -> alpha_iso < 0 -> alpha_ter > 0) produces +Fy (leftward).
	var alpha_ter: float = clampf(-eff_alpha_iso, -0.55, 0.55)
	var gamma_clamped: float = clampf(inputs.camber_gamma_rad, -0.25, 0.25)

	var mf62_out: Vector2 = eval_ter_mf62_raw(
		alpha_ter,
		gamma_clamped,
		fz,
		inputs.inflation_pressure_kpa,
		config.enable_ply_steer_asymmetry
	)
	var mf62_camber0: Vector2 = eval_ter_mf62_raw(
		0.0,
		gamma_clamped,
		fz,
		inputs.inflation_pressure_kpa,
		config.enable_ply_steer_asymmetry
	)

	var grip_scale: float = inputs.surface_mu_scale * config.ter_lateral_grip_scale
	var fy_pure: float = mf62_out.x * grip_scale
	var mz_pure: float = mf62_out.y * grip_scale

	state.static_camber_fy_n = mf62_camber0.x * grip_scale
	state.fy_pure_mf62_n = fy_pure
	state.mz_pure_mf62_nm = mz_pure
	state.delta_fy_pinn_n = 0.0
	state.delta_mz_pinn_nm = 0.0

	# Evaluate subclass residual hook (0.0 for pure PacejkaMF62; PINN residual for TeRTireGreyBoxModel)
	var residual_out: Vector2 = _evaluate_residual_hook(alpha_ter, inputs, state, config, fz, grip_scale)
	var fy_uncombined: float = fy_pure + residual_out.x
	var mz_uncombined: float = mz_pure + residual_out.y

	# Evaluate explicitly separated uncalibrated longitudinal companion & friction ellipse
	var companion: Vector3 = _evaluate_uncalibrated_longitudinal_companion(
		eff_kappa, fy_uncombined, fz, inputs.surface_mu_scale, config, alpha_ter, gamma_clamped, inputs.inflation_pressure_kpa
	)
	out.fx_n = companion.x
	out.fy_n = companion.y
	var gyx: float = companion.z
	state.fx_companion_uncalibrated_n = companion.x
	state.combined_slip_factor_gyx = gyx

	# Compute lateral friction capacity and combined-slip utilization diagnostics
	var fz0: float = TeRTireWeightsData.FZ0_N
	var p0: float = TeRTireWeightsData.P0_KPA
	var dfz: float = (fz - fz0) / (fz0 + EPS)
	var dpi: float = (inputs.inflation_pressure_kpa - p0) / (p0 + EPS)
	var mu_y_peak: float = (float(params["p_Dy1"]) + float(params["p_Dy2"]) * dfz) * (1.0 + float(params["p_py1"]) * dpi) * (1.0 - float(params["p_Dy3"]) * gamma_clamped * gamma_clamped)
	var fy_cap_n: float = maxf(absf(mu_y_peak) * fz * grip_scale, EPS)
	var fx_cap_n: float = maxf(config.companion_longitudinal_peak_mu * inputs.surface_mu_scale * fz, EPS)
	var util_x: float = out.fx_n / fx_cap_n
	var util_y: float = out.fy_n / fy_cap_n
	state.combined_slip_utilization = clampf(sqrt(util_x * util_x + util_y * util_y), 0.0, 1.5)
	state.lateral_force_reserve_n = maxf(0.0, fy_cap_n * gyx - absf(out.fy_n))
	state.is_saturated = state.combined_slip_utilization > 0.92 or gyx < 0.5 or absf(alpha_ter) > 0.18
	out.mz_nm = mz_uncombined * gyx
	out.mx_nm = 0.0
	out.my_nm = -config.rolling_resistance_coeff * fz * config.unloaded_radius_m * tanh(inputs.vx_contact_mps / 0.1)
	state.prev_slip_angle_rad = inputs.slip_angle_alpha_rad
	return out

## Overridden by `TeRTireGreyBoxModel` to add the 7-input PINN residual.
func _evaluate_residual_hook(
	_alpha_ter: float,
	_inputs: TireInputs,
	_state: TireState,
	_config: TireConfiguration,
	_fz: float,
	_grip_scale: float
) -> Vector2:
	return Vector2.ZERO

## Explicitly separated, clearly labeled uncalibrated longitudinal slip & friction-ellipse companion.
## Includes Pacejka MF6.2 bidirectional combined-slip coupling:
## - Longitudinal force reduction under sideslip `G_x_alpha = cos(C_xa * atan(B_xa * alpha))`
## - Lateral force reduction under longitudinal slip `G_y_kappa` scaled against the combined friction envelope
## Returns Vector3(fx_combined, fy_combined, gyx_factor).
func _evaluate_uncalibrated_longitudinal_companion(
	eff_kappa: float,
	fy_candidate_n: float,
	fz_n: float,
	surface_mu_scale: float,
	config: TireConfiguration,
	alpha_ter_rad: float = 0.0,
	_gamma_rad: float = 0.0,
	_pressure_kpa: float = TeRTireWeightsData.P0_KPA
) -> Vector3:
	var fx_max: float = maxf(config.companion_longitudinal_peak_mu * surface_mu_scale * fz_n, EPS)
	var kx: float = config.companion_longitudinal_kx_per_fz * fz_n
	# Smooth Magic-Formula saturation for pure longitudinal force Fx0
	var bx: float = kx / (1.35 * fx_max + EPS)
	var fx0: float = fx_max * sin(1.35 * atan(bx * eff_kappa))

	# Pacejka combined-slip longitudinal reduction factor G_x_alpha(alpha):
	# When the tire is cornering at slip angle alpha, longitudinal traction shares the contact patch
	# rather than taking 100% unilateral priority over lateral cornering force.
	var b_xa: float = 9.5 * cos(atan(6.0 * eff_kappa))
	var g_x_alpha: float = clampf(cos(1.0 * atan(b_xa * alpha_ter_rad)), 0.25, 1.0)
	var fx_combined: float = fx0 * g_x_alpha

	# Friction ellipse coupling factor Gyx: scales against the Hoosier R20 total combined envelope
	# (lateral mu_y ~ 2.2 vs longitudinal mu_x ~ 1.45), retaining realistic cornering reserve under moderate throttle.
	var combined_env_x: float = fx_max * 1.18
	var ratio_x: float = clampf(absf(fx_combined) / combined_env_x, 0.0, 0.94)
	var gyx: float = sqrt(maxf(1.0 - ratio_x * ratio_x, 0.12))
	var fy_combined: float = fy_candidate_n * gyx
	return Vector3(fx_combined, fy_combined, gyx)
