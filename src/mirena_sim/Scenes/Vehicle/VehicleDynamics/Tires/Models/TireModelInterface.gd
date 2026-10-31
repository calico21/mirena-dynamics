extends RefCounted
class_name TireModelInterface

## Abstract interface contract for replaceable tire models in MirenaSim VehicleDynamics.
##
## Every concrete tire model must:
## 1. Extend `TireModelInterface`.
## 2. Compute `TireForces` strictly from `(inputs: TireInputs, state: TireState, config: TireConfiguration)`
##    without accessing scene nodes, ROS topics, or chassis globals.
## 3. Return zero forces (`fx=0, fy=0, fz=0, mx=0, my=0, mz=0`) when `inputs.normal_load_fz_n <= 0.0`.
## 4. Handle invalid or non-finite inputs safely by returning zero forces with `valid = false` and
##    `FaultFlag.NON_FINITE_INPUT` set.
## 5. Report honestly via `is_calibrated()` whether the model represents an experimentally calibrated
##    formulation or a test/unimplemented contract stub.

## Human-readable identifier of the tire model implementation.
func get_model_name() -> String:
	return "TireModelInterface(Abstract)"

## Return true ONLY if this model is backed by experimentally calibrated tire parameters.
## Must return false for contract placeholders and linear unit-test stubs.
func is_calibrated() -> bool:
	return false

## Return true if the physical equations for this model have been implemented.
func is_implemented() -> bool:
	return false

## Reset internal mutable `TireState` to equilibrium defaults.
func reset_state(state: TireState, config: TireConfiguration = null) -> void:
	if state == null:
		return
	var temp_c: float = config.nominal_temperature_c if config != null else 65.0
	state.reset(temp_c)

## Evaluate tire forces and moments in the ISO 8855 Steered Wheel Contact Frame.
## Base implementation acts as the `UNIMPLEMENTED_CONTRACT` placeholder:
## validates inputs, enforces contact loss semantics, and raises `UNIMPLEMENTED_MODEL_CALLED`
## when non-zero load is evaluated on an unimplemented model.
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


## Uncalibrated Linear + Friction-Ellipse Test Stub (`TireModelType.LINEAR_TEST_STUB`).
##
## Purpose:
## Enables independent unit and integration testing of coordinate signs, slip relaxation,
## combined-slip friction-ellipse bounding, contact loss, and chassis force/moment assembly
## in Phase 1 BEFORE the Phase 2 Pacejka MF6.2 / TeR-Tire Grey-Box models are integrated.
## Explicitly reports `is_calibrated() == false`.
class LinearTestStubTireModel extends TireModelInterface:
	func get_model_name() -> String:
		return "LinearTestStubTireModel(Uncalibrated)"

	func is_calibrated() -> bool:
		return false

	func is_implemented() -> bool:
		return true

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
			out.zero()
			return out

		var fz: float = minf(inputs.normal_load_fz_n, config.fz_max_n)
		out.fz_n = inputs.normal_load_fz_n

		# Optional first-order slip relaxation
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

		# Uncalibrated linear force generation:
		# - Positive kappa (driving) -> positive Fx (+X_wheel)
		# - Positive alpha (sliding left, vy > 0) -> negative Fy (-Y_wheel, restoring force)
		var kx_n: float = config.stub_kx_per_fz * fz
		var ky_n_rad: float = config.stub_cornering_stiffness_n_rad * (fz / maxf(config.fz_nominal_n, 1.0))

		var fx_raw: float = kx_n * eff_kappa
		var fy_raw: float = -ky_n_rad * eff_alpha

		# Combined-slip friction circle / ellipse saturation: sqrt(Fx^2 + Fy^2) <= mu * Fz
		var f_max: float = maxf(0.0, config.stub_peak_mu * inputs.surface_mu_scale * fz)
		var f_mag: float = sqrt(fx_raw * fx_raw + fy_raw * fy_raw)
		if f_mag > f_max and f_mag > VehicleUnits.EPSILON_STRICT:
			var scale: float = f_max / f_mag
			fx_raw *= scale
			fy_raw *= scale
			state.is_saturated = true
			out.fault_flags |= VehicleDynamicsTypes.FaultFlag.TIRE_SATURATION
		else:
			state.is_saturated = false

		out.fx_n = fx_raw
		out.fy_n = fy_raw
		# Self-aligning moment Mz opposes slip angle via pneumatic trail t: Mz = -t * Fy
		out.mz_nm = -config.stub_pneumatic_trail_m * fy_raw
		# Rolling resistance moment opposes forward rotation
		if absf(inputs.wheel_omega_rad_s) > VehicleUnits.EPSILON_SMALL:
			out.my_nm = -signf(inputs.wheel_omega_rad_s) * config.rolling_resistance_coeff * fz * inputs.effective_radius_m
		else:
			out.my_nm = 0.0
		out.mx_nm = 0.0
		return out
