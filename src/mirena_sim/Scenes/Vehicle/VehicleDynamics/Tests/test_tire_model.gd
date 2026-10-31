extends SceneTree
## Unit test suite for Tires/ subsystem contract, TireModelRegistry, and Phase 1
## uncalibrated LINEAR_TEST_STUB behavior.


func _init() -> void:
	var result := run_suite()
	if result["failed"] > 0:
		quit(1)
	else:
		quit(0)


static func _make_inputs(
	fz_n: float,
	kappa: float,
	alpha_rad: float,
	vx_mps: float = 10.0,
	dt_s: float = 0.0025
) -> TireInputs:
	var inp := TireInputs.new()
	inp.corner = VehicleDynamicsTypes.WheelCorner.FL
	inp.normal_load_fz_n = fz_n
	inp.slip_ratio_kappa = kappa
	inp.slip_angle_alpha_rad = alpha_rad
	inp.vx_contact_mps = vx_mps
	inp.vy_contact_mps = tan(alpha_rad) * vx_mps
	inp.wheel_omega_rad_s = (vx_mps * (1.0 + kappa)) / 0.20
	inp.effective_radius_m = 0.20
	inp.inflation_pressure_kpa = 82.7
	inp.tire_temperature_c = 65.0
	inp.surface_mu_scale = 1.0
	inp.dt_s = dt_s
	return inp


static func run_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_tire_model] Starting Tire Subsystem Contract Tests ===")

	# 1. Registry and uncalibrated status verification
	var cfg := TireConfiguration.new()
	cfg.model_type = VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB
	var tire_model := TireModel.new(cfg.model_type)
	var active_iface: TireModelInterface = tire_model.get_active_model()
	if active_iface != null and not active_iface.is_calibrated() and active_iface.is_implemented():
		passed += 1
		print("  [PASS] LINEAR_TEST_STUB reports is_calibrated() == false and is_implemented() == true")
	else:
		failed += 1
		print("  [FAIL] LINEAR_TEST_STUB calibration status check failed")

	# 2. Base TireModelInterface contract reports UNIMPLEMENTED_MODEL_CALLED when not overridden
	var base_cfg := TireConfiguration.new()
	base_cfg.model_type = VehicleDynamicsTypes.TireModelType.UNIMPLEMENTED_CONTRACT
	var base_model := TireModel.new(base_cfg.model_type)
	var state := TireState.new()
	var inputs := _make_inputs(700.0, 0.05, 0.05)
	var base_forces: TireForces = base_model.evaluate_wheel_forces(inputs, state, base_cfg)
	if (base_forces.fault_flags & VehicleDynamicsTypes.FaultFlag.UNIMPLEMENTED_MODEL_CALLED) != 0 and base_forces.fx_n == 0.0 and base_forces.fy_n == 0.0:
		passed += 1
		print("  [PASS] Base TireModelInterface flags UNIMPLEMENTED_MODEL_CALLED and outputs zero planar force")
	else:
		failed += 1
		print("  [FAIL] Base TireModelInterface contract check failed")

	# 3. Contact loss (Fz <= 0) produces strictly zero forces and moments
	state.reset()
	var zero_fz_inputs := _make_inputs(0.0, 0.15, 0.10)
	var zero_forces: TireForces = tire_model.evaluate_wheel_forces(zero_fz_inputs, state, cfg)
	if zero_forces.fault_flags == 0 and zero_forces.fx_n == 0.0 and zero_forces.fy_n == 0.0 and zero_forces.mz_nm == 0.0:
		passed += 1
		print("  [PASS] Zero normal load (Fz = 0) yields Fx = Fy = Mz = 0")
	else:
		failed += 1
		print("  [FAIL] Zero normal load check failed")

	# 4. Sign conventions: positive slip ratio -> +Fx; positive slip angle -> -Fy, +Mz
	cfg.enable_relaxation = false
	state.reset()
	var sign_inputs := _make_inputs(700.0, 0.05, 0.04)
	var sign_forces: TireForces = tire_model.evaluate_wheel_forces(sign_inputs, state, cfg)
	if sign_forces.fx_n > 0.0 and sign_forces.fy_n < 0.0 and sign_forces.mz_nm > 0.0:
		passed += 1
		print("  [PASS] Sign conventions: kappa > 0 => Fx > 0; alpha > 0 => Fy < 0, Mz > 0")
	else:
		failed += 1
		print("  [FAIL] Tire force sign convention check failed: Fx=%f Fy=%f Mz=%f" % [
			sign_forces.fx_n, sign_forces.fy_n, sign_forces.mz_nm
		])

	# 5. Combined-slip friction circle saturation
	state.reset()
	var sat_inputs := _make_inputs(700.0, 0.80, 0.60)
	var sat_forces: TireForces = tire_model.evaluate_wheel_forces(sat_inputs, state, cfg)
	var max_f: float = 700.0 * cfg.stub_peak_mu
	var actual_f: float = sqrt(sat_forces.fx_n * sat_forces.fx_n + sat_forces.fy_n * sat_forces.fy_n)
	if actual_f <= max_f * 1.0001 and state.is_saturated and (sat_forces.fault_flags & VehicleDynamicsTypes.FaultFlag.TIRE_SATURATION) != 0:
		passed += 1
		print("  [PASS] Combined-slip friction envelope bounded (|F|=%.1f N <= %.1f N)" % [actual_f, max_f])
	else:
		failed += 1
		print("  [FAIL] Combined-slip friction envelope exceeded bound: %f vs %f" % [actual_f, max_f])

	# 6. First-order transient slip relaxation convergence
	var cfg_relax := TireConfiguration.new()
	cfg_relax.model_type = VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB
	cfg_relax.enable_relaxation = true
	state.reset()
	var relax_inputs := _make_inputs(700.0, 0.05, 0.04)
	var f_first: TireForces = tire_model.evaluate_wheel_forces(relax_inputs, state, cfg_relax)
	var first_fx: float = f_first.fx_n
	var f_steady: TireForces = f_first
	for _i in range(200):
		f_steady = tire_model.evaluate_wheel_forces(relax_inputs, state, cfg_relax)
	if first_fx > 0.0 and first_fx < f_steady.fx_n and absf(state.relaxed_slip_ratio - 0.05) < 1e-3:
		passed += 1
		print("  [PASS] Transient slip relaxation converges smoothly to steady state")
	else:
		failed += 1
		print("  [FAIL] Transient slip relaxation check failed (first=%f, steady=%f)" % [first_fx, f_steady.fx_n])

	# 7. Non-finite input rejection
	state.reset()
	var nan_inputs := _make_inputs(NAN, 0.05, 0.04)
	var nan_forces: TireForces = tire_model.evaluate_wheel_forces(nan_inputs, state, cfg)
	if not nan_forces.valid and (nan_forces.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_INPUT) != 0 and nan_forces.fx_n == 0.0:
		passed += 1
		print("  [PASS] Non-finite (NaN) TireInputs safely rejected with NON_FINITE_INPUT fault")
	else:
		failed += 1
		print("  [FAIL] Non-finite TireInputs rejection check failed")

	# 8. TeR_Tire Pacejka MF6.2 exact parity against PyTorch reference (Hoosier 16.0x7.5-10 R20)
	var mf62_model := PacejkaMF62TireModel.new()
	var ref_mf62: Vector2 = mf62_model.eval_ter_mf62_raw(0.05, 0.0, 700.0, 82.7, true)
	if mf62_model.is_calibrated() and absf(ref_mf62.x - 1162.642) < 0.05 and absf(ref_mf62.y - (-18.910)) < 0.05:
		passed += 1
		print("  [PASS] TeR_Tire PacejkaMF62TireModel matches PyTorch reference (Fy0=%.3f N, Mz0=%.3f Nm)" % [ref_mf62.x, ref_mf62.y])
	else:
		failed += 1
		print("  [FAIL] TeR_Tire PacejkaMF62TireModel mismatch: Fy0=%f Mz0=%f" % [ref_mf62.x, ref_mf62.y])

	# 9. TeR_Tire 7-input PINN ResidualTireMLP exact parity against pinn_residual_best.pth
	var greybox_model := TeRTireGreyBoxModel.new()
	var ref_res_zero: Vector2 = greybox_model.eval_ter_pinn_residual_raw(0.0, 0.0, 0.0, 700.0, 82.7, 62.0, 1.5)
	var ref_res_005: Vector2 = greybox_model.eval_ter_pinn_residual_raw(0.05, 0.0, 0.0, 700.0, 82.7, 62.0, 1.5)
	if (
		greybox_model.is_calibrated() and
		absf(ref_res_zero.x) < 1e-6 and
		absf(ref_res_zero.y) < 1e-6 and
		absf(ref_res_005.x - (-73.951)) < 0.05 and
		absf(ref_res_005.y - (-0.702)) < 0.05
	):
		passed += 1
		print("  [PASS] TeR_Tire 7D PINN Residual matches PyTorch checkpoint (dFy=%.3f N, dMz=%.3f Nm, zero-gate=0)" % [ref_res_005.x, ref_res_005.y])
	else:
		failed += 1
		print("  [FAIL] TeR_Tire 7D PINN Residual mismatch: dFy=%f dMz=%f" % [ref_res_005.x, ref_res_005.y])

	# 10. Full TeRTireGreyBoxModel evaluation via TireModelRegistry & ISO sign convention check
	var gb_cfg := TireConfiguration.new()
	gb_cfg.model_type = VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN
	gb_cfg.enable_ply_steer_asymmetry = false
	var gb_tire := TireModel.new(gb_cfg.model_type)
	var gb_state := TireState.new()
	# Left turn (+steer -> vy_contact < 0 -> slip_angle_alpha_rad = -0.05 rad) must produce +Fy > 0
	var gb_inp := _make_inputs(700.0, 0.02, -0.05)
	gb_inp.tire_temperature_c = 62.0
	var gb_forces: TireForces = gb_tire.evaluate_wheel_forces(gb_inp, gb_state, gb_cfg)
	if (
		gb_forces.fy_n > 900.0 and
		gb_forces.fx_n > 0.0 and
		gb_forces.mz_nm < 0.0 and
		gb_state.fy_pure_mf62_n > 1000.0 and
		absf(gb_state.delta_fy_pinn_n) > 10.0 and
		gb_state.fx_companion_uncalibrated_n > 0.0
	):
		passed += 1
		print("  [PASS] TeRTireGreyBoxModel ISO evaluation verified (Fy=%.1f N, PureMF62=%.1f N, PINN_dFy=%.1f N, CompanionFx=%.1f N)" % [
			gb_forces.fy_n, gb_state.fy_pure_mf62_n, gb_state.delta_fy_pinn_n, gb_state.fx_companion_uncalibrated_n
		])
	else:
		failed += 1
		print("  [FAIL] TeRTireGreyBoxModel ISO evaluation failed: Fy=%f Fx=%f Mz=%f" % [
			gb_forces.fy_n, gb_forces.fx_n, gb_forces.mz_nm
		])

	print("=== [test_tire_model] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}
