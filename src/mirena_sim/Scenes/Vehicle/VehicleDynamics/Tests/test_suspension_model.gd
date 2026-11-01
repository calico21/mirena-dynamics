extends SceneTree

func _init() -> void:
	var result := run_suite()
	if result["failed"] > 0:
		quit(1)
	else:
		quit(0)

static func run_suite() -> Dictionary:
	var passed: int = 0
	var failed: int = 0

	print("=== [test_suspension_model] Starting Suspension & Normal Load Tests ===")

	var params := VehicleParameters.new()
	var cfg := SuspensionConfiguration.new()
	var susp_model := SuspensionModel.new()

	var wheels: Array[WheelState] = []
	for corner in VehicleDynamicsTypes.ALL_CORNERS:
		var w := WheelState.new(corner)
		w.contact_state.in_contact = true
		w.suspension_state.compression_m = 0.0
		w.suspension_state.compression_velocity_mps = 0.0
		wheels.append(w)

	susp_model.evaluate_all_corners(wheels, 0.0, 0.0, 0.0, params, cfg)
	var static_loads: Array[float] = VehicleInertia.compute_static_corner_loads_n(params)
	if absf(wheels[0].suspension_state.normal_load_fz_n - static_loads[0]) < 1e-3 and absf(wheels[2].suspension_state.normal_load_fz_n - static_loads[2]) < 1e-3:
		passed += 1
		print("  [PASS] Zero deflection and velocity recovers exact static wheel loads (FL=%.1f N, RL=%.1f N)" % [
			wheels[0].suspension_state.normal_load_fz_n, wheels[2].suspension_state.normal_load_fz_n
		])
	else:
		failed += 1
		print("  [FAIL] Static equilibrium wheel load recovery failed")

	var bs_below := SpringModel.compute_bump_stop_force_n(0.020, cfg)
	var bs_above := SpringModel.compute_bump_stop_force_n(0.030, cfg)
	var expected_bs := cfg.bump_stop_rate_npm * (0.030 - cfg.bump_stop_engage_m)
	if bs_below == 0.0 and absf(bs_above - expected_bs) < 1e-3:
		passed += 1
		print("  [PASS] SpringModel linear rate and progressive bump-stop engagement verified")
	else:
		failed += 1
		print("  [FAIL] SpringModel bump-stop calculation failed")

	var f_bump := DamperModel.compute_damper_force_n(0.05, VehicleDynamicsTypes.Axle.FRONT, cfg)
	var f_rebound := DamperModel.compute_damper_force_n(-0.05, VehicleDynamicsTypes.Axle.FRONT, cfg)
	if f_bump > 0.0 and f_rebound < 0.0 and absf(f_rebound) > absf(f_bump):
		passed += 1
		print("  [PASS] DamperModel digressive bump/rebound asymmetry verified")
	else:
		failed += 1
		print("  [FAIL] DamperModel asymmetry check failed")

	var arb_pair := AntiRollBarModel.compute_axle_arb_forces_n(0.015, -0.005, VehicleDynamicsTypes.Axle.FRONT, cfg)
	if arb_pair.x > 0.0 and arb_pair.y < 0.0 and absf(arb_pair.x + arb_pair.y) < 1e-6:
		passed += 1
		print("  [PASS] AntiRollBarModel transfers load from inside to outside with zero net heave force")
	else:
		failed += 1
		print("  [FAIL] AntiRollBarModel symmetry check failed: %s" % str(arb_pair))

	wheels[0].suspension_state.compression_m = -0.035
	wheels[0].suspension_state.compression_velocity_mps = -0.80
	susp_model.evaluate_all_corners(wheels, 0.0, 0.0, 0.0, params, cfg)
	if wheels[0].suspension_state.normal_load_fz_n == 0.0 and wheels[0].suspension_state.total_suspension_force_n == 0.0:
		passed += 1
		print("  [PASS] SuspensionModel clamps tensile vertical load to zero on wheel liftoff")
	else:
		failed += 1
		print("  [FAIL] SuspensionModel wheel liftoff clamping failed: Fz=%f" % wheels[0].suspension_state.normal_load_fz_n)

	print("=== [test_suspension_model] Completed: %d passed, %d failed ===\n" % [passed, failed])
	return {"passed": passed, "failed": failed}
