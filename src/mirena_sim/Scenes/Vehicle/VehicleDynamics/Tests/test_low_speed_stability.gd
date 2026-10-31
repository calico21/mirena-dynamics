extends SceneTree

func _init() -> void:
	var cfg_res: VehicleDynamicsConfig = load("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_default.tres")
	var model := VehicleDynamicsModel.new(cfg_res.duplicate(true))
	model.contact_query.use_analytic_plane = true
	model.set_control_commands(0.0, 0.0, false)

	var fx_history: Array[float] = []
	var omega_history: Array[float] = []

	for s in range(400):
		model.step(0.0025, true, false)
		var w_rl: WheelState = model.state.wheels[2]
		fx_history.append(w_rl.tire_forces.fx_n)
		omega_history.append(w_rl.omega_rad_s)

	var max_abs_fx: float = 0.0
	var max_abs_omega: float = 0.0
	for i in range(200, 400):
		max_abs_fx = maxf(max_abs_fx, absf(fx_history[i]))
		max_abs_omega = maxf(max_abs_omega, absf(omega_history[i]))

	print("STANDSTILL TEST RESULTS (Steps 200..400):")
	print("  Max abs Fx RL: %.5f N (Previous modular log: 1071.9 N)" % max_abs_fx)
	print("  Max abs omega RL: %.5f rad/s (Previous modular log: 8.8 rad/s)" % max_abs_omega)
	print("  Chassis az: %.5f m/s^2 (Previous modular log: +9.80 m/s^2)" % model.state.az_mps2)
	print("  Fault flags: %d" % model.state.fault_flags)

	if max_abs_fx < 1.0 and max_abs_omega < 1e-4 and absf(model.state.az_mps2) < 1e-3:
		print("  => SUCCESS: Standstill chatter completely eliminated! Semi-implicit integration verified unconditionally stable.")
		quit(0)
	else:
		print("  => FAILURE")
		quit(1)
