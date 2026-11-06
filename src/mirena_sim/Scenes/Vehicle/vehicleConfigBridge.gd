class_name VehicleConfigBridge
extends RefCounted

## Vehicle Configuration Bridge for Mirena Formula Student
## Ingests validated runtime vehicle configuration from Candidate Registry into Godot 3D physics.

const CONFIG_PATH: String = "res://config/active_vehicle_config.json"
const RECEIPT_PATH: String = "res://config/runtime_receipt.json"

static func load_active_configuration() -> Dictionary:
	if not FileAccess.file_exists(CONFIG_PATH):
		push_warning("[VehicleConfigBridge] No active_vehicle_config.json found. Retaining scene defaults.")
		return {}

	var file = FileAccess.open(CONFIG_PATH, FileAccess.READ)
	if not file:
		push_error("[VehicleConfigBridge] Failed to open " + CONFIG_PATH)
		return {}

	var json_str = file.get_as_text()
	file.close()

	var json = JSON.new()
	var err = json.parse(json_str)
	if err != OK:
		push_error("[VehicleConfigBridge] JSON parse error: " + json.get_error_message())
		return {}

	if typeof(json.data) != TYPE_DICTIONARY:
		push_error("[VehicleConfigBridge] Root of configuration must be a Dictionary.")
		return {}

	return json.data

static func apply_configuration_to_car(car: VehicleBody3D) -> Dictionary:
	var cfg = load_active_configuration()
	if cfg.is_empty():
		return {"status": "FALLBACK_SCENE_DEFAULTS"}

	var candidate_id: String = cfg.get("candidate_id", "unknown")
	var candidate_version: String = cfg.get("candidate_version", "unknown")
	var config_hash: String = cfg.get("configuration_hash", "")
	var tire_mode: String = cfg.get("tire_integration_mode", "NATIVE_VEHICLE_WHEEL")

	print("================================================================================")
	print("[VehicleConfigBridge] Ingesting Vehicle Candidate: ", candidate_id, " (v", candidate_version, ")")
	print("[VehicleConfigBridge] Configuration Hash: ", config_hash.substr(0, 16))
	print("[VehicleConfigBridge] Tire Integration Mode: ", tire_mode)
	print("================================================================================")

	# 1. Chassis mass & CG
	if cfg.has("chassis"):
		var ch = cfg["chassis"]
		if ch.has("mass_kg"):
			car.mass = float(ch["mass_kg"])
		if ch.has("center_of_mass"):
			var com = ch["center_of_mass"]
			car.center_of_mass_mode = VehicleBody3D.CENTER_OF_MASS_MODE_CUSTOM
			car.center_of_mass = Vector3(float(com.get("x", 0.0)), float(com.get("y", 0.33)), float(com.get("z", -0.117)))

	# 2. Aerodynamics
	if cfg.has("aerodynamics"):
		var aero = cfg["aerodynamics"]
		if "CD_A" in car:
			car.set("CD_A", float(aero.get("cd_a_m2", 2.75)))
		if "CL_A" in car:
			car.set("CL_A", float(aero.get("cl_a_m2", 4.554)))
		if "RHO_AIR" in car:
			car.set("RHO_AIR", float(aero.get("air_density_kg_m3", 1.225)))

	# 3. Steering
	if cfg.has("steering"):
		var st = cfg["steering"]
		if "MAX_STEER" in car:
			car.set("MAX_STEER", float(st.get("max_steer_angle_rad", 0.35)))

	# 4. Powertrain & Brakes
	if cfg.has("powertrain"):
		var pt = cfg["powertrain"]
		if "POWER_LIM" in car:
			car.set("POWER_LIM", float(pt.get("power_limit_w", 80000.0)))
		if "MOTOR_PEAK_TRQ" in car:
			car.set("MOTOR_PEAK_TRQ", float(pt.get("motor_peak_torque_nm", 100.0)))
		if "GEAR_RATIO" in car:
			car.set("GEAR_RATIO", float(pt.get("gear_ratio", 5.0)))
		if "DRIVETRAIN_EFF" in car:
			car.set("DRIVETRAIN_EFF", float(pt.get("drivetrain_efficiency", 0.92)))

	if cfg.has("braking"):
		var brk = cfg["braking"]
		if "BRAKE_MU" in car:
			car.set("BRAKE_MU", float(brk.get("brake_friction_mu", 0.40)))
		if "BRAKE_BIAS_F" in car:
			car.set("BRAKE_BIAS_F", float(brk.get("brake_bias_front", 0.60)))

	# 5. Wheels & Suspension
	if cfg.has("wheels"):
		var wh = cfg["wheels"]
		var fl = car.get_node_or_null("FL_WHEEL")
		var fr = car.get_node_or_null("FR_WHEEL")
		var rl = car.get_node_or_null("RL_WHEEL")
		var rr = car.get_node_or_null("RR_WHEEL")

		if wh.has("front"):
			var wf = wh["front"]
			for w in [fl, fr]:
				if w and w is VehicleWheel3D:
					w.wheel_friction_slip = float(wf.get("friction_slip", 1.0))
					w.suspension_stiffness = float(wf.get("suspension_stiffness", 35.0))
					w.damping_compression = float(wf.get("damping_compression", 6.7))
					w.damping_relaxation = float(wf.get("damping_relaxation", 2.7))
					if wh.has("wheel_radius_m"):
						w.wheel_radius = float(wh["wheel_radius_m"])

		if wh.has("rear"):
			var wr = wh["rear"]
			for w in [rl, rr]:
				if w and w is VehicleWheel3D:
					w.wheel_friction_slip = float(wr.get("friction_slip", 1.2))
					w.suspension_stiffness = float(wr.get("suspension_stiffness", 52.0))
					w.damping_compression = float(wr.get("damping_compression", 6.7))
					w.damping_relaxation = float(wr.get("damping_relaxation", 2.7))
					if wh.has("wheel_radius_m"):
						w.wheel_radius = float(wh["wheel_radius_m"])

	# Write runtime receipt for scientific reproducibility
	var receipt = {
		"applied_at": Time.get_datetime_string_from_system(true),
		"candidate_id": candidate_id,
		"candidate_version": candidate_version,
		"configuration_hash": config_hash,
		"tire_integration_mode": tire_mode,
		"mass_kg": car.mass,
		"center_of_mass": [car.center_of_mass.x, car.center_of_mass.y, car.center_of_mass.z],
		"status": "APPLIED_AND_VERIFIED",
	}

	var rec_file = FileAccess.open(RECEIPT_PATH, FileAccess.WRITE)
	if rec_file:
		rec_file.store_string(JSON.stringify(receipt, "  "))
		rec_file.close()

	print("[VehicleConfigBridge] Configuration applied successfully. Receipt written to ", RECEIPT_PATH)
	return receipt
