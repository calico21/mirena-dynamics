extends RefCounted
class_name TelemetryImporter

const WHEEL_RADIUS_REAL_M: float = 0.2032

static func load_imu_compare_real_bundle(dir_path: String, max_rows: int = 60000) -> TelemetryRecording:
	var rec := TelemetryRecording.new("Real Vehicle Bag (TeR26 VN-300 + CAN)", true)
	rec.source_path = dir_path
	rec.source_format = "imu_compare Multi-CSV Real Bundle"
	rec.physics_mode_label = "REAL_VEHICLE_HARDWARE"
	rec.tire_setup_label = "Hoosier 16.0x7.5-10 R20 (Physical)"
	rec.track_condition = "Real Track Session (rosbag2_2026_10_06)"
	rec.known_limitations = PackedStringArray([
		"No physical wheel-force transducers (Fx, Fy, Fz, Mz are not directly measured on the real car).",
		"No physical suspension potentiometer channels in the recorded bag.",
		"No direct pitot/aero load cells on the real car.",
		"Front wheel speed Hall sensors (FL, FR) are 0 RPM in this bag; rear wheel speeds (RL, RR) and INS odometry are present.",
	])

	var imu_path: String = dir_path.path_join("real_imu.csv")
	var ctl_path: String = dir_path.path_join("real_control.csv")
	var wheels_path: String = dir_path.path_join("real_wheels.csv")
	var odom_path: String = dir_path.path_join("real_odom.csv")

	var imu_unix_t0: float = 0.0
	var imu_rel_t0: float = 0.0
	var have_imu_clock_map: bool = false
	if FileAccess.file_exists(imu_path):
		var imu_data: Dictionary = _read_csv_columns(imu_path, max_rows)
		var t_rel: PackedFloat64Array = imu_data.get("t_rel", PackedFloat64Array())
		var t_unix: PackedFloat64Array = imu_data.get("t_unix", PackedFloat64Array())
		if t_rel.size() > 0 and t_unix.size() == t_rel.size():
			imu_unix_t0 = t_unix[0]
			imu_rel_t0 = t_rel[0]
			have_imu_clock_map = true

		_add_series(rec, "imu.ax", "Longitudinal Accel ax", "m/s^2", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("ax", PackedFloat64Array()),
			"ISO 8855 car/cog (+X fwd)", "/sensors/imu linear_acceleration.x (gravity-free)")
		_add_series(rec, "imu.ay", "Lateral Accel ay", "m/s^2", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("ay", PackedFloat64Array()),
			"ISO 8855 car/cog (+Y left)", "/sensors/imu linear_acceleration.y (gravity-free)")
		_add_series(rec, "imu.az", "Vertical Accel az", "m/s^2", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("az", PackedFloat64Array()),
			"ISO 8855 car/cog (+Z up)", "/sensors/imu linear_acceleration.z (gravity-free)")
		_add_series(rec, "imu.gx", "Roll Rate gx", "rad/s", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("gx", PackedFloat64Array()),
			"ISO 8855 car/cog (+X roll)", "/sensors/imu angular_velocity.x")
		_add_series(rec, "imu.gy", "Pitch Rate gy", "rad/s", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("gy", PackedFloat64Array()),
			"ISO 8855 car/cog (+Y pitch)", "/sensors/imu angular_velocity.y")
		_add_series(rec, "imu.gz", "Yaw Rate gz (r)", "rad/s", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("gz", PackedFloat64Array()),
			"ISO 8855 car/cog (+Z CCW yaw)", "/sensors/imu angular_velocity.z")

		_add_series(rec, "chassis.r", "Chassis Yaw Rate r", "rad/s", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, imu_data.get("gz", PackedFloat64Array()),
			"ISO 8855 car/cog (+Z CCW yaw)", "Directly measured by VN-300 gyro gz")

	if FileAccess.file_exists(ctl_path):
		var ctl_data: Dictionary = _read_csv_columns(ctl_path, max_rows * 3)
		var t_rel: PackedFloat64Array = ctl_data.get("t_rel", PackedFloat64Array())
		_add_series(rec, "control.gas", "Throttle / Gas Command", "[-1..1]", "powertrain",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, ctl_data.get("gas", PackedFloat64Array()),
			"Normalized command", "/control/can gas")
		_add_series(rec, "control.steer_rad", "Steering Command", "rad", "steering",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel, ctl_data.get("steer_rad", PackedFloat64Array()),
			"Positive = Left turn (+Z)", "/control/can steer_angle converted from deg to rad")

	if FileAccess.file_exists(odom_path):
		var odom_data: Dictionary = _read_csv_columns(odom_path, max_rows)
		var t_epoch: PackedFloat64Array = odom_data.get("t", PackedFloat64Array())
		var t_rel_odom := PackedFloat64Array()
		t_rel_odom.resize(t_epoch.size())
		for i in range(t_epoch.size()):
			t_rel_odom[i] = (t_epoch[i] - imu_unix_t0 + imu_rel_t0) if have_imu_clock_map else (t_epoch[i] - t_epoch[0])
		_add_series(rec, "chassis.x", "Position X", "m", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel_odom, odom_data.get("x", PackedFloat64Array()),
			"Odom frame X (m)", "/sensors/imu/odom pose.position.x")
		_add_series(rec, "chassis.y", "Position Y", "m", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel_odom, odom_data.get("y", PackedFloat64Array()),
			"Odom frame Y (m)", "/sensors/imu/odom pose.position.y")
		_add_series(rec, "chassis.psi", "Heading / Yaw Angle psi", "rad", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel_odom, odom_data.get("yaw", PackedFloat64Array()),
			"Odom yaw (rad)", "/sensors/imu/odom orientation yaw")
		var vx: PackedFloat64Array = odom_data.get("vx", PackedFloat64Array())
		var vy: PackedFloat64Array = odom_data.get("vy", PackedFloat64Array())
		var speed_odom := PackedFloat64Array()
		speed_odom.resize(vx.size())
		for i in range(vx.size()):
			speed_odom[i] = sqrt(vx[i] * vx[i] + vy[i] * vy[i])
		_add_series(rec, "chassis.u", "Longitudinal Speed u", "m/s", "motion",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel_odom, speed_odom,
			"Forward speed magnitude (m/s)", "/sensors/imu/odom twist magnitude")

	if FileAccess.file_exists(wheels_path):
		var w_data: Dictionary = _read_csv_columns(wheels_path, max_rows)
		var t_epoch: PackedFloat64Array = w_data.get("t", PackedFloat64Array())
		var t_rel_w := PackedFloat64Array()
		t_rel_w.resize(t_epoch.size())
		for i in range(t_epoch.size()):
			t_rel_w[i] = (t_epoch[i] - imu_unix_t0 + imu_rel_t0) if have_imu_clock_map else (t_epoch[i] - t_epoch[0])
		var rl_rpm: PackedFloat64Array = w_data.get("rl", PackedFloat64Array())
		var rr_rpm: PackedFloat64Array = w_data.get("rr", PackedFloat64Array())
		_add_series(rec, "wheel.rl_rpm", "Rear Left Wheel Speed", "RPM", "powertrain",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel_w, rl_rpm,
			"Wheel rotational speed (RPM)", "/sensors/wheel_speeds rl")
		_add_series(rec, "wheel.rr_rpm", "Rear Right Wheel Speed", "RPM", "powertrain",
			TelemetryChannel.SourceType.MEASURED_REAL, t_rel_w, rr_rpm,
			"Wheel rotational speed (RPM)", "/sensors/wheel_speeds rr")

		var v_wheel := PackedFloat64Array()
		v_wheel.resize(rl_rpm.size())
		for i in range(rl_rpm.size()):
			v_wheel[i] = 0.5 * (rl_rpm[i] + rr_rpm[i]) * (TAU / 60.0) * WHEEL_RADIUS_REAL_M * 0.935
		_add_series(rec, "chassis.u_wheel_est", "Wheel-Derived Speed Estimate", "m/s", "powertrain",
			TelemetryChannel.SourceType.DERIVED_ESTIMATE, t_rel_w, v_wheel,
			"Forward speed from rear RPM (m/s)", "Derived: 0.5*(RL+RR)*2*pi/60*0.2032 * 0.935 slip calibration")
		if not rec.has_usable_channel("chassis.u"):
			_add_series(rec, "chassis.u", "Longitudinal Speed u (Wheel Est.)", "m/s", "motion",
				TelemetryChannel.SourceType.DERIVED_ESTIMATE, t_rel_w, v_wheel,
				"Forward speed from rear RPM (m/s)", "Derived from /sensors/wheel_speeds")

	_register_unavailable_real_channels(rec)
	rec.detect_maneuver_onset()
	return rec

static func load_imu_compare_sim_bundle(dir_path: String, suffix: String = "front10_rear12", max_rows: int = 60000) -> TelemetryRecording:
	var tag: String = ("_" + suffix) if not suffix.is_empty() else ""
	var label: String = suffix if not suffix.is_empty() else "default"
	var rec := TelemetryRecording.new("Sim Reference (%s)" % label, false)
	rec.source_path = dir_path
	rec.source_format = "imu_compare Multi-CSV Sim Bundle (%s)" % label
	if suffix.begins_with("modular"):
		rec.physics_mode_label = "MODULAR_VEHICLE_DYNAMICS"
		rec.tire_setup_label = "TeR_Tire Pacejka MF6.2 + 7D PINN (%s)" % label
	else:
		rec.physics_mode_label = "LEGACY_VEHICLE_BODY_3D"
		rec.tire_setup_label = "Godot VehicleWheel3D (%s)" % label
	rec.track_condition = "MirenaSim Replay against rosbag2_2026_10_06"

	var imu_path: String = dir_path.path_join("sim_imu%s.csv" % tag)
	var ctl_path: String = dir_path.path_join("sim_control%s.csv" % tag)
	var state_path: String = dir_path.path_join("sim_state%s.csv" % tag)

	var a_slope: float = 1.0
	var b_intercept: float = 0.0
	var have_sim_clock_fit: bool = false

	if FileAccess.file_exists(imu_path):
		var imu_data: Dictionary = _read_csv_columns(imu_path, max_rows)
		var t_rel: PackedFloat64Array = imu_data.get("t_rel", PackedFloat64Array())
		var t_unix: PackedFloat64Array = imu_data.get("t_unix", PackedFloat64Array())
		if t_rel.size() >= 2 and t_unix.size() == t_rel.size():
			var dt_u: float = t_unix[t_unix.size() - 1] - t_unix[0]
			if absf(dt_u) > 1e-6:
				a_slope = (t_rel[t_rel.size() - 1] - t_rel[0]) / dt_u
				b_intercept = t_rel[0] - a_slope * t_unix[0]
				have_sim_clock_fit = true

		_add_series(rec, "imu.ax", "Longitudinal Accel ax", "m/s^2", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, imu_data.get("ax", PackedFloat64Array()),
			"ISO 8855 car/cog (+X fwd)", "Sim /sensors/imu linear_acceleration.x")
		_add_series(rec, "imu.ay", "Lateral Accel ay", "m/s^2", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, imu_data.get("ay", PackedFloat64Array()),
			"ISO 8855 car/cog (+Y left)", "Sim /sensors/imu linear_acceleration.y")
		_add_series(rec, "imu.az", "Vertical Accel az", "m/s^2", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, imu_data.get("az", PackedFloat64Array()),
			"ISO 8855 car/cog (+Z up)", "Sim /sensors/imu linear_acceleration.z")
		_add_series(rec, "imu.gx", "Roll Rate gx", "rad/s", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, imu_data.get("gx", PackedFloat64Array()),
			"ISO 8855 car/cog (+X roll)", "Sim /sensors/imu angular_velocity.x")
		_add_series(rec, "imu.gy", "Pitch Rate gy", "rad/s", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, imu_data.get("gy", PackedFloat64Array()),
			"ISO 8855 car/cog (+Y pitch)", "Sim /sensors/imu angular_velocity.y")
		_add_series(rec, "imu.gz", "Yaw Rate gz (r)", "rad/s", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, imu_data.get("gz", PackedFloat64Array()),
			"ISO 8855 car/cog (+Z CCW yaw)", "Sim /sensors/imu angular_velocity.z")

	if FileAccess.file_exists(ctl_path):
		var ctl_data: Dictionary = _read_csv_columns(ctl_path, max_rows * 3)
		var t_rel: PackedFloat64Array = ctl_data.get("t_rel", PackedFloat64Array())
		_add_series(rec, "control.gas", "Throttle / Gas Command", "[-1..1]", "powertrain",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, ctl_data.get("gas", PackedFloat64Array()),
			"Normalized command", "Sim /control gas")
		_add_series(rec, "control.steer_rad", "Steering Command", "rad", "steering",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel, ctl_data.get("steer_rad", PackedFloat64Array()),
			"Positive = Left turn (+Z)", "Sim /control steer_angle (rad)")

	if FileAccess.file_exists(state_path):
		var st_data: Dictionary = _read_csv_columns(state_path, max_rows)
		var t_sim: PackedFloat64Array = st_data.get("t", PackedFloat64Array())
		var t_rel_st := PackedFloat64Array()
		t_rel_st.resize(t_sim.size())
		for i in range(t_sim.size()):
			t_rel_st[i] = (a_slope * t_sim[i] + b_intercept) if have_sim_clock_fit else t_sim[i]
		_add_series(rec, "chassis.x", "Position X", "m", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel_st, st_data.get("x", PackedFloat64Array()),
			"World X (m)", "Sim /debug/state/car x")
		_add_series(rec, "chassis.y", "Position Y", "m", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel_st, st_data.get("y", PackedFloat64Array()),
			"World Y (m)", "Sim /debug/state/car y")
		_add_series(rec, "chassis.psi", "Heading / Yaw Angle psi", "rad", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel_st, st_data.get("psi", PackedFloat64Array()),
			"World yaw (rad)", "Sim /debug/state/car psi")
		_add_series(rec, "chassis.u", "Longitudinal Speed u", "m/s", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel_st, st_data.get("u", PackedFloat64Array()),
			"Body +X forward speed (m/s)", "Sim /debug/state/car u")
		_add_series(rec, "chassis.v", "Lateral Speed v", "m/s", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel_st, st_data.get("v", PackedFloat64Array()),
			"Body +Y lateral speed (m/s)", "Sim /debug/state/car v")
		_add_series(rec, "chassis.r", "Chassis Yaw Rate r", "rad/s", "motion",
			TelemetryChannel.SourceType.SIMULATED_LEGACY, t_rel_st, st_data.get("omega", PackedFloat64Array()),
			"Body +Z yaw rate (rad/s)", "Sim /debug/state/car omega")

	rec.detect_maneuver_onset()
	return rec

static func load_single_csv(file_path: String, is_real: bool = false, max_rows: int = 60000) -> TelemetryRecording:
	var rec := TelemetryRecording.new(file_path.get_file(), is_real)
	rec.source_path = file_path
	if not FileAccess.file_exists(file_path):
		rec.known_limitations.append("File does not exist: %s" % file_path)
		return rec

	var cols: Dictionary = _read_csv_columns(file_path, max_rows)
	var src_type: TelemetryChannel.SourceType = (
		TelemetryChannel.SourceType.MEASURED_REAL if is_real else TelemetryChannel.SourceType.SIMULATED_MODULAR
	)

	if cols.has("u_mps") and cols.has("t_s"):
		rec.source_format = "MirenaSim DynamicsLogger 30-Column CSV"
		rec.physics_mode_label = "MODULAR_VEHICLE_DYNAMICS"
		var t: PackedFloat64Array = cols["t_s"]
		_add_series(rec, "chassis.u", "Longitudinal Speed u", "m/s", "motion", src_type, t, cols.get("u_mps", PackedFloat64Array()))
		_add_series(rec, "chassis.v", "Lateral Speed v", "m/s", "motion", src_type, t, cols.get("v_mps", PackedFloat64Array()))
		_add_series(rec, "chassis.r", "Chassis Yaw Rate r", "rad/s", "motion", src_type, t, cols.get("yaw_rate_rad_s", PackedFloat64Array()))
		_add_series(rec, "imu.gz", "Yaw Rate gz (r)", "rad/s", "motion", src_type, t, cols.get("yaw_rate_rad_s", PackedFloat64Array()))
		_add_series(rec, "imu.ax", "Longitudinal Accel ax", "m/s^2", "motion", src_type, t, cols.get("ax_mps2", PackedFloat64Array()))
		_add_series(rec, "imu.ay", "Lateral Accel ay", "m/s^2", "motion", src_type, t, cols.get("ay_mps2", PackedFloat64Array()))
		_add_series(rec, "control.gas", "Throttle Command", "[-1..1]", "powertrain", src_type, t, cols.get("throttle_cmd", PackedFloat64Array()))
		_add_series(rec, "control.steer_rad", "Steering Command", "rad", "steering", src_type, t, cols.get("steer_cmd_rad", PackedFloat64Array()))
		_add_series(rec, "tire.fz_fl", "Normal Load Fz FL", "N", "tires", src_type, t, cols.get("fz_fl_n", PackedFloat64Array()))
		_add_series(rec, "tire.fz_fr", "Normal Load Fz FR", "N", "tires", src_type, t, cols.get("fz_fr_n", PackedFloat64Array()))
		_add_series(rec, "tire.fz_rl", "Normal Load Fz RL", "N", "tires", src_type, t, cols.get("fz_rl_n", PackedFloat64Array()))
		_add_series(rec, "tire.fz_rr", "Normal Load Fz RR", "N", "tires", src_type, t, cols.get("fz_rr_n", PackedFloat64Array()))
		_add_series(rec, "tire.fx_rl", "Longitudinal Force Fx RL", "N", "tires", TelemetryChannel.SourceType.DERIVED_ESTIMATE, t, cols.get("fx_rl_n", PackedFloat64Array()))
		_add_series(rec, "tire.fx_rr", "Longitudinal Force Fx RR", "N", "tires", TelemetryChannel.SourceType.DERIVED_ESTIMATE, t, cols.get("fx_rr_n", PackedFloat64Array()))
		_add_series(rec, "tire.fy_fl", "Lateral Force Fy FL", "N", "tires", src_type, t, cols.get("fy_fl_n", PackedFloat64Array()))
		_add_series(rec, "tire.fy_fr", "Lateral Force Fy FR", "N", "tires", src_type, t, cols.get("fy_fr_n", PackedFloat64Array()))
		_add_series(rec, "tire.fy_rl", "Lateral Force Fy RL", "N", "tires", src_type, t, cols.get("fy_rl_n", PackedFloat64Array()))
		_add_series(rec, "tire.fy_rr", "Lateral Force Fy RR", "N", "tires", src_type, t, cols.get("fy_rr_n", PackedFloat64Array()))
		_add_series(rec, "wheel.rl_rpm", "Rear Left Wheel Speed", "RPM", "powertrain", src_type, t, cols.get("rl_rpm_cond", PackedFloat64Array()))
		_add_series(rec, "wheel.rr_rpm", "Rear Right Wheel Speed", "RPM", "powertrain", src_type, t, cols.get("rr_rpm_cond", PackedFloat64Array()))
		_add_series(rec, "aero.drag_n", "Aero Drag Force", "N", "aero", src_type, t, cols.get("aero_drag_n", PackedFloat64Array()))
		_add_series(rec, "aero.downforce_front_n", "Aero Downforce Front", "N", "aero", src_type, t, cols.get("aero_downforce_front_n", PackedFloat64Array()))
		_add_series(rec, "aero.downforce_rear_n", "Aero Downforce Rear", "N", "aero", src_type, t, cols.get("aero_downforce_rear_n", PackedFloat64Array()))
	elif cols.has("ax") and cols.has("t_rel"):
		rec.source_format = "IMU CSV Log"
		var t: PackedFloat64Array = cols["t_rel"]
		_add_series(rec, "imu.ax", "Longitudinal Accel ax", "m/s^2", "motion", src_type, t, cols.get("ax", PackedFloat64Array()))
		_add_series(rec, "imu.ay", "Lateral Accel ay", "m/s^2", "motion", src_type, t, cols.get("ay", PackedFloat64Array()))
		_add_series(rec, "imu.az", "Vertical Accel az", "m/s^2", "motion", src_type, t, cols.get("az", PackedFloat64Array()))
		_add_series(rec, "imu.gx", "Roll Rate gx", "rad/s", "motion", src_type, t, cols.get("gx", PackedFloat64Array()))
		_add_series(rec, "imu.gy", "Pitch Rate gy", "rad/s", "motion", src_type, t, cols.get("gy", PackedFloat64Array()))
		_add_series(rec, "imu.gz", "Yaw Rate gz (r)", "rad/s", "motion", src_type, t, cols.get("gz", PackedFloat64Array()))
		_add_series(rec, "chassis.r", "Chassis Yaw Rate r", "rad/s", "motion", src_type, t, cols.get("gz", PackedFloat64Array()))

	rec.detect_maneuver_onset()
	return rec

static func inspect_rosbag2_directory(bag_dir_path: String) -> Dictionary:
	var meta_path: String = bag_dir_path.path_join("metadata.yaml")
	var info := {
		"valid": false,
		"bag_path": bag_dir_path,
		"storage_identifier": "",
		"topics": [],
		"message": "",
	}
	if not FileAccess.file_exists(meta_path):
		info["message"] = "No metadata.yaml found in %s" % bag_dir_path
		return info

	var f := FileAccess.open(meta_path, FileAccess.READ)
	if f == null:
		info["message"] = "Could not open %s" % meta_path
		return info

	info["valid"] = true
	var text: String = f.get_as_text()
	var topics: Array[String] = []
	for line in text.split("\n"):
		var s: String = line.strip_edges()
		if s.begins_with("storage_identifier:"):
			info["storage_identifier"] = s.split(":", true, 1)[1].strip_edges()
		elif s.begins_with("name: /"):
			topics.append(s.split(":", true, 1)[1].strip_edges())
	info["topics"] = topics
	info["message"] = "ROS 2 bag verified (%d topics, storage=%s)." % [topics.size(), info["storage_identifier"]]
	return info

static func _register_unavailable_real_channels(rec: TelemetryRecording) -> void:
	var unavail_specs := [
		["tire.fx_rl", "Longitudinal Tire Force Fx RL", "N", "tires", "No wheel-force transducer on real car"],
		["tire.fx_rr", "Longitudinal Tire Force Fx RR", "N", "tires", "No wheel-force transducer on real car"],
		["tire.fy_fl", "Lateral Tire Force Fy FL", "N", "tires", "No wheel-force transducer on real car"],
		["tire.fy_fr", "Lateral Tire Force Fy FR", "N", "tires", "No wheel-force transducer on real car"],
		["tire.fy_rl", "Lateral Tire Force Fy RL", "N", "tires", "No wheel-force transducer on real car"],
		["tire.fy_rr", "Lateral Tire Force Fy RR", "N", "tires", "No wheel-force transducer on real car"],
		["tire.fz_fl", "Normal Load Fz FL", "N", "tires", "No vertical load cell on real car"],
		["tire.fz_fr", "Normal Load Fz FR", "N", "tires", "No vertical load cell on real car"],
		["tire.fz_rl", "Normal Load Fz RL", "N", "tires", "No vertical load cell on real car"],
		["tire.fz_rr", "Normal Load Fz RR", "N", "tires", "No vertical load cell on real car"],
		["susp.heave_fl", "Suspension Compression FL", "m", "suspension", "No suspension potentiometer in real bag"],
		["susp.heave_fr", "Suspension Compression FR", "m", "suspension", "No suspension potentiometer in real bag"],
		["susp.heave_rl", "Suspension Compression RL", "m", "suspension", "No suspension potentiometer in real bag"],
		["susp.heave_rr", "Suspension Compression RR", "m", "suspension", "No suspension potentiometer in real bag"],
		["aero.drag_n", "Aero Drag Force", "N", "aero", "No direct aero load cells on real car"],
		["aero.downforce_front_n", "Aero Front Downforce", "N", "aero", "No direct aero load cells on real car"],
		["aero.downforce_rear_n", "Aero Rear Downforce", "N", "aero", "No direct aero load cells on real car"],
	]
	for spec in unavail_specs:
		if not rec.channels.has(spec[0]):
			rec.add_channel(TelemetryChannel.new(
				spec[0], spec[1], spec[2], spec[3],
				TelemetryChannel.SourceType.UNAVAILABLE,
				"ISO 8855", spec[4]
			))

static func _add_series(
	rec: TelemetryRecording,
	ch_id: String,
	name: String,
	unit: String,
	category: String,
	source: TelemetryChannel.SourceType,
	t_arr: PackedFloat64Array,
	v_arr: PackedFloat64Array,
	frame: String = "ISO 8855 (+X fwd, +Y left, +Z up)",
	note: String = ""
) -> void:
	if t_arr.is_empty() or v_arr.is_empty() or t_arr.size() != v_arr.size():
		return
	var ch := TelemetryChannel.new(ch_id, name, unit, category, source, frame, note)

	var prev_t: float = -INF
	for i in range(t_arr.size()):
		var ti: float = t_arr[i]
		var vi: float = v_arr[i]
		if not is_nan(ti) and not is_inf(ti) and not is_nan(vi) and not is_inf(vi) and ti > prev_t:
			ch.time_s.append(ti)
			ch.raw_values.append(vi)
			ch.conditioned_values.append(vi)
			prev_t = ti
	rec.add_channel(ch)

static func _read_csv_columns(file_path: String, max_rows: int = 60000) -> Dictionary:
	var out: Dictionary = {}
	var f := FileAccess.open(file_path, FileAccess.READ)
	if f == null:
		return out
	if f.eof_reached():
		return out

	var header_line: String = f.get_line().strip_edges()
	if header_line.is_empty():
		return out
	var headers: PackedStringArray = header_line.split(",")
	var num_cols: int = headers.size()
	var col_arrays: Array[PackedFloat64Array] = []
	var is_numeric_col: Array[bool] = []
	for i in range(num_cols):
		col_arrays.append(PackedFloat64Array())
		is_numeric_col.append(headers[i] != "frame_id" and headers[i] != "mode" and headers[i] != "tire_model")

	var file_len: int = f.get_length()
	var approx_rows: int = maxi(1, int(file_len / 75))
	var stride: int = maxi(1, int(ceil(float(approx_rows) / float(maxi(max_rows, 1000)))))

	var row_idx: int = 0
	while not f.eof_reached():
		var line: String = f.get_line()
		if line.is_empty():
			continue
		row_idx += 1
		if stride > 1 and (row_idx % stride) != 1:
			continue
		var parts: PackedStringArray = line.split(",")
		if parts.size() < num_cols:
			continue
		for c in range(num_cols):
			if is_numeric_col[c]:
				col_arrays[c].append(parts[c].to_float())

	for c in range(num_cols):
		if is_numeric_col[c]:
			out[headers[c]] = col_arrays[c]
	return out
