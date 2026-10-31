extends SceneTree

const DT_SIM: float = 0.0025 # 400 Hz
const DECIMATION: int = 4   # Output at 100 Hz

func _init() -> void:
	var args: PackedStringArray = OS.get_cmdline_user_args()
	var config_path: String = "res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_default.tres"
	var out_prefix: String = "modular_baseline"
	var duration_s: float = 281.0
	var offset_s: float = 0.0

	var i: int = 0
	while i < args.size():
		var arg: String = args[i]
		if arg == "--config" and i + 1 < args.size():
			config_path = args[i + 1]
			i += 2
		elif arg == "--out-prefix" and i + 1 < args.size():
			out_prefix = args[i + 1]
			i += 2
		elif arg == "--duration" and i + 1 < args.size():
			duration_s = float(args[i + 1])
			i += 2
		elif arg == "--offset" and i + 1 < args.size():
			offset_s = float(args[i + 1])
			i += 2
		else:
			i += 1

	print("=== Running Modular Open-Loop Replay ===")
	print("  Config: %s" % config_path)
	print("  Out prefix: %s" % out_prefix)
	print("  Duration: %.1f s" % duration_s)

	var base_dir: String = ProjectSettings.globalize_path("res://../../imu_compare")
	if not DirAccess.dir_exists_absolute(base_dir):
		base_dir = "/home/alexr/eRacing/mirena_ws-SIM/imu_compare"

	# 1. Load real_control.csv
	var ctl_file_path: String = base_dir.path_join("real_control.csv")
	var ctl_file := FileAccess.open(ctl_file_path, FileAccess.READ)
	if ctl_file == null:
		printerr("Cannot open %s" % ctl_file_path)
		quit(1)
		return

	var header_line := ctl_file.get_line()
	var ctl_t_rel: Array[float] = []
	var ctl_t_unix: Array[float] = []
	var ctl_gas: Array[float] = []
	var ctl_steer: Array[float] = []

	var onset_t_rel: float = -1.0
	var onset_t_unix: float = 0.0

	while not ctl_file.eof_reached():
		var line := ctl_file.get_line().strip_edges()
		if line.is_empty():
			continue
		var parts := line.split(",")
		if parts.size() < 5:
			continue
		var tu := float(parts[0])
		var tr := float(parts[1])
		var g := float(parts[2])
		var st := float(parts[4])

		ctl_t_unix.append(tu)
		ctl_t_rel.append(tr)
		ctl_gas.append(g)
		ctl_steer.append(st)

		if onset_t_rel < 0.0 and g > 0.1:
			onset_t_rel = tr
			onset_t_unix = tu

	ctl_file.close()
	print("  Loaded %d control samples. Onset at t_rel=%.3f s" % [ctl_t_rel.size(), onset_t_rel])

	# 2. Setup VehicleDynamicsModel
	var cfg_res: VehicleDynamicsConfig = load(config_path)
	if cfg_res == null:
		printerr("Failed to load config: %s" % config_path)
		quit(1)
		return

	var model := VehicleDynamicsModel.new(cfg_res.duplicate(true))
	model.contact_query.use_analytic_plane = true
	model.reset()

	# 3. Open output CSV files
	var imu_out_path := base_dir.path_join("sim_imu_%s.csv" % out_prefix)
	var ctl_out_path := base_dir.path_join("sim_control_%s.csv" % out_prefix)
	var state_out_path := base_dir.path_join("sim_state_%s.csv" % out_prefix)
	var logger_out_path := base_dir.path_join("reports/modular_replay_%s.csv" % out_prefix)

	var imu_out := FileAccess.open(imu_out_path, FileAccess.WRITE)
	var ctl_out := FileAccess.open(ctl_out_path, FileAccess.WRITE)
	var state_out := FileAccess.open(state_out_path, FileAccess.WRITE)

	imu_out.store_line("t_unix,t_rel,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,frame_id")
	ctl_out.store_line("t_unix,t_rel,gas,steer_raw,steer_rad,frame_id")
	state_out.store_line("t_s,tau_s,u_mps,v_mps,w_mps,ax_mps2,ay_mps2,az_mps2,r_rad_s,roll_rad,pitch_rad,yaw_rad,x_m,y_m,rl_rpm,rr_rpm,fz_rl_n,fz_rr_n")

	var logger := DynamicsLogger.new()
	logger.logging_enabled = true
	logger.max_buffered_samples = 120000

	# 4. Step through replay continuously
	var total_steps: int = int(duration_s / DT_SIM)
	var ctl_idx: int = 0
	var n_ctl: int = ctl_t_rel.size()

	# Low-pass filter matching VN-300 sensor response (tau = 0.02 s, fc ~ 8.0 Hz)
	var lpf_tau: float = 0.02
	var lpf_alpha: float = 1.0 - exp(-DT_SIM / lpf_tau)
	var filt_ax: float = 0.0
	var filt_ay: float = 0.0
	var filt_az: float = 0.0
	var filt_gx: float = 0.0
	var filt_gy: float = 0.0
	var filt_gz: float = 0.0

	for step_idx in range(total_steps):
		var tau: float = float(step_idx) * DT_SIM
		var t_rel_target: float = onset_t_rel + tau + offset_s
		var t_unix_target: float = onset_t_unix + tau + offset_s

		# Linear interpolation of control inputs
		while ctl_idx + 1 < n_ctl and ctl_t_rel[ctl_idx + 1] < t_rel_target:
			ctl_idx += 1

		var cur_gas: float = 0.0
		var cur_steer: float = 0.0
		if ctl_idx + 1 < n_ctl:
			var t0: float = ctl_t_rel[ctl_idx]
			var t1: float = ctl_t_rel[ctl_idx + 1]
			var frac: float = clampf((t_rel_target - t0) / maxf(t1 - t0, 1e-6), 0.0, 1.0)
			cur_gas = lerpf(ctl_gas[ctl_idx], ctl_gas[ctl_idx + 1], frac)
			cur_steer = lerpf(ctl_steer[ctl_idx], ctl_steer[ctl_idx + 1], frac)
		else:
			cur_gas = ctl_gas[ctl_idx]
			cur_steer = ctl_steer[ctl_idx]

		model.set_control_commands(cur_gas, cur_steer, false)
		model.step(DT_SIM, true, false)

		# Apply IMU sensor filter
		filt_ax = lerpf(filt_ax, model.state.ax_mps2, lpf_alpha)
		filt_ay = lerpf(filt_ay, model.state.ay_mps2, lpf_alpha)
		filt_az = lerpf(filt_az, model.state.az_mps2, lpf_alpha)
		filt_gx = lerpf(filt_gx, model.state.p_rad_s, lpf_alpha)
		filt_gy = lerpf(filt_gy, model.state.q_rad_s, lpf_alpha)
		filt_gz = lerpf(filt_gz, model.state.r_rad_s, lpf_alpha)

		# Output decimated samples at 100 Hz
		if (step_idx % DECIMATION) == 0:
			logger.record_sample(model.diagnostics)
			var q: Quaternion = model.state.global_transform_godot.basis.get_rotation_quaternion()
			imu_out.store_line("%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,car/cog" % [
				t_unix_target, t_rel_target,
				filt_ax, filt_ay, filt_az,
				filt_gx, filt_gy, filt_gz,
				q.x, q.y, q.z, q.w
			])

			ctl_out.store_line("%.9f,%.9f,%.4f,%.4f,%.6f,car/cog" % [
				t_unix_target, t_rel_target,
				cur_gas, rad_to_deg(cur_steer), cur_steer
			])

			var w_rl: WheelState = model.state.wheels[2]
			var w_rr: WheelState = model.state.wheels[3]
			state_out.store_line("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f" % [
				t_rel_target, tau,
				model.state.u_mps, model.state.v_mps, model.state.w_mps,
				filt_ax, filt_ay, filt_az, filt_gz,
				model.state.roll_rad, model.state.pitch_rad, model.state.yaw_rad,
				model.state.x_odom_m, model.state.y_odom_m,
				w_rl.get_rpm(), w_rr.get_rpm(),
				w_rl.suspension_state.normal_load_fz_n, w_rr.suspension_state.normal_load_fz_n
			])

	imu_out.close()
	ctl_out.close()
	state_out.close()

	logger.export_csv(logger_out_path, true)
	print("  Finished %d simulation steps. Outputs written to:" % total_steps)
	print("    %s" % imu_out_path)
	print("    %s" % ctl_out_path)
	print("    %s" % state_out_path)
	print("    %s" % logger_out_path)
	quit(0)
