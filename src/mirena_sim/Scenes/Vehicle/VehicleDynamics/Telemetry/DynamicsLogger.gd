extends RefCounted
class_name DynamicsLogger

## Optional offline buffered CSV logger for `DynamicsDiagnostics` snapshots.
##
## Designed so that:
## - Production simulation runs have zero file-I/O overhead (`logging_enabled = false` by default).
## - Offline validation runs can buffer samples in memory and flush once at the end of a maneuver
##   to avoid per-frame disk I/O jitter.

var logging_enabled: bool = false
var max_buffered_samples: int = 40000
var _buffer: Array[Dictionary] = []
var simulation_metadata: Dictionary = {
	"physics_mode": "CUSTOM_RIGID_BODY_3D",
	"config_id": "vehicle_default",
	"tire_model": "TeRTireGreyBoxModel",
	"git_commit": "a0d0415",
	"physics_rate_hz": 400.0,
	"substeps": 1,
}

func clear() -> void:
	_buffer.clear()

func get_sample_count() -> int:
	return _buffer.size()

## Append a diagnostic sample to the in-memory ring/linear buffer if `logging_enabled == true`.
func record_sample(diagnostics: DynamicsDiagnostics) -> void:
	if not logging_enabled or diagnostics == null:
		return
	if _buffer.size() >= max_buffered_samples:
		return
	_buffer.append(diagnostics.to_dictionary())

## Return CSV header string matching either the legacy 30 columns or the expanded 47-column schema.
static func get_csv_header(expanded: bool = false) -> String:
	if not expanded:
		return ",".join([
			"t_s", "u_mps", "v_mps", "ax_mps2", "ay_mps2", "az_mps2", "yaw_rate_rad_s",
			"fz_fl_n", "fz_fr_n", "fz_rl_n", "fz_rr_n",
			"kappa_fl", "kappa_fr", "kappa_rl", "kappa_rr",
			"alpha_fl_rad", "alpha_fr_rad", "alpha_rl_rad", "alpha_rr_rad",
			"fx_fl_n", "fx_fr_n", "fx_rl_n", "fx_rr_n",
			"fy_fl_n", "fy_fr_n", "fy_rl_n", "fy_rr_n",
			"rpm_rl", "rpm_rr", "fault_flags"
		])
	return ",".join([
		"t_s", "throttle_cmd", "brake_cmd", "steer_cmd_rad",
		"steer_fl_rad", "steer_fr_rad",
		"u_mps", "v_mps", "w_mps", "ax_mps2", "ay_mps2", "az_mps2", "yaw_rate_rad_s",
		"roll_rad", "pitch_rad", "beta_rad", "psi_rad", "x_m", "y_m",
		"fz_fl_n", "fz_fr_n", "fz_rl_n", "fz_rr_n",
		"kappa_fl", "kappa_fr", "kappa_rl", "kappa_rr",
		"alpha_fl_rad", "alpha_fr_rad", "alpha_rl_rad", "alpha_rr_rad",
		"fx_fl_n", "fx_fr_n", "fx_rl_n", "fx_rr_n",
		"fy_fl_n", "fy_fr_n", "fy_rl_n", "fy_rr_n",
		"omega_fl", "omega_fr", "omega_rl", "omega_rr",
		"rpm_fl", "rpm_fr", "rpm_rl", "rpm_rr",
		"susp_comp_fl_m", "susp_comp_fr_m", "susp_comp_rl_m", "susp_comp_rr_m",
		"contact_fl", "contact_fr", "contact_rl", "contact_rr",
		"fault_flags"
	])

## Export buffered samples to a CSV file on disk. Returns true on success.
func export_csv(file_path: String, expanded: bool = false) -> bool:
	var file := FileAccess.open(file_path, FileAccess.WRITE)
	if file == null:
		return false
	file.store_line(get_csv_header(expanded))
	for row in _buffer:
		var fz: Array = row["wheel_fz_n"]
		var kp: Array = row["wheel_kappa"]
		var al: Array = row["wheel_alpha_rad"]
		var fx: Array = row["wheel_fx_n"]
		var fy: Array = row["wheel_fy_n"]
		var rpm: Array = row["wheel_rpm"]

		if not expanded:
			var line := "%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d" % [
				row["timestamp_s"], row["u_mps"], row["v_mps"],
				row["ax_mps2"], row["ay_mps2"], row["az_mps2"], row["yaw_rate_rad_s"],
				fz[0], fz[1], fz[2], fz[3],
				kp[0], kp[1], kp[2], kp[3],
				al[0], al[1], al[2], al[3],
				fx[0], fx[1], fx[2], fx[3],
				fy[0], fy[1], fy[2], fy[3],
				rpm[2], rpm[3], row["fault_flags"]
			]
			file.store_line(line)
		else:
			var st: Array = row.get("wheel_steer_angle_rad", [0.0, 0.0, 0.0, 0.0])
			var om: Array = row.get("wheel_omega_rad_s", [0.0, 0.0, 0.0, 0.0])
			var sc: Array = row.get("wheel_compression_m", [0.0, 0.0, 0.0, 0.0])
			var ct: Array = row.get("wheel_in_contact", [true, true, true, true])
			var line := "%.6f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%d" % [
				row["timestamp_s"], row.get("throttle_cmd", 0.0), row.get("brake_cmd", 0.0), row.get("steer_cmd_rad", 0.0),
				st[0], st[1],
				row["u_mps"], row["v_mps"], row.get("w_mps", 0.0),
				row["ax_mps2"], row["ay_mps2"], row["az_mps2"], row["yaw_rate_rad_s"],
				row.get("roll_rad", 0.0), row.get("pitch_rad", 0.0), row.get("beta_rad", 0.0), row.get("psi_rad", 0.0),
				row.get("x_m", 0.0), row.get("y_m", 0.0),
				fz[0], fz[1], fz[2], fz[3],
				kp[0], kp[1], kp[2], kp[3],
				al[0], al[1], al[2], al[3],
				fx[0], fx[1], fx[2], fx[3],
				fy[0], fy[1], fy[2], fy[3],
				om[0], om[1], om[2], om[3],
				rpm[0], rpm[1], rpm[2], rpm[3],
				sc[0], sc[1], sc[2], sc[3],
				1 if ct[0] else 0, 1 if ct[1] else 0, 1 if ct[2] else 0, 1 if ct[3] else 0,
				row["fault_flags"]
			]
			file.store_line(line)
	file.close()
	return true
