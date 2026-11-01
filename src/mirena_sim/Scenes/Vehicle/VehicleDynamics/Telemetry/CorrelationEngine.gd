extends RefCounted
class_name CorrelationEngine

enum EvaluationStatus {
	NOT_EVALUATED = 0,
	AWAITING_REFERENCE_DATA = 1,
	INSUFFICIENT_DATA = 2,
	EVALUATED = 3,
	VALIDATION_WARNING = 4,
}

const MIN_OVERLAP_SAMPLES: int = 10
const EPS: float = 1.0e-9

static func status_to_string(st: EvaluationStatus) -> String:
	match st:
		EvaluationStatus.NOT_EVALUATED:
			return "Not evaluated"
		EvaluationStatus.AWAITING_REFERENCE_DATA:
			return "Awaiting reference data"
		EvaluationStatus.INSUFFICIENT_DATA:
			return "Insufficient data"
		EvaluationStatus.EVALUATED:
			return "Evaluated"
		EvaluationStatus.VALIDATION_WARNING:
			return "Validation warning"
		_:
			return "Unknown"

static func status_to_color(st: EvaluationStatus) -> Color:
	match st:
		EvaluationStatus.EVALUATED:
			return Color(0.22, 0.84, 0.45)
		EvaluationStatus.VALIDATION_WARNING:
			return Color(0.96, 0.68, 0.18)
		EvaluationStatus.AWAITING_REFERENCE_DATA:
			return Color(0.45, 0.68, 0.95)
		EvaluationStatus.INSUFFICIENT_DATA:
			return Color(0.88, 0.42, 0.35)
		EvaluationStatus.NOT_EVALUATED, _:
			return Color(0.62, 0.65, 0.70)

static func evaluate_channel(
	ref_rec: TelemetryRecording,
	sim_rec: TelemetryRecording,
	channel_id: String,
	tau_start_s: float = 20.0,
	tau_end_s: float = 95.0,
	dt_grid_s: float = 0.02,
	global_sim_time_offset_s: float = 0.0,
	align_by_maneuver_onset: bool = true,
	remove_pre_onset_bias: bool = true
) -> Dictionary:
	var result := {
		"channel_id": channel_id,
		"display_name": channel_id,
		"unit": "",
		"category_id": "motion",
		"ref_source": TelemetryChannel.SourceType.UNAVAILABLE,
		"sim_source": TelemetryChannel.SourceType.UNAVAILABLE,
		"status": EvaluationStatus.NOT_EVALUATED,
		"status_text": status_to_string(EvaluationStatus.NOT_EVALUATED),
		"status_reason": "",
		"sample_count": 0,
		"rmse": NAN,
		"mae": NAN,
		"bias": NAN,
		"pearson_r": NAN,
		"nrmse_std": NAN,
		"nrmse_range": NAN,
		"peak_abs_error": NAN,
		"peak_amp_diff": NAN,
		"gain_sim_over_ref": NAN,
		"estimated_lag_s": NAN,
		"ref_mean": NAN,
		"sim_mean": NAN,
		"ref_std": NAN,
		"sim_std": NAN,
		"aligned_time_s": PackedFloat64Array(),
		"aligned_ref": PackedFloat64Array(),
		"aligned_sim": PackedFloat64Array(),
		"residual": PackedFloat64Array(),
	}

	if sim_rec == null:
		result["status"] = EvaluationStatus.NOT_EVALUATED
		result["status_text"] = status_to_string(EvaluationStatus.NOT_EVALUATED)
		result["status_reason"] = "No simulation recording loaded."
		return result

	var sim_ch: TelemetryChannel = sim_rec.get_channel(channel_id)
	var ref_ch: TelemetryChannel = ref_rec.get_channel(channel_id) if ref_rec != null else null

	if sim_ch != null:
		result["display_name"] = sim_ch.display_name
		result["unit"] = sim_ch.unit
		result["category_id"] = sim_ch.category_id
		result["sim_source"] = sim_ch.source_type
	if ref_ch != null:
		if result["display_name"] == channel_id and not ref_ch.display_name.is_empty():
			result["display_name"] = ref_ch.display_name
		if result["unit"].is_empty():
			result["unit"] = ref_ch.unit
		result["category_id"] = ref_ch.category_id
		result["ref_source"] = ref_ch.source_type

	if ref_rec == null or ref_ch == null or ref_ch.source_type == TelemetryChannel.SourceType.UNAVAILABLE or not ref_ch.has_data():
		result["status"] = EvaluationStatus.AWAITING_REFERENCE_DATA
		result["status_text"] = status_to_string(EvaluationStatus.AWAITING_REFERENCE_DATA)
		result["status_reason"] = (
			ref_ch.provenance_note if (ref_ch != null and not ref_ch.provenance_note.is_empty())
			else "No real-vehicle measurement channel available for '%s'." % channel_id
		)
		return result

	if sim_ch == null or not sim_ch.has_data():
		result["status"] = EvaluationStatus.INSUFFICIENT_DATA
		result["status_text"] = status_to_string(EvaluationStatus.INSUFFICIENT_DATA)
		result["status_reason"] = "Simulation recording has no samples for '%s'." % channel_id
		return result

	var onset_ref: float = ref_rec.maneuver_onset_time_s if align_by_maneuver_onset else 0.0
	var onset_sim: float = sim_rec.maneuver_onset_time_s if align_by_maneuver_onset else 0.0

	var ref_tau_min: float = ref_ch.time_s[0] - onset_ref
	var ref_tau_max: float = ref_ch.time_s[ref_ch.time_s.size() - 1] - onset_ref
	var sim_tau_min: float = sim_ch.time_s[0] - onset_sim + global_sim_time_offset_s
	var sim_tau_max: float = sim_ch.time_s[sim_ch.time_s.size() - 1] - onset_sim + global_sim_time_offset_s

	var win_start: float = maxf(tau_start_s, maxf(ref_tau_min, sim_tau_min))
	var win_end: float = minf(tau_end_s, minf(ref_tau_max, sim_tau_max))
	var step: float = maxf(dt_grid_s, 0.002)

	if win_end - win_start < step * float(MIN_OVERLAP_SAMPLES):
		result["status"] = EvaluationStatus.INSUFFICIENT_DATA
		result["status_text"] = status_to_string(EvaluationStatus.INSUFFICIENT_DATA)
		result["status_reason"] = "Insufficient overlapping time span ([%.2f, %.2f] s)." % [win_start, win_end]
		return result

	var bias_ref: float = 0.0
	var bias_sim: float = 0.0
	if remove_pre_onset_bias and channel_id.begins_with("imu."):
		bias_ref = _compute_window_mean(ref_ch, onset_ref - 15.0, onset_ref - 1.0)
		bias_sim = _compute_window_mean(sim_ch, onset_sim - 15.0, onset_sim - 1.0)

	var n_samples: int = int(floor((win_end - win_start) / step)) + 1
	var grid_t := PackedFloat64Array()
	var arr_ref := PackedFloat64Array()
	var arr_sim := PackedFloat64Array()
	var arr_res := PackedFloat64Array()
	grid_t.resize(n_samples)
	arr_ref.resize(n_samples)
	arr_sim.resize(n_samples)
	arr_res.resize(n_samples)

	var sum_ref: float = 0.0
	var sum_sim: float = 0.0
	var sum_sq_err: float = 0.0
	var sum_abs_err: float = 0.0
	var sum_err: float = 0.0
	var peak_abs_err: float = 0.0
	var peak_abs_ref: float = 0.0
	var peak_abs_sim: float = 0.0
	var ref_min: float = INF
	var ref_max: float = -INF

	for i in range(n_samples):
		var tau: float = win_start + float(i) * step
		var r_val: float = ref_ch.sample_at(tau + onset_ref, false) - bias_ref
		var s_val: float = sim_ch.sample_at(tau + onset_sim - global_sim_time_offset_s, false) - bias_sim
		var err: float = s_val - r_val

		grid_t[i] = tau
		arr_ref[i] = r_val
		arr_sim[i] = s_val
		arr_res[i] = err

		sum_ref += r_val
		sum_sim += s_val
		sum_sq_err += err * err
		sum_abs_err += absf(err)
		sum_err += err
		peak_abs_err = maxf(peak_abs_err, absf(err))
		peak_abs_ref = maxf(peak_abs_ref, absf(r_val))
		peak_abs_sim = maxf(peak_abs_sim, absf(s_val))
		ref_min = minf(ref_min, r_val)
		ref_max = maxf(ref_max, r_val)

	var inv_n: float = 1.0 / float(n_samples)
	var mean_ref: float = sum_ref * inv_n
	var mean_sim: float = sum_sim * inv_n
	var rmse: float = sqrt(sum_sq_err * inv_n)
	var mae: float = sum_abs_err * inv_n
	var bias_err: float = sum_err * inv_n

	var var_ref_acc: float = 0.0
	var var_sim_acc: float = 0.0
	var cov_acc: float = 0.0
	for i in range(n_samples):
		var dr: float = arr_ref[i] - mean_ref
		var ds: float = arr_sim[i] - mean_sim
		var_ref_acc += dr * dr
		var_sim_acc += ds * ds
		cov_acc += dr * ds

	var std_ref: float = sqrt(var_ref_acc * inv_n)
	var std_sim: float = sqrt(var_sim_acc * inv_n)
	var denom_corr: float = sqrt(var_ref_acc * var_sim_acc)
	var pearson_r: float = (cov_acc / denom_corr) if denom_corr > EPS else 0.0
	var gain_ratio: float = (cov_acc / var_ref_acc) if var_ref_acc > EPS else NAN
	var nrmse_std: float = rmse / maxf(std_ref, 1e-4)
	var nrmse_range: float = rmse / maxf(ref_max - ref_min, 1e-4)
	var lag_s: float = _estimate_bounded_lag_s(arr_ref, arr_sim, mean_ref, mean_sim, denom_corr, step, 2.5)

	var eval_status: EvaluationStatus = EvaluationStatus.EVALUATED
	var reason: String = "Quantitatively evaluated across %d samples ([%.1f, %.1f] s)." % [n_samples, win_start, win_end]
	if pearson_r < 0.20 or nrmse_std > 1.35:
		eval_status = EvaluationStatus.VALIDATION_WARNING
		reason = "Discrepancy warning: r=%.2f, NRMSE=%.2f, gain=%.2f." % [pearson_r, nrmse_std, gain_ratio]

	result["status"] = eval_status
	result["status_text"] = status_to_string(eval_status)
	result["status_reason"] = reason
	result["sample_count"] = n_samples
	result["rmse"] = rmse
	result["mae"] = mae
	result["bias"] = bias_err
	result["pearson_r"] = pearson_r
	result["nrmse_std"] = nrmse_std
	result["nrmse_range"] = nrmse_range
	result["peak_abs_error"] = peak_abs_err
	result["peak_amp_diff"] = peak_abs_sim - peak_abs_ref
	result["gain_sim_over_ref"] = gain_ratio
	result["estimated_lag_s"] = lag_s
	result["ref_mean"] = mean_ref
	result["sim_mean"] = mean_sim
	result["ref_std"] = std_ref
	result["sim_std"] = std_sim
	result["aligned_time_s"] = grid_t
	result["aligned_ref"] = arr_ref
	result["aligned_sim"] = arr_sim
	result["residual"] = arr_res
	return result

static func evaluate_all_channels(
	ref_rec: TelemetryRecording,
	sim_rec: TelemetryRecording,
	tau_start_s: float = 20.0,
	tau_end_s: float = 95.0,
	dt_grid_s: float = 0.02,
	global_sim_time_offset_s: float = 0.0
) -> Dictionary:
	var channel_list := PackedStringArray([
		"chassis.u", "chassis.v", "imu.ax", "imu.ay", "imu.gz", "chassis.r",
		"imu.gx", "imu.gy", "imu.az", "chassis.psi", "chassis.x", "chassis.y",
		"control.steer_rad", "control.gas",
		"wheel.rl_rpm", "wheel.rr_rpm",
		"tire.fx_rl", "tire.fx_rr", "tire.fy_fl", "tire.fy_fr", "tire.fy_rl", "tire.fy_rr",
		"tire.fz_fl", "tire.fz_fr", "tire.fz_rl", "tire.fz_rr",
		"susp.heave_fl", "susp.heave_fr", "susp.heave_rl", "susp.heave_rr",
		"aero.drag_n", "aero.downforce_front_n", "aero.downforce_rear_n",
	])
	var metrics_by_channel := {}
	for ch_id in channel_list:
		metrics_by_channel[ch_id] = evaluate_channel(
			ref_rec, sim_rec, ch_id, tau_start_s, tau_end_s, dt_grid_s, global_sim_time_offset_s
		)
	return metrics_by_channel

const PRIMARY_CORRELATION_WEIGHTS: Dictionary = {
	"imu.ay": 0.30,
	"imu.gz": 0.30,
	"chassis.u": 0.20,
	"imu.ax": 0.15,
	"chassis.v": 0.05,
}

const MIN_AGGREGATE_SAMPLES: int = 30
const MIN_DYNAMIC_STD: float = 1.0e-3

static func compute_aggregate_vehicle_correlation(
	all_metrics: Dictionary,
	context_mode: String = "LAST"
) -> Dictionary:
	var summary := {
		"state": "N/A",
		"correlation_pct": NAN,
		"fisher_r": NAN,
		"mean_nrmse_std": NAN,
		"mean_nrmse_range": NAN,
		"valid_channel_count": 0,
		"min_sample_count": 0,
		"hud_text": "CORR: N/A",
		"hud_color": Color(0.68, 0.72, 0.78),
		"details": "No real reference telemetry loaded.",
		"channel_scores": {},
	}

	if all_metrics.is_empty():
		return summary

	var any_ref_present: bool = false
	var any_insufficient: bool = false
	var z_weighted_sum: float = 0.0
	var nrmse_std_sum: float = 0.0
	var nrmse_rng_sum: float = 0.0
	var weight_sum: float = 0.0
	var valid_count: int = 0
	var min_samples: int = 999999999
	var ch_scores: Dictionary = {}

	for ch_id in PRIMARY_CORRELATION_WEIGHTS.keys():
		if not all_metrics.has(ch_id):
			continue
		var m: Dictionary = all_metrics[ch_id]
		var st: int = int(m.get("status", EvaluationStatus.NOT_EVALUATED))
		if st != EvaluationStatus.AWAITING_REFERENCE_DATA and st != EvaluationStatus.NOT_EVALUATED:
			any_ref_present = true
		if st == EvaluationStatus.INSUFFICIENT_DATA:
			any_insufficient = true
			continue
		if st != EvaluationStatus.EVALUATED and st != EvaluationStatus.VALIDATION_WARNING:
			continue

		var n_samp: int = int(m.get("sample_count", 0))
		if n_samp < MIN_AGGREGATE_SAMPLES:
			any_insufficient = true
			continue

		var r_std: float = float(m.get("ref_std", 0.0))
		var s_std: float = float(m.get("sim_std", 0.0))
		if not is_finite(r_std) or not is_finite(s_std) or r_std < MIN_DYNAMIC_STD or s_std < MIN_DYNAMIC_STD:

			continue

		var r_val: float = float(m.get("pearson_r", NAN))
		var nrmse_s: float = float(m.get("nrmse_std", NAN))
		var nrmse_r: float = float(m.get("nrmse_range", NAN))
		if not is_finite(r_val) or not is_finite(nrmse_s) or not is_finite(nrmse_r):
			continue

		var w: float = float(PRIMARY_CORRELATION_WEIGHTS[ch_id])
		var r_clamped: float = clampf(r_val, -0.999, 0.999)
		var z_k: float = 0.5 * log((1.0 + r_clamped) / (1.0 - r_clamped))
		z_weighted_sum += w * z_k
		nrmse_std_sum += w * clampf(nrmse_s, 0.0, 5.0)
		nrmse_rng_sum += w * clampf(nrmse_r, 0.0, 5.0)
		weight_sum += w
		valid_count += 1
		min_samples = mini(min_samples, n_samp)

		var amp_k: float = clampf(1.0 - 0.5 * nrmse_r, 0.0, 1.0)
		var pct_k: float = 100.0 * clampf(maxf(0.0, r_clamped) * (0.75 + 0.25 * amp_k), 0.0, 1.0)
		ch_scores[ch_id] = pct_k

	if not any_ref_present:
		return summary

	if valid_count == 0 or weight_sum <= EPS:
		if context_mode == "STANDBY":
			summary["state"] = "STANDBY"
			summary["hud_text"] = "CORR: STANDBY"
			summary["hud_color"] = Color(0.45, 0.74, 0.96)
			summary["details"] = "Reference data ready — awaiting maneuver onset / sample window."
		elif any_insufficient:
			summary["state"] = "INSUF_DATA"
			summary["hud_text"] = "CORR: INSUF DATA"
			summary["hud_color"] = Color(0.92, 0.56, 0.36)
			summary["details"] = "Fewer than %d overlapping dynamic samples in window." % MIN_AGGREGATE_SAMPLES
		return summary

	var z_bar: float = z_weighted_sum / weight_sum
	var r_bar: float = tanh(z_bar)
	var nrmse_std_bar: float = nrmse_std_sum / weight_sum
	var nrmse_rng_bar: float = nrmse_rng_sum / weight_sum
	var amp_factor: float = clampf(1.0 - 0.5 * nrmse_rng_bar, 0.0, 1.0)
	var corr_pct: float = 100.0 * clampf(maxf(0.0, r_bar) * (0.75 + 0.25 * amp_factor), 0.0, 1.0)

	summary["correlation_pct"] = corr_pct
	summary["fisher_r"] = r_bar
	summary["mean_nrmse_std"] = nrmse_std_bar
	summary["mean_nrmse_range"] = nrmse_rng_bar
	summary["valid_channel_count"] = valid_count
	summary["min_sample_count"] = min_samples if min_samples < 999999999 else 0
	summary["channel_scores"] = ch_scores
	summary["channel_metrics"] = all_metrics

	var is_warn: bool = (corr_pct < 50.0 or r_bar < 0.35 or nrmse_std_bar > 1.35)
	if is_warn:
		summary["state"] = "WARN"
		summary["hud_text"] = "CORR: %.1f%% [WARN]" % corr_pct
		summary["hud_color"] = Color(0.96, 0.68, 0.20)
	elif context_mode == "LIVE":
		summary["state"] = "LIVE"
		summary["hud_text"] = "CORR: %.1f%% (LIVE)" % corr_pct
		summary["hud_color"] = Color(0.26, 0.92, 0.54)
	else:
		summary["state"] = "LAST"
		summary["hud_text"] = "CORR: %.1f%% (LAST)" % corr_pct
		summary["hud_color"] = Color(0.42, 0.88, 0.62)

	summary["details"] = "Fisher-z r=%+.2f | NRMSE_rng=%.2f | %d channels (N>=%d)" % [
		r_bar, nrmse_rng_bar, valid_count, summary["min_sample_count"]
	]
	return summary

static func estimate_global_sync_offset_s(
	ref_rec: TelemetryRecording,
	sim_rec: TelemetryRecording,
	sync_channel_id: String = "imu.gz",
	tau_start_s: float = 20.0,
	tau_end_s: float = 95.0
) -> float:
	var res: Dictionary = evaluate_channel(ref_rec, sim_rec, sync_channel_id, tau_start_s, tau_end_s, 0.01, 0.0)
	var lag: float = float(res.get("estimated_lag_s", 0.0))
	if is_nan(lag) or is_inf(lag):
		return 0.0
	return lag

static func _compute_window_mean(ch: TelemetryChannel, t_lo: float, t_hi: float) -> float:
	var sum_v: float = 0.0
	var count: int = 0
	for i in range(ch.time_s.size()):
		var t: float = ch.time_s[i]
		if t >= t_lo and t <= t_hi:
			sum_v += ch.raw_values[i]
			count += 1
	return (sum_v / float(count)) if count >= 5 else 0.0

static func _estimate_bounded_lag_s(
	ref_arr: PackedFloat64Array,
	sim_arr: PackedFloat64Array,
	mean_ref: float,
	mean_sim: float,
	denom_corr: float,
	dt_s: float,
	max_lag_s: float
) -> float:
	var n: int = ref_arr.size()
	if n < 20 or denom_corr <= EPS:
		return 0.0
	var max_k: int = mini(int(round(max_lag_s / dt_s)), n / 4)
	var best_lag_k: int = 0
	var best_corr: float = -INF
	for k in range(-max_k, max_k + 1):
		var i_start: int = maxi(0, -k)
		var i_end: int = mini(n, n - k)
		var acc: float = 0.0
		for i in range(i_start, i_end):
			acc += (ref_arr[i] - mean_ref) * (sim_arr[i + k] - mean_sim)
		var c: float = acc / denom_corr
		if c > best_corr:
			best_corr = c
			best_lag_k = k
	return float(best_lag_k) * dt_s
