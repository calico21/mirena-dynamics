extends RefCounted
class_name TelemetryChannel

enum SourceType {
	MEASURED_REAL = 0,
	SIMULATED_MODULAR = 1,
	SIMULATED_LEGACY = 2,
	DERIVED_ESTIMATE = 3,
	REFERENCE_MODEL = 4,
	UNAVAILABLE = 5,
}

var channel_id: String = ""

var display_name: String = ""

var category_id: String = "motion"

var unit: String = ""

var frame_convention: String = "ISO 8855 (+X fwd, +Y left, +Z up)"

var source_type: SourceType = SourceType.UNAVAILABLE

var provenance_note: String = ""

var time_s: PackedFloat64Array = PackedFloat64Array()

var raw_values: PackedFloat64Array = PackedFloat64Array()

var conditioned_values: PackedFloat64Array = PackedFloat64Array()

var removed_bias: float = 0.0

func _init(
	p_id: String = "",
	p_name: String = "",
	p_unit: String = "",
	p_category: String = "motion",
	p_source: SourceType = SourceType.UNAVAILABLE,
	p_frame: String = "ISO 8855 (+X fwd, +Y left, +Z up)",
	p_note: String = ""
) -> void:
	channel_id = p_id
	display_name = p_name
	unit = p_unit
	category_id = p_category
	source_type = p_source
	frame_convention = p_frame
	provenance_note = p_note

func append_sample(t: float, val: float, cond_val: float = NAN) -> void:
	time_s.append(t)
	raw_values.append(val)
	conditioned_values.append(val if is_nan(cond_val) else cond_val)

func get_sample_count() -> int:
	return time_s.size()

func has_data() -> bool:
	return source_type != SourceType.UNAVAILABLE and time_s.size() >= 2

func get_sampling_rate_hz() -> float:
	var n: int = time_s.size()
	if n < 2:
		return 0.0
	var dt_total: float = time_s[n - 1] - time_s[0]
	if dt_total <= 1e-9:
		return 0.0
	return float(n - 1) / dt_total

func sample_at(t_query: float, use_conditioned: bool = true) -> float:
	var n: int = time_s.size()
	if n == 0:
		return 0.0
	var vals: PackedFloat64Array = conditioned_values if (use_conditioned and conditioned_values.size() == n) else raw_values
	if n == 1 or t_query <= time_s[0]:
		return vals[0]
	if t_query >= time_s[n - 1]:
		return vals[n - 1]

	var lo: int = 0
	var hi: int = n - 1
	while hi - lo > 1:
		var mid: int = (lo + hi) >> 1
		if time_s[mid] <= t_query:
			lo = mid
		else:
			hi = mid
	var t0: float = time_s[lo]
	var t1: float = time_s[hi]
	var span: float = t1 - t0
	if span <= 1e-12:
		return vals[lo]
	var alpha: float = (t_query - t0) / span
	return lerpf(vals[lo], vals[hi], alpha)

func apply_bias_removal(t_bias_start: float, t_bias_end: float) -> void:
	var n: int = time_s.size()
	if n == 0:
		return
	var sum_val: float = 0.0
	var count: int = 0
	for i in range(n):
		var t: float = time_s[i]
		if t >= t_bias_start and t <= t_bias_end:
			sum_val += raw_values[i]
			count += 1
	removed_bias = (sum_val / float(count)) if count >= 3 else 0.0
	conditioned_values.resize(n)
	for i in range(n):
		conditioned_values[i] = raw_values[i] - removed_bias

static func source_type_name(st: SourceType) -> String:
	match st:
		SourceType.MEASURED_REAL:
			return "Measured (Real Car)"
		SourceType.SIMULATED_MODULAR:
			return "Simulated (Modular Dynamics)"
		SourceType.SIMULATED_LEGACY:
			return "Simulated (Legacy VehicleBody3D)"
		SourceType.DERIVED_ESTIMATE:
			return "Derived / Estimated"
		SourceType.REFERENCE_MODEL:
			return "Reference Model"
		SourceType.UNAVAILABLE, _:
			return "Unavailable (No Sensor)"
