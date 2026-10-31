extends RefCounted
class_name TelemetryChannel

## Single time-series signal channel inside a `TelemetryRecording`.
##
## Preserves both raw and conditioned values, explicit physical units, coordinate-frame
## convention, and honest provenance (`SourceType`) so the dashboard and correlation engine
## never confuse measured real-car quantities with simulated or derived estimates.

enum SourceType {
	MEASURED_REAL = 0,     ## Directly measured by physical vehicle hardware (e.g., VN-300 IMU, Hall encoders, CAN bus).
	SIMULATED_MODULAR = 1, ## Computed by the modular VehicleDynamics physics engine.
	SIMULATED_LEGACY = 2,  ## Computed by the legacy Godot VehicleBody3D physics engine.
	DERIVED_ESTIMATE = 3,  ## Derived/calculated quantity (e.g., wheel-speed odometry speed, companion longitudinal tire force).
	REFERENCE_MODEL = 4,   ## Analytical or reference model trace (e.g., bicycle model yaw-rate reference).
	UNAVAILABLE = 5,       ## Signal not present in this recording (e.g., real-car suspension travel or real tire forces).
}

## Canonical internal channel key (e.g., "imu.ax", "imu.gz", "chassis.u", "wheel.rl_rpm").
var channel_id: String = ""
## Human-readable display label (e.g., "Longitudinal Accel ax").
var display_name: String = ""
## Vehicle-dynamics dashboard area (1..8 corresponding to Sections 3.1..3.8).
var category_id: String = "motion"
## Physical unit string (e.g., "m/s^2", "rad/s", "m/s", "N", "N*m", "rad", "RPM").
var unit: String = ""
## Coordinate frame and sign convention description (e.g., "ISO 8855 car/cog (+X fwd, +Y left, +Z up)").
var frame_convention: String = "ISO 8855 (+X fwd, +Y left, +Z up)"
## Honest provenance of this channel.
var source_type: SourceType = SourceType.UNAVAILABLE
## Detailed provenance note (e.g., "/sensors/imu gravity-free linear_acceleration.x").
var provenance_note: String = ""
## Monotonically increasing relative timestamp array (s).
var time_s: PackedFloat64Array = PackedFloat64Array()
## Raw unconditioned signal samples in SI/canonical units.
var raw_values: PackedFloat64Array = PackedFloat64Array()
## Conditioned signal samples (e.g., static bias removed or matched LPF applied).
var conditioned_values: PackedFloat64Array = PackedFloat64Array()
## Estimated static bias removed during conditioning (in channel units).
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

## Append a single sample `(t, val)` to both raw and conditioned buffers.
func append_sample(t: float, val: float, cond_val: float = NAN) -> void:
	time_s.append(t)
	raw_values.append(val)
	conditioned_values.append(val if is_nan(cond_val) else cond_val)

## Return number of samples in this channel.
func get_sample_count() -> int:
	return time_s.size()

## Return true if this channel contains usable time-series data.
func has_data() -> bool:
	return source_type != SourceType.UNAVAILABLE and time_s.size() >= 2

## Estimate median sampling rate (Hz).
func get_sampling_rate_hz() -> float:
	var n: int = time_s.size()
	if n < 2:
		return 0.0
	var dt_total: float = time_s[n - 1] - time_s[0]
	if dt_total <= 1e-9:
		return 0.0
	return float(n - 1) / dt_total

## Evaluate channel by linear interpolation at query time `t_query` (using conditioned values by default).
func sample_at(t_query: float, use_conditioned: bool = true) -> float:
	var n: int = time_s.size()
	if n == 0:
		return 0.0
	var vals: PackedFloat64Array = conditioned_values if (use_conditioned and conditioned_values.size() == n) else raw_values
	if n == 1 or t_query <= time_s[0]:
		return vals[0]
	if t_query >= time_s[n - 1]:
		return vals[n - 1]

	# Binary search for bracket [lo, hi]
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

## Remove static bias computed over `[t_bias_start, t_bias_end]` and populate `conditioned_values`.
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

## Return human-readable badge string for `SourceType`.
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
