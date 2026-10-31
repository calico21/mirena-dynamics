extends RefCounted
class_name TelemetryRecording

## Common internal representation for any vehicle telemetry dataset:
## - Real vehicle ROS 2 bag / CSV exports (`imu_compare/real_*.csv`)
## - Saved or replayed simulation runs (`imu_compare/sim_*.csv` or `DynamicsLogger` CSVs)
## - Live in-memory recordings captured directly from `MirenaCar` during driving.

## Recording identifier / session label.
var recording_name: String = "Untitled Recording"
## Source file or directory path on disk (empty if live in-memory).
var source_path: String = ""
## Source format description (e.g., "imu_compare Multi-CSV Bundle", "MirenaSim DynamicsLogger CSV", "Live Buffer").
var source_format: String = "Live Buffer"
## Recording date / timestamp string.
var recording_date: String = ""
## Active physics engine identifier (e.g., "REAL_HARDWARE_TER26", "MODULAR_VEHICLE_DYNAMICS", "LEGACY_VEHICLE_BODY_3D").
var physics_mode_label: String = ""
## Active tire model / setup identifier (e.g., "TeR_Tire Grey-Box PINN", "Godot Wheel Friction front10_rear12").
var tire_setup_label: String = ""
## Track or test condition description.
var track_condition: String = ""
## True if this recording represents real physical vehicle measurements.
var is_real_vehicle: bool = false
## Detected maneuver onset time (first sample where gas > 0.1 or |steer| > 0.05 rad) in relative seconds.
var maneuver_onset_time_s: float = 0.0
## Explicit list of missing or unsupported signals in this recording.
var known_limitations: PackedStringArray = PackedStringArray()
## Dictionary mapping canonical `channel_id: String` -> `TelemetryChannel`.
var channels: Dictionary = {}

func _init(p_name: String = "Untitled Recording", p_is_real: bool = false) -> void:
	recording_name = p_name
	is_real_vehicle = p_is_real

## Register or overwrite a `TelemetryChannel`.
func add_channel(channel: TelemetryChannel) -> void:
	if channel == null or channel.channel_id.is_empty():
		return
	channels[channel.channel_id] = channel

## Retrieve a `TelemetryChannel` by its canonical `channel_id`, or `null` if absent.
func get_channel(channel_id: String) -> TelemetryChannel:
	return channels.get(channel_id, null)

## Return true if a channel exists and contains at least 2 samples.
func has_usable_channel(channel_id: String) -> bool:
	var ch: TelemetryChannel = get_channel(channel_id)
	return ch != null and ch.has_data()

## Return all registered channel IDs sorted alphabetically.
func get_channel_ids() -> PackedStringArray:
	var keys := PackedStringArray()
	for k in channels.keys():
		keys.append(str(k))
	keys.sort()
	return keys

## Return overall `[t_min, t_max]` time span across all usable channels (s).
func get_time_range_s() -> Vector2:
	var t_min: float = INF
	var t_max: float = -INF
	for k in channels.keys():
		var ch: TelemetryChannel = channels[k]
		if ch != null and ch.has_data():
			t_min = minf(t_min, ch.time_s[0])
			t_max = maxf(t_max, ch.time_s[ch.time_s.size() - 1])
	if t_min == INF or t_max == -INF:
		return Vector2.ZERO
	return Vector2(t_min, t_max)

## Detect maneuver onset time `t_onset` where `control.gas > 0.1` (or `|control.steer_rad| > 0.05`),
## matching `imu_compare/compare_imu.py::first_onset`.
func detect_maneuver_onset() -> float:
	var gas_ch: TelemetryChannel = get_channel("control.gas")
	if gas_ch != null and gas_ch.has_data():
		for i in range(gas_ch.time_s.size()):
			if gas_ch.raw_values[i] > 0.1:
				maneuver_onset_time_s = gas_ch.time_s[i]
				return maneuver_onset_time_s
	var steer_ch: TelemetryChannel = get_channel("control.steer_rad")
	if steer_ch != null and steer_ch.has_data():
		for i in range(steer_ch.time_s.size()):
			if absf(steer_ch.raw_values[i]) > 0.05:
				maneuver_onset_time_s = steer_ch.time_s[i]
				return maneuver_onset_time_s
	var rng: Vector2 = get_time_range_s()
	maneuver_onset_time_s = rng.x
	return maneuver_onset_time_s

## Shift all channel timestamps by `-offset_s` (e.g., to align `tau = t_rel - t_onset` so `tau = 0` is throttle onset).
func shift_all_timestamps(offset_s: float) -> void:
	if absf(offset_s) <= 1e-12:
		return
	for k in channels.keys():
		var ch: TelemetryChannel = channels[k]
		if ch != null:
			for i in range(ch.time_s.size()):
				ch.time_s[i] -= offset_s
	maneuver_onset_time_s -= offset_s
