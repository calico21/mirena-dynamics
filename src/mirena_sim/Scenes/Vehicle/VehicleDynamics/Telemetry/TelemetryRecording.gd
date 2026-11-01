extends RefCounted
class_name TelemetryRecording

var recording_name: String = "Untitled Recording"

var source_path: String = ""

var source_format: String = "Live Buffer"

var recording_date: String = ""

var physics_mode_label: String = ""

var tire_setup_label: String = ""

var track_condition: String = ""

var is_real_vehicle: bool = false

var maneuver_onset_time_s: float = 0.0

var known_limitations: PackedStringArray = PackedStringArray()

var channels: Dictionary = {}

func _init(p_name: String = "Untitled Recording", p_is_real: bool = false) -> void:
	recording_name = p_name
	is_real_vehicle = p_is_real

func add_channel(channel: TelemetryChannel) -> void:
	if channel == null or channel.channel_id.is_empty():
		return
	channels[channel.channel_id] = channel

func get_channel(channel_id: String) -> TelemetryChannel:
	return channels.get(channel_id, null)

func has_usable_channel(channel_id: String) -> bool:
	var ch: TelemetryChannel = get_channel(channel_id)
	return ch != null and ch.has_data()

func get_channel_ids() -> PackedStringArray:
	var keys := PackedStringArray()
	for k in channels.keys():
		keys.append(str(k))
	keys.sort()
	return keys

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

func shift_all_timestamps(offset_s: float) -> void:
	if absf(offset_s) <= 1e-12:
		return
	for k in channels.keys():
		var ch: TelemetryChannel = channels[k]
		if ch != null:
			for i in range(ch.time_s.size()):
				ch.time_s[i] -= offset_s
	maneuver_onset_time_s -= offset_s
