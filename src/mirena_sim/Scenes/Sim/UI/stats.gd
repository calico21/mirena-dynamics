extends PanelContainer

const MirenaUITheme = preload("res://Scenes/Sim/UI/MirenaUITheme.gd")

## Persistent Horizontal Engineering Metrics Top Bar for MirenaSim
## Designed in exact visual consistency with the Options Menu panel.
## Displays:
## - Row 1: Simulator status (FPS, Time, Cones), Active Physics Mode & Tire Model,
##          Dataset / Evaluation Window, Quantitative Correlation badge, and
##          Integrated Motorsport Applications Dock (VD, CORR, LOG, TIRE, SUSP, SETUP, GRAPHS, MENU).
## - Row 2: Live Vehicle Telemetry (Speed, Ax, Ay, Az, Yaw Rate, Sideslip β, Roll, Pitch)
##          with highest visual priority on Speed, Ax, and Ay, and explicit units on all channels.

@export var car: MirenaCar

# --- Row 1 Nodes ---
@onready var fps_label: Label = find_child("FpsLabel", true, false) as Label
@onready var timer_label: Label = find_child("TimerLabel", true, false) as Label
@onready var fallen_cones_label: Label = find_child("FallenConesLabel", true, false) as Label
@onready var correlation_label: Label = find_child("CorrelationLabel", true, false) as Label

@onready var physics_mode_label: Label = find_child("PhysicsModeLabel", true, false) as Label
@onready var physics_mode_badge: PanelContainer = find_child("PhysicsModeBadge", true, false) as PanelContainer
@onready var dataset_label: Label = find_child("DatasetLabel", true, false) as Label
@onready var correlation_badge: PanelContainer = find_child("CorrelationBadge", true, false) as PanelContainer

# --- Row 2 Live Telemetry Nodes (Prioritized: Speed, Ax, Ay) ---
@onready var live_tag_label: Label = find_child("LiveTagLabel", true, false) as Label
@onready var speed_metric: Label = find_child("SpeedMetric", true, false) as Label
@onready var ax_metric: Label = find_child("AxMetric", true, false) as Label
@onready var ay_metric: Label = find_child("AyMetric", true, false) as Label
@onready var az_metric: Label = find_child("AzMetric", true, false) as Label
@onready var yaw_metric: Label = find_child("YawMetric", true, false) as Label
@onready var beta_metric: Label = find_child("BetaMetric", true, false) as Label
@onready var roll_metric: Label = find_child("RollMetric", true, false) as Label
@onready var pitch_metric: Label = find_child("PitchMetric", true, false) as Label

# Aliases for backward compatibility with tests
@onready var gz_metric: Label = find_child("GzMetric", true, false) as Label
@onready var u_metric: Label = find_child("UMetric", true, false) as Label
@onready var v_metric: Label = find_child("VMetric", true, false) as Label

# Legacy proxy labels for 100% backward compatibility
@onready var _legacy_corr_lbl: Label = get_node_or_null("MarginContainer/VBoxContainer/CorrelationLabel") as Label
@onready var _legacy_fps_lbl: Label = get_node_or_null("MarginContainer/VBoxContainer/FpsLabel") as Label
@onready var _legacy_timer_lbl: Label = get_node_or_null("MarginContainer/VBoxContainer/TimerLabel") as Label
@onready var _legacy_cones_lbl: Label = get_node_or_null("MarginContainer/VBoxContainer/FallenConesLabel") as Label

var elapsed_time: float = 0.0
var _corr_refresh_accum: float = 0.25 # Refresh correlation at ~4 Hz
var _live_telemetry_accum: float = 0.10 # Refresh live vehicle telemetry at 10 Hz

func _ready() -> void:
	# Apply exact Options Menu styling: translucent dark-grey panel with subtle slate border
	add_theme_stylebox_override("panel", MirenaUITheme.create_panel_stylebox(0.72, 0.60, 12, 6, 4))
	
	if fallen_cones_label != null:
		fallen_cones_label.text = "Cones: 0"
	if timer_label != null:
		timer_label.text = "Time: 0.00"
	if correlation_label != null:
		correlation_label.text = "CORR: N/A"

	_resolve_car_reference()
	_refresh_correlation_indicator()
	_refresh_live_telemetry()

func _resolve_car_reference() -> void:
	if car != null and is_instance_valid(car):
		return
	if get_tree() != null:
		car = get_tree().get_first_node_in_group("Car") as MirenaCar
		if car == null:
			car = get_tree().get_first_node_in_group("vehicle") as MirenaCar
	var sim_autoload: Node = get_node_or_null("/root/Sim")
	if car == null and sim_autoload != null and sim_autoload.get("car") is MirenaCar:
		car = sim_autoload.get("car") as MirenaCar
	if car == null and get_parent() != null and get_parent().get_parent() != null:
		car = get_parent().get_parent().get_node_or_null("Car") as MirenaCar

	if car != null:
		if not car.cone_hit.is_connected(_on_cone_hit):
			car.cone_hit.connect(_on_cone_hit)

func _process(delta: float) -> void:
	# 1. Update FPS
	var fps_str: String = "FPS: %d" % Engine.get_frames_per_second()
	if fps_label != null:
		fps_label.text = fps_str
	if _legacy_fps_lbl != null and _legacy_fps_lbl != fps_label:
		_legacy_fps_lbl.text = fps_str
	
	# 2. Update Timer
	elapsed_time += delta
	var time_str: String = "Time: %.2f" % elapsed_time
	if timer_label != null:
		timer_label.text = time_str
	if _legacy_timer_lbl != null and _legacy_timer_lbl != timer_label:
		_legacy_timer_lbl.text = time_str

	# 3. Update Live Vehicle Telemetry at 10 Hz (100 ms)
	_live_telemetry_accum += delta
	if _live_telemetry_accum >= 0.10:
		_live_telemetry_accum = 0.0
		_refresh_live_telemetry()

	# 4. Update Correlation & Status at ~4 Hz (250 ms, independent from live telemetry)
	_corr_refresh_accum += delta
	if _corr_refresh_accum >= 0.25:
		_corr_refresh_accum = 0.0
		_refresh_correlation_indicator()

## Live Telemetry Refresh (10 Hz)
## Reads fresh values directly from car.get_live_dynamics_snapshot()
func _refresh_live_telemetry() -> void:
	if car == null or not is_instance_valid(car):
		_resolve_car_reference()
	if car == null or not car.has_method("get_live_dynamics_snapshot"):
		return
	
	var snap: Dictionary = car.call("get_live_dynamics_snapshot")
	var is_legacy: bool = bool(snap.get("is_legacy", false))
	var is_ay_est: bool = bool(snap.get("is_ay_estimated", false))
	var is_ax_avail: bool = bool(snap.get("is_ax_available", true))
	var is_az_avail: bool = bool(snap.get("is_az_available", true))

	# Tag pill
	if live_tag_label != null:
		if is_legacy:
			live_tag_label.text = "LIVE [LEGACY]"
			live_tag_label.add_theme_color_override("font_color", Color(0.95, 0.72, 0.20))
			live_tag_label.tooltip_text = "Legacy Godot VehicleBody3D (continuous rigid-body kinematics)"
		else:
			live_tag_label.text = "LIVE [MODULAR]"
			live_tag_label.add_theme_color_override("font_color", Color(0.30, 0.95, 0.60))
			live_tag_label.tooltip_text = "MirenaSim Modular 6-DOF dynamic snapshot (true body-fixed kinematics and IMU acceleration)"

	# 1. SPEED: sqrt(u^2 + v^2) * 3.6 [km/h] (HIGH VISUAL PRIORITY)
	var u_mps: float = float(snap.get("chassis.u", 0.0))
	var v_mps: float = float(snap.get("chassis.v", 0.0))
	var speed_kmh: float = sqrt(u_mps * u_mps + v_mps * v_mps) * 3.6
	var speed_target: Label = speed_metric if speed_metric != null else u_metric
	if speed_target != null:
		speed_target.text = "SPEED: %4.1f km/h" % speed_kmh
		# Prominent Options Menu cyan styling
		speed_target.add_theme_color_override("font_color", MirenaUITheme.ACCENT_COLOR)
		speed_target.tooltip_text = "Total Ground Speed: %.1f km/h (%.2f m/s)\nLongitudinal Velocity u: %+.2f m/s\nLateral Velocity v: %+.2f m/s" % [
			speed_kmh, speed_kmh / 3.6, u_mps, v_mps
		]
	if u_metric != null and u_metric != speed_target:
		u_metric.text = "U: %+.1f m/s" % u_mps

	# 2. AX: Longitudinal acceleration [m/s²] (HIGH VISUAL PRIORITY)
	if ax_metric != null:
		if is_ax_avail:
			var ax_mps2: float = float(snap.get("imu.ax", 0.0))
			ax_metric.text = "AX: %+5.2f m/s²" % ax_mps2
			var col_ax: Color = MirenaUITheme.COLOR_AX_ACCEL if ax_mps2 >= 0.0 else MirenaUITheme.COLOR_AX_BRAKE
			ax_metric.add_theme_color_override("font_color", col_ax)
			ax_metric.tooltip_text = "Longitudinal Acceleration (Ax): %+.2f m/s² (%+.2fg)\nFrame: Vehicle Body (+X Forward)\nDefinition: Gravity-free kinematic body acceleration" % [
				ax_mps2, ax_mps2 / 9.80665
			]
		else:
			ax_metric.text = "AX: N/A"
			ax_metric.add_theme_color_override("font_color", MirenaUITheme.TEXT_MUTED)
			ax_metric.tooltip_text = "Longitudinal Acceleration (Ax): Not available"

	# 3. AY: Lateral acceleration [m/s²] (HIGH VISUAL PRIORITY)
	if ay_metric != null:
		var ay_mps2: float = float(snap.get("imu.ay", 0.0))
		if is_ay_est:
			ay_metric.text = "AY: %+5.2f [EST]" % ay_mps2
			ay_metric.add_theme_color_override("font_color", Color(0.95, 0.72, 0.25))
			ay_metric.tooltip_text = "Lateral Acceleration (Ay): %+.2f m/s²\nSource: Kinematic Estimate (u * r) in Legacy mode" % ay_mps2
		else:
			ay_metric.text = "AY: %+5.2f m/s²" % ay_mps2
			var abs_ay: float = absf(ay_mps2)
			var col_ay: Color = Color(0.35, 0.90, 0.55) if abs_ay < 4.0 else (Color(0.40, 0.85, 0.95) if abs_ay < 8.0 else Color(0.95, 0.70, 0.25))
			ay_metric.add_theme_color_override("font_color", col_ay)
			ay_metric.tooltip_text = "Lateral Acceleration (Ay): %+.2f m/s² (%+.2fg)\nFrame: Vehicle Body (+Y Left)\nDefinition: Gravity-free kinematic body acceleration" % [
				ay_mps2, ay_mps2 / 9.80665
			]

	# 4. AZ: Vertical acceleration [m/s²]
	if az_metric != null:
		if is_az_avail:
			var az_mps2: float = float(snap.get("imu.az", 0.0))
			az_metric.text = "AZ: %+4.2f m/s²" % az_mps2
			az_metric.add_theme_color_override("font_color", MirenaUITheme.COLOR_AZ)
			az_metric.tooltip_text = "Vertical Kinematic Acceleration (Az): %+.2f m/s²\nFrame: Vehicle Body (+Z Up)\nDefinition: Gravity-free kinematic body acceleration (0 at rest on level ground)" % az_mps2
		else:
			az_metric.text = "AZ: N/A"
			az_metric.add_theme_color_override("font_color", MirenaUITheme.TEXT_MUTED)
			az_metric.tooltip_text = "Vertical Acceleration (Az): Not available"

	# 5. YAW RATE: [rad/s]
	var gz_target: Label = yaw_metric if yaw_metric != null else gz_metric
	if gz_target != null:
		var gz_rads: float = float(snap.get("imu.gz", snap.get("chassis.r", 0.0)))
		gz_target.text = "YAW RATE: %+5.2f rad/s" % gz_rads
		gz_target.add_theme_color_override("font_color", MirenaUITheme.TEXT_SECONDARY)
		gz_target.tooltip_text = "Chassis Yaw Rate (r / gz): %+.2f rad/s (%+.1f deg/s)\nPositive Leftward (+Z axis)" % [
			gz_rads, rad_to_deg(gz_rads)
		]
	if gz_metric != null and gz_metric != gz_target:
		var gz_rads: float = float(snap.get("imu.gz", snap.get("chassis.r", 0.0)))
		gz_metric.text = "GZ: %+5.2f rad/s" % gz_rads

	# 6. SIDESLIP: beta [deg]
	var beta_target: Label = beta_metric if beta_metric != null else v_metric
	if beta_target != null:
		var beta_deg: float = float(snap.get("chassis.beta_deg", 0.0))
		if is_legacy and is_ay_est:
			beta_target.text = "SIDESLIP β: %+4.1f° [EST]" % beta_deg
			beta_target.add_theme_color_override("font_color", Color(0.95, 0.72, 0.25))
			beta_target.tooltip_text = "Vehicle Sideslip Angle (β): %+.1f°\nSource: Kinematic atan2(v, u)" % beta_deg
		else:
			beta_target.text = "SIDESLIP β: %+4.1f°" % beta_deg
			var col_beta: Color = MirenaUITheme.TEXT_SECONDARY if absf(beta_deg) < 4.0 else Color(0.95, 0.65, 0.25)
			beta_target.add_theme_color_override("font_color", col_beta)
			beta_target.tooltip_text = "Vehicle CoG Sideslip Angle (β): %+.1f°\nSource: Modular Kinematics atan2(v, u)" % beta_deg
	if v_metric != null and v_metric != beta_target:
		v_metric.text = "V: %+.2f m/s" % v_mps

	# 7. ROLL: [deg]
	if roll_metric != null:
		var roll_deg: float = float(snap.get("chassis.roll_deg", 0.0))
		roll_metric.text = "ROLL: %+4.1f°" % roll_deg
		roll_metric.add_theme_color_override("font_color", MirenaUITheme.TEXT_SECONDARY)
		roll_metric.tooltip_text = "Sprung Mass Roll Angle (φ): %+.1f°\nUnits: degrees (Right-down positive)" % roll_deg

	# 8. PITCH: [deg]
	if pitch_metric != null:
		var pitch_deg: float = float(snap.get("chassis.pitch_deg", 0.0))
		pitch_metric.text = "PITCH: %+4.1f°" % pitch_deg
		pitch_metric.add_theme_color_override("font_color", MirenaUITheme.TEXT_SECONDARY)
		pitch_metric.tooltip_text = "Sprung Mass Pitch Angle (θ): %+.1f°\nUnits: degrees (Nose-up positive)" % pitch_deg

## Correlation Status Refresh (~4 Hz)
## Updates Row 1 indicators independently from live telemetry
func _refresh_correlation_indicator() -> void:
	if car == null or not is_instance_valid(car):
		_resolve_car_reference()
	
	var summary: Dictionary = {}
	if car != null and car.has_method("get_hud_correlation_summary"):
		summary = car.call("get_hud_correlation_summary")
	
	var corr_text: String = str(summary.get("hud_text", "CORR: N/A"))
	var corr_col: Color = summary.get("hud_color", Color(0.68, 0.72, 0.78))
	var details_str: String = str(summary.get("details", ""))
	
	if correlation_label != null:
		correlation_label.text = corr_text
		correlation_label.add_theme_color_override("font_color", corr_col)
		correlation_label.tooltip_text = details_str
	if _legacy_corr_lbl != null and _legacy_corr_lbl != correlation_label:
		_legacy_corr_lbl.text = corr_text
		_legacy_corr_lbl.add_theme_color_override("font_color", corr_col)

	# 1. Physics Mode Badge
	var p_mode_text: String = str(summary.get("physics_mode_label", "MODULAR 6-DOF: TeR_Tire MF6.2"))
	var p_mode_col: Color = summary.get("physics_mode_color", Color(0.30, 0.95, 0.60))
	if physics_mode_label != null:
		physics_mode_label.text = p_mode_text
		physics_mode_label.add_theme_color_override("font_color", p_mode_col)
		physics_mode_label.tooltip_text = "Physics Authority: %s\nTire Model: %s" % [
			p_mode_text, str(summary.get("tire_model_label", "TeR_Tire"))
		]
	
	if physics_mode_badge != null:
		var is_modular: bool = p_mode_text.begins_with("MODULAR")
		physics_mode_badge.add_theme_stylebox_override("panel", MirenaUITheme.create_pill_badge_stylebox(is_modular, not is_modular))

	# 2. Correlation Score Badge
	if correlation_badge != null:
		correlation_badge.add_theme_stylebox_override("panel", MirenaUITheme.create_pill_badge_stylebox(true, false))

	# 3. Dataset & Evaluation Window Info
	var ref_name: String = str(summary.get("ref_dataset_name", "TeR26 VN-300 Real"))
	var sim_name: String = str(summary.get("sim_dataset_name", "modular_calibrated"))
	var win_label: String = str(summary.get("eval_window_label", "tau: 20..281s"))
	if dataset_label != null:
		if ref_name == "None" or sim_name == "None":
			dataset_label.text = "Ref: No Dataset Loaded | Sim: Live Standby"
			dataset_label.tooltip_text = "No reference dataset loaded for correlation."
		else:
			dataset_label.text = "Ref: %s | Sim: %s (%s)" % [ref_name, sim_name, win_label]
			dataset_label.tooltip_text = "Real Reference: %s\nSimulation Run: %s\nEvaluation Window: %s\nPress F7 for Multi-Channel Correlation Dashboard" % [
				ref_name, sim_name, win_label
			]

func _on_cone_hit(count: int) -> void:
	var cones_str: String = "Cones: %d" % count
	if fallen_cones_label != null:
		fallen_cones_label.text = cones_str
	if _legacy_cones_lbl != null and _legacy_cones_lbl != fallen_cones_label:
		_legacy_cones_lbl.text = cones_str
