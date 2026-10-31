extends Window
class_name VehicleDynamicsDashboardWindow

## Separate, resizable, independently navigable native Godot `Window` for:
## - Mode A: Live Vehicle-Dynamics Monitoring & Real-Time Parameter Tuning
## - Mode B: Real-versus-Simulation Quantitative Correlation Analysis & Control Replay
##
## Covers all 8 required vehicle-dynamics areas (Sections 3.1 - 3.8):
## 1. Vehicle Motion & Chassis Dynamics
## 2. Steering & Lateral Response
## 3. Tires & Tire-Road Interaction (with explicit TeR_Tire MF6.2 + 7D PINN vs Companion Fx provenance)
## 4. Suspension & Wheel Loads
## 5. Powertrain & Longitudinal Response
## 6. Aerodynamics
## 7. Sensors, ROS & Measurement Consistency
## 8. Numerical Stability & Simulation Health

enum DashboardMode {
	LIVE_MONITORING = 0,
	CORRELATION_ANALYSIS = 1,
}

const CATEGORY_SPECS: Array[Dictionary] = [
	{
		"id": "motion",
		"title": "3.1 Vehicle Motion & Chassis",
		"primary_channels": ["imu.ax", "imu.ay", "imu.gz", "chassis.u", "chassis.v", "chassis.beta_deg", "chassis.fy_net_n", "chassis.mz_net_nm", "chassis.psi"],
		"default_primary": "imu.ax",
		"default_secondary": "imu.gz",
	},
	{
		"id": "steering",
		"title": "3.2 Steering & Lateral Response",
		"primary_channels": ["control.steer_rad", "steer.fl_rad", "steer.fr_rad", "imu.gz", "imu.ay", "chassis.beta_deg", "axle.fy_front_n", "axle.fy_rear_n"],
		"default_primary": "imu.gz",
		"default_secondary": "control.steer_rad",
	},
	{
		"id": "tires",
		"title": "3.3 Tires & Road Interaction",
		"primary_channels": [
			"tire.fy_fl", "tire.fy_fr", "tire.fy_rl", "tire.fy_rr",
			"tire.fy0_fl", "tire.fy0_rl", "tire.static_fy_fl", "tire.static_fy_fr",
			"tire.pinn_dfy_fl", "tire.pinn_dfy_rl",
			"tire.fx_rl", "tire.fx_rr", "tire.util_rl", "tire.util_rr",
			"axle.fy_reserve_rear_n", "tire.fz_fl", "tire.fz_rl"
		],
		"default_primary": "tire.fy_fl",
		"default_secondary": "tire.fz_rl",
	},
	{
		"id": "suspension",
		"title": "3.4 Suspension & Wheel Loads",
		"primary_channels": ["susp.heave_fl", "susp.heave_fr", "susp.heave_rl", "susp.heave_rr", "tire.fz_fl", "tire.fz_fr", "tire.fz_rl", "tire.fz_rr"],
		"default_primary": "susp.heave_fl",
		"default_secondary": "tire.fz_fl",
	},
	{
		"id": "powertrain",
		"title": "3.5 Powertrain & Longitudinal",
		"primary_channels": ["control.gas", "wheel.rl_rpm", "wheel.rr_rpm", "chassis.u", "imu.ax", "tire.fx_rl"],
		"default_primary": "wheel.rl_rpm",
		"default_secondary": "imu.ax",
	},
	{
		"id": "aero",
		"title": "3.6 Aerodynamics",
		"primary_channels": ["aero.drag_n", "aero.downforce_front_n", "aero.downforce_rear_n", "chassis.u"],
		"default_primary": "aero.downforce_rear_n",
		"default_secondary": "aero.drag_n",
	},
	{
		"id": "sensors",
		"title": "3.7 Sensors & ROS Consistency",
		"primary_channels": ["imu.ax", "imu.ay", "imu.az", "imu.gx", "imu.gy", "imu.gz", "wheel.rl_rpm"],
		"default_primary": "imu.ax",
		"default_secondary": "imu.ay",
	},
	{
		"id": "health",
		"title": "3.8 Numerical Health & Stability",
		"primary_channels": ["chassis.u", "imu.ax", "imu.gz"],
		"default_primary": "chassis.u",
		"default_secondary": "imu.gz",
	},
]

var target_car: Node = null
var active_mode: DashboardMode = DashboardMode.LIVE_MONITORING
var active_category_index: int = 0
var selected_primary_channel: String = "imu.ax"
var selected_secondary_channel: String = "imu.gz"

## Live ring-buffer arrays (last 20 seconds at 50 Hz = 1000 samples)
const LIVE_HISTORY_MAX: int = 1000
var _live_time_s: PackedFloat64Array = PackedFloat64Array()
var _live_traces: Dictionary = {}
var _live_sample_accum_s: float = 0.0
var _live_elapsed_s: float = 0.0

## Session recording into `TelemetryRecording` when `is_recording_live` is active
var is_recording_live: bool = false
var live_session_recording: TelemetryRecording = null

## Correlation datasets (Mode B)
var real_reference_recording: TelemetryRecording = null
var sim_comparison_recording: TelemetryRecording = null
var legacy_baseline_recording: TelemetryRecording = null
var cached_correlation_metrics: Dictionary = {}
var corr_tau_start_s: float = 20.0
var corr_tau_end_s: float = 95.0
var corr_global_offset_s: float = 0.0

## UI Controls
var _mode_banner_panel: PanelContainer
var _mode_banner_label: Label
var _mode_select: OptionButton
var _physics_mode_select: OptionButton
var _tire_model_select: OptionButton
var _record_btn: Button
var _save_csv_btn: Button
var _bundle_select: OptionButton
var _replay_btn: Button
var _tau_start_spin: SpinBox
var _tau_end_spin: SpinBox
var _offset_spin: SpinBox
var _auto_align_btn: Button
var _category_buttons: Array[Button] = []
var _primary_channel_select: OptionButton
var _secondary_channel_select: OptionButton
var _top_plot: SignalPlotCanvas
var _bottom_plot: SignalPlotCanvas
var _right_tabs: TabContainer
var _live_readout_label: RichTextLabel
var _metrics_readout_label: RichTextLabel
var _status_footer_label: Label
var _file_dialog: FileDialog
var _ui_refresh_accum: float = 0.0
var _updating_controls_from_car: bool = false

## Parameter spinboxes for live tuning
var _mass_spin: SpinBox
var _grip_scale_spin: SpinBox
var _pinn_check: CheckBox
var _spring_f_spin: SpinBox
var _spring_r_spin: SpinBox
var _arb_f_spin: SpinBox
var _arb_r_spin: SpinBox
var _cl_spin: SpinBox
var _cd_spin: SpinBox
var _power_spin: SpinBox
var _brake_bias_spin: SpinBox

func _init() -> void:
	title = "MirenaSim — Vehicle Dynamics & Real-vs-Simulation Correlation Dashboard"
	size = Vector2i(1360, 860)
	min_size = Vector2i(960, 620)
	unresizable = false
	wrap_controls = false
	close_requested.connect(func(): hide())
	_init_live_trace_buffers()

func _ready() -> void:
	_build_ui()
	_preload_default_imu_compare_bundle()
	_select_category(0)

func bind_to_car(car: Node) -> void:
	target_car = car
	_sync_tuning_controls_from_car()

func _init_live_trace_buffers() -> void:
	var keys := [
		"chassis.u", "chassis.v", "chassis.r", "chassis.x", "chassis.y", "chassis.psi",
		"chassis.roll_deg", "chassis.pitch_deg", "chassis.beta_deg",
		"chassis.fy_net_n", "chassis.fx_net_n", "chassis.mz_net_nm",
		"axle.fy_front_n", "axle.fy_rear_n", "axle.fy_reserve_rear_n",
		"imu.ax", "imu.ay", "imu.az", "imu.gx", "imu.gy", "imu.gz",
		"control.gas", "control.steer_rad", "steer.fl_rad", "steer.fr_rad", "steer.ackermann_diff_deg",
		"tire.fx_fl", "tire.fx_fr", "tire.fx_rl", "tire.fx_rr",
		"tire.fy_fl", "tire.fy_fr", "tire.fy_rl", "tire.fy_rr",
		"tire.fy0_fl", "tire.fy0_fr", "tire.fy0_rl", "tire.fy0_rr",
		"tire.static_fy_fl", "tire.static_fy_fr", "tire.static_fy_rl", "tire.static_fy_rr",
		"tire.fz_fl", "tire.fz_fr", "tire.fz_rl", "tire.fz_rr",
		"tire.mz_fl", "tire.mz_fr", "tire.mz_rl", "tire.mz_rr",
		"tire.alpha_fl", "tire.alpha_fr", "tire.alpha_rl", "tire.alpha_rr",
		"tire.alpha_dot_fl", "tire.alpha_dot_fr", "tire.alpha_dot_rl", "tire.alpha_dot_rr",
		"tire.gamma_fl", "tire.gamma_fr", "tire.gamma_rl", "tire.gamma_rr",
		"tire.kappa_fl", "tire.kappa_fr", "tire.kappa_rl", "tire.kappa_rr",
		"tire.pinn_dfy_fl", "tire.pinn_dfy_fr", "tire.pinn_dfy_rl", "tire.pinn_dfy_rr",
		"tire.util_fl", "tire.util_fr", "tire.util_rl", "tire.util_rr",
		"tire.reserve_fl", "tire.reserve_fr", "tire.reserve_rl", "tire.reserve_rr",
		"susp.heave_fl", "susp.heave_fr", "susp.heave_rl", "susp.heave_rr",
		"susp.vel_fl", "susp.vel_fr", "susp.vel_rl", "susp.vel_rr",
		"susp.f_spring_fl", "susp.f_spring_rl", "susp.f_damper_fl", "susp.f_damper_rl",
		"susp.f_arb_fl", "susp.f_arb_fr", "susp.f_arb_rl", "susp.f_arb_rr",
		"wheel.fl_rpm", "wheel.fr_rpm", "wheel.rl_rpm", "wheel.rr_rpm",
		"powertrain.drive_torque_rl", "powertrain.drive_torque_rr",
		"powertrain.brake_torque_fl", "powertrain.brake_torque_rl",
		"aero.drag_n", "aero.downforce_front_n", "aero.downforce_rear_n",
		"health.dt_ms", "health.contact_count", "health.fault_flags",
	]
	for k in keys:
		_live_traces[k] = PackedFloat64Array()

func _build_ui() -> void:
	var bg := ColorRect.new()
	bg.color = Color(0.06, 0.07, 0.09, 1.0)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	add_child(bg)

	var root_margin := MarginContainer.new()
	root_margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root_margin.add_theme_constant_override("margin_left", 8)
	root_margin.add_theme_constant_override("margin_right", 8)
	root_margin.add_theme_constant_override("margin_top", 8)
	root_margin.add_theme_constant_override("margin_bottom", 8)
	add_child(root_margin)

	var main_vbox := VBoxContainer.new()
	main_vbox.add_theme_constant_override("separation", 6)
	root_margin.add_child(main_vbox)

	# 1. Prominent Mode Banner
	_mode_banner_panel = PanelContainer.new()
	var banner_style := StyleBoxFlat.new()
	banner_style.bg_color = Color(0.11, 0.30, 0.22, 1.0)
	banner_style.set_corner_radius_all(4)
	banner_style.content_margin_left = 10
	banner_style.content_margin_right = 10
	banner_style.content_margin_top = 4
	banner_style.content_margin_bottom = 4
	_mode_banner_panel.add_theme_stylebox_override("panel", banner_style)
	_mode_banner_label = Label.new()
	_mode_banner_label.text = "MODE A — LIVE SIMULATION MONITORING (Real-Time Running Model Diagnostics)"
	_mode_banner_label.add_theme_font_size_override("font_size", 13)
	_mode_banner_label.add_theme_color_override("font_color", Color(0.92, 1.0, 0.95))
	_mode_banner_panel.add_child(_mode_banner_label)
	main_vbox.add_child(_mode_banner_panel)

	# 2. Top Toolbar Row 1: Mode, Physics Authority, Tire Model, Recording & Replay
	var toolbar_row1 := HBoxContainer.new()
	toolbar_row1.add_theme_constant_override("separation", 8)
	main_vbox.add_child(toolbar_row1)

	toolbar_row1.add_child(_make_label("View Mode:"))
	_mode_select = OptionButton.new()
	_mode_select.add_item("Mode A: Live Simulation Monitoring", DashboardMode.LIVE_MONITORING)
	_mode_select.add_item("Mode B: Real-vs-Simulation Correlation", DashboardMode.CORRELATION_ANALYSIS)
	_mode_select.item_selected.connect(_on_mode_selected)
	toolbar_row1.add_child(_mode_select)

	toolbar_row1.add_child(_make_label("Physics Engine:"))
	_physics_mode_select = OptionButton.new()
	_physics_mode_select.add_item("Modular VehicleDynamics (6-DOF)", 1)
	_physics_mode_select.add_item("Legacy Godot VehicleBody3D", 0)
	_physics_mode_select.item_selected.connect(_on_physics_mode_selected)
	toolbar_row1.add_child(_physics_mode_select)

	toolbar_row1.add_child(_make_label("Tire Model:"))
	_tire_model_select = OptionButton.new()
	_tire_model_select.add_item("TeR_Tire Grey-Box (MF6.2 + PINN)", 0)
	_tire_model_select.add_item("TeR_Tire Pacejka MF6.2 Baseline", 1)
	_tire_model_select.add_item("Linear Test Stub (Uncalibrated)", 2)
	_tire_model_select.item_selected.connect(_on_tire_model_selected)
	toolbar_row1.add_child(_tire_model_select)

	_record_btn = Button.new()
	_record_btn.text = "Start Live Recording"
	_record_btn.toggle_mode = true
	_record_btn.toggled.connect(_on_record_toggled)
	toolbar_row1.add_child(_record_btn)

	_save_csv_btn = Button.new()
	_save_csv_btn.text = "Export Sim CSV"
	_save_csv_btn.pressed.connect(_on_save_csv_pressed)
	toolbar_row1.add_child(_save_csv_btn)

	_replay_btn = Button.new()
	_replay_btn.text = "Replay Real Bag Controls on Car"
	_replay_btn.toggle_mode = true
	_replay_btn.toggled.connect(_on_replay_toggled)
	toolbar_row1.add_child(_replay_btn)

	# 3. Top Toolbar Row 2: Correlation Dataset Selector & Time Alignment Controls
	var toolbar_row2 := HBoxContainer.new()
	toolbar_row2.add_theme_constant_override("separation", 8)
	main_vbox.add_child(toolbar_row2)

	toolbar_row2.add_child(_make_label("Correlation Dataset:"))
	_bundle_select = OptionButton.new()
	_bundle_select.add_item("imu_compare: Real Bag vs. Modular Calibrated (TeR26 MF6.2+PINN)", 0)
	_bundle_select.add_item("imu_compare: Real Bag vs. Modular Baseline (vehicle_default)", 1)
	_bundle_select.add_item("imu_compare: Real Bag vs. Sim (front10_rear12)", 2)
	_bundle_select.add_item("imu_compare: Real Bag vs. Sim (grip10)", 3)
	_bundle_select.add_item("imu_compare: Real Bag vs. Sim (grip14)", 4)
	_bundle_select.add_item("imu_compare: Real Bag vs. Sim (pre_aero)", 5)
	_bundle_select.add_item("Real Bag vs. Live Recorded Modular Run", 6)
	_bundle_select.add_item("Load Custom CSV / ROS2 Bag...", 7)
	_bundle_select.item_selected.connect(_on_bundle_selected)
	toolbar_row2.add_child(_bundle_select)

	toolbar_row2.add_child(_make_label("Window tau_start [s]:"))
	_tau_start_spin = SpinBox.new()
	_tau_start_spin.min_value = -20.0
	_tau_start_spin.max_value = 350.0
	_tau_start_spin.step = 1.0
	_tau_start_spin.value = corr_tau_start_s
	_tau_start_spin.value_changed.connect(func(v: float):
		corr_tau_start_s = v
		_recompute_correlation()
	)
	toolbar_row2.add_child(_tau_start_spin)

	toolbar_row2.add_child(_make_label("tau_end [s]:"))
	_tau_end_spin = SpinBox.new()
	_tau_end_spin.min_value = 1.0
	_tau_end_spin.max_value = 400.0
	_tau_end_spin.step = 1.0
	_tau_end_spin.value = corr_tau_end_s
	_tau_end_spin.value_changed.connect(func(v: float):
		corr_tau_end_s = v
		_recompute_correlation()
	)
	toolbar_row2.add_child(_tau_end_spin)

	toolbar_row2.add_child(_make_label("Global Offset dt [s]:"))
	_offset_spin = SpinBox.new()
	_offset_spin.min_value = -20.0
	_offset_spin.max_value = 20.0
	_offset_spin.step = 0.01
	_offset_spin.value = corr_global_offset_s
	_offset_spin.value_changed.connect(func(v: float):
		corr_global_offset_s = v
		_recompute_correlation()
	)
	toolbar_row2.add_child(_offset_spin)

	_auto_align_btn = Button.new()
	_auto_align_btn.text = "Auto-Align Lag (gz)"
	_auto_align_btn.pressed.connect(_on_auto_align_pressed)
	toolbar_row2.add_child(_auto_align_btn)

	# 4. Main Content Split: [Left Category Sidebar] | [Center Plots] | [Right Inspector Tabs]
	var body_hbox := HBoxContainer.new()
	body_hbox.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body_hbox.add_theme_constant_override("separation", 8)
	main_vbox.add_child(body_hbox)

	# 4A. Left Sidebar (8 Vehicle-Dynamics Areas)
	var left_panel := PanelContainer.new()
	left_panel.custom_minimum_size = Vector2(245, 0)
	var left_vbox := VBoxContainer.new()
	left_vbox.add_theme_constant_override("separation", 4)
	left_panel.add_child(left_vbox)

	var cat_header := Label.new()
	cat_header.text = "VEHICLE DYNAMICS AREAS"
	cat_header.add_theme_font_size_override("font_size", 12)
	cat_header.add_theme_color_override("font_color", Color(0.65, 0.78, 0.95))
	left_vbox.add_child(cat_header)

	for i in range(CATEGORY_SPECS.size()):
		var btn := Button.new()
		btn.alignment = HORIZONTAL_ALIGNMENT_LEFT
		btn.text = str(CATEGORY_SPECS[i]["title"])
		btn.toggle_mode = true
		var idx: int = i
		btn.pressed.connect(func(): _select_category(idx))
		left_vbox.add_child(btn)
		_category_buttons.append(btn)

	left_vbox.add_child(HSeparator.new())
	var provenance_legend := RichTextLabel.new()
	provenance_legend.bbcode_enabled = true
	provenance_legend.fit_content = true
	provenance_legend.scroll_active = false
	provenance_legend.text = (
		"[b]Signal Provenance Legend[/b]\n" +
		"[color=#4ad674]● Measured (Real Car)[/color]\n" +
		"[color=#eb7538]● Simulated (Modular 6-DOF)[/color]\n" +
		"[color=#a678de]● Simulated (Legacy Body3D)[/color]\n" +
		"[color=#ebd246]● Derived / Companion Est.[/color]\n" +
		"[color=#73adeb]● Awaiting Reference Data[/color]"
	)
	left_vbox.add_child(provenance_legend)
	body_hbox.add_child(left_panel)

	# 4B. Center Multi-Plot Column
	var center_vbox := VBoxContainer.new()
	center_vbox.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	center_vbox.size_flags_vertical = Control.SIZE_EXPAND_FILL
	center_vbox.add_theme_constant_override("separation", 6)
	body_hbox.add_child(center_vbox)

	var plot_sel_bar := HBoxContainer.new()
	plot_sel_bar.add_theme_constant_override("separation", 8)
	center_vbox.add_child(plot_sel_bar)

	plot_sel_bar.add_child(_make_label("Top Plot Signal:"))
	_primary_channel_select = OptionButton.new()
	_primary_channel_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_primary_channel_select.item_selected.connect(_on_primary_channel_selected)
	plot_sel_bar.add_child(_primary_channel_select)

	plot_sel_bar.add_child(_make_label("Bottom Plot Signal:"))
	_secondary_channel_select = OptionButton.new()
	_secondary_channel_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_secondary_channel_select.item_selected.connect(_on_secondary_channel_selected)
	plot_sel_bar.add_child(_secondary_channel_select)

	_top_plot = SignalPlotCanvas.new()
	center_vbox.add_child(_top_plot)

	_bottom_plot = SignalPlotCanvas.new()
	center_vbox.add_child(_bottom_plot)

	# 4C. Right Inspector Panel (Tabs: Live Diagnostics | Correlation Metrics | Live Tuning)
	_right_tabs = TabContainer.new()
	_right_tabs.custom_minimum_size = Vector2(440, 0)
	body_hbox.add_child(_right_tabs)

	# Tab 0: Live State, Forces & Provenance
	var tab_live := MarginContainer.new()
	tab_live.name = "Live State & Provenance"
	_live_readout_label = RichTextLabel.new()
	_live_readout_label.bbcode_enabled = true
	_live_readout_label.selection_enabled = true
	_live_readout_label.add_theme_font_size_override("normal_font_size", 12)
	_live_readout_label.add_theme_font_size_override("bold_font_size", 12)
	_live_readout_label.add_theme_font_size_override("mono_font_size", 11)
	tab_live.add_child(_live_readout_label)
	_right_tabs.add_child(tab_live)

	# Tab 1: Quantitative Correlation Metrics
	var tab_metrics := MarginContainer.new()
	tab_metrics.name = "Correlation Metrics"
	_metrics_readout_label = RichTextLabel.new()
	_metrics_readout_label.bbcode_enabled = true
	_metrics_readout_label.selection_enabled = true
	_metrics_readout_label.add_theme_font_size_override("normal_font_size", 12)
	_metrics_readout_label.add_theme_font_size_override("bold_font_size", 12)
	_metrics_readout_label.add_theme_font_size_override("mono_font_size", 11)
	tab_metrics.add_child(_metrics_readout_label)
	_right_tabs.add_child(tab_metrics)

	# Tab 2: Live Parameter Tuning
	var tab_tuning := ScrollContainer.new()
	tab_tuning.name = "Live Physics Tuning"
	var tuning_vbox := VBoxContainer.new()
	tuning_vbox.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	tuning_vbox.add_theme_constant_override("separation", 6)
	tab_tuning.add_child(tuning_vbox)
	_build_tuning_controls(tuning_vbox)
	_right_tabs.add_child(tab_tuning)

	# 5. Footer Status Bar
	_status_footer_label = Label.new()
	_status_footer_label.text = "Ready. Use mouse wheel on plots to zoom, right-drag to pan, double-click to reset zoom."
	_status_footer_label.add_theme_font_size_override("font_size", 11)
	_status_footer_label.add_theme_color_override("font_color", Color(0.62, 0.68, 0.78))
	main_vbox.add_child(_status_footer_label)

	# File Dialog for custom CSV / ROS2 bag loading
	_file_dialog = FileDialog.new()
	_file_dialog.access = FileDialog.ACCESS_FILESYSTEM
	_file_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	_file_dialog.filters = PackedStringArray(["*.csv ; Telemetry CSV Files", "*.yaml ; ROS 2 Bag metadata.yaml"])
	_file_dialog.file_selected.connect(_on_custom_file_selected)
	add_child(_file_dialog)

func _build_tuning_controls(parent: VBoxContainer) -> void:
	var note := Label.new()
	note.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	note.text = "Adjusting any parameter below immediately updates the active Modular VehicleDynamics model on MirenaCar in real time."
	note.add_theme_font_size_override("font_size", 11)
	note.add_theme_color_override("font_color", Color(0.75, 0.86, 0.98))
	parent.add_child(note)

	_mass_spin = _add_tuning_row(parent, "Total Vehicle Mass [kg]:", 150.0, 600.0, 5.0, 300.0)
	_grip_scale_spin = _add_tuning_row(parent, "TeR_Tire Lateral Grip Scale:", 0.20, 2.00, 0.05, 1.00)
	_pinn_check = CheckBox.new()
	_pinn_check.text = "Enable TeR_Tire 7D PINN Residual Network"
	_pinn_check.button_pressed = true
	_pinn_check.toggled.connect(func(_v: bool): _apply_tuning_to_car())
	parent.add_child(_pinn_check)

	_spring_f_spin = _add_tuning_row(parent, "Front Spring Stiffness [N/m]:", 8000.0, 90000.0, 1000.0, 28000.0)
	_spring_r_spin = _add_tuning_row(parent, "Rear Spring Stiffness [N/m]:", 8000.0, 90000.0, 1000.0, 32000.0)
	_arb_f_spin = _add_tuning_row(parent, "Front Anti-Roll Bar [N/m]:", 0.0, 40000.0, 500.0, 6000.0)
	_arb_r_spin = _add_tuning_row(parent, "Rear Anti-Roll Bar [N/m]:", 0.0, 40000.0, 500.0, 5000.0)
	_cl_spin = _add_tuning_row(parent, "Aero Downforce Coeff Cl:", 0.0, 6.0, 0.10, 3.63)
	_cd_spin = _add_tuning_row(parent, "Aero Drag Coeff Cd:", 0.0, 3.0, 0.05, 1.45)
	_power_spin = _add_tuning_row(parent, "Power Limit [kW]:", 10.0, 160.0, 2.0, 80.0)
	_brake_bias_spin = _add_tuning_row(parent, "Front Brake Bias [0..1]:", 0.30, 0.85, 0.02, 0.60)

	var btn_row := HBoxContainer.new()
	var reset_defaults_btn := Button.new()
	reset_defaults_btn.text = "Restore Default Parameters"
	reset_defaults_btn.pressed.connect(_on_restore_default_params)
	btn_row.add_child(reset_defaults_btn)

	var reset_car_btn := Button.new()
	reset_car_btn.text = "Respawn Car on Track (R)"
	reset_car_btn.pressed.connect(func():
		if target_car != null and target_car.has_method("reset_position"):
			target_car.call("reset_position")
	)
	btn_row.add_child(reset_car_btn)
	parent.add_child(btn_row)

func _add_tuning_row(parent: VBoxContainer, label_text: String, min_v: float, max_v: float, step_v: float, def_v: float) -> SpinBox:
	var row := HBoxContainer.new()
	var lbl := Label.new()
	lbl.text = label_text
	lbl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	lbl.add_theme_font_size_override("font_size", 12)
	row.add_child(lbl)

	var sb := SpinBox.new()
	sb.min_value = min_v
	sb.max_value = max_v
	sb.step = step_v
	sb.value = def_v
	sb.custom_minimum_size = Vector2(120, 0)
	sb.value_changed.connect(func(_v: float): _apply_tuning_to_car())
	row.add_child(sb)
	parent.add_child(row)
	return sb

func _make_label(txt: String) -> Label:
	var l := Label.new()
	l.text = txt
	l.add_theme_font_size_override("font_size", 12)
	return l

func _preload_default_imu_compare_bundle() -> void:
	var imu_dir: String = ProjectSettings.globalize_path("res://").get_base_dir().get_base_dir().path_join("imu_compare")
	if not DirAccess.dir_exists_absolute(imu_dir):
		imu_dir = "/home/alexr/eRacing/mirena_ws-SIM/imu_compare"
	if DirAccess.dir_exists_absolute(imu_dir):
		real_reference_recording = TelemetryImporter.load_imu_compare_real_bundle(imu_dir, 25000)
		var def_sim: String = "modular_calibrated" if FileAccess.file_exists(imu_dir.path_join("sim_imu_modular_calibrated.csv")) else "modular_baseline"
		if not FileAccess.file_exists(imu_dir.path_join("sim_imu_%s.csv" % def_sim)):
			def_sim = "front10_rear12"
		sim_comparison_recording = TelemetryImporter.load_imu_compare_sim_bundle(imu_dir, def_sim, 25000)
		legacy_baseline_recording = TelemetryImporter.load_imu_compare_sim_bundle(imu_dir, "front10_rear12", 25000)
		_recompute_correlation()

func _recompute_correlation() -> void:
	if real_reference_recording == null or sim_comparison_recording == null:
		cached_correlation_metrics.clear()
		return
	cached_correlation_metrics = CorrelationEngine.evaluate_all_channels(
		real_reference_recording,
		sim_comparison_recording,
		corr_tau_start_s,
		corr_tau_end_s,
		0.05,
		corr_global_offset_s
	)
	_update_category_button_badges()
	_refresh_plots_and_tables()

func _update_category_button_badges() -> void:
	for i in range(CATEGORY_SPECS.size()):
		var spec: Dictionary = CATEGORY_SPECS[i]
		var base_title: String = str(spec["title"])
		if active_mode == DashboardMode.LIVE_MONITORING:
			_category_buttons[i].text = "%s  [LIVE]" % base_title
		else:
			var def_ch: String = str(spec["default_primary"])
			var m: Dictionary = cached_correlation_metrics.get(def_ch, {})
			var st: int = int(m.get("status", CorrelationEngine.EvaluationStatus.NOT_EVALUATED))
			var badge: String = "—"
			match st:
				CorrelationEngine.EvaluationStatus.EVALUATED:
					badge = "EVAL"
				CorrelationEngine.EvaluationStatus.VALIDATION_WARNING:
					badge = "WARN"
				CorrelationEngine.EvaluationStatus.AWAITING_REFERENCE_DATA:
					badge = "NO REF"
				CorrelationEngine.EvaluationStatus.INSUFFICIENT_DATA:
					badge = "LOW N"
			_category_buttons[i].text = "%s  [%s]" % [base_title, badge]

func _select_category(index: int) -> void:
	active_category_index = clampi(index, 0, CATEGORY_SPECS.size() - 1)
	for i in range(_category_buttons.size()):
		_category_buttons[i].set_pressed_no_signal(i == active_category_index)

	var spec: Dictionary = CATEGORY_SPECS[active_category_index]
	var ch_list: Array = spec["primary_channels"]
	_primary_channel_select.clear()
	_secondary_channel_select.clear()
	for i in range(ch_list.size()):
		var cid: String = str(ch_list[i])
		_primary_channel_select.add_item(cid, i)
		_secondary_channel_select.add_item(cid, i)
		if cid == str(spec["default_primary"]):
			_primary_channel_select.select(i)
			selected_primary_channel = cid
		if cid == str(spec["default_secondary"]):
			_secondary_channel_select.select(i)
			selected_secondary_channel = cid

	_update_category_button_badges()
	_refresh_plots_and_tables()

func _on_mode_selected(index: int) -> void:
	active_mode = DashboardMode.CORRELATION_ANALYSIS if index == 1 else DashboardMode.LIVE_MONITORING
	var banner_style := _mode_banner_panel.get_theme_stylebox("panel") as StyleBoxFlat
	if active_mode == DashboardMode.LIVE_MONITORING:
		if banner_style:
			banner_style.bg_color = Color(0.11, 0.30, 0.22, 1.0)
		_mode_banner_label.text = "MODE A — LIVE SIMULATION MONITORING (Real-Time Running Model Diagnostics — No Real Comparison Active)"
		_right_tabs.current_tab = 0
	else:
		if banner_style:
			banner_style.bg_color = Color(0.14, 0.24, 0.42, 1.0)
		_mode_banner_label.text = "MODE B — REAL-VS-SIMULATION QUANTITATIVE CORRELATION ANALYSIS (Aligned Time Series, Residuals & Metrics)"
		_right_tabs.current_tab = 1
		_recompute_correlation()
	_update_category_button_badges()
	_refresh_plots_and_tables()

func _on_physics_mode_selected(index: int) -> void:
	if target_car == null:
		return
	var use_modular: bool = (index == 0)
	if target_car.has_method("set_physics_engine_mode"):
		target_car.call("set_physics_engine_mode", 1 if use_modular else 0)

func _on_tire_model_selected(index: int) -> void:
	if target_car == null:
		return
	var model_enum: int = VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN
	if index == 1:
		model_enum = VehicleDynamicsTypes.TireModelType.PACEJKA_MF62
	elif index == 2:
		model_enum = VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB
	if target_car.has_method("set_active_tire_model_type"):
		target_car.call("set_active_tire_model_type", model_enum)

func _on_record_toggled(pressed: bool) -> void:
	is_recording_live = pressed
	if pressed:
		_record_btn.text = "Stop Recording (Active)"
		live_session_recording = TelemetryRecording.new("Modular Live Run", false)
		live_session_recording.physics_mode_label = "MODULAR_VEHICLE_DYNAMICS"
		live_session_recording.tire_setup_label = _tire_model_select.get_item_text(_tire_model_select.selected)
		_init_live_recording_channels(live_session_recording)
		_status_footer_label.text = "Started recording live simulation telemetry session."
	else:
		_record_btn.text = "Start Live Recording"
		if live_session_recording != null:
			live_session_recording.detect_maneuver_onset()
			_status_footer_label.text = "Stopped live recording (%d samples captured). Select 'Real Bag vs. Live Recorded Modular Run' to compare." % [
				live_session_recording.get_channel("imu.ax").get_sample_count() if live_session_recording.get_channel("imu.ax") != null else 0
			]

func _init_live_recording_channels(rec: TelemetryRecording) -> void:
	var specs := [
		["imu.ax", "Longitudinal Accel ax", "m/s^2", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["imu.ay", "Lateral Accel ay", "m/s^2", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["imu.az", "Vertical Accel az", "m/s^2", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["imu.gx", "Roll Rate gx", "rad/s", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["imu.gy", "Pitch Rate gy", "rad/s", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["imu.gz", "Yaw Rate gz (r)", "rad/s", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["chassis.u", "Longitudinal Speed u", "m/s", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["chassis.v", "Lateral Speed v", "m/s", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["chassis.r", "Chassis Yaw Rate r", "rad/s", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["chassis.x", "Position X", "m", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["chassis.y", "Position Y", "m", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["chassis.psi", "Heading psi", "rad", "motion", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["control.gas", "Throttle Command", "[-1..1]", "powertrain", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["control.steer_rad", "Steering Command", "rad", "steering", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["wheel.rl_rpm", "Rear Left Wheel RPM", "RPM", "powertrain", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["wheel.rr_rpm", "Rear Right Wheel RPM", "RPM", "powertrain", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fy_fl", "Lateral Tire Force Fy FL", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fy_fr", "Lateral Tire Force Fy FR", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fy_rl", "Lateral Tire Force Fy RL", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fy_rr", "Lateral Tire Force Fy RR", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fx_rl", "Companion Long. Force Fx RL", "N", "tires", TelemetryChannel.SourceType.DERIVED_ESTIMATE],
		["tire.fx_rr", "Companion Long. Force Fx RR", "N", "tires", TelemetryChannel.SourceType.DERIVED_ESTIMATE],
		["tire.fz_fl", "Normal Load Fz FL", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fz_fr", "Normal Load Fz FR", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fz_rl", "Normal Load Fz RL", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["tire.fz_rr", "Normal Load Fz RR", "N", "tires", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["susp.heave_fl", "Suspension Compression FL", "m", "suspension", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["susp.heave_fr", "Suspension Compression FR", "m", "suspension", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["susp.heave_rl", "Suspension Compression RL", "m", "suspension", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["susp.heave_rr", "Suspension Compression RR", "m", "suspension", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["aero.drag_n", "Aero Drag Force", "N", "aero", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["aero.downforce_front_n", "Aero Front Downforce", "N", "aero", TelemetryChannel.SourceType.SIMULATED_MODULAR],
		["aero.downforce_rear_n", "Aero Rear Downforce", "N", "aero", TelemetryChannel.SourceType.SIMULATED_MODULAR],
	]
	for s in specs:
		rec.add_channel(TelemetryChannel.new(s[0], s[1], s[2], s[3], s[4]))

func _on_save_csv_pressed() -> void:
	if target_car != null and target_car.has_method("save_dynamics_csv_log"):
		var out_path: String = "user://modular_vehicle_dynamics_log.csv"
		var ok: bool = target_car.call("save_dynamics_csv_log", out_path)
		if ok:
			_status_footer_label.text = "Saved 30-column VehicleDynamics CSV log to %s" % ProjectSettings.globalize_path(out_path)
		else:
			_status_footer_label.text = "No logged samples yet — enable 'Start Live Recording' first."

func _on_replay_toggled(pressed: bool) -> void:
	if target_car == null:
		return
	if pressed:
		if real_reference_recording == null:
			_preload_default_imu_compare_bundle()
		if real_reference_recording != null and target_car.has_method("start_telemetry_control_replay"):
			target_car.call("start_telemetry_control_replay", real_reference_recording, true)
			if not is_recording_live:
				_record_btn.button_pressed = true
			_replay_btn.text = "Stop Control Replay (Running)"
			_status_footer_label.text = "Replaying real_control.csv commands on MirenaCar (starting at maneuver onset)."
	else:
		if target_car.has_method("stop_telemetry_control_replay"):
			target_car.call("stop_telemetry_control_replay")
		_replay_btn.text = "Replay Real Bag Controls on Car"
		_status_footer_label.text = "Stopped control replay."

func _on_bundle_selected(index: int) -> void:
	var imu_dir: String = "/home/alexr/eRacing/mirena_ws-SIM/imu_compare"
	if index >= 0 and index <= 5:
		var suffixes := ["modular_calibrated", "modular_baseline", "front10_rear12", "grip10", "grip14", "pre_aero"]
		if real_reference_recording == null:
			real_reference_recording = TelemetryImporter.load_imu_compare_real_bundle(imu_dir, 25000)
		sim_comparison_recording = TelemetryImporter.load_imu_compare_sim_bundle(imu_dir, suffixes[index], 25000)
		_recompute_correlation()
		_status_footer_label.text = "Loaded imu_compare bundle: Real Bag vs. Sim (%s)" % suffixes[index]
	elif index == 6:
		if live_session_recording != null and live_session_recording.has_usable_channel("imu.ax"):
			sim_comparison_recording = live_session_recording
			sim_comparison_recording.detect_maneuver_onset()
			var trng: Vector2 = sim_comparison_recording.get_time_range_s()
			corr_tau_start_s = 0.0
			corr_tau_end_s = maxf(5.0, trng.y - sim_comparison_recording.maneuver_onset_time_s)
			_tau_start_spin.set_value_no_signal(corr_tau_start_s)
			_tau_end_spin.set_value_no_signal(corr_tau_end_s)
			_recompute_correlation()
			_status_footer_label.text = "Comparing Real Vehicle Bag against Live Recorded Modular Run."
		else:
			_status_footer_label.text = "Record a live modular run first using 'Start Live Recording' or 'Replay Real Bag Controls'."
	elif index == 7:
		_file_dialog.popup_centered_ratio(0.65)

func _on_custom_file_selected(path: String) -> void:
	if path.ends_with("metadata.yaml"):
		var bag_info: Dictionary = TelemetryImporter.inspect_rosbag2_directory(path.get_base_dir())
		_status_footer_label.text = str(bag_info.get("message", "Inspected ROS 2 bag."))
	elif path.ends_with(".csv"):
		sim_comparison_recording = TelemetryImporter.load_single_csv(path, false, 30000)
		_recompute_correlation()
		_status_footer_label.text = "Loaded custom simulation CSV: %s" % path.get_file()

func _on_auto_align_pressed() -> void:
	if real_reference_recording == null or sim_comparison_recording == null:
		return
	var est_lag: float = CorrelationEngine.estimate_global_sync_offset_s(
		real_reference_recording, sim_comparison_recording, "imu.gz", corr_tau_start_s, corr_tau_end_s
	)
	corr_global_offset_s = est_lag
	_offset_spin.set_value_no_signal(corr_global_offset_s)
	_recompute_correlation()
	_status_footer_label.text = "Applied global synchronization offset dt = %+.3f s estimated from yaw rate (imu.gz)." % est_lag

func _on_primary_channel_selected(index: int) -> void:
	selected_primary_channel = _primary_channel_select.get_item_text(index)
	_refresh_plots_and_tables()

func _on_secondary_channel_selected(index: int) -> void:
	selected_secondary_channel = _secondary_channel_select.get_item_text(index)
	_refresh_plots_and_tables()

func _sync_tuning_controls_from_car() -> void:
	if target_car == null or _mass_spin == null:
		return
	var ctrl: Node = target_car.get("dynamics_controller")
	if ctrl == null or ctrl.get("config") == null:
		return
	_updating_controls_from_car = true
	var cfg: VehicleDynamicsConfig = ctrl.get("config")
	_mass_spin.set_value_no_signal(cfg.vehicle_parameters.total_mass_kg)
	_grip_scale_spin.set_value_no_signal(cfg.tire_config.ter_lateral_grip_scale)
	_pinn_check.set_pressed_no_signal(cfg.tire_config.enable_pinn_residual)
	_spring_f_spin.set_value_no_signal(cfg.suspension_config.spring_rate_front_npm)
	_spring_r_spin.set_value_no_signal(cfg.suspension_config.spring_rate_rear_npm)
	_arb_f_spin.set_value_no_signal(cfg.suspension_config.arb_rate_front_npm)
	_arb_r_spin.set_value_no_signal(cfg.suspension_config.arb_rate_rear_npm)
	_cl_spin.set_value_no_signal(cfg.aerodynamics_config.cl_ref)
	_cd_spin.set_value_no_signal(cfg.aerodynamics_config.cd_ref)
	_power_spin.set_value_no_signal(cfg.powertrain_config.power_limit_w / 1000.0)
	_brake_bias_spin.set_value_no_signal(cfg.brake_config.brake_bias_front)
	_updating_controls_from_car = false

func _apply_tuning_to_car() -> void:
	if _updating_controls_from_car or target_car == null:
		return
	var ctrl: Node = target_car.get("dynamics_controller")
	if ctrl == null or ctrl.get("config") == null:
		return
	var cfg: VehicleDynamicsConfig = ctrl.get("config")
	cfg.vehicle_parameters.total_mass_kg = _mass_spin.value
	cfg.tire_config.ter_lateral_grip_scale = _grip_scale_spin.value
	cfg.tire_config.enable_pinn_residual = _pinn_check.button_pressed
	cfg.suspension_config.spring_rate_front_npm = _spring_f_spin.value
	cfg.suspension_config.spring_rate_rear_npm = _spring_r_spin.value
	cfg.suspension_config.arb_rate_front_npm = _arb_f_spin.value
	cfg.suspension_config.arb_rate_rear_npm = _arb_r_spin.value
	cfg.aerodynamics_config.cl_ref = _cl_spin.value
	cfg.aerodynamics_config.cd_ref = _cd_spin.value
	cfg.powertrain_config.power_limit_w = _power_spin.value * 1000.0
	cfg.brake_config.brake_bias_front = _brake_bias_spin.value
	if target_car is RigidBody3D:
		(target_car as RigidBody3D).mass = cfg.vehicle_parameters.total_mass_kg

func _on_restore_default_params() -> void:
	if target_car == null:
		return
	var ctrl: Node = target_car.get("dynamics_controller")
	if ctrl == null:
		return
	var fresh_cfg := VehicleDynamicsConfig.new()
	fresh_cfg.ensure_subresources()
	fresh_cfg.integration_mode = ctrl.get("config").integration_mode if ctrl.get("config") != null else VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D
	ctrl.set("config", fresh_cfg)
	if ctrl.get("model") != null:
		ctrl.get("model").set("config", fresh_cfg)
	_sync_tuning_controls_from_car()
	_apply_tuning_to_car()

## Called every physics frame (or from `_physics_process`) to sample live telemetry from `target_car`.
func _physics_process(delta: float) -> void:
	if target_car == null or not is_instance_valid(target_car):
		return
	_live_elapsed_s += delta
	_live_sample_accum_s += delta
	# Record live buffer at 50 Hz (every 20 ms)
	if _live_sample_accum_s >= 0.02:
		_live_sample_accum_s = 0.0
		_capture_live_sample(_live_elapsed_s)

func _process(delta: float) -> void:
	if not visible:
		return
	_ui_refresh_accum += delta
	if _ui_refresh_accum >= 0.08:
		_ui_refresh_accum = 0.0
		_refresh_plots_and_tables()

func _capture_live_sample(t_now: float) -> void:
	if target_car == null:
		return
	_live_time_s.append(t_now)
	if _live_time_s.size() > LIVE_HISTORY_MAX:
		_live_time_s.remove_at(0)

	var snap: Dictionary = {}
	if target_car.has_method("get_live_dynamics_snapshot"):
		snap = target_car.call("get_live_dynamics_snapshot")

	for k in _live_traces.keys():
		var arr: PackedFloat64Array = _live_traces[k]
		var val: float = float(snap.get(k, 0.0))
		arr.append(val)
		if arr.size() > LIVE_HISTORY_MAX:
			arr.remove_at(0)
		if is_recording_live and live_session_recording != null:
			var ch: TelemetryChannel = live_session_recording.get_channel(k)
			if ch != null:
				ch.append_sample(t_now, val)

func _refresh_plots_and_tables() -> void:
	if active_mode == DashboardMode.LIVE_MONITORING:
		_refresh_live_plots()
		_refresh_live_readout()
	else:
		_refresh_correlation_plots()
		_refresh_correlation_readout()
		_refresh_live_readout()

func _refresh_live_plots() -> void:
	_top_plot.clear_traces()
	_top_plot.plot_mode = SignalPlotCanvas.PlotMode.TIME_SERIES
	_top_plot.plot_title = "Live Time History — %s" % selected_primary_channel
	_top_plot.x_axis_label = "Sim Time [s]"
	_top_plot.y_axis_label = selected_primary_channel

	if _live_traces.has(selected_primary_channel):
		_top_plot.add_trace(
			selected_primary_channel,
			_live_time_s,
			_live_traces[selected_primary_channel],
			Color(0.25, 0.90, 0.55),
			2.0
		)
	# Overlay partner corner on Tires/Suspension/Powertrain
	if selected_primary_channel == "tire.fy_fl" and _live_traces.has("tire.fy_fr"):
		_top_plot.add_trace("tire.fy_fr", _live_time_s, _live_traces["tire.fy_fr"], Color(0.35, 0.72, 0.98), 1.6)
	elif selected_primary_channel == "susp.heave_fl" and _live_traces.has("susp.heave_rl"):
		_top_plot.add_trace("susp.heave_rl", _live_time_s, _live_traces["susp.heave_rl"], Color(0.95, 0.65, 0.25), 1.6)

	_bottom_plot.clear_traces()
	var cat_id: String = str(CATEGORY_SPECS[active_category_index]["id"])
	if cat_id == "tires" and _live_traces.has("tire.fx_rl") and _live_traces.has("tire.fy_rl"):
		_bottom_plot.plot_mode = SignalPlotCanvas.PlotMode.XY_SCATTER
		_bottom_plot.plot_title = "Live Rear-Left Friction Circle (Fx vs Fy) + Companion Envelope"
		_bottom_plot.x_axis_label = "Longitudinal Force Fx [N]"
		_bottom_plot.y_axis_label = "Lateral Force Fy [N]"
		var fz_rl_arr: PackedFloat64Array = _live_traces.get("tire.fz_rl", PackedFloat64Array())
		var fz_now: float = fz_rl_arr[fz_rl_arr.size() - 1] if fz_rl_arr.size() > 0 else 750.0
		_bottom_plot.xy_reference_circle_radius = fz_now * 1.45
		_bottom_plot.add_trace("RL Tire Force (Fx, Fy)", _live_traces["tire.fx_rl"], _live_traces["tire.fy_rl"], Color(0.95, 0.55, 0.25), 2.0)
	else:
		_bottom_plot.plot_mode = SignalPlotCanvas.PlotMode.TIME_SERIES
		_bottom_plot.plot_title = "Live Time History — %s" % selected_secondary_channel
		_bottom_plot.x_axis_label = "Sim Time [s]"
		_bottom_plot.y_axis_label = selected_secondary_channel
		if _live_traces.has(selected_secondary_channel):
			_bottom_plot.add_trace(
				selected_secondary_channel,
				_live_time_s,
				_live_traces[selected_secondary_channel],
				Color(0.95, 0.62, 0.24),
				2.0
			)

func _refresh_correlation_plots() -> void:
	_top_plot.clear_traces()
	var m1: Dictionary = cached_correlation_metrics.get(selected_primary_channel, {})
	var st1: int = int(m1.get("status", CorrelationEngine.EvaluationStatus.NOT_EVALUATED))
	_top_plot.plot_mode = SignalPlotCanvas.PlotMode.TIME_SERIES_WITH_RESIDUAL
	_top_plot.plot_title = "%s [%s] — %s" % [
		str(m1.get("display_name", selected_primary_channel)),
		str(m1.get("unit", "")),
		str(m1.get("status_text", "Not evaluated"))
	]
	_top_plot.x_axis_label = "Maneuver Time tau [s]"
	_top_plot.y_axis_label = "%s [%s]" % [selected_primary_channel, str(m1.get("unit", ""))]

	if st1 == CorrelationEngine.EvaluationStatus.EVALUATED or st1 == CorrelationEngine.EvaluationStatus.VALIDATION_WARNING:
		var t_arr: PackedFloat64Array = m1["aligned_time_s"]
		_top_plot.add_trace("Real Vehicle (Measured)", t_arr, m1["aligned_ref"], Color(0.24, 0.68, 0.96), 2.0)
		_top_plot.add_trace("Simulation", t_arr, m1["aligned_sim"], Color(0.96, 0.50, 0.22), 1.8)
		_top_plot.set_residual_trace(t_arr, m1["residual"], Color(0.94, 0.36, 0.36))
	elif sim_comparison_recording != null and sim_comparison_recording.has_usable_channel(selected_primary_channel):
		var sch: TelemetryChannel = sim_comparison_recording.get_channel(selected_primary_channel)
		_top_plot.plot_mode = SignalPlotCanvas.PlotMode.TIME_SERIES
		_top_plot.add_trace("Simulation (Awaiting Real Reference)", sch.time_s, sch.raw_values, Color(0.96, 0.50, 0.22), 1.8)

	_bottom_plot.clear_traces()
	var m2: Dictionary = cached_correlation_metrics.get(selected_secondary_channel, {})
	var st2: int = int(m2.get("status", CorrelationEngine.EvaluationStatus.NOT_EVALUATED))
	_bottom_plot.plot_mode = SignalPlotCanvas.PlotMode.TIME_SERIES_WITH_RESIDUAL
	_bottom_plot.plot_title = "%s [%s] — %s" % [
		str(m2.get("display_name", selected_secondary_channel)),
		str(m2.get("unit", "")),
		str(m2.get("status_text", "Not evaluated"))
	]
	_bottom_plot.x_axis_label = "Maneuver Time tau [s]"
	_bottom_plot.y_axis_label = "%s [%s]" % [selected_secondary_channel, str(m2.get("unit", ""))]

	if st2 == CorrelationEngine.EvaluationStatus.EVALUATED or st2 == CorrelationEngine.EvaluationStatus.VALIDATION_WARNING:
		var t_arr2: PackedFloat64Array = m2["aligned_time_s"]
		_bottom_plot.add_trace("Real Vehicle (Measured)", t_arr2, m2["aligned_ref"], Color(0.24, 0.68, 0.96), 2.0)
		_bottom_plot.add_trace("Simulation", t_arr2, m2["aligned_sim"], Color(0.96, 0.50, 0.22), 1.8)
		_bottom_plot.set_residual_trace(t_arr2, m2["residual"], Color(0.94, 0.36, 0.36))
	elif sim_comparison_recording != null and sim_comparison_recording.has_usable_channel(selected_secondary_channel):
		var sch2: TelemetryChannel = sim_comparison_recording.get_channel(selected_secondary_channel)
		_bottom_plot.plot_mode = SignalPlotCanvas.PlotMode.TIME_SERIES
		_bottom_plot.add_trace("Simulation (Awaiting Real Reference)", sch2.time_s, sch2.raw_values, Color(0.96, 0.50, 0.22), 1.8)

func _refresh_live_readout() -> void:
	if target_car == null or not target_car.has_method("get_live_dynamics_snapshot"):
		_live_readout_label.text = "[b]No MirenaCar bound to dashboard.[/b]"
		return
	var s: Dictionary = target_car.call("get_live_dynamics_snapshot")
	var phys_mode_str: String = str(s.get("physics_mode_name", "MODULAR_VEHICLE_DYNAMICS"))
	var tire_name_str: String = str(s.get("tire_model_name", "TeR_Tire Grey-Box PINN"))

	var lines := PackedStringArray()
	lines.append("[b][color=#7fd4ff]ACTIVE AUTHORITY & CONFIGURATION[/color][/b]")
	lines.append("• Physics Authority: [b][color=#4ad674]%s[/color][/b]" % phys_mode_str)
	lines.append("• Active Tire Model: [b]%s[/b]" % tire_name_str)
	lines.append("• Sole Force Authority Verified: [b]%s[/b]" % ("YES (Legacy Wheels Detached)" if bool(s.get("modular_sole_authority", true)) else "Legacy VehicleBody3D Active"))
	lines.append("")

	lines.append("[b][color=#7fd4ff]3.1 CHASSIS MOTION & NET WRENCH (ISO 8855 +X Fwd, +Y Left, +Z Up)[/color][/b]")
	lines.append("• Speed u: [b]%+.2f m/s[/b] (%.1f km/h) | Lateral v: [b]%+.2f m/s[/b] | Sideslip beta: [b]%+.2f deg[/b]" % [
		float(s.get("chassis.u", 0.0)), float(s.get("chassis.u", 0.0)) * 3.6,
		float(s.get("chassis.v", 0.0)), float(s.get("chassis.beta_deg", 0.0))
	])
	lines.append("• Accel ax: [b]%+.2f m/s²[/b] | Lat Accel ay: [b]%+.2f m/s²[/b]" % [
		float(s.get("imu.ax", 0.0)), float(s.get("imu.ay", 0.0))
	])
	lines.append("• Yaw Rate r (gz): [b]%+.3f rad/s[/b] | Heading psi: [b]%+.2f rad[/b]" % [
		float(s.get("chassis.r", 0.0)), float(s.get("chassis.psi", 0.0))
	])
	lines.append("• Net Lateral Force Fy_net: [b]%+.1f N[/b] | Net Yaw Moment Mz_net: [b]%+.1f Nm[/b]" % [
		float(s.get("chassis.fy_net_n", 0.0)), float(s.get("chassis.mz_net_nm", 0.0))
	])
	lines.append("• Axle Lateral Forces: Front Fy_F=[b]%+.1f N[/b] | Rear Fy_R=[b]%+.1f N[/b] | Rear Reserve=[b]%.0f N[/b]" % [
		float(s.get("axle.fy_front_n", 0.0)), float(s.get("axle.fy_rear_n", 0.0)), float(s.get("axle.fy_reserve_rear_n", 0.0))
	])
	lines.append("• Roll: [b]%+.2f deg[/b] | Pitch: [b]%+.2f deg[/b]" % [
		float(s.get("chassis.roll_deg", 0.0)), float(s.get("chassis.pitch_deg", 0.0))
	])
	lines.append("")

	lines.append("[b][color=#7fd4ff]3.2 & 3.3 STEERING & TeR_Tire INTERACTION[/color][/b]")
	lines.append("• Steer Cmd: [b]%+.3f rad[/b] (%+.1f°) -> Road Wheels: FL=[b]%+.3f[/b], FR=[b]%+.3f rad[/b] (Ackermann d=%+.2f°)" % [
		float(s.get("control.steer_rad", 0.0)), rad_to_deg(float(s.get("control.steer_rad", 0.0))),
		float(s.get("steer.fl_rad", 0.0)), float(s.get("steer.fr_rad", 0.0)),
		float(s.get("steer.ackermann_diff_deg", 0.0))
	])
	lines.append("• Slip Angle alpha [rad]:     FL=%+.3f | FR=%+.3f | RL=%+.3f | RR=%+.3f" % [
		float(s.get("tire.alpha_fl", 0.0)), float(s.get("tire.alpha_fr", 0.0)),
		float(s.get("tire.alpha_rl", 0.0)), float(s.get("tire.alpha_rr", 0.0))
	])
	lines.append("• Slip Rate alpha_dot [rad/s]: FL=%+.2f | FR=%+.2f | RL=%+.2f | RR=%+.2f" % [
		float(s.get("tire.alpha_dot_fl", 0.0)), float(s.get("tire.alpha_dot_fr", 0.0)),
		float(s.get("tire.alpha_dot_rl", 0.0)), float(s.get("tire.alpha_dot_rr", 0.0))
	])
	lines.append("• ISO Inclination gamma [deg]: FL=%+.2f | FR=%+.2f | RL=%+.2f | RR=%+.2f" % [
		rad_to_deg(float(s.get("tire.gamma_fl", 0.0))), rad_to_deg(float(s.get("tire.gamma_fr", 0.0))),
		rad_to_deg(float(s.get("tire.gamma_rl", 0.0))), rad_to_deg(float(s.get("tire.gamma_rr", 0.0)))
	])
	lines.append("• Slip Ratio kappa [-]:       FL=%+.3f | FR=%+.3f | RL=%+.3f | RR=%+.3f" % [
		float(s.get("tire.kappa_fl", 0.0)), float(s.get("tire.kappa_fr", 0.0)),
		float(s.get("tire.kappa_rl", 0.0)), float(s.get("tire.kappa_rr", 0.0))
	])
	lines.append("• Static Camber Thrust Fy_c0 [N]: FL=%+.1f | FR=%+.1f | RL=%+.1f | RR=%+.1f" % [
		float(s.get("tire.static_fy_fl", 0.0)), float(s.get("tire.static_fy_fr", 0.0)),
		float(s.get("tire.static_fy_rl", 0.0)), float(s.get("tire.static_fy_rr", 0.0))
	])
	lines.append("• Pacejka Baseline Fy0 [N]:   FL=%+.0f | FR=%+.0f | RL=%+.0f | RR=%+.0f" % [
		float(s.get("tire.fy0_fl", 0.0)), float(s.get("tire.fy0_fr", 0.0)),
		float(s.get("tire.fy0_rl", 0.0)), float(s.get("tire.fy0_rr", 0.0))
	])
	lines.append("• PINN Residual dFy [N]:      FL=%+.1f | FR=%+.1f | RL=%+.1f | RR=%+.1f" % [
		float(s.get("tire.pinn_dfy_fl", 0.0)), float(s.get("tire.pinn_dfy_fr", 0.0)),
		float(s.get("tire.pinn_dfy_rl", 0.0)), float(s.get("tire.pinn_dfy_rr", 0.0))
	])
	lines.append("• Final Lateral Force Fy [N] [color=#4ad674](Calibrated TeR_Tire)[/color]:\n  FL=%+.0f | FR=%+.0f | RL=%+.0f | RR=%+.0f" % [
		float(s.get("tire.fy_fl", 0.0)), float(s.get("tire.fy_fr", 0.0)),
		float(s.get("tire.fy_rl", 0.0)), float(s.get("tire.fy_rr", 0.0))
	])
	lines.append("• Long. Force Fx [N] [color=#ebd246](Companion Slip-Ellipse)[/color]:\n  FL=%+.0f | FR=%+.0f | RL=%+.0f | RR=%+.0f" % [
		float(s.get("tire.fx_fl", 0.0)), float(s.get("tire.fx_fr", 0.0)),
		float(s.get("tire.fx_rl", 0.0)), float(s.get("tire.fx_rr", 0.0))
	])
	lines.append("• Combined-Slip Util [%%]:     FL=%3.0f%% | FR=%3.0f%% | RL=%3.0f%% | RR=%3.0f%%" % [
		float(s.get("tire.util_fl", 0.0)) * 100.0, float(s.get("tire.util_fr", 0.0)) * 100.0,
		float(s.get("tire.util_rl", 0.0)) * 100.0, float(s.get("tire.util_rr", 0.0)) * 100.0
	])
	lines.append("• Aligning Mz [Nm]:           FL=%+.1f | FR=%+.1f | RL=%+.1f | RR=%+.1f" % [
		float(s.get("tire.mz_fl", 0.0)), float(s.get("tire.mz_fr", 0.0)),
		float(s.get("tire.mz_rl", 0.0)), float(s.get("tire.mz_rr", 0.0))
	])
	lines.append("")

	lines.append("[b][color=#7fd4ff]3.4 SUSPENSION & NORMAL LOADS[/color][/b]")
	lines.append("• Normal Load Fz [N]: FL=%.0f | FR=%.0f | RL=%.0f | RR=%.0f" % [
		float(s.get("tire.fz_fl", 0.0)), float(s.get("tire.fz_fr", 0.0)),
		float(s.get("tire.fz_rl", 0.0)), float(s.get("tire.fz_rr", 0.0))
	])
	lines.append("• Heave Deflection [mm]: FL=%.1f | FR=%.1f | RL=%.1f | RR=%.1f" % [
		float(s.get("susp.heave_fl", 0.0)) * 1000.0, float(s.get("susp.heave_fr", 0.0)) * 1000.0,
		float(s.get("susp.heave_rl", 0.0)) * 1000.0, float(s.get("susp.heave_rr", 0.0)) * 1000.0
	])
	lines.append("• Spring / Damper FL [N]: F_spr=%.0f | F_dmp=%+.0f | F_arb=%+.0f" % [
		float(s.get("susp.f_spring_fl", 0.0)), float(s.get("susp.f_damper_fl", 0.0)), float(s.get("susp.f_arb_fl", 0.0))
	])
	lines.append("")

	lines.append("[b][color=#7fd4ff]3.5 & 3.6 POWERTRAIN, BRAKES & AERODYNAMICS[/color][/b]")
	lines.append("• Gas Cmd: %+.2f | Drive Torque RL/RR: %.1f / %.1f Nm" % [
		float(s.get("control.gas", 0.0)),
		float(s.get("powertrain.drive_torque_rl", 0.0)),
		float(s.get("powertrain.drive_torque_rr", 0.0))
	])
	lines.append("• Wheel Speeds [RPM]: FL=%.0f | FR=%.0f | RL=%.0f | RR=%.0f" % [
		float(s.get("wheel.fl_rpm", 0.0)), float(s.get("wheel.fr_rpm", 0.0)),
		float(s.get("wheel.rl_rpm", 0.0)), float(s.get("wheel.rr_rpm", 0.0))
	])
	lines.append("• Aero Downforce F/R: %.1f N / %.1f N | Drag: %.1f N" % [
		float(s.get("aero.downforce_front_n", 0.0)),
		float(s.get("aero.downforce_rear_n", 0.0)),
		float(s.get("aero.drag_n", 0.0))
	])
	lines.append("")

	lines.append("[b][color=#7fd4ff]3.8 NUMERICAL STABILITY & HEALTH[/color][/b]")
	lines.append("• Physics Step dt: %.2f ms | Grounded Wheels: %d/4 | Fault Mask: 0x%02X" % [
		float(s.get("health.dt_ms", 2.5)),
		int(s.get("health.contact_count", 4)),
		int(s.get("health.fault_flags", 0))
	])

	_live_readout_label.text = "\n".join(lines)

func _refresh_correlation_readout() -> void:
	var lines := PackedStringArray()
	lines.append("[b][color=#7fd4ff]REAL-VS-SIMULATION CORRELATION SUMMARY[/color][/b]")
	var agg: Dictionary = CorrelationEngine.compute_aggregate_vehicle_correlation(cached_correlation_metrics, "LAST")
	lines.append("• Overall Vehicle Correlation (HUD Metric): [b][color=#4ad674]%s[/color][/b]" % str(agg.get("hud_text", "CORR: N/A")))
	lines.append("  [color=#9ec2e6]↳ %s[/color]" % str(agg.get("details", "")))
	if real_reference_recording != null:
		lines.append("• Real Reference: [b]%s[/b] (onset=%.2fs)" % [
			real_reference_recording.recording_name, real_reference_recording.maneuver_onset_time_s
		])
	if sim_comparison_recording != null:
		lines.append("• Sim Recording: [b]%s[/b] (onset=%.2fs, mode=%s)" % [
			sim_comparison_recording.recording_name,
			sim_comparison_recording.maneuver_onset_time_s,
			sim_comparison_recording.physics_mode_label
		])
	lines.append("• Evaluated Window: tau in [%.1f, %.1f] s | Global Offset: %+.3f s" % [
		corr_tau_start_s, corr_tau_end_s, corr_global_offset_s
	])
	lines.append("")

	var spec: Dictionary = CATEGORY_SPECS[active_category_index]
	lines.append("[b][color=#ebd246]ACTIVE AREA: %s[/color][/b]" % str(spec["title"]))
	lines.append("[code]Signal          | Status     | RMSE   | MAE    | Bias   |   r   | NRMSE | Lag[s][/code]")
	lines.append("[code]----------------+------------+--------+--------+--------+-------+-------+-------[/code]")

	var ch_ids: Array = spec["primary_channels"]
	for cid_var in ch_ids:
		var cid: String = str(cid_var)
		var m: Dictionary = cached_correlation_metrics.get(cid, {})
		var st: int = int(m.get("status", CorrelationEngine.EvaluationStatus.NOT_EVALUATED))
		var st_short: String = "NOT_EVAL"
		var col_hex: String = "#9ea6b5"
		match st:
			CorrelationEngine.EvaluationStatus.EVALUATED:
				st_short = "Evaluated "
				col_hex = "#4ad674"
			CorrelationEngine.EvaluationStatus.VALIDATION_WARNING:
				st_short = "ValWarning"
				col_hex = "#f5ae2e"
			CorrelationEngine.EvaluationStatus.AWAITING_REFERENCE_DATA:
				st_short = "AwaitRef  "
				col_hex = "#73adeb"
			CorrelationEngine.EvaluationStatus.INSUFFICIENT_DATA:
				st_short = "InsuffData"
				col_hex = "#e06b59"

		if st == CorrelationEngine.EvaluationStatus.EVALUATED or st == CorrelationEngine.EvaluationStatus.VALIDATION_WARNING:
			lines.append("[code]%-15s | [color=%s]%s[/color] | %6.3f | %6.3f | %+6.3f | %+5.2f | %5.2f | %+5.2f[/code]" % [
				cid.substr(0, 15),
				col_hex,
				st_short,
				float(m.get("rmse", 0.0)),
				float(m.get("mae", 0.0)),
				float(m.get("bias", 0.0)),
				float(m.get("pearson_r", 0.0)),
				float(m.get("nrmse_std", 0.0)),
				float(m.get("estimated_lag_s", 0.0)),
			])
		else:
			lines.append("[code]%-15s | [color=%s]%s[/color] |   —    |   —    |   —    |   —   |   —   |   —  [/code]" % [
				cid.substr(0, 15), col_hex, st_short
			])
			var reason: String = str(m.get("status_reason", ""))
			if not reason.is_empty():
				lines.append("  [color=#8fa3bf]↳ %s[/color]" % reason)

	lines.append("")
	lines.append("[b][color=#7fd4ff]DETAILED METRICS FOR SELECTED SIGNAL (%s)[/color][/b]" % selected_primary_channel)
	var mp: Dictionary = cached_correlation_metrics.get(selected_primary_channel, {})
	if not mp.is_empty():
		lines.append("• Evaluation Status: [b]%s[/b]" % str(mp.get("status_text", "Not evaluated")))
		lines.append("• Real Source Provenance: %s" % TelemetryChannel.source_type_name(int(mp.get("ref_source", TelemetryChannel.SourceType.UNAVAILABLE))))
		lines.append("• Sim Source Provenance:  %s" % TelemetryChannel.source_type_name(int(mp.get("sim_source", TelemetryChannel.SourceType.UNAVAILABLE))))
		if int(mp.get("sample_count", 0)) > 0:
			lines.append("• Overlapping Samples: %d" % int(mp.get("sample_count", 0)))
			lines.append("• RMSE: %.4f %s | MAE: %.4f %s | Bias: %+.4f %s" % [
				float(mp.get("rmse", 0.0)), str(mp.get("unit", "")),
				float(mp.get("mae", 0.0)), str(mp.get("unit", "")),
				float(mp.get("bias", 0.0)), str(mp.get("unit", ""))
			])
			lines.append("• Pearson r: %+.3f | NRMSE (std): %.3f | NRMSE (range): %.3f" % [
				float(mp.get("pearson_r", 0.0)),
				float(mp.get("nrmse_std", 0.0)),
				float(mp.get("nrmse_range", 0.0))
			])
			lines.append("• Peak Abs Error: %.4f | Peak Amp Diff: %+.4f | Sim/Real Gain: %.3f" % [
				float(mp.get("peak_abs_error", 0.0)),
				float(mp.get("peak_amp_diff", 0.0)),
				float(mp.get("gain_sim_over_ref", 0.0))
			])
			lines.append("• Cross-Correlation Lag Estimate: %+.3f s" % float(mp.get("estimated_lag_s", 0.0)))

	if real_reference_recording != null and not real_reference_recording.known_limitations.is_empty():
		lines.append("")
		lines.append("[b][color=#f5ae2e]RECORDING LIMITATIONS & SENSOR AVAILABILITY[/color][/b]")
		for lim in real_reference_recording.known_limitations:
			lines.append("• " + lim)

	_metrics_readout_label.text = "\n".join(lines)
