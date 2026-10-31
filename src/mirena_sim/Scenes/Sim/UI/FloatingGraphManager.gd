extends Control
class_name FloatingGraphManager

const FloatingGraphPanel = preload("res://Scenes/Sim/UI/FloatingGraphPanel.gd")
const GraphConfigDialog = preload("res://Scenes/Sim/UI/GraphConfigDialog.gd")
const MirenaUITheme = preload("res://Scenes/Sim/UI/MirenaUITheme.gd")

## Manages floating telemetry graph panels in MirenaSim.
## Styled to match the Options Menu visual design system.
## Features:
## - Creation, destruction, repositioning, and resizing of floating panels.
## - GRAPHS button popup menu (Add, Available, Presets, Show/Hide, Reset, Manage).
## - Standard motorsport presets (Vehicle Dynamics, Tires, Suspension, Powertrain).
## - Real-time telemetry feed from car dynamics snapshot to active panels.
## - Layout persistence to user://floating_graphs.cfg.

const CONFIG_PATH: String = "user://floating_graphs.cfg"

@export var car: MirenaCar
@export var graphs_button: Button

var panels: Array[FloatingGraphPanel] = []
var config_dialog: GraphConfigDialog = null
var graphs_menu: PopupMenu = null
var available_menu: PopupMenu = null
var presets_menu: PopupMenu = null

var _elapsed_sim_time: float = 0.0
var _save_timer: float = 0.0
var _is_layout_dirty: bool = false
var _are_graphs_hidden: bool = false

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_resolve_car_reference()
	_setup_dialog()
	_setup_popup_menu()
	
	if graphs_button != null:
		graphs_button.pressed.connect(_on_graphs_button_pressed)
	
	# Load persisted layout or load default preset
	call_deferred("_load_or_create_default_layout")

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

func _setup_dialog() -> void:
	config_dialog = GraphConfigDialog.new()
	config_dialog.visible = false
	add_child(config_dialog)
	config_dialog.applied.connect(_on_config_applied)
	config_dialog.deleted.connect(_on_config_deleted)
	config_dialog.cancelled.connect(_on_config_cancelled)

func _setup_popup_menu() -> void:
	graphs_menu = PopupMenu.new()
	graphs_menu.name = "GraphsPopupMenu"
	add_child(graphs_menu)
	MirenaUITheme.style_popup_menu(graphs_menu)
	
	# Submenu 1: Available Graphs
	available_menu = PopupMenu.new()
	available_menu.name = "AvailableGraphsSubmenu"
	graphs_menu.add_child(available_menu)
	MirenaUITheme.style_popup_menu(available_menu)
	_populate_available_menu()
	available_menu.id_pressed.connect(_on_available_graph_selected)
	
	# Submenu 2: Graph Presets
	presets_menu = PopupMenu.new()
	presets_menu.name = "PresetsSubmenu"
	graphs_menu.add_child(presets_menu)
	MirenaUITheme.style_popup_menu(presets_menu)
	_populate_presets_menu()
	presets_menu.id_pressed.connect(_on_preset_selected)
	
	# Main Menu Structure
	graphs_menu.add_item("➕ Add Custom Graph...", 100)
	graphs_menu.add_separator()
	graphs_menu.add_submenu_node_item("📊 Quick Add Graph", available_menu, 200)
	graphs_menu.add_submenu_node_item("📑 Load Preset Layout", presets_menu, 300)
	graphs_menu.add_separator()
	graphs_menu.add_item("👁️ Show All Graphs", 400)
	graphs_menu.add_item("🚫 Hide All Graphs", 401)
	graphs_menu.add_item("🔄 Reset to Default Layout", 402)
	graphs_menu.add_separator()
	graphs_menu.add_item("⚙️ Manage Active Graphs...", 500)
	
	graphs_menu.id_pressed.connect(_on_graphs_menu_id_pressed)

func _populate_available_menu() -> void:
	available_menu.clear()
	available_menu.add_item("Speed vs Time", 1)
	available_menu.add_item("Longitudinal & Lateral Accel (Ax, Ay)", 2)
	available_menu.add_item("G-G Diagram (Ay vs Ax)", 3)
	available_menu.add_item("Chassis Yaw Rate", 4)
	available_menu.add_item("Body Sideslip Angle (Beta)", 5)
	available_menu.add_item("Normal Wheel Loads (Fz FL/FR/RL/RR)", 6)
	available_menu.add_item("Tire Slip Angles (Alpha)", 7)
	available_menu.add_item("Suspension Deflection / Heave", 8)
	available_menu.add_item("Speedometer Gauge", 9)
	available_menu.add_item("Lateral Accel Distribution", 10)

func _populate_presets_menu() -> void:
	presets_menu.clear()
	presets_menu.add_item("Vehicle Dynamics (Speed, Accel, G-G, Yaw)", 1)
	presets_menu.add_item("Tire Analysis (Slip, Fy, Fz Loads)", 2)
	presets_menu.add_item("Suspension Analysis (Deflection & Heave)", 3)
	presets_menu.add_item("Powertrain & Controls (Throttle, Brake, RPM)", 4)

func _on_graphs_button_pressed() -> void:
	if graphs_menu == null:
		return
	if graphs_button != null:
		var btn_rect: Rect2 = graphs_button.get_global_rect()
		var popup_pos := Vector2(btn_rect.position.x, btn_rect.end.y + 4.0)
		graphs_menu.position = Vector2i(int(popup_pos.x), int(popup_pos.y))
	else:
		graphs_menu.position = Vector2i(100, 100)
	graphs_menu.popup()

func _on_graphs_menu_id_pressed(id: int) -> void:
	match id:
		100: # Add Custom Graph
			open_add_graph_dialog()
		400: # Show All
			set_all_graphs_visible(true)
		401: # Hide All
			set_all_graphs_visible(false)
		402: # Reset to Default
			reset_layout()
		500: # Manage Active Graphs
			open_manage_graphs_dialog()

func _on_available_graph_selected(id: int) -> void:
	match id:
		1: # Speed vs Time
			create_graph({
				"title": "Speed vs Time",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["chassis.speed_kmh"],
				"time_window_s": 15.0,
				"auto_scale_y": true
			})
		2: # Accel Ax, Ay
			create_graph({
				"title": "Accelerations (Ax & Ay)",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["imu.ax", "imu.ay"],
				"time_window_s": 15.0,
				"auto_scale_y": false,
				"manual_min_y": -15.0,
				"manual_max_y": 15.0
			})
		3: # G-G Diagram
			create_graph({
				"title": "G-G Acceleration Diagram",
				"type": FloatingGraphPanel.GraphType.XY_PLOT,
				"x_channel": "imu.ay",
				"y_channels": ["imu.ax"],
				"time_window_s": 20.0,
				"auto_scale_x": false,
				"manual_min_x": -15.0,
				"manual_max_x": 15.0,
				"auto_scale_y": false,
				"manual_min_y": -15.0,
				"manual_max_y": 15.0
			})
		4: # Yaw Rate
			create_graph({
				"title": "Chassis Yaw Rate",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["imu.gz"],
				"time_window_s": 15.0,
				"auto_scale_y": true
			})
		5: # Sideslip Beta
			create_graph({
				"title": "Sideslip Angle β",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["chassis.beta_deg"],
				"time_window_s": 15.0,
				"auto_scale_y": true
			})
		6: # Normal Wheel Loads
			create_graph({
				"title": "Normal Loads (Fz)",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["wheel.fz_fl", "wheel.fz_fr", "wheel.fz_rl", "wheel.fz_rr"],
				"time_window_s": 15.0,
				"auto_scale_y": true
			})
		7: # Slip Angles
			create_graph({
				"title": "Slip Angles (Alpha)",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["tire.alpha_fl", "tire.alpha_fr", "tire.alpha_rl", "tire.alpha_rr"],
				"time_window_s": 15.0,
				"auto_scale_y": true
			})
		8: # Suspension Heave
			create_graph({
				"title": "Suspension Deflection",
				"type": FloatingGraphPanel.GraphType.TIME_SERIES,
				"x_channel": "time",
				"y_channels": ["susp.deflection_fl", "susp.deflection_fr", "susp.deflection_rl", "susp.deflection_rr"],
				"time_window_s": 15.0,
				"auto_scale_y": true
			})
		9: # Speedometer Gauge
			create_graph({
				"title": "Speedometer",
				"type": FloatingGraphPanel.GraphType.GAUGE,
				"y_channels": ["chassis.speed_kmh"],
				"auto_scale_y": false,
				"manual_min_y": 0.0,
				"manual_max_y": 140.0
			})
		10: # Lateral Accel Distribution
			create_graph({
				"title": "Ay Distribution",
				"type": FloatingGraphPanel.GraphType.HISTOGRAM,
				"y_channels": ["imu.ay"],
				"auto_scale_y": false,
				"manual_min_y": -15.0,
				"manual_max_y": 15.0
			})
	_save_layout_soon()

func _on_preset_selected(id: int) -> void:
	match id:
		1:
			load_preset_vehicle_dynamics()
		2:
			load_preset_tire_analysis()
		3:
			load_preset_suspension_analysis()
		4:
			load_preset_powertrain()
	_save_layout_soon()

## Creates a new FloatingGraphPanel from a dictionary specification
func create_graph(cfg: Dictionary) -> FloatingGraphPanel:
	var panel := FloatingGraphPanel.new()
	panel.graph_id = cfg.get("id", "graph_%d" % Time.get_ticks_msec())
	panel.graph_title = cfg.get("title", "Telemetry Graph")
	panel.graph_type = int(cfg.get("type", FloatingGraphPanel.GraphType.TIME_SERIES))
	panel.x_channel = cfg.get("x_channel", "time")
	var ys: Array = cfg.get("y_channels", ["chassis.speed_kmh"])
	panel.y_channels.clear()
	for y in ys:
		panel.y_channels.append(str(y))
	panel.time_window_s = float(cfg.get("time_window_s", 15.0))
	panel.auto_scale_y = bool(cfg.get("auto_scale_y", true))
	panel.manual_min_y = float(cfg.get("manual_min_y", -10.0))
	panel.manual_max_y = float(cfg.get("manual_max_y", 10.0))
	panel.auto_scale_x = bool(cfg.get("auto_scale_x", true))
	panel.manual_min_x = float(cfg.get("manual_min_x", -10.0))
	panel.manual_max_x = float(cfg.get("manual_max_x", 10.0))
	panel.show_grid = bool(cfg.get("show_grid", true))
	panel.show_legend = bool(cfg.get("show_legend", true))
	panel.is_collapsed = bool(cfg.get("is_collapsed", false))
	
	add_child(panel)
	panels.append(panel)
	
	panel.closed.connect(_on_panel_closed)
	panel.configure_requested.connect(_on_panel_configure_requested)
	panel.layout_changed.connect(_on_panel_layout_changed)
	
	# Position panel
	if cfg.has("pos_x") and cfg.has("pos_y"):
		panel.position = Vector2(float(cfg.get("pos_x")), float(cfg.get("pos_y")))
	else:
		# Cascade position
		var offset_idx: int = panels.size() - 1
		panel.position = Vector2(24.0 + (offset_idx % 4) * 30.0, 96.0 + (offset_idx % 4) * 30.0)
	
	if cfg.has("size_w") and cfg.has("size_h"):
		panel.size = Vector2(float(cfg.get("size_w")), float(cfg.get("size_h")))
	
	panel.visible = !_are_graphs_hidden
	return panel

func remove_graph(panel: FloatingGraphPanel) -> void:
	if panel == null:
		return
	panels.erase(panel)
	panel.queue_free()
	_save_layout_soon()

func clear_all_graphs() -> void:
	for p in panels:
		p.queue_free()
	panels.clear()

func set_all_graphs_visible(is_vis: bool) -> void:
	_are_graphs_hidden = !is_vis
	for p in panels:
		p.visible = is_vis

func open_add_graph_dialog() -> void:
	if config_dialog == null:
		return
	var dummy_panel := FloatingGraphPanel.new()
	dummy_panel.graph_id = "graph_%d" % Time.get_ticks_msec()
	dummy_panel.graph_title = "New Telemetry Graph"
	dummy_panel.y_channels = ["chassis.speed_kmh"]
	config_dialog.populate_from_panel(dummy_panel)
	_center_dialog(config_dialog)
	config_dialog.visible = true
	dummy_panel.queue_free()

func open_manage_graphs_dialog() -> void:
	if panels.is_empty():
		open_add_graph_dialog()
		return
	_on_panel_configure_requested(panels[0])

func _on_panel_configure_requested(panel: FloatingGraphPanel) -> void:
	if config_dialog == null or panel == null:
		return
	config_dialog.populate_from_panel(panel)
	_center_dialog(config_dialog)
	config_dialog.visible = true

func _on_panel_closed(panel: FloatingGraphPanel) -> void:
	remove_graph(panel)

func _on_panel_layout_changed(_panel: FloatingGraphPanel) -> void:
	_save_layout_soon()

func _on_config_applied(cfg: Dictionary) -> void:
	var gid: String = str(cfg.get("id", ""))
	var existing_panel: FloatingGraphPanel = null
	for p in panels:
		if p.graph_id == gid:
			existing_panel = p
			break
	
	if existing_panel != null:
		existing_panel.deserialize_config(cfg)
	else:
		create_graph(cfg)
	
	if config_dialog != null:
		config_dialog.visible = false
	_save_layout_soon()

func _on_config_deleted(gid: String) -> void:
	for p in panels:
		if p.graph_id == gid:
			remove_graph(p)
			break
	if config_dialog != null:
		config_dialog.visible = false

func _on_config_cancelled() -> void:
	if config_dialog != null:
		config_dialog.visible = false

func _center_dialog(dlg: Control) -> void:
	var vp_size: Vector2 = get_viewport_rect().size
	dlg.position = Vector2(
		maxf(20.0, (vp_size.x - dlg.size.x) * 0.5),
		maxf(80.0, (vp_size.y - dlg.size.y) * 0.5)
	)
	dlg.move_to_front()

# --- PRESET IMPLEMENTATIONS ---

func load_preset_vehicle_dynamics() -> void:
	clear_all_graphs()
	var vp_size: Vector2 = get_viewport_rect().size
	if vp_size.x < 100.0:
		vp_size = Vector2(1280, 720)
	
	# 1. Top-Left: Speed & Kinematics vs Time
	create_graph({
		"id": "vd_speed_accel",
		"title": "Speed & Acceleration",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["chassis.speed_kmh", "imu.ax", "imu.ay"],
		"time_window_s": 15.0,
		"auto_scale_y": true,
		"pos_x": 20.0,
		"pos_y": 90.0,
		"size_w": 380.0,
		"size_h": 220.0
	})
	
	# 2. Top-Right: Yaw Rate & Sideslip Beta
	create_graph({
		"id": "vd_yaw_beta",
		"title": "Yaw Rate & Sideslip β",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["imu.gz", "chassis.beta_deg"],
		"time_window_s": 15.0,
		"auto_scale_y": true,
		"pos_x": maxf(420.0, vp_size.x - 390.0),
		"pos_y": 90.0,
		"size_w": 370.0,
		"size_h": 220.0
	})
	
	# 3. Bottom-Right: G-G Diagram (Configurable XY Plot)
	create_graph({
		"id": "vd_gg_diagram",
		"title": "G-G Acceleration Diagram",
		"type": FloatingGraphPanel.GraphType.XY_PLOT,
		"x_channel": "imu.ay",
		"y_channels": ["imu.ax"],
		"time_window_s": 25.0,
		"auto_scale_x": false,
		"manual_min_x": -15.0,
		"manual_max_x": 15.0,
		"auto_scale_y": false,
		"manual_min_y": -15.0,
		"manual_max_y": 15.0,
		"pos_x": maxf(420.0, vp_size.x - 330.0),
		"pos_y": maxf(330.0, vp_size.y - 270.0),
		"size_w": 310.0,
		"size_h": 250.0
	})

func load_preset_tire_analysis() -> void:
	clear_all_graphs()
	var vp_size: Vector2 = get_viewport_rect().size
	if vp_size.x < 100.0:
		vp_size = Vector2(1280, 720)
	
	# 1. Slip angles FL/FR/RL/RR
	create_graph({
		"id": "tire_alpha",
		"title": "Tire Slip Angles (α)",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["tire.alpha_fl", "tire.alpha_fr", "tire.alpha_rl", "tire.alpha_rr"],
		"time_window_s": 15.0,
		"auto_scale_y": true,
		"pos_x": 20.0,
		"pos_y": 90.0,
		"size_w": 380.0,
		"size_h": 220.0
	})
	
	# 2. Wheel Normal Loads Fz
	create_graph({
		"id": "tire_fz",
		"title": "Dynamic Normal Loads (Fz)",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["wheel.fz_fl", "wheel.fz_fr", "wheel.fz_rl", "wheel.fz_rr"],
		"time_window_s": 15.0,
		"auto_scale_y": true,
		"pos_x": maxf(420.0, vp_size.x - 390.0),
		"pos_y": 90.0,
		"size_w": 370.0,
		"size_h": 220.0
	})
	
	# 3. Tire Fy vs Alpha Front Left
	create_graph({
		"id": "tire_alpha_fy_fl",
		"title": "FL Cornering: α vs Fy",
		"type": FloatingGraphPanel.GraphType.XY_PLOT,
		"x_channel": "tire.alpha_fl",
		"y_channels": ["tire.fy_fl"],
		"time_window_s": 25.0,
		"auto_scale_x": true,
		"auto_scale_y": true,
		"pos_x": maxf(420.0, vp_size.x - 330.0),
		"pos_y": maxf(330.0, vp_size.y - 270.0),
		"size_w": 310.0,
		"size_h": 250.0
	})

func load_preset_suspension_analysis() -> void:
	clear_all_graphs()
	var vp_size: Vector2 = get_viewport_rect().size
	if vp_size.x < 100.0:
		vp_size = Vector2(1280, 720)
	
	create_graph({
		"id": "susp_deflection",
		"title": "Suspension Deflections",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["susp.deflection_fl", "susp.deflection_fr", "susp.deflection_rl", "susp.deflection_rr"],
		"time_window_s": 15.0,
		"auto_scale_y": true,
		"pos_x": 20.0,
		"pos_y": 90.0,
		"size_w": 380.0,
		"size_h": 220.0
	})
	
	create_graph({
		"id": "chassis_attitudes",
		"title": "Roll & Pitch Angles",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["chassis.roll_deg", "chassis.pitch_deg"],
		"time_window_s": 15.0,
		"auto_scale_y": true,
		"pos_x": maxf(420.0, vp_size.x - 390.0),
		"pos_y": 90.0,
		"size_w": 370.0,
		"size_h": 220.0
	})

func load_preset_powertrain() -> void:
	clear_all_graphs()
	var vp_size: Vector2 = get_viewport_rect().size
	if vp_size.x < 100.0:
		vp_size = Vector2(1280, 720)
	
	create_graph({
		"id": "driver_inputs",
		"title": "Throttle & Brake Demand",
		"type": FloatingGraphPanel.GraphType.TIME_SERIES,
		"x_channel": "time",
		"y_channels": ["control.throttle", "control.brake"],
		"time_window_s": 15.0,
		"auto_scale_y": false,
		"manual_min_y": 0.0,
		"manual_max_y": 1.0,
		"pos_x": 20.0,
		"pos_y": 90.0,
		"size_w": 380.0,
		"size_h": 220.0
	})
	
	create_graph({
		"id": "speed_gauge",
		"title": "Vehicle Speedometer",
		"type": FloatingGraphPanel.GraphType.GAUGE,
		"y_channels": ["chassis.speed_kmh"],
		"auto_scale_y": false,
		"manual_min_y": 0.0,
		"manual_max_y": 140.0,
		"pos_x": maxf(420.0, vp_size.x - 300.0),
		"pos_y": 90.0,
		"size_w": 280.0,
		"size_h": 220.0
	})

func reset_layout() -> void:
	load_preset_vehicle_dynamics()
	_save_layout_soon()

# --- PERSISTENCE: user://floating_graphs.cfg ---

func _save_layout_soon() -> void:
	_is_layout_dirty = true
	_save_timer = 0.5 # Debounce 500 ms

func save_layout() -> void:
	_is_layout_dirty = false
	var cfg := ConfigFile.new()
	cfg.set_value("system", "panel_count", panels.size())
	cfg.set_value("system", "graphs_hidden", _are_graphs_hidden)
	
	for i in range(panels.size()):
		var p: FloatingGraphPanel = panels[i]
		var section: String = "panel_%d" % i
		var data: Dictionary = p.serialize_config()
		for k in data:
			cfg.set_value(section, k, data[k])
	
	cfg.save(CONFIG_PATH)

func load_layout() -> bool:
	var cfg := ConfigFile.new()
	var err: Error = cfg.load(CONFIG_PATH)
	if err != OK:
		return false
	
	clear_all_graphs()
	var count: int = int(cfg.get_value("system", "panel_count", 0))
	_are_graphs_hidden = bool(cfg.get_value("system", "graphs_hidden", false))
	
	if count <= 0:
		return false
	
	for i in range(count):
		var section: String = "panel_%d" % i
		if not cfg.has_section(section):
			continue
		var data: Dictionary = {}
		for k in cfg.get_section_keys(section):
			data[k] = cfg.get_value(section, k)
		create_graph(data)
	
	return true

func _load_or_create_default_layout() -> void:
	var loaded: bool = load_layout()
	if not loaded or panels.is_empty():
		load_preset_vehicle_dynamics()

# --- TELEMETRY TICK PROCESS ---

func _process(delta: float) -> void:
	_elapsed_sim_time += delta
	
	# Layout autosave debounce
	if _is_layout_dirty:
		_save_timer -= delta
		if _save_timer <= 0.0:
			save_layout()
	
	if panels.is_empty() or _are_graphs_hidden:
		return
	
	if car == null or not is_instance_valid(car):
		_resolve_car_reference()
	if car == null or not car.has_method("get_live_dynamics_snapshot"):
		return
	
	var snap: Dictionary = car.call("get_live_dynamics_snapshot")
	
	# Push telemetry snapshot to all open panels
	for p in panels:
		if p.visible and not p.is_paused:
			p.sample_snapshot(snap, _elapsed_sim_time)
