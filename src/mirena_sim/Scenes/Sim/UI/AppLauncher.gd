extends PanelContainer
class_name VehicleAppLauncher

const MirenaUITheme = preload("res://Scenes/Sim/UI/MirenaUITheme.gd")
const FloatingGraphManager = preload("res://Scenes/Sim/UI/FloatingGraphManager.gd")

## Motorsport Engineering Applications Launcher Dock
## Integrated into the Top HUD in exact visual alignment with the Options Menu theme.
## Exposes one-click shortcuts to real existing diagnostic, telemetry, and setup tools:
## 1. VD: Vehicle Dynamics Overview (F7 Mode A)
## 2. CORR: Diagnostics & Quantitative Correlation (F7 Mode B)
## 3. LOG: Telemetry Recorder & Data Logger
## 4. TIRE: Tire & Road Interaction / Friction Circle Analysis
## 5. SUSP: Suspension Deflections & Dynamic Normal Loads
## 6. SETUP: Real-Time Vehicle Setup & Parameter Tuning
## 7. GRAPHS: Configurable Floating Telemetry Graphs Menu
## 8. MENU: Simulation Options, AS Missions & Track Menu

@export var car: MirenaCar
@export var main_menu: Control

@onready var btn_vd: Button = find_child("BtnVD", true, false) as Button
@onready var btn_corr: Button = find_child("BtnCorr", true, false) as Button
@onready var btn_log: Button = find_child("BtnLog", true, false) as Button
@onready var btn_tire: Button = find_child("BtnTire", true, false) as Button
@onready var btn_susp: Button = find_child("BtnSusp", true, false) as Button
@onready var btn_setup: Button = find_child("BtnSetup", true, false) as Button
@onready var btn_graphs: Button = find_child("BtnGraphs", true, false) as Button
@onready var btn_menu: Button = find_child("BtnMenu", true, false) as Button

func _ready() -> void:
	# Apply Options Menu aesthetic
	add_theme_stylebox_override("panel", MirenaUITheme.create_panel_stylebox(0.55, 0.40, 4, 2, 4))
	_resolve_references()
	_style_buttons()
	_connect_buttons()

func _resolve_references() -> void:
	if car == null or not is_instance_valid(car):
		if get_tree() != null:
			car = get_tree().get_first_node_in_group("Car") as MirenaCar
			if car == null:
				car = get_tree().get_first_node_in_group("vehicle") as MirenaCar
		var sim_autoload: Node = get_node_or_null("/root/Sim")
		if car == null and sim_autoload != null and sim_autoload.get("car") is MirenaCar:
			car = sim_autoload.get("car") as MirenaCar

	if main_menu == null:
		var sim_ui: Node = get_parent()
		while sim_ui != null and not (sim_ui is Control and sim_ui.has_node("MainMenu")):
			sim_ui = sim_ui.get_parent()
		if sim_ui != null:
			main_menu = sim_ui.get_node_or_null("MainMenu") as Control

func _style_buttons() -> void:
	var btns: Array[Button] = [btn_vd, btn_corr, btn_log, btn_tire, btn_susp, btn_setup, btn_graphs, btn_menu]
	for b in btns:
		if b != null:
			MirenaUITheme.style_button(b)
			b.custom_minimum_size = Vector2(36, 22)
			b.add_theme_font_size_override("font_size", 10)

func _connect_buttons() -> void:
	if btn_vd != null and not btn_vd.pressed.is_connected(_on_btn_vd_pressed):
		btn_vd.pressed.connect(_on_btn_vd_pressed)
	if btn_corr != null and not btn_corr.pressed.is_connected(_on_btn_corr_pressed):
		btn_corr.pressed.connect(_on_btn_corr_pressed)
	if btn_log != null and not btn_log.pressed.is_connected(_on_btn_log_pressed):
		btn_log.pressed.connect(_on_btn_log_pressed)
	if btn_tire != null and not btn_tire.pressed.is_connected(_on_btn_tire_pressed):
		btn_tire.pressed.connect(_on_btn_tire_pressed)
	if btn_susp != null and not btn_susp.pressed.is_connected(_on_btn_susp_pressed):
		btn_susp.pressed.connect(_on_btn_susp_pressed)
	if btn_setup != null and not btn_setup.pressed.is_connected(_on_btn_setup_pressed):
		btn_setup.pressed.connect(_on_btn_setup_pressed)
	if btn_graphs != null and not btn_graphs.pressed.is_connected(_on_btn_graphs_pressed):
		btn_graphs.pressed.connect(_on_btn_graphs_pressed)
	if btn_menu != null and not btn_menu.pressed.is_connected(_on_btn_menu_pressed):
		btn_menu.pressed.connect(_on_btn_menu_pressed)

func _get_or_open_dashboard() -> VehicleDynamicsDashboardWindow:
	if car == null or not is_instance_valid(car):
		_resolve_references()
	if car != null and car.has_method("open_dynamics_dashboard_window"):
		return car.call("open_dynamics_dashboard_window") as VehicleDynamicsDashboardWindow
	return null

func _on_btn_vd_pressed() -> void:
	var dash := _get_or_open_dashboard()
	if dash != null:
		dash.show()
		dash.grab_focus()
		if dash.has_method("_on_mode_selected"):
			dash.call("_on_mode_selected", 0) # Mode A: Live Simulation Monitoring
		if dash.has_method("_select_category"):
			dash.call("_select_category", 0) # 3.1 Vehicle Motion & Chassis

func _on_btn_corr_pressed() -> void:
	var dash := _get_or_open_dashboard()
	if dash != null:
		dash.show()
		dash.grab_focus()
		if dash.has_method("_on_mode_selected"):
			dash.call("_on_mode_selected", 1) # Mode B: Real-vs-Simulation Correlation
		if dash.has_method("_select_category"):
			dash.call("_select_category", 0)

func _on_btn_log_pressed() -> void:
	var dash := _get_or_open_dashboard()
	if dash != null:
		dash.show()
		dash.grab_focus()
		if dash.has_method("_on_mode_selected"):
			dash.call("_on_mode_selected", 0)
		if dash.has_method("_select_category"):
			dash.call("_select_category", 6) # 3.7 Sensors & ROS Consistency / Logger

func _on_btn_tire_pressed() -> void:
	var dash := _get_or_open_dashboard()
	if dash != null:
		dash.show()
		dash.grab_focus()
		if dash.has_method("_on_mode_selected"):
			dash.call("_on_mode_selected", 0)
		if dash.has_method("_select_category"):
			dash.call("_select_category", 2) # 3.3 Tires & Road Interaction

func _on_btn_susp_pressed() -> void:
	var dash := _get_or_open_dashboard()
	if dash != null:
		dash.show()
		dash.grab_focus()
		if dash.has_method("_on_mode_selected"):
			dash.call("_on_mode_selected", 0)
		if dash.has_method("_select_category"):
			dash.call("_select_category", 3) # 3.4 Suspension & Wheel Loads

func _on_btn_setup_pressed() -> void:
	var dash := _get_or_open_dashboard()
	if dash != null:
		dash.show()
		dash.grab_focus()
		if dash.has_method("_on_mode_selected"):
			dash.call("_on_mode_selected", 0)
		if dash.has_method("_select_category"):
			dash.call("_select_category", 0)

func _on_btn_graphs_pressed() -> void:
	# Locate FloatingGraphManager in tree
	var root: Node = get_tree().root if get_tree() != null else null
	var graph_mgr: FloatingGraphManager = null
	if root != null:
		graph_mgr = root.find_child("FloatingGraphManager", true, false) as FloatingGraphManager
	if graph_mgr != null:
		graph_mgr.graphs_button = btn_graphs
		graph_mgr._on_graphs_button_pressed()

func _on_btn_menu_pressed() -> void:
	if main_menu == null:
		_resolve_references()
	if main_menu != null:
		main_menu.visible = !main_menu.visible
