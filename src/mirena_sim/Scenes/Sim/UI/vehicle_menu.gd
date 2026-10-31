extends MarginContainer

# Text for each MissionInfo.status value (index matches mirena_common/MissionInfo constants).
const MISSION_STATUS_NAMES := ["INACTIVE", "CONFIGURING", "READY", "RUNNING", "FINISHED", "FAILED"]


var _physics_mode_select: OptionButton = null
var _tire_model_select: OptionButton = null

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	_build_dynamics_controls()

func _build_dynamics_controls() -> void:
	var vbox: VBoxContainer = get_node_or_null("Vehicle") as VBoxContainer
	if vbox == null:
		return

	var sep := HSeparator.new()
	vbox.add_child(sep)

	var phys_row := HBoxContainer.new()
	phys_row.add_theme_constant_override("separation", 10)
	var phys_lbl := Label.new()
	phys_lbl.text = "PHYSICS:"
	phys_row.add_child(phys_lbl)

	_physics_mode_select = OptionButton.new()
	_physics_mode_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_physics_mode_select.add_item("Modular VehicleDynamics (6-DOF)", 1)
	_physics_mode_select.add_item("Legacy VehicleBody3D (Raycast)", 0)
	_physics_mode_select.selected = 0
	_physics_mode_select.item_selected.connect(_on_physics_engine_selected)
	phys_row.add_child(_physics_mode_select)
	vbox.add_child(phys_row)

	var tire_row := HBoxContainer.new()
	tire_row.add_theme_constant_override("separation", 10)
	var tire_lbl := Label.new()
	tire_lbl.text = "TIRE:"
	tire_row.add_child(tire_lbl)

	_tire_model_select = OptionButton.new()
	_tire_model_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_tire_model_select.add_item("TeR_Tire Grey-Box (MF6.2 + PINN)", 0)
	_tire_model_select.add_item("Pacejka MF6.2 Analytical", 1)
	_tire_model_select.add_item("Linear Reference Stub", 2)
	_tire_model_select.selected = 0
	_tire_model_select.item_selected.connect(_on_tire_model_selected)
	tire_row.add_child(_tire_model_select)
	vbox.add_child(tire_row)

	var dash_btn := Button.new()
	dash_btn.text = "Open Dynamics & Correlation Window (F7)"
	dash_btn.pressed.connect(func():
		if Sim.car != null and Sim.car.has_method("toggle_dynamics_dashboard_window"):
			Sim.car.toggle_dynamics_dashboard_window()
	)
	vbox.add_child(dash_btn)

func _on_physics_engine_selected(index: int) -> void:
	if Sim.car != null and Sim.car.has_method("set_physics_engine_mode"):
		var mode_id: int = _physics_mode_select.get_item_id(index)
		Sim.car.set_physics_engine_mode(mode_id)

func _on_tire_model_selected(index: int) -> void:
	if Sim.car != null and Sim.car.has_method("set_active_tire_model_type"):
		match index:
			0:
				Sim.car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN)
			1:
				Sim.car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.PACEJKA_MF62)
			2:
				Sim.car.set_active_tire_model_type(VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB)

func _process(delta):
	var ms_idx: int = Sim.car._mission_info.status
	var ms_text: String = MISSION_STATUS_NAMES[ms_idx] if ms_idx >= 0 and ms_idx < MISSION_STATUS_NAMES.size() else "UNKNOWN"
	$Vehicle/MissionStatus/Label.text = "MISSION_STATUS: " + ms_text
	if _physics_mode_select != null and Sim.car != null:
		var target_idx: int = 0 if Sim.car.physics_engine_mode == MirenaCar.PhysicsEngineMode.MODULAR_VEHICLE_DYNAMICS else 1
		if _physics_mode_select.selected != target_idx:
			_physics_mode_select.selected = target_idx

func _on_pilot_mode_item_selected(index: int) -> void:
	match index:
		0:
			Sim.car.pilot = MirenaCar.PilotMode.MANUAL
		1:
			Sim.car.pilot = MirenaCar.PilotMode.ROS
		2:
			Sim.car.path = Sim.track.track_path
			Sim.car.pilot = MirenaCar.PilotMode.TRACK_RAIL
		3:
			Sim.car.pilot = MirenaCar.PilotMode.NO_PILOT

	

func _on_reset_car_button_pressed() -> void:
	Sim.car.reset_position()


func _on_as_state_item_selected(index: int) -> void:
	# item ids carry the mirena_common/AsInfo AS_STATUS_* enum values
	Sim.car._as_info.status = $Vehicle/AsStatus/StateSelector/State.get_item_id(index)
	Sim.car._as_info_pub.publish(Sim.car._as_info)


func _on_mission_item_selected(index: int) -> void:
	# item ids carry the mirena_common/AsInfo AS_MISSION_* enum values
	Sim.car._as_info.mission = $Vehicle/AsStatus/MissionSelector/Mission.get_item_id(index)
	Sim.car._as_info_pub.publish(Sim.car._as_info)
