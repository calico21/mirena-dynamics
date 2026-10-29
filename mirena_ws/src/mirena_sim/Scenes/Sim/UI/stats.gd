extends PanelContainer

@export var car: MirenaCar

@onready var fps_label: Label = $MarginContainer/VBoxContainer/FpsLabel
@onready var timer_label: Label = $MarginContainer/VBoxContainer/TimerLabel
@onready var fallen_cones_label: Label = $MarginContainer/VBoxContainer/FallenConesLabel

var elapsed_time: float = 0.0

func _ready() -> void:
	# 1. Set default text immediately
	fallen_cones_label.text = "Fallen Cones: 0"
	timer_label.text = "Time: 0.00"

	# 2. Auto-find the car if it wasn't assigned manually in the Inspector
	if not car:
		car = get_tree().get_first_node_in_group("Car") as MirenaCar

	# 3. Connect signal
	if car:
		if not car.cone_hit.is_connected(_on_cone_hit):
			car.cone_hit.connect(_on_cone_hit)
	else:
		push_warning("MirenaHud: No MirenaCar found in scene!")

func _process(delta: float) -> void:
	# Update FPS
	fps_label.text = "FPS: %d" % Engine.get_frames_per_second()
	
	# Update Timer
	elapsed_time += delta
	timer_label.text = "Time: %.2f" % elapsed_time

func _on_cone_hit(count: int) -> void:
	# Update Fallen Cones when a collision occurs
	fallen_cones_label.text = "Fallen Cones: %d" % count
