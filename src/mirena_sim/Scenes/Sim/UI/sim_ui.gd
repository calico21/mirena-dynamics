extends Control
class_name MirenaHud

const FloatingGraphManager = preload("res://Scenes/Sim/UI/FloatingGraphManager.gd")

@onready var fps_label: Label = find_child("FpsLabel", true, false) as Label
@onready var timer_label: Label = find_child("TimerLabel", true, false) as Label
@onready var fallen_cones_label: Label = find_child("FallenConesLabel", true, false) as Label
@onready var correlation_label: Label = find_child("CorrelationLabel", true, false) as Label
@onready var main_menu: Control = find_child("MainMenu", true, false) as Control
@onready var app_launcher: Node = find_child("AppLauncher", true, false)
@onready var floating_graphs: FloatingGraphManager = find_child("FloatingGraphManager", true, false) as FloatingGraphManager

func _input(event):
	# Toggle Options Menu visibility while keeping top-right Stats HUD (FPS, Time, Cones, CORR) always visible
	if event.is_action_pressed("open_sim_menu") and main_menu != null:
		main_menu.visible = !main_menu.visible
