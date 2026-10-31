extends Camera3D

## Camera with flying script attached to it.
class_name Freecam3D

## Customize your own toggle key to avoid collisions with your current mappings.
@export var toggle_key: Key = KEY_TAB
## Speed up / down by scrolling the mouse whell down / up
@export var invert_speed_controls: bool = false

@export var overlay_text: bool = true

## Pivot node for camera looking around
@onready var pivot := Node3D.new()


const MAX_SPEED := 4
const MIN_SPEED := 0.1
const ACCELERATION := 0.1
const MOUSE_SENSITIVITY := 0.002

## The current maximum speed. Lower or higher it by scrolling the mouse wheel.
var target_speed := MIN_SPEED
## Movement velocity.
var velocity := Vector3.ZERO


## Sets up pivot and UI overlay elements.
func _setup_nodes() -> void:
	self.add_sibling(pivot)
	pivot.position = position
	pivot.rotation = rotation
	pivot.name = "FreecamPivot"
	self.reparent(pivot)
	self.position = Vector3.ZERO
	self.rotation = Vector3.ZERO
	


func _ready() -> void:
	_setup_nodes.call_deferred()
	_add_keybindings()
	Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
	Sim.register_camera.call_deferred(self)



func _process(delta: float) -> void:
	if current:
		var dir = Vector3.ZERO
		if Input.is_action_pressed("__debug_camera_forward"): 	dir.z -= 1
		if Input.is_action_pressed("__debug_camera_back"): 		dir.z += 1
		if Input.is_action_pressed("__debug_camera_left"): 		dir.x -= 1
		if Input.is_action_pressed("__debug_camera_right"): 	dir.x += 1
		if Input.is_action_pressed("__debug_camera_up"): 		dir.y += 1
		if Input.is_action_pressed("__debug_camera_down"): 		dir.y -= 1
		
		dir = dir.normalized()
		dir = dir.rotated(Vector3.UP, pivot.rotation.y)
		
		velocity = lerp(velocity, dir * target_speed, ACCELERATION)
		pivot.position += velocity


func _input(event: InputEvent) -> void:
	if not current:
		return

	# Toggle mouse capture with TAB
	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == toggle_key:
			var captured = Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED
			Input.set_mouse_mode(
				Input.MOUSE_MODE_VISIBLE if captured
				else Input.MOUSE_MODE_CAPTURED
			)
			get_viewport().set_input_as_handled()
			return

		# Always release mouse with ESC
		if event.keycode == KEY_ESCAPE:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			get_viewport().set_input_as_handled()
			return

	# Rotate camera only while mouse is captured
	if event is InputEventMouseMotion:
		if Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED:
			pivot.rotate_y(-event.relative.x * MOUSE_SENSITIVITY)
			rotate_x(-event.relative.y * MOUSE_SENSITIVITY)
			rotation.x = clamp(rotation.x, -PI/2, PI/2)

	var speed_up = func():
		target_speed = clamp(target_speed + 0.15, MIN_SPEED, MAX_SPEED)

	var slow_down = func():
		target_speed = clamp(target_speed - 0.15, MIN_SPEED, MAX_SPEED)

	# Speed control with mouse wheel
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			slow_down.call() if invert_speed_controls else speed_up.call()

		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			speed_up.call() if invert_speed_controls else slow_down.call()





func _make_label(text: String) -> Label:
	var l = Label.new()
	l.text = text
	return l


func _add_keybindings() -> void:
	var actions = InputMap.get_actions()
	if "__debug_camera_forward" not in actions: _add_key_input_action("__debug_camera_forward", KEY_W)
	if "__debug_camera_back" 	not in actions: _add_key_input_action("__debug_camera_back", KEY_S)
	if "__debug_camera_left" 	not in actions: _add_key_input_action("__debug_camera_left", KEY_A)
	if "__debug_camera_right" 	not in actions: _add_key_input_action("__debug_camera_right", KEY_D)
	if "__debug_camera_up" 		not in actions: _add_key_input_action("__debug_camera_up", KEY_SPACE)
	if "__debug_camera_down" 	not in actions: _add_key_input_action("__debug_camera_down", KEY_SHIFT)
	if "__debug_camera_toggle" 	not in actions: _add_key_input_action("__debug_camera_toggle", toggle_key)


func _add_key_input_action(name: String, key: Key) -> void:
	var ev = InputEventKey.new()
	ev.physical_keycode = key
	
	InputMap.add_action(name)
	InputMap.action_add_event(name, ev)
