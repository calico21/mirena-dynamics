extends Control

var _node: RosNode
var _image_sub: RosSubscriber
var _control_pub: RosPublisher

const BRAKE_F = 20
const MAX_STEER = deg_to_rad(40)

# Class member variables to persist state between frames
var _steer_smoothed: float = 0.0
var gas: float = 0.0
var steering: float = 0.0
var brake: float = 0.0
# Last camera resolution applied to the window; used to resize only on change.
var _last_res: Vector2i = Vector2i.ZERO
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	_node = RosNode.new()
	_node.init("mirena_remote", "")
	
	var sensor_qos = RosQoS.new()
	sensor_qos.reliability = RosQoS.BEST_EFFORT
	
	_image_sub = _node.create_subscription("/sensors/camera/image", RosSensorMsgsImage, _on_image, sensor_qos)
	_control_pub = _node.create_publisher("/control", RosMirenaCommonCarControl)

func _on_image(msg) -> void:
	if msg.data.is_empty():
		return

	# Pick the Godot format from the ROS encoding so the viewer works with both
	# the real camera (bgr8) and the sim camera (rgba8). Godot has no native BGR
	# format, so bgr(a)8 is uploaded as RGB(A)8 and the change_channels shader
	# swaps R/B on the GPU (see the swap_rb uniform below).
	var fmt: int
	var swap_rb: bool = false
	match msg.encoding:
		"rgb8":
			fmt = Image.FORMAT_RGB8
		"rgba8":
			fmt = Image.FORMAT_RGBA8
		"bgr8":
			fmt = Image.FORMAT_RGB8
			swap_rb = true
		"bgra8":
			fmt = Image.FORMAT_RGBA8
			swap_rb = true
		"mono8":
			fmt = Image.FORMAT_L8
		_:
			push_error("Unsupported camera image encoding: '%s'" % msg.encoding)
			return

	# Guard against a truncated/mismatched buffer instead of letting the engine
	# assert (which then leaves an empty Image and a second error).
	var expected: int = msg.width * msg.height * _bytes_per_pixel(fmt)
	if msg.data.size() != expected:
		push_warning("Camera image size mismatch: got %d, expected %d (%dx%d %s)" %
			[msg.data.size(), expected, msg.width, msg.height, msg.encoding])
		return

	var img: Image = Image.create_from_data(msg.width, msg.height, false, fmt, msg.data)
	if img and not img.is_empty():
		(%CameraFeed.material as ShaderMaterial).set_shader_parameter("swap_rb", swap_rb)
		%CameraFeed.texture = ImageTexture.create_from_image(img)

		# Resize the app window to the camera resolution, but only when it changes
		# so we don't fight manual resizes or thrash the window every frame.
		var res: Vector2i = Vector2i(msg.width, msg.height)
		if res != _last_res and res.x > 0 and res.y > 0:
			_last_res = res
			get_window().size = res


static func _bytes_per_pixel(fmt: int) -> int:
	match fmt:
		Image.FORMAT_L8: return 1
		Image.FORMAT_RGB8: return 3
		Image.FORMAT_RGBA8: return 4
	return 4


func _process(delta: float) -> void:

	# 2. Handle Inputs
	var steer_input = Input.get_action_strength("manual_steer_l") - Input.get_action_strength("manual_steer_r")
	
	# Update our class variable smoothly
	_steer_smoothed = _smooth_steer(_steer_smoothed, steer_input, delta, 2.0)
	
	# Assign values to our local class variables instead of 'self' properties that don't exist
	gas = Input.get_action_strength("manual_gas_pos") - Input.get_action_strength("manual_gas_neg")
	steering = _steer_smoothed * MAX_STEER
	#brake = Input.get_action_strength("EBS") * BRAKE_F
	
	var msg = RosMirenaCommonCarControl.new()
	msg.header.stamp = _node.now()
	msg.gas = gas
	msg.steer_angle = steering
	_control_pub.publish(msg)
	
	%GasLabel.text = "Gas: %6f" % gas
	%SteerLabel.text = "Steer: %6f" % steering

func _smooth_steer(current: float, target: float, delta: float, speed: float) -> float:
	var diff = target - current
	var t = clamp(abs(diff), 0.0, 1.0)
	var _ease = t * t * (3.0 - 2.0 * t) 
	current += sign(diff) * _ease * speed * delta
	if sign(target - current) != sign(diff): 
		current = target
	return clamp(current, -1.0, 1.0)
