extends Node3D
class_name RosTransformManager

## Central static-transform manager for the simulated vehicle — the sim
## counterpart of mirena_hardware's `mirena_transform` node. It collects every
## sensor mount from the scene tree (the sim's source of truth, where
## transforms.yaml is the real car's) and latches them on /tf_static from a
## single ROS node, so both vehicle providers expose the same TF contract and
## are swappable packages.
##
## Sensors declare their frames by implementing:
##   get_static_transforms() -> Array[Dictionary]
## returning entries of the form:
##   {"transform": Transform3D, "frame_id": String, "parent_frame_id": String}
## Sensors must NOT broadcast TF themselves — one owner per tree layer.

@export_group("ROS 2 Settings")
@export var ros_namespace: String = ""
## Root of the subtree scanned for sensors. Defaults to the parent (the vehicle body).
@export var vehicle_root: NodePath

var _node: RosNode
var _tf_broadcaster: RosTfBroadcaster

func _ready() -> void:
	_node = RosNode.new()
	_node.init(name.to_snake_case(), ros_namespace.to_snake_case())
	_tf_broadcaster = _node.create_tf_broadcaster()
	# Defer so every sensor's _ready has run before their mounts are collected.
	_broadcast_all.call_deferred()

func _broadcast_all() -> void:
	var root: Node = get_node(vehicle_root) if not vehicle_root.is_empty() else get_parent()
	if not root:
		push_error("RosTransformManager: no vehicle root to scan")
		return

	var count := 0
	for spec in _collect_specs(root):
		_tf_broadcaster.send_transform(
			spec["transform"], spec["frame_id"], spec["parent_frame_id"], true)
		count += 1
	print("RosTransformManager: latched %d static transforms" % count)

func _collect_specs(node: Node) -> Array:
	var specs: Array = []
	if node != self and node.has_method("get_static_transforms"):
		specs.append_array(node.get_static_transforms())
	for child in node.get_children():
		specs.append_array(_collect_specs(child))
	return specs
