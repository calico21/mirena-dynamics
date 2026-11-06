extends Node3D
class_name Ter26SuspensionVisualizer
## Optional 3D visualizer for TeR26 wishbones, pushrods/pullrods, tie-rods, and rockers.

@export var enabled: bool = false
var last_diagnostics: Dictionary = {}

func update_from_diagnostics(diag: Dictionary) -> void:
	last_diagnostics = diag
