extends RefCounted
class_name Ter26VehicleAdapter
## High-level GDScript adapter around the native C++20 `Ter26VehicleBridge` GDExtension.
## Queries ground height and normals under all 4 wheels via Godot PhysicsDirectSpaceState3D
## (Layer 1 = Track/Environment) and steps the 14-DOF multibody engine at 400 Hz.

var native_bridge: Object = null
var is_native_available: bool = false

const NOMINAL_WHEEL_OFFSETS_GODOT: Array[Vector3] = [
	Vector3(-0.587, 0.208, -0.993), # FL
	Vector3( 0.587, 0.208, -0.993), # FR
	Vector3(-0.587, 0.208,  0.600), # RL
	Vector3( 0.587, 0.208,  0.600)  # RR
]

func _init() -> void:
	if ClassDB.class_exists("Ter26VehicleBridge"):
		native_bridge = ClassDB.instantiate("Ter26VehicleBridge")
		is_native_available = (native_bridge != null)

func reset_to_transform(transform_godot: Transform3D) -> void:
	if is_native_available:
		native_bridge.call("reset_to_transform_godot", transform_godot)

func query_ground_planes_and_step(
	vehicle_node: Node3D,
	dt_s: float,
	throttle: float,
	brake: float,
	steering_rad: float,
	ebs_active: bool
) -> Dictionary:
	if not is_native_available:
		return {}

	var ground_pts := PackedFloat64Array()
	var ground_nrms := PackedFloat64Array()
	ground_pts.resize(12)
	ground_nrms.resize(12)

	var space_state: PhysicsDirectSpaceState3D = null
	if vehicle_node.is_inside_tree() and vehicle_node.get_world_3d() != null:
		space_state = vehicle_node.get_world_3d().direct_space_state

	var chassis_tf: Transform3D = native_bridge.call("get_chassis_transform_godot")

	for i in range(4):
		var wc_world: Vector3 = chassis_tf * NOMINAL_WHEEL_OFFSETS_GODOT[i]
		var pt_ground := Vector3(wc_world.x, 0.0, wc_world.z)
		var nrm_ground := Vector3(0.0, 1.0, 0.0)

		if space_state != null:
			var ray_from := wc_world + Vector3(0.0, 0.50, 0.0)
			var ray_to   := wc_world - Vector3(0.0, 1.20, 0.0)
			var query := PhysicsRayQueryParameters3D.create(ray_from, ray_to, 1) # Layer 1 = Track
			if vehicle_node is CollisionObject3D:
				query.exclude = [(vehicle_node as CollisionObject3D).get_rid()]
			var hit: Dictionary = space_state.intersect_ray(query)
			if not hit.is_empty():
				pt_ground = hit["position"]
				nrm_ground = hit["normal"]

		ground_pts[3 * i + 0] = pt_ground.x
		ground_pts[3 * i + 1] = pt_ground.y
		ground_pts[3 * i + 2] = pt_ground.z

		ground_nrms[3 * i + 0] = nrm_ground.x
		ground_nrms[3 * i + 1] = nrm_ground.y
		ground_nrms[3 * i + 2] = nrm_ground.z

	return native_bridge.call(
		"step_godot",
		dt_s,
		throttle,
		brake,
		steering_rad,
		ebs_active,
		ground_pts,
		ground_nrms
	)
