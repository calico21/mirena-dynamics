extends RefCounted
class_name Ter26WheelVisualizer
## Synchronizes the 4 VehicleWheel3D nodes ($FL_WHEEL, $FR_WHEEL, $RL_WHEEL, $RR_WHEEL)
## from the C++20 14-DOF multibody engine's 3D suspension kinematics and wheel spin state,
## while neutralizing Godot's built-in raycast vehicle forces so there is only ONE physical authority.

static func neutralize_godot_wheel_physics(wheels: Array[VehicleWheel3D]) -> void:
	for w in wheels:
		if w == null:
			continue
		w.wheel_friction_slip = 0.0
		w.suspension_stiffness = 0.0
		w.damping_compression = 0.0
		w.damping_relaxation = 0.0
		w.engine_force = 0.0
		w.brake = 0.0

static func sync_wheels_from_bridge(bridge: Object, wheels: Array[VehicleWheel3D]) -> void:
	if bridge == null:
		return
	for i in range(min(4, wheels.size())):
		var w: VehicleWheel3D = wheels[i]
		if w != null:
			w.transform = bridge.call("get_wheel_local_transform_godot", i)
