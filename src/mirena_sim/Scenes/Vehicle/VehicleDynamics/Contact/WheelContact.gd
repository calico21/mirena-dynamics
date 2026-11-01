extends RefCounted
class_name WheelContact

func evaluate_wheel_contact(
	wheel: WheelState,
	chassis_global_tf_godot: Transform3D,
	chassis_lin_vel_world_godot: Vector3,
	chassis_ang_vel_world_godot: Vector3,
	params: VehicleParameters,
	susp_cfg: SuspensionConfiguration,
	contact_query: ContactQuery
) -> void:
	if wheel == null or params == null or susp_cfg == null or contact_query == null:
		return

	var corner: VehicleDynamicsTypes.WheelCorner = wheel.corner
	var hub_mount_local_godot: Vector3 = params.get_wheel_mount_godot(corner)
	var hub_world_godot: Vector3 = chassis_global_tf_godot * hub_mount_local_godot
	var down_world_godot: Vector3 = -chassis_global_tf_godot.basis.y.normalized()

	var unloaded_r: float = params.wheel_unloaded_radius_m
	var max_droop: float = susp_cfg.max_droop_travel_m
	var max_ray_len: float = unloaded_r + max_droop

	var res: Dictionary = contact_query.query_ground(hub_world_godot, down_world_godot, max_ray_len)
	var hit: bool = res["hit"]
	var dist_m: float = res["distance_m"]

	wheel.contact_state.in_contact = hit
	wheel.contact_state.contact_point_world = res["point_world"]
	wheel.contact_state.contact_normal_world = res["normal_world"]
	wheel.contact_state.contact_normal_vehicle = VehicleCoordinates.world_godot_to_vehicle_body(
		res["normal_world"], chassis_global_tf_godot.basis
	)
	wheel.contact_state.hub_to_ground_distance_m = dist_m
	wheel.contact_state.surface_friction_scale = res["friction_scale"]

	if hit:

		var compression: float = clampf(
			unloaded_r - dist_m,
			-susp_cfg.max_droop_travel_m,
			susp_cfg.max_compression_travel_m
		)
		wheel.suspension_state.compression_m = compression

		var arm_world: Vector3 = hub_world_godot - chassis_global_tf_godot.origin
		var hub_vel_world: Vector3 = chassis_lin_vel_world_godot + chassis_ang_vel_world_godot.cross(arm_world)
		wheel.suspension_state.compression_velocity_mps = hub_vel_world.dot(down_world_godot)
		wheel.effective_radius_m = maxf(0.05, unloaded_r)
	else:
		wheel.suspension_state.compression_m = -susp_cfg.max_droop_travel_m
		wheel.suspension_state.compression_velocity_mps = 0.0
		wheel.effective_radius_m = unloaded_r
