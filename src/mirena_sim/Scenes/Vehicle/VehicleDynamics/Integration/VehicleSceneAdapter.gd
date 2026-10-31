extends RefCounted
class_name VehicleSceneAdapter

## Bridges the Godot vehicle scene hierarchy (`mirena_car.tscn`) and simulator interactions
## (wheel visual meshes, cameras, INS sensor reset, cone collision layers, and lap gates)
## without coupling physical subsystem modules to scene-tree paths.
##
## Preserves all external MirenaSim scene contracts:
## - `"vehicle"` group membership for `Gate` (`gate.gd`) detection.
## - Cone collision layer/mask bit 2 (`cone_collision_set`).
## - Camera registration with `Sim.register_camera` (`$TPCam`, `$FPCam`).
## - `$INS.reset()` invocation on vehicle respawn.
## - Visual wheel mesh updates (steering yaw, suspension vertical jounce, and rolling spin).

var chassis_body: RigidBody3D = null
var wheel_visual_nodes: Array[Node3D] = [null, null, null, null]
var rear_smoke_nodes: Array[CPUParticles3D] = [null, null]
var ins_node: Node = null
var ros_camera_node: Node = null
var tp_cam: Camera3D = null
var fp_cam: Camera3D = null

## Bind scene child references explicitly from the vehicle root node.
func bind_scene_nodes(vehicle_root: RigidBody3D, explicit_wheels: Array[Node3D] = []) -> void:
	chassis_body = vehicle_root
	if vehicle_root == null:
		return

	if explicit_wheels.size() == VehicleDynamicsTypes.WHEEL_COUNT:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			wheel_visual_nodes[i] = explicit_wheels[i]
	else:
		var holder: Node = vehicle_root.get_node_or_null("ModularWheelHolder")
		var names := ["FL_WHEEL", "FR_WHEEL", "RL_WHEEL", "RR_WHEEL"]
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			var wnode: Node3D = vehicle_root.get_node_or_null(names[i]) as Node3D
			if wnode == null and holder != null:
				wnode = holder.get_node_or_null(names[i]) as Node3D
			wheel_visual_nodes[i] = wnode

	ins_node = vehicle_root.get_node_or_null("INS")
	ros_camera_node = vehicle_root.get_node_or_null("Camera")
	tp_cam = vehicle_root.get_node_or_null("TPCam") as Camera3D
	fp_cam = vehicle_root.get_node_or_null("FPCam") as Camera3D

	# Setup rear wheel smoke emitters for wheel-spin-in-air visual effects
	var smoke_names := ["RL_Smoke", "RR_Smoke"]
	for s_idx in range(2):
		var sname: String = smoke_names[s_idx]
		var existing: CPUParticles3D = vehicle_root.get_node_or_null(sname) as CPUParticles3D
		if existing == null:
			existing = _create_smoke_emitter(sname)
			vehicle_root.add_child(existing)
		rear_smoke_nodes[s_idx] = existing

## Helper to create and configure a CPUParticles3D node for rear wheel smoke.
func _create_smoke_emitter(emitter_name: String) -> CPUParticles3D:
	var smoke := CPUParticles3D.new()
	smoke.name = emitter_name
	smoke.emitting = false
	smoke.amount = 32
	smoke.lifetime = 0.75
	smoke.one_shot = false
	smoke.explosiveness = 0.05
	smoke.randomness = 0.35
	smoke.local_coords = false # Particles stay in world space drifting behind vehicle
	smoke.direction = Vector3(0.0, 0.5, 1.0) # Upwards and rearwards in Godot car frame (+Z is backward)
	smoke.spread = 30.0
	smoke.gravity = Vector3(0.0, 1.0, 0.0) # Subtle buoyant rise
	smoke.initial_velocity_min = 1.0
	smoke.initial_velocity_max = 3.5
	smoke.damping_min = 0.5
	smoke.damping_max = 1.2
	smoke.scale_amount_min = 0.8
	smoke.scale_amount_max = 2.5
	smoke.color = Color(0.88, 0.88, 0.92, 0.35)

	var mesh := SphereMesh.new()
	mesh.radius = 0.14
	mesh.height = 0.28
	mesh.radial_segments = 6
	mesh.rings = 4
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.albedo_color = Color(0.88, 0.88, 0.92, 0.30)
	mesh.material = mat
	smoke.mesh = mesh
	return smoke

## Enable or disable cone physical collisions (layer/mask bit 2), matching `vehicleMotion.gd`.
func set_cone_collision_enabled(enable: bool) -> void:
	if chassis_body == null:
		return
	chassis_body.collision_layer = (chassis_body.collision_layer & ~2) | (2 * int(enable))
	chassis_body.collision_mask = (chassis_body.collision_mask & ~2) | (2 * int(enable))

## Notify the onboard simulated INS (`ros_ins.gd`) to wipe its finite-difference velocity and error states on respawn.
func reset_sensors() -> void:
	if ins_node != null and ins_node.has_method("reset"):
		ins_node.reset()
	for smoke in rear_smoke_nodes:
		if smoke != null and is_instance_valid(smoke) and smoke.is_inside_tree():
			smoke.emitting = false

## Update visual wheel node transforms from `VehicleState` and `VehicleParameters` when running in custom rigid-body mode.
func update_wheel_visuals(state: VehicleState, params: VehicleParameters) -> void:
	if state == null or params == null:
		return
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var node: Node3D = wheel_visual_nodes[i]
		if node != null:
			if not (node is VehicleWheel3D and node.get_parent() == chassis_body):
				var w: WheelState = state.wheels[i]
				var mount_pos: Vector3 = params.get_wheel_mount_godot(w.corner)
				mount_pos.y += w.suspension_state.compression_m
				node.position = mount_pos
				# Preserve the 180-deg Y rest orientation used by mirena_car.tscn's VehicleWheel3D meshes
				var rest_basis := Basis(Vector3.UP, PI) if (node is VehicleWheel3D) else Basis.IDENTITY
				# Rotate about +Y_godot for steering (+steer = left) and about -X_godot for forward spin
				var steer_basis := Basis(Vector3.UP, w.steering_angle_rad)
				var spin_basis := Basis(Vector3.LEFT, w.spin_angle_rad)
				node.basis = steer_basis * spin_basis * rest_basis

		# Rear wheel smoke effect for RL (corner index 2) and RR (corner index 3)
		if i == VehicleDynamicsTypes.WheelCorner.RL or i == VehicleDynamicsTypes.WheelCorner.RR:
			var smoke_idx: int = 0 if i == VehicleDynamicsTypes.WheelCorner.RL else 1
			var smoke: CPUParticles3D = rear_smoke_nodes[smoke_idx]
			if smoke != null and is_instance_valid(smoke):
				var w: WheelState = state.wheels[i]
				var mount_pos: Vector3 = params.get_wheel_mount_godot(w.corner)
				mount_pos.y += w.suspension_state.compression_m
				smoke.position = mount_pos

				# Trigger smoke when wheel is in the air / unweighted AND spinning past maximum slip ratio
				var is_in_air: bool = (not w.contact_state.in_contact) or (w.suspension_state.normal_load_fz_n < 50.0)
				var is_spinning_excess: bool = (
					absf(w.slip_ratio_kappa) > 0.20 or
					(absf(w.omega_rad_s) * params.wheel_unloaded_radius_m - maxf(state.u_mps, 0.0) > 1.5 and absf(w.omega_rad_s) > 15.0)
				)
				if smoke.is_inside_tree():
					smoke.emitting = is_in_air and is_spinning_excess
