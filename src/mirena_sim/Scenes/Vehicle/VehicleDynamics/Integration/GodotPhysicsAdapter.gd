extends RefCounted
class_name GodotPhysicsAdapter

static func configure_rigid_body(body: RigidBody3D, params: VehicleParameters) -> void:
	if body == null or params == null or not params.is_valid():
		return
	body.mass = params.total_mass_kg
	body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	body.center_of_mass = params.center_of_mass_godot
	body.inertia = VehicleInertia.get_inertia_godot(params)

static func sync_state_from_body(
	body: RigidBody3D,
	state: VehicleState,
	contact_query: ContactQuery = null
) -> void:
	if body == null or state == null:
		return
	state.global_transform_godot = body.global_transform
	state.linear_velocity_world_godot = body.linear_velocity
	state.angular_velocity_world_godot = body.angular_velocity

	if contact_query != null and not contact_query.use_analytic_plane and Engine.is_in_physics_frame() and body.is_inside_tree() and body.get_world_3d() != null:
		contact_query.physics_space_state = body.get_world_3d().direct_space_state
		contact_query.exclude_rids = [body.get_rid()]

static func apply_forces_to_body(
	body: RigidBody3D,
	state: VehicleState,
	integration_mode: VehicleDynamicsTypes.IntegrationMode
) -> bool:
	if integration_mode != VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D:
		return false
	if body == null or state == null:
		return false
	if not VehicleUnits.is_finite_vec3(state.total_force_world_godot) \
			or not VehicleUnits.is_finite_vec3(state.total_moment_world_godot):
		return false

	body.apply_central_force(state.total_force_world_godot)
	body.apply_torque(state.total_moment_world_godot)
	return true
