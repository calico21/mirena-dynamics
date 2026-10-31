extends RefCounted
class_name GodotPhysicsAdapter

## Designated single-point physics integration adapter between `VehicleDynamicsModel` and Godot's 3D physics server.
##
## Architectural Guarantee Against Force Double-Counting:
## 1. When `integration_mode == INACTIVE_LEGACY_PASSTHROUGH` (Phase 1 default):
##    This adapter applies ZERO forces and ZERO torques to the chassis node. The legacy `VehicleBody3D`
##    (`vehicleMotion.gd`) owns 100% of vehicle physics.
## 2. When `integration_mode == CUSTOM_RIGID_BODY_3D` (Phase 2+ replacement):
##    - If the host body is a `VehicleBody3D`, all child `VehicleWheel3D` nodes have their `engine_force`
##      and `brake` zeroed and their built-in suspension/friction disabled, OR the root node is converted
##      to a pure `RigidBody3D` (recommended in `Documentation/INTEGRATION_PLAN.md`).
##    - Exactly ONE call to `body.apply_central_force(state.total_force_world_godot)` and
##      `body.apply_torque(state.total_moment_world_godot)` applies the sum of tire, suspension, and
##      aerodynamic forces/moments about the Center of Mass.

## Configure a Godot `RigidBody3D` mass, CoM, and inertia properties from `VehicleParameters`.
static func configure_rigid_body(body: RigidBody3D, params: VehicleParameters) -> void:
	if body == null or params == null or not params.is_valid():
		return
	body.mass = params.total_mass_kg
	body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	body.center_of_mass = params.center_of_mass_godot
	body.inertia = VehicleInertia.get_inertia_godot(params)

## Read current world transform and velocities from a Godot `RigidBody3D` into `VehicleState`,
## and update `ContactQuery` with the body's `PhysicsDirectSpaceState3D`.
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

## Apply the assembled net forces and moments from `VehicleState` to `body` ONLY when
## `integration_mode == CUSTOM_RIGID_BODY_3D`.
## Returns true if forces were applied, or false if blocked by `integration_mode` or fault flags.
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
