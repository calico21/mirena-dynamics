extends RefCounted
class_name Ter26VehicleCollisionBridge
## Bridges discrete external collisions in Godot 4.7 (cones on Layer 4, barriers/walls on Layer 1)
## with the C++20 14-DOF multibody impulse-momentum solver:
##   Delta_v = M(q)^-1 * J_c(q)^T * Impulse
## and applies the equal-and-opposite impulse to the Godot RigidBody3D collider.

const CONE_LIFT_IMPULSE_NS := 0.35

static func handle_body_collision(
	vehicle_body: RigidBody3D,
	bridge: Object,
	other_body: Node
) -> bool:
	if bridge == null or other_body == null:
		return false

	# Ground/floor surfaces are handled by the 4 HPITM wheel contacts.
	if other_body is StaticBody3D and (
		other_body.name.contains("Track") or
		other_body.name.contains("Ground") or
		other_body.name.contains("Floor")
	):
		return false

	var is_cone: bool = other_body.is_in_group("Cones") or other_body.is_in_group("cone")
	var other_pos: Vector3 = vehicle_body.global_position
	var other_vel: Vector3 = Vector3.ZERO
	var other_mass: float = 0.20 if is_cone else 1.0e6
	var restitution: float = 0.30 if is_cone else 0.15

	if other_body is Node3D:
		other_pos = (other_body as Node3D).global_position
	if other_body is RigidBody3D:
		var rb := other_body as RigidBody3D
		other_vel = rb.linear_velocity
		other_mass = maxf(0.05, rb.mass)

	var normal_godot: Vector3 = vehicle_body.global_position - other_pos
	normal_godot.y = 0.0
	if normal_godot.length_squared() < 1e-6:
		normal_godot = Vector3(0.0, 0.0, 1.0)
	else:
		normal_godot = normal_godot.normalized()

	var impulse_on_other: Vector3 = bridge.call(
		"apply_collision_impulse_godot",
		other_pos, normal_godot, other_vel, other_mass, restitution
	)

	# Single application, by absolute velocity assignment. apply_central_impulse
	# updates only the physics server; the cached linear_velocity getter stays
	# stale until the next sync, so the post-collision state would not be
	# observable in the same frame. The setter updates both.
	if other_body is RigidBody3D and impulse_on_other.length_squared() > 1e-8:
		var rb2 := other_body as RigidBody3D
		var total_imp := impulse_on_other + Vector3(0.0, CONE_LIFT_IMPULSE_NS, 0.0)
		rb2.linear_velocity += total_imp / maxf(0.05, rb2.mass)

	return is_cone