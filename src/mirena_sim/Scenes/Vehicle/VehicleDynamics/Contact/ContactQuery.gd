extends RefCounted
class_name ContactQuery

## Pluggable ground-surface query interface for wheel-ground contact evaluation.
##
## Decouples the physics model from Godot's 3D scene tree so that:
## - Unit and integration tests can supply an analytic flat or sloped plane (`use_analytic_plane = true`)
##   without instantiating a 3D physics world.
## - Scene execution can supply a `PhysicsDirectSpaceState3D` raycast query via `GodotPhysicsAdapter`.

## When true, queries evaluate against an analytic plane defined by `plane_origin_world` and `plane_normal_world`.
var use_analytic_plane: bool = true
var plane_origin_world: Vector3 = Vector3.ZERO
var plane_normal_world: Vector3 = Vector3.UP
var default_friction_scale: float = 1.0

## Optional Godot 3D physics space state for raycasting in live scenes.
var physics_space_state: PhysicsDirectSpaceState3D = null
## Collision mask for ground raycasts when `use_analytic_plane` is false.
var collision_mask: int = 1
## Excluded RIDs (e.g., the vehicle's own RigidBody3D) for scene raycasts.
var exclude_rids: Array[RID] = []

## Query ground height and normal beneath `ray_origin_world` along `ray_dir_world` (unit vector)
## up to `max_distance_m`.
## Returns a Dictionary:
##   {
##     "hit": bool,
##     "point_world": Vector3,
##     "normal_world": Vector3,
##     "distance_m": float,
##     "friction_scale": float,
##   }
func query_ground(ray_origin_world: Vector3, ray_dir_world: Vector3, max_distance_m: float) -> Dictionary:
	if not use_analytic_plane and physics_space_state != null:
		var query := PhysicsRayQueryParameters3D.create(
			ray_origin_world,
			ray_origin_world + ray_dir_world * max_distance_m,
			collision_mask,
			exclude_rids
		)
		var hit_dict: Dictionary = physics_space_state.intersect_ray(query)
		if not hit_dict.is_empty():
			var hit_pos: Vector3 = hit_dict["position"]
			var hit_norm: Vector3 = (hit_dict["normal"] as Vector3).normalized()
			var dist: float = ray_origin_world.distance_to(hit_pos)
			return {
				"hit": true,
				"point_world": hit_pos,
				"normal_world": hit_norm,
				"distance_m": dist,
				"friction_scale": default_friction_scale,
			}
		return {
			"hit": false,
			"point_world": ray_origin_world + ray_dir_world * max_distance_m,
			"normal_world": Vector3.UP,
			"distance_m": max_distance_m,
			"friction_scale": default_friction_scale,
		}

	# Analytic plane intersection: (ray_origin + t * ray_dir - plane_origin) . plane_normal = 0
	var n: Vector3 = plane_normal_world.normalized()
	var denom: float = ray_dir_world.dot(n)
	if absf(denom) < VehicleUnits.EPSILON_STRICT:
		return {
			"hit": false,
			"point_world": ray_origin_world + ray_dir_world * max_distance_m,
			"normal_world": n,
			"distance_m": max_distance_m,
			"friction_scale": default_friction_scale,
		}
	var t: float = (plane_origin_world - ray_origin_world).dot(n) / denom
	if t >= 0.0 and t <= max_distance_m:
		return {
			"hit": true,
			"point_world": ray_origin_world + ray_dir_world * t,
			"normal_world": n,
			"distance_m": t,
			"friction_scale": default_friction_scale,
		}
	return {
		"hit": false,
		"point_world": ray_origin_world + ray_dir_world * max_distance_m,
		"normal_world": n,
		"distance_m": max_distance_m,
		"friction_scale": default_friction_scale,
	}
