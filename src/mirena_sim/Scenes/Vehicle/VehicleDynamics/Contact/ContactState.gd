extends RefCounted
class_name ContactState

## Mutable per-wheel ground contact geometry and surface state.
##
## Coordinate frames:
## - `contact_point_world` and `contact_normal_world` are in the Godot World Frame (Y-up).
## - `contact_normal_vehicle` is in the ISO 8855 / ROS 2 Vehicle Body Frame (+Z up).

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL
## True when the tire patch is in contact with the ground surface.
var in_contact: bool = true
## Contact patch location in Godot world coordinates (m).
var contact_point_world: Vector3 = Vector3.ZERO
## Unit outward surface normal in Godot world coordinates (default +Y up).
var contact_normal_world: Vector3 = Vector3.UP
## Unit outward surface normal in ISO 8855 / ROS 2 Vehicle Body Frame (default +Z up).
var contact_normal_vehicle: Vector3 = Vector3(0.0, 0.0, 1.0)
## Vertical distance from wheel hub to ground contact along suspension axis (m).
var hub_to_ground_distance_m: float = 0.20
## Tire radial compression / deflection (m), >= 0 when in contact.
var tire_deflection_m: float = 0.0
## Local surface friction scaling multiplier (1.0 = nominal dry asphalt).
var surface_friction_scale: float = 1.0
## Velocity of the ground surface at the contact point in Godot world coordinates (m/s).
var surface_velocity_world: Vector3 = Vector3.ZERO

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	reset()

## Reset contact state to nominal level ground contact.
func reset(nominal_radius_m: float = 0.20) -> void:
	in_contact = true
	contact_point_world = Vector3.ZERO
	contact_normal_world = Vector3.UP
	contact_normal_vehicle = Vector3(0.0, 0.0, 1.0)
	hub_to_ground_distance_m = nominal_radius_m
	tire_deflection_m = 0.0
	surface_friction_scale = 1.0
	surface_velocity_world = Vector3.ZERO

## Verify all contact quantities are finite.
func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_vec3(contact_point_world)
		and VehicleUnits.is_finite_vec3(contact_normal_world)
		and VehicleUnits.is_finite_vec3(contact_normal_vehicle)
		and VehicleUnits.is_finite_scalar(hub_to_ground_distance_m)
		and VehicleUnits.is_finite_scalar(tire_deflection_m)
		and VehicleUnits.is_finite_scalar(surface_friction_scale)
		and surface_friction_scale >= 0.0
	)
