extends RefCounted
class_name ContactState

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

var in_contact: bool = true

var contact_point_world: Vector3 = Vector3.ZERO

var contact_normal_world: Vector3 = Vector3.UP

var contact_normal_vehicle: Vector3 = Vector3(0.0, 0.0, 1.0)

var hub_to_ground_distance_m: float = 0.20

var tire_deflection_m: float = 0.0

var surface_friction_scale: float = 1.0

var surface_velocity_world: Vector3 = Vector3.ZERO

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	reset()

func reset(nominal_radius_m: float = 0.20) -> void:
	in_contact = true
	contact_point_world = Vector3.ZERO
	contact_normal_world = Vector3.UP
	contact_normal_vehicle = Vector3(0.0, 0.0, 1.0)
	hub_to_ground_distance_m = nominal_radius_m
	tire_deflection_m = 0.0
	surface_friction_scale = 1.0
	surface_velocity_world = Vector3.ZERO

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
