extends Resource
class_name VehicleParameters

@export_group("Mass Properties")

@export var total_mass_kg: float = 300.0

@export var unsprung_mass_front_kg: float = 7.74

@export var unsprung_mass_rear_kg: float = 7.76

@export var gravity_mps2: float = 9.81

@export_group("Center of Mass & Inertia Tensor")

@export var center_of_mass_godot: Vector3 = Vector3(0.0, 0.33, -0.117)

@export var ixx_kgm2: float = 45.0

@export var iyy_kgm2: float = 85.0

@export var izz_kgm2: float = 120.0

@export_group("Chassis & Wheelbase Geometry")

@export var lf_m: float = 0.876

@export var lr_m: float = 0.717

@export var track_front_m: float = 1.174

@export var track_rear_m: float = 1.174

@export var wheel_hub_height_godot_m: float = 0.208

@export_group("Wheel Physical Properties")

@export var wheel_unloaded_radius_m: float = 0.20

@export var wheel_inertia_kgm2: float = 0.32

func get_sprung_mass_kg() -> float:
	var unsprung_total: float = 2.0 * (unsprung_mass_front_kg + unsprung_mass_rear_kg)
	return maxf(0.0, total_mass_kg - unsprung_total)

func get_wheelbase_m() -> float:
	return lf_m + lr_m

func get_center_of_mass_vehicle() -> Vector3:
	return VehicleCoordinates.godot_to_vehicle_vec(center_of_mass_godot)

func get_inertia_diagonal_vehicle() -> Vector3:
	return Vector3(ixx_kgm2, iyy_kgm2, izz_kgm2)

func get_wheel_offset_from_com_vehicle(corner: VehicleDynamicsTypes.WheelCorner) -> Vector3:
	var x_rel: float = lf_m if VehicleDynamicsTypes.is_front_corner(corner) else -lr_m
	var half_track: float = 0.5 * (track_front_m if VehicleDynamicsTypes.is_front_corner(corner) else track_rear_m)
	var y_rel: float = half_track if VehicleDynamicsTypes.is_left_corner(corner) else -half_track
	var z_rel: float = wheel_hub_height_godot_m - center_of_mass_godot.y
	return Vector3(x_rel, y_rel, z_rel)

func get_wheel_mount_godot(corner: VehicleDynamicsTypes.WheelCorner) -> Vector3:
	var com_veh: Vector3 = get_center_of_mass_vehicle()
	var offset_veh: Vector3 = get_wheel_offset_from_com_vehicle(corner)
	return VehicleCoordinates.vehicle_to_godot_vec(com_veh + offset_veh)

func is_valid() -> bool:
	if not VehicleUnits.is_finite_scalar(total_mass_kg) or total_mass_kg <= 0.0:
		return false
	if not VehicleUnits.is_finite_scalar(ixx_kgm2) or ixx_kgm2 <= 0.0:
		return false
	if not VehicleUnits.is_finite_scalar(iyy_kgm2) or iyy_kgm2 <= 0.0:
		return false
	if not VehicleUnits.is_finite_scalar(izz_kgm2) or izz_kgm2 <= 0.0:
		return false
	if lf_m <= 0.0 or lr_m <= 0.0 or track_front_m <= 0.0 or track_rear_m <= 0.0:
		return false
	if wheel_unloaded_radius_m <= 0.0 or wheel_inertia_kgm2 <= 0.0:
		return false
	return true
