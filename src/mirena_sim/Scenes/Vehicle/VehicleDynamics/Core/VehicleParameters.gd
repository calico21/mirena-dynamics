extends Resource
class_name VehicleParameters

## Source-of-truth physical mass, geometry, and inertia parameters for the vehicle chassis and wheels.
##
## All quantities are expressed in SI units and in the ISO 8855 / ROS 2 Vehicle Body Frame
## (+X forward, +Y left, +Z up) unless explicitly suffixed with `_godot`.
## See Documentation/PARAMETER_PROVENANCE.md for the verified origin of every default value.

@export_group("Mass Properties")
## Total vehicle mass including driver (kg). Verified in vehicleMotion.gd (MASS_KG=300.0) and mirena_car.tscn.
@export var total_mass_kg: float = 300.0
## Unsprung mass per front corner (upright + wheel + tire + brake) (kg). Source: TeR26 CAD spec.
@export var unsprung_mass_front_kg: float = 7.74
## Unsprung mass per rear corner (kg). Source: TeR26 CAD spec.
@export var unsprung_mass_rear_kg: float = 7.76
## Gravitational acceleration (m/s^2). Verified in vehicleMotion.gd (GRAVITY=9.81).
@export var gravity_mps2: float = 9.81

@export_group("Center of Mass & Inertia Tensor")
## Center of mass in Godot chassis-local coordinates (m). Verified in mirena_car.tscn: Vector3(0, 0.33, -0.117).
@export var center_of_mass_godot: Vector3 = Vector3(0.0, 0.33, -0.117)
## Roll moment of inertia Ixx about vehicle +X axis (kg*m^2). Source: TeR26 estimate (45.0 kg*m^2; provisional).
@export var ixx_kgm2: float = 45.0
## Pitch moment of inertia Iyy about vehicle +Y axis (kg*m^2). Source: TeR26 estimate (85.0 kg*m^2; provisional).
@export var iyy_kgm2: float = 85.0
## Yaw moment of inertia Izz about vehicle +Z axis (kg*m^2). Source: dbm_model.py calibrated FS yaw inertia (120.0 kg*m^2).
@export var izz_kgm2: float = 120.0

@export_group("Chassis & Wheelbase Geometry")
## Distance from CoM to front axle along vehicle +X (m).
## In mirena_car.tscn: front wheel z = -0.993, CoM z = -0.117 => lf = 0.876 m (TeR26 CAD is 0.8525 m).
@export var lf_m: float = 0.876
## Distance from CoM to rear axle along vehicle -X (m).
## In mirena_car.tscn: rear wheel z = +0.600, CoM z = -0.117 => lr = 0.717 m (TeR26 CAD is 0.6975 m).
@export var lr_m: float = 0.717
## Front track width (hub to hub) (m). In mirena_car.tscn: 2 * 0.587 = 1.174 m (mirena_state.cpp uses 1.20 m).
@export var track_front_m: float = 1.174
## Rear track width (hub to hub) (m). In mirena_car.tscn: 2 * 0.587 = 1.174 m.
@export var track_rear_m: float = 1.174
## Nominal wheel hub height in Godot chassis-local frame at rest (m). Verified in mirena_car.tscn (y = 0.208).
@export var wheel_hub_height_godot_m: float = 0.208

@export_group("Wheel Physical Properties")
## Unloaded / nominal rolling radius of the tire (m).
## Verified in vehicleMotion.gd (WHEEL_RADIUS = 0.20) and mirena_state.cpp (wheel_radius = 0.20).
## Note: Hoosier 16.0x7.5-10 unloaded radius is 0.2032 m in TeR_Tire/config/tire_specs.yaml.
@export var wheel_unloaded_radius_m: float = 0.20
## Rotational inertia of one wheel-tire-rotor assembly about its spin axis (kg*m^2).
## Provisional value from TeR26/TeR27 specs (0.32 - 1.20 kg*m^2); requires experimental validation.
@export var wheel_inertia_kgm2: float = 0.32

## Return derived sprung mass (kg).
func get_sprung_mass_kg() -> float:
	var unsprung_total: float = 2.0 * (unsprung_mass_front_kg + unsprung_mass_rear_kg)
	return maxf(0.0, total_mass_kg - unsprung_total)

## Return total wheelbase L = lf + lr (m).
func get_wheelbase_m() -> float:
	return lf_m + lr_m

## Return Center of Mass position in ISO/ROS Vehicle Body coordinates (+X forward, +Y left, +Z up).
func get_center_of_mass_vehicle() -> Vector3:
	return VehicleCoordinates.godot_to_vehicle_vec(center_of_mass_godot)

## Return the 3x3 diagonal principal inertia tensor in the Vehicle Body Frame (kg*m^2).
func get_inertia_diagonal_vehicle() -> Vector3:
	return Vector3(ixx_kgm2, iyy_kgm2, izz_kgm2)

## Return the nominal hub position of a corner relative to the CoM in the Vehicle Body Frame (+X fwd, +Y left, +Z up).
func get_wheel_offset_from_com_vehicle(corner: VehicleDynamicsTypes.WheelCorner) -> Vector3:
	var x_rel: float = lf_m if VehicleDynamicsTypes.is_front_corner(corner) else -lr_m
	var half_track: float = 0.5 * (track_front_m if VehicleDynamicsTypes.is_front_corner(corner) else track_rear_m)
	var y_rel: float = half_track if VehicleDynamicsTypes.is_left_corner(corner) else -half_track
	var z_rel: float = wheel_hub_height_godot_m - center_of_mass_godot.y
	return Vector3(x_rel, y_rel, z_rel)

## Return the nominal wheel attachment position in Godot chassis-local coordinates (matching mirena_car.tscn).
func get_wheel_mount_godot(corner: VehicleDynamicsTypes.WheelCorner) -> Vector3:
	var com_veh: Vector3 = get_center_of_mass_vehicle()
	var offset_veh: Vector3 = get_wheel_offset_from_com_vehicle(corner)
	return VehicleCoordinates.vehicle_to_godot_vec(com_veh + offset_veh)

## Validate physical consistency of parameters.
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
