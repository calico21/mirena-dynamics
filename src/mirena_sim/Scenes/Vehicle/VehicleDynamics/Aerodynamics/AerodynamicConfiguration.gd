extends Resource
class_name AerodynamicConfiguration

@export_group("Reference Aerodynamic Coefficients")

@export var air_density_kgpm3: float = 1.225

@export var frontal_area_m2: float = 1.10

@export var cd_ref: float = 2.50

@export var cl_ref: float = 4.14

@export var min_airspeed_mps: float = 0.1

@export_group("Aero Distribution & Ground Effect")

@export var aero_split_front: float = 0.45

@export var enable_ground_effect: bool = false

@export var h_aero_ref_m: float = 0.040

@export var k_ground_front: float = 0.30
@export var k_ground_rear: float = 0.45

func get_cd_area_m2() -> float:
	return cd_ref * frontal_area_m2

func get_cl_area_m2() -> float:
	return cl_ref * frontal_area_m2

func is_valid() -> bool:
	if air_density_kgpm3 < 0.0 or frontal_area_m2 <= 0.0:
		return false
	if cd_ref < 0.0 or aero_split_front < 0.0 or aero_split_front > 1.0:
		return false
	return true
