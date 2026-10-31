extends Resource
class_name AerodynamicConfiguration

## Configuration resource for the aerodynamic subsystem (drag, downforce, aero balance, ride-height sensitivity).
##
## Defaults match the verified parameters in `src/mirena_sim/Scenes/Vehicle/vehicleMotion.gd`:
## - RHO_AIR = 1.225 kg/m^3
## - A_ref = 1.10 m^2
## - Cd_ref = 2.50 (CD_A = 2.50 * 1.10 = 2.75 m^2)
## - Cl_ref = 4.14 (CL_A = 4.14 * 1.10 = 4.554 m^2, positive = downward downforce)

@export_group("Reference Aerodynamic Coefficients")
## Ambient air density rho (kg/m^3). Verified in vehicleMotion.gd (RHO_AIR = 1.225).
@export var air_density_kgpm3: float = 1.225
## Frontal reference area A_ref (m^2). Verified in vehicleMotion.gd and ter26.py (1.10 m^2).
@export var frontal_area_m2: float = 1.10
## Reference drag coefficient Cd_ref (dimensionless). Verified in vehicleMotion.gd (2.50).
@export var cd_ref: float = 2.50
## Reference downforce (negative lift) coefficient Cl_ref (dimensionless, positive = downward force).
## Verified in vehicleMotion.gd and ter26.py (4.14).
@export var cl_ref: float = 4.14
## Minimum airspeed (m/s) below which aerodynamic forces are neglected.
## Verified in vehicleMotion.gd line 448 (speed > 0.1).
@export var min_airspeed_mps: float = 0.1

@export_group("Aero Distribution & Ground Effect")
## Fraction of total downforce acting on the front axle [0.0, 1.0].
## Source: ter26.py (aero_split_f = 0.45 -> 45% front, 55% rear).
@export var aero_split_front: float = 0.45
## Enable ride-height and pitch sensitivity on downforce (ground effect).
## Disabled by default for baseline parity with vehicleMotion.gd.
@export var enable_ground_effect: bool = false
## Reference ride height for Cl_ref (m). Source: ter26.py (0.040 m).
@export var h_aero_ref_m: float = 0.040
## Front and rear ground-effect sensitivity coefficients. Source: ter26.py (0.30 front, 0.45 rear).
@export var k_ground_front: float = 0.30
@export var k_ground_rear: float = 0.45

## Return effective drag area Cd * A (m^2).
func get_cd_area_m2() -> float:
	return cd_ref * frontal_area_m2

## Return effective downforce area Cl * A (m^2).
func get_cl_area_m2() -> float:
	return cl_ref * frontal_area_m2

## Validate aerodynamic configuration.
func is_valid() -> bool:
	if air_density_kgpm3 < 0.0 or frontal_area_m2 <= 0.0:
		return false
	if cd_ref < 0.0 or aero_split_front < 0.0 or aero_split_front > 1.0:
		return false
	return true
