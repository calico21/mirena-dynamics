extends RefCounted
class_name TireForces

## Output force and moment contract produced by `TireModelInterface.evaluate_forces()`.
##
## Coordinate Frame (ISO 8855 Steered Wheel / Contact Patch Frame):
## - `fx_n`: Longitudinal tire force (N). Positive (+X_wheel) accelerates the vehicle forward.
## - `fy_n`: Lateral tire force (N). Positive (+Y_wheel) pushes the wheel/vehicle to the left.
## - `fz_n`: Vertical normal load at the contact patch (N, >= 0). Positive (+Z_wheel) pushes upward.
## - `mx_nm`: Overturning moment about +X_wheel (N*m).
## - `my_nm`: Rolling resistance moment about +Y_wheel (N*m).
## - `mz_nm`: Self-aligning moment about +Z_wheel (N*m).

var fx_n: float = 0.0
var fy_n: float = 0.0
var fz_n: float = 0.0
var mx_nm: float = 0.0
var my_nm: float = 0.0
var mz_nm: float = 0.0
## True if forces were computed from valid inputs and are finite.
var valid: bool = true
## Bitmask of `VehicleDynamicsTypes.FaultFlag` encountered during evaluation.
var fault_flags: int = VehicleDynamicsTypes.FaultFlag.NONE

## Reset all forces, moments, and fault flags to zero.
func zero() -> void:
	fx_n = 0.0
	fy_n = 0.0
	fz_n = 0.0
	mx_nm = 0.0
	my_nm = 0.0
	mz_nm = 0.0
	valid = true
	fault_flags = VehicleDynamicsTypes.FaultFlag.NONE

## Return the translational force vector (Fx, Fy, Fz) in the Steered Wheel Frame (N).
func get_force_vector_wheel() -> Vector3:
	return Vector3(fx_n, fy_n, fz_n)

## Return the moment vector (Mx, My, Mz) in the Steered Wheel Frame (N*m).
func get_moment_vector_wheel() -> Vector3:
	return Vector3(mx_nm, my_nm, mz_nm)

## Verify that all force and moment components are finite.
func is_finite() -> bool:
	return (
		VehicleUnits.is_finite_scalar(fx_n)
		and VehicleUnits.is_finite_scalar(fy_n)
		and VehicleUnits.is_finite_scalar(fz_n)
		and VehicleUnits.is_finite_scalar(mx_nm)
		and VehicleUnits.is_finite_scalar(my_nm)
		and VehicleUnits.is_finite_scalar(mz_nm)
	)
