extends RefCounted
class_name TireForces

var fx_n: float = 0.0
var fy_n: float = 0.0
var fz_n: float = 0.0
var mx_nm: float = 0.0
var my_nm: float = 0.0
var mz_nm: float = 0.0

var valid: bool = true

var fault_flags: int = VehicleDynamicsTypes.FaultFlag.NONE

func zero() -> void:
	fx_n = 0.0
	fy_n = 0.0
	fz_n = 0.0
	mx_nm = 0.0
	my_nm = 0.0
	mz_nm = 0.0
	valid = true
	fault_flags = VehicleDynamicsTypes.FaultFlag.NONE

func get_force_vector_wheel() -> Vector3:
	return Vector3(fx_n, fy_n, fz_n)

func get_moment_vector_wheel() -> Vector3:
	return Vector3(mx_nm, my_nm, mz_nm)

func is_finite() -> bool:
	return (
		VehicleUnits.is_finite_scalar(fx_n)
		and VehicleUnits.is_finite_scalar(fy_n)
		and VehicleUnits.is_finite_scalar(fz_n)
		and VehicleUnits.is_finite_scalar(mx_nm)
		and VehicleUnits.is_finite_scalar(my_nm)
		and VehicleUnits.is_finite_scalar(mz_nm)
	)
