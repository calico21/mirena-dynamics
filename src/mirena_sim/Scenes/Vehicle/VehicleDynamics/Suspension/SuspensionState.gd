extends RefCounted
class_name SuspensionState

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

var compression_m: float = 0.0

var compression_velocity_mps: float = 0.0

var static_preload_force_n: float = 0.0

var spring_force_n: float = 0.0

var damper_force_n: float = 0.0

var arb_force_n: float = 0.0

var bump_stop_force_n: float = 0.0

var total_suspension_force_n: float = 0.0

var normal_load_fz_n: float = 0.0

var dynamic_camber_rad: float = 0.0

var bump_steer_rad: float = 0.0

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	reset()

func reset(static_fz_n: float = 0.0, static_camber: float = 0.0) -> void:
	compression_m = 0.0
	compression_velocity_mps = 0.0
	static_preload_force_n = maxf(0.0, static_fz_n)
	spring_force_n = static_preload_force_n
	damper_force_n = 0.0
	arb_force_n = 0.0
	bump_stop_force_n = 0.0
	total_suspension_force_n = static_preload_force_n
	normal_load_fz_n = static_preload_force_n
	dynamic_camber_rad = static_camber
	bump_steer_rad = 0.0

func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_scalar(compression_m)
		and VehicleUnits.is_finite_scalar(compression_velocity_mps)
		and VehicleUnits.is_finite_scalar(spring_force_n)
		and VehicleUnits.is_finite_scalar(damper_force_n)
		and VehicleUnits.is_finite_scalar(arb_force_n)
		and VehicleUnits.is_finite_scalar(bump_stop_force_n)
		and VehicleUnits.is_finite_scalar(total_suspension_force_n)
		and VehicleUnits.is_finite_scalar(normal_load_fz_n)
		and normal_load_fz_n >= 0.0
	)
