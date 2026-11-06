extends Node3D
class_name Ter26VehicleDebugDraw
## Debug telemetry overlay for contact patch forces (Fx, Fy, Fz) and joint loads (f_pushrod, f_damper).

@export var draw_tire_forces: bool = false
@export var force_scale_m_per_N: float = 0.0005

func update_debug_vectors(_diag: Dictionary) -> void:
	pass
