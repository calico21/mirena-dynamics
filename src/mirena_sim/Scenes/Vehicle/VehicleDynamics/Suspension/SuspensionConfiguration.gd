extends Resource
class_name SuspensionConfiguration

@export_group("Springs & Ride Height")

@export var spring_rate_front_npm: float = 35030.0

@export var spring_rate_rear_npm: float = 52540.0

@export var motion_ratio_front: float = 1.14

@export var motion_ratio_rear: float = 1.16

@export var rest_length_front_m: float = 0.05
@export var rest_length_rear_m: float = 0.05

@export var max_compression_travel_m: float = 0.080

@export var max_droop_travel_m: float = 0.060

@export_group("Bump Stops & Preload")

@export var bump_stop_engage_m: float = 0.025

@export var bump_stop_rate_npm: float = 50000.0

@export var damper_gas_preload_front_n: float = 120.0
@export var damper_gas_preload_rear_n: float = 120.0

@export var auto_static_preload: bool = true

@export var manual_preload_front_n: float = 650.0
@export var manual_preload_rear_n: float = 800.0

@export_group("Digressive Dampers")

@export var damper_c_low_front_nspm: float = 1800.0

@export var damper_c_low_rear_nspm: float = 1600.0

@export var damper_c_high_front_nspm: float = 720.0

@export var damper_c_high_rear_nspm: float = 640.0

@export var damper_v_knee_front_mps: float = 0.10
@export var damper_v_knee_rear_mps: float = 0.10

@export var rebound_ratio_front: float = 1.60
@export var rebound_ratio_rear: float = 1.60

@export_group("Anti-Roll Bars (ARB)")

@export var arb_rate_front_npm: float = 16000.0

@export var arb_rate_rear_npm: float = 1500.0

@export_group("Kinematics & Compliance (K&C)")

@export var static_camber_front_rad: float = deg_to_rad(-2.0)

@export var static_camber_rear_rad: float = deg_to_rad(-1.5)

@export var camber_per_m_travel_front_radpm: float = deg_to_rad(-25.0)
@export var camber_per_m_travel_rear_radpm: float = deg_to_rad(-20.0)

@export var camber_gain_roll_front: float = -0.80
@export var camber_gain_roll_rear: float = -0.65

@export var roll_center_height_front_m: float = 0.040
@export var roll_center_height_rear_m: float = 0.060

func get_effective_wheel_rate_npm(axle: VehicleDynamicsTypes.Axle) -> float:
	if axle == VehicleDynamicsTypes.Axle.FRONT:
		var mr: float = maxf(motion_ratio_front, 1e-4)
		return spring_rate_front_npm / (mr * mr)
	else:
		var mr: float = maxf(motion_ratio_rear, 1e-4)
		return spring_rate_rear_npm / (mr * mr)

func is_valid() -> bool:
	if spring_rate_front_npm <= 0.0 or spring_rate_rear_npm <= 0.0:
		return false
	if motion_ratio_front <= 0.0 or motion_ratio_rear <= 0.0:
		return false
	if damper_c_low_front_nspm < 0.0 or damper_c_low_rear_nspm < 0.0:
		return false
	if damper_v_knee_front_mps <= 0.0 or damper_v_knee_rear_mps <= 0.0:
		return false
	if arb_rate_front_npm < 0.0 or arb_rate_rear_npm < 0.0:
		return false
	return true
