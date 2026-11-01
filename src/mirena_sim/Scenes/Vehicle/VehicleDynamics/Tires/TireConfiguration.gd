extends Resource
class_name TireConfiguration

@export_group("Model Selection")

@export var model_type: VehicleDynamicsTypes.TireModelType = VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN

@export var external_params_path: String = ""

@export_group("Physical Dimensions & Nominal Conditions (Hoosier 16.0x7.5-10 R20)")

@export var unloaded_radius_m: float = 0.20

@export var width_m: float = 0.1905

@export var fz_nominal_n: float = 700.0

@export var fz_max_n: float = 4000.0

@export var nominal_pressure_kpa: float = 82.7

@export var nominal_temperature_c: float = 65.0

@export var vertical_stiffness_npm: float = 110000.0

@export_group("Regularization & Transient Relaxation")

@export var vx_min_regularization_mps: float = 0.25

@export var enable_relaxation: bool = false

@export var relaxation_length_long_m: float = 0.12

@export var relaxation_length_lat_m: float = 0.15

@export_group("TeR_Tire Calibrated MF6.2 & Grey-Box PINN Settings")

@export var ter_lateral_grip_scale: float = 1.0

@export var enable_ply_steer_asymmetry: bool = false

@export var enable_pinn_residual: bool = true

@export var nominal_delta_temp_c: float = 1.5

@export var companion_longitudinal_peak_mu: float = 1.45

@export var companion_longitudinal_kx_per_fz: float = 18.0

@export_group("Uncalibrated Linear Test Stub Parameters (Testing Only)")

@export var stub_kx_per_fz: float = 15.0

@export var stub_cornering_stiffness_n_rad: float = 25000.0

@export var stub_peak_mu: float = 1.20

@export var stub_pneumatic_trail_m: float = 0.015

@export var rolling_resistance_coeff: float = 0.015

func is_valid() -> bool:
	if unloaded_radius_m <= 0.0 or fz_nominal_n <= 0.0 or fz_max_n < fz_nominal_n:
		return false
	if nominal_pressure_kpa <= 0.0 or vertical_stiffness_npm <= 0.0:
		return false
	if vx_min_regularization_mps <= 0.0:
		return false
	if relaxation_length_long_m <= 0.0 or relaxation_length_lat_m <= 0.0:
		return false
	return true
