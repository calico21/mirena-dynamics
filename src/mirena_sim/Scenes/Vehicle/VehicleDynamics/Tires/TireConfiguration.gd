extends Resource
class_name TireConfiguration

## Configuration resource for the replaceable tire-model subsystem.
##
## Physical specifications default to the Hoosier 16.0x7.5-10 (R20 compound on 8-inch rim)
## documented in `/home/alexr/eRacing/TeR_Tire/config/tire_specs.yaml`.
##
## IMPORTANT: Selecting `LINEAR_TEST_STUB` uses a simplified linear-with-friction-ellipse
## test model strictly for unit/integration contract verification. It is NOT a calibrated
## tire model. High-fidelity Pacejka MF6.2 and TeR-Tire Grey-Box models are scheduled for Phase 2+.

@export_group("Model Selection")
## Active tire model implementation. Defaults to TeR_Tire Grey-Box (Pacejka MF6.2 + 7-input PINN Residual).
@export var model_type: VehicleDynamicsTypes.TireModelType = VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN
## Optional path to external calibrated parameter file (e.g., TeR_Tire pacejka_mf62_params.yaml).
@export var external_params_path: String = ""

@export_group("Physical Dimensions & Nominal Conditions (Hoosier 16.0x7.5-10 R20)")
## Unloaded tire radius R0 (m). Source: TeR_Tire/config/tire_specs.yaml (0.2032 m; sim bus uses 0.20 m).
@export var unloaded_radius_m: float = 0.20
## Tire section width (m). Source: TeR_Tire/config/tire_specs.yaml (7.5 in = 0.1905 m).
@export var width_m: float = 0.1905
## Nominal vertical load Fz0 (N). Source: TeR_Tire/config/tire_specs.yaml (700.0 N).
@export var fz_nominal_n: float = 700.0
## Maximum allowable normal load before clamping in empirical equations (N).
@export var fz_max_n: float = 4000.0
## Nominal cold/working inflation pressure p0 (kPa). Source: TeR_Tire/config/tire_specs.yaml (82.7 kPa = 12 psi).
@export var nominal_pressure_kpa: float = 82.7
## Nominal operating temperature (C). Source: TeR_Tire/config/tire_specs.yaml (55-75 C window).
@export var nominal_temperature_c: float = 65.0
## Radial vertical elastic stiffness Cz (N/m). Source: TeR_Tire/config/tire_specs.yaml (110000.0 N/m).
@export var vertical_stiffness_npm: float = 110000.0

@export_group("Regularization & Transient Relaxation")
## Low-speed longitudinal velocity regularization epsilon_vx (m/s) to prevent division by zero in slip equations.
@export var vx_min_regularization_mps: float = 0.25
## Enable first-order transient slip relaxation dynamics.
@export var enable_relaxation: bool = false
## Longitudinal relaxation length sigma_kappa (m). Provisional baseline.
@export var relaxation_length_long_m: float = 0.12
## Lateral relaxation length sigma_alpha (m). Provisional baseline.
@export var relaxation_length_lat_m: float = 0.15

@export_group("TeR_Tire Calibrated MF6.2 & Grey-Box PINN Settings")
## Scale factor applied to calibrated TeR_Tire lateral grip (1.0 = raw TTC Round 9 Hoosier R20 fit;
## ~0.55 matches typical outdoor dusty-asphalt vs indoor Calspan sandpaper test belt).
@export var ter_lateral_grip_scale: float = 1.0
## Include single-tire TTC ply-steer / conicity zero-slip offsets (S_Hy, S_Vy, q_Hz1, q_Dz6).
## Default false for full-vehicle symmetry at 0 deg steer/camber; set true for 1:1 single-tire rig match.
@export var enable_ply_steer_asymmetry: bool = false
## Enable the 7-input ResidualTireMLP correction on top of Pacejka MF6.2 when using TER_GREYBOX_PINN.
@export var enable_pinn_residual: bool = true
## Rib temperature spread Delta_T (C) fed into the 7-input ResidualTireMLP.
@export var nominal_delta_temp_c: float = 1.5
## Separate, explicitly labeled uncalibrated longitudinal peak friction coefficient mu_x
## (since TeR_Tire only fits pure lateral Fy0 and aligning moment Mz0).
@export var companion_longitudinal_peak_mu: float = 1.45
## Separate, explicitly labeled uncalibrated longitudinal slip stiffness Kx per unit Fz (1/slip).
@export var companion_longitudinal_kx_per_fz: float = 18.0

@export_group("Uncalibrated Linear Test Stub Parameters (Testing Only)")
## Uncalibrated longitudinal slip stiffness per unit Fz (1/slip) for LINEAR_TEST_STUB only.
@export var stub_kx_per_fz: float = 15.0
## Uncalibrated cornering stiffness at nominal load (N/rad) for LINEAR_TEST_STUB only.
## Source: TeR_Tire/config/tire_specs.yaml initial estimate (25000.0 N/rad @ Fz0=700 N).
@export var stub_cornering_stiffness_n_rad: float = 25000.0
## Peak friction coefficient mu0 at nominal load for LINEAR_TEST_STUB friction ellipse.
## Note: mirena_car.tscn uses 1.0 front / 1.2 rear; dbm_model.py uses D=2.33.
@export var stub_peak_mu: float = 1.20
## Pneumatic trail approximation (m) for LINEAR_TEST_STUB aligning moment Mz.
@export var stub_pneumatic_trail_m: float = 0.015
## Rolling resistance coefficient Cr0 for My calculation.
@export var rolling_resistance_coeff: float = 0.015

## Validate configuration parameters.
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
