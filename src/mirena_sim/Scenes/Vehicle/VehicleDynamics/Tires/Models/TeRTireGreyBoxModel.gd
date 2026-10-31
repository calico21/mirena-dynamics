extends PacejkaMF62TireModel
class_name TeRTireGreyBoxModel

## Hybrid Grey-Box Tire Model (`TireModelType.TER_GREYBOX_PINN`):
## Calibrated Pacejka MF6.2 Analytical Baseline + 7-Input Physics-Informed Neural Network (PINN) Residual.
##
## Directly ports `TeR_Tire/models/grey_box_tire.py` and `TeR_Tire/models/network.py` (`ResidualTireMLP`)
## using the trained weights from `TeR_Tire/export/weights/pinn_residual_best.pth`.
##
## Architecture of `ResidualTireMLP`:
## - 7 Inputs: `[alpha, alpha_dot, gamma, Fz, P, T_avg, Delta_T]`
## - Input normalization: `x_norm = (x - x_mean) / (x_std + 1e-6)`
## - Hidden Layer 0: `Linear(7 -> 64) + SiLU`
## - Hidden Layer 2: `Linear(64 -> 64) + SiLU`
## - Output Layer 4: `Linear(64 -> 2)`
## - Bounded Output Scaling:
##   - `raw_delta_fy = 600.0 * tanh(raw_out[0] * 250.0 / 600.0)`
##   - `raw_delta_mz = 30.0 * tanh(raw_out[1] * 10.0 / 30.0)`
## - Zero-Slip Physics Gate:
##   - `gate = tanh(15.0 * |alpha| + 5.0 * |gamma| + 2.0 * |alpha_dot|)`
##   - Ensures exact zero residual at `alpha = gamma = alpha_dot = 0`.

const IN_DIM: int = 7
const HIDDEN_DIM: int = 64
const OUT_SCALE_FY: float = 250.0
const OUT_SCALE_MZ: float = 10.0
const MAX_DELTA_FY: float = 600.0
const MAX_DELTA_MZ: float = 30.0

var _x_mean: PackedFloat32Array
var _x_std: PackedFloat32Array
var _w0: PackedFloat32Array
var _b0: PackedFloat32Array
var _w2: PackedFloat32Array
var _b2: PackedFloat32Array
var _w4: PackedFloat32Array
var _b4: PackedFloat32Array
var _h0_buf: PackedFloat32Array
var _h1_buf: PackedFloat32Array

func _init() -> void:
	_x_mean = TeRTireWeightsData.get_x_mean()
	_x_std = TeRTireWeightsData.get_x_std()
	_w0 = TeRTireWeightsData.get_w0()
	_b0 = TeRTireWeightsData.get_b0()
	_w2 = TeRTireWeightsData.get_w2()
	_b2 = TeRTireWeightsData.get_b2()
	_w4 = TeRTireWeightsData.get_w4()
	_b4 = TeRTireWeightsData.get_b4()
	_h0_buf.resize(HIDDEN_DIM)
	_h1_buf.resize(HIDDEN_DIM)

func get_model_name() -> String:
	return "TeR_Tire Grey-Box (Pacejka MF6.2 + 7D PINN Residual)"

func get_provenance_metadata() -> Dictionary:
	return {
		"tire_spec": TeRTireWeightsData.TIRE_NAME,
		"source_module": "src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Tires/TeR_Tire",
		"fy_provenance": "CALIBRATED_TER_MF62 + PINN_RESIDUAL (pinn_residual_best.pth)",
		"mz_provenance": "CALIBRATED_TER_MF62 + PINN_RESIDUAL (pinn_residual_best.pth)",
		"fx_provenance": "UNCALIBRATED_COMPANION_SLIP_ELLIPSE (TeR_Tire has no longitudinal fit)",
		"combined_slip_provenance": "FRICTION_ELLIPSE_COMPANION",
	}

## Exact port of `TeR_Tire/models/network.py::ResidualTireMLP.forward`.
## Directly callable by unit tests to verify 1:1 agreement with PyTorch.
func eval_ter_pinn_residual_raw(
	alpha_ter_rad: float,
	alpha_dot_rad_s: float,
	gamma_rad: float,
	fz_n: float,
	pressure_kpa: float = TeRTireWeightsData.P0_KPA,
	temp_avg_c: float = 62.0,
	delta_temp_c: float = 1.5
) -> Vector2:
	var x0: float = (alpha_ter_rad - _x_mean[0]) / (_x_std[0] + EPS)
	var x1: float = (alpha_dot_rad_s - _x_mean[1]) / (_x_std[1] + EPS)
	var x2: float = (gamma_rad - _x_mean[2]) / (_x_std[2] + EPS)
	var x3: float = (fz_n - _x_mean[3]) / (_x_std[3] + EPS)
	var x4: float = (pressure_kpa - _x_mean[4]) / (_x_std[4] + EPS)
	var x5: float = (temp_avg_c - _x_mean[5]) / (_x_std[5] + EPS)
	var x6: float = (delta_temp_c - _x_mean[6]) / (_x_std[6] + EPS)

	# Layer 0: Linear(7 -> 64) + SiLU
	for i in range(HIDDEN_DIM):
		var row_off: int = i * IN_DIM
		var z: float = _b0[i] \
			+ _w0[row_off + 0] * x0 \
			+ _w0[row_off + 1] * x1 \
			+ _w0[row_off + 2] * x2 \
			+ _w0[row_off + 3] * x3 \
			+ _w0[row_off + 4] * x4 \
			+ _w0[row_off + 5] * x5 \
			+ _w0[row_off + 6] * x6
		_h0_buf[i] = z / (1.0 + exp(-clampf(z, -40.0, 40.0)))

	# Layer 2: Linear(64 -> 64) + SiLU
	for i in range(HIDDEN_DIM):
		var row_off: int = i * HIDDEN_DIM
		var z: float = _b2[i]
		for j in range(HIDDEN_DIM):
			z += _w2[row_off + j] * _h0_buf[j]
		_h1_buf[i] = z / (1.0 + exp(-clampf(z, -40.0, 40.0)))

	# Layer 4: Linear(64 -> 2)
	var raw_0: float = _b4[0]
	var raw_1: float = _b4[1]
	for j in range(HIDDEN_DIM):
		var h1_j: float = _h1_buf[j]
		raw_0 += _w4[j] * h1_j
		raw_1 += _w4[HIDDEN_DIM + j] * h1_j

	var raw_delta_fy: float = MAX_DELTA_FY * tanh(raw_0 * OUT_SCALE_FY / MAX_DELTA_FY)
	var raw_delta_mz: float = MAX_DELTA_MZ * tanh(raw_1 * OUT_SCALE_MZ / MAX_DELTA_MZ)

	var gate: float = tanh(15.0 * absf(alpha_ter_rad) + 5.0 * absf(gamma_rad) + 2.0 * absf(alpha_dot_rad_s))
	return Vector2(raw_delta_fy * gate, raw_delta_mz * gate)

func _evaluate_residual_hook(
	alpha_ter: float,
	inputs: TireInputs,
	state: TireState,
	config: TireConfiguration,
	fz: float,
	grip_scale: float
) -> Vector2:
	if not config.enable_pinn_residual:
		state.delta_fy_pinn_n = 0.0
		state.delta_mz_pinn_nm = 0.0
		return Vector2.ZERO

	# Convert ISO slip-angle rate to TeR convention (-alpha_dot_iso) and clamp to physical range
	var alpha_dot_ter: float = clampf(-inputs.slip_angle_rate_rad_s, -2.5, 2.5)
	var gamma_clamped: float = clampf(inputs.camber_gamma_rad, -0.25, 0.25)

	# Evaluate odd-symmetrized PINN residual with respect to lateral kinematic inputs (alpha, alpha_dot, gamma)
	# when enable_ply_steer_asymmetry is false so single-tire rig MLP bias offsets do not create a unilateral
	# lateral pull at zero slip angle on a 4-wheel vehicle, while preserving exact raw evaluation in
	# eval_ter_pinn_residual_raw() and when ply-steer rig asymmetry is enabled.
	var res_pos: Vector2 = eval_ter_pinn_residual_raw(
		alpha_ter,
		alpha_dot_ter,
		gamma_clamped,
		fz,
		inputs.inflation_pressure_kpa,
		inputs.tire_temperature_c,
		config.nominal_delta_temp_c
	)
	var res: Vector2 = res_pos
	if not config.enable_ply_steer_asymmetry:
		var res_neg: Vector2 = eval_ter_pinn_residual_raw(
			-alpha_ter,
			-alpha_dot_ter,
			-gamma_clamped,
			fz,
			inputs.inflation_pressure_kpa,
			inputs.tire_temperature_c,
			config.nominal_delta_temp_c
		)
		res = 0.5 * (res_pos - res_neg)

	# Smoothly gate dynamic PINN residual by rolling speed so transient neural network corrections
	# are active when rolling and vanish cleanly at zero wheel rolling velocity.
	var vx_eps: float = maxf(config.vx_min_regularization_mps, 0.25)
	var rolling_gate: float = clampf(absf(inputs.vx_contact_mps) / vx_eps, 0.0, 1.0)
	var dfy: float = res.x * grip_scale * rolling_gate
	var dmz: float = res.y * grip_scale * rolling_gate
	state.delta_fy_pinn_n = dfy
	state.delta_mz_pinn_nm = dmz
	return Vector2(dfy, dmz)
