class_name PacejkaTireModelGD
extends RefCounted

## Pure GDScript implementation of Pacejka Magic Formula 6.2 per-wheel tire dynamics.
## Uses the calibrated Hoosier 16.0x7.5-10 R20 characterization from TeR_Tire.

var fz0: float = 700.0
var p0: float = 82.7
var r0: float = 0.2032
var eps: float = 1e-6

# Lateral parameters (p_*y*)
var p_Cy1: float = 1.4484286
var p_Dy1: float = 2.4903758
var p_Dy2: float = -0.2707577
var p_Dy3: float = 15.00096
var p_Ey1: float = 0.9099938
var p_Ey2: float = -3.5097189
var p_Ey3: float = 0.0252961
var p_Ey4: float = -3.0557888
var p_Hy1: float = 0.0006034
var p_Hy2: float = -0.0013349
var p_Hy3: float = -0.1342268
var p_Ky1: float = 46.849857
var p_Ky2: float = 1.6477934
var p_Vy1: float = 0.0152580
var p_Vy2: float = 0.0439268
var p_Vy3: float = 0.7625932
var p_Vy4: float = -1.6821796
var p_py1: float = -0.1676939
var p_py2: float = -0.2414611

# Aligning moment parameters (q_*z*)
var q_Bz1: float = 24.912182
var q_Bz2: float = 16.187218
var q_Bz3: float = 15.291947
var q_Bz5: float = 5.188725
var q_Cz1: float = 17.054585
var q_Dz1: float = 0.0836906
var q_Dz2: float = -0.0009979
var q_Dz3: float = -3.577241
var q_Dz6: float = -0.0063087
var q_Dz7: float = 0.0248343
var q_Dz8: float = -4.100142
var q_Dz9: float = 1.555570
var q_Ez1: float = -5.362194
var q_Ez2: float = -1.152544
var q_Ez4: float = 2.088430
var q_Ez5: float = -4.150985
var q_Hz1: float = 0.0052826
var q_Hz2: float = -0.0128239
var q_Hz3: float = 0.7302535
var q_Hz4: float = -0.2901489
var q_pz1: float = -0.3766476

func evaluate_pure_lateral(fz: float, alpha: float, gamma: float = 0.0, pressure: float = 82.7) -> Vector2:
	if fz <= 5.0:
		return Vector2.ZERO

	var dfz: float = (fz - fz0) / (fz0 + eps)
	var dpi: float = (pressure - p0) / (p0 + eps)

	# 1. Peak lateral friction
	var mu_y: float = (p_Dy1 + p_Dy2 * dfz) * (1.0 + p_py1 * dpi) * (1.0 - p_Dy3 * (gamma * gamma))
	mu_y = maxf(0.1, mu_y)
	var Dy: float = mu_y * fz
	var Cy: float = p_Cy1

	# 2. Cornering stiffness Kya0
	var atan_arg: float = fz / (p_Ky2 * fz0 + eps)
	var Kya0: float = p_Ky1 * fz0 * sin(2.0 * atan(atan_arg)) * (1.0 + p_py2 * dpi)
	Kya0 = maxf(100.0, Kya0)

	# 3. Stiffness factor By
	var By: float = Kya0 / (Cy * Dy + eps)

	# 4. Shifts
	var S_Hy: float = p_Hy1 + p_Hy2 * dfz + p_Hy3 * gamma
	var S_Vy: float = fz * ((p_Vy1 + p_Vy2 * dfz) + (p_Vy3 + p_Vy4 * dfz) * gamma)
	var alpha_y: float = alpha + S_Hy

	# 5. Curvature factor Ey
	var sign_ay: float = signf(alpha_y) if alpha_y != 0.0 else 1.0
	var Ey: float = tanh(p_Ey1 + p_Ey2 * dfz) * (1.0 - (p_Ey3 + p_Ey4 * gamma) * sign_ay)
	Ey = clampf(Ey, -5.0, 0.99)

	# 6. Lateral Force Fy0
	var B_ay: float = By * alpha_y
	var atan_B_ay: float = atan(B_ay)
	var inside_sin: float = Cy * atan(B_ay - Ey * (B_ay - atan_B_ay))
	var Fy0: float = Dy * sin(inside_sin) + S_Vy

	# 7. Self-aligning torque Mz0
	var Fy0_prime: float = Fy0 - S_Vy
	var alpha_t: float = alpha + q_Hz1 + q_Hz2 * dfz + (q_Hz3 + q_Hz4 * dfz) * gamma
	var Dt: float = fz * (r0 / fz0) * (q_Dz1 + q_Dz2 * dfz) * (1.0 + q_pz1 * dpi) * (1.0 - q_Dz3 * absf(gamma))
	var Ct: float = q_Cz1
	var Bt: float = (q_Bz1 + q_Bz2 * dfz + q_Bz3 * (dfz * dfz)) * (1.0 + q_Bz5 * absf(gamma)) * (r0 / (fz0 * Dt + eps))
	var sign_at: float = signf(alpha_t) if alpha_t != 0.0 else 1.0
	var Et: float = tanh(q_Ez1 + q_Ez2 * dfz) * (1.0 + (q_Ez4 + q_Ez5 * gamma) * sign_at)
	var B_at: float = Bt * alpha_t
	var trail: float = Dt * cos(Ct * atan(B_at - Et * (B_at - atan(B_at)))) * cos(alpha)

	var alpha_r: float = alpha + S_Hy + (S_Vy / (Kya0 + eps))
	var Dr: float = fz * r0 * ((q_Dz6 + q_Dz7 * dfz) + (q_Dz8 + q_Dz9 * dfz) * gamma)
	var Br: float = r0 / fz0
	var Cr: float = 1.0
	var Mzr: float = Dr * cos(Cr * atan(Br * alpha_r))
	var Mz0: float = -trail * Fy0_prime + Mzr

	return Vector2(Fy0, Mz0)
