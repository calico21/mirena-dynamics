extends RefCounted
class_name WheelRotationalDynamics

## Integrates individual wheel rotational spin dynamics:
##   I_w * d(omega_w)/dt = tau_drive - tau_brake_signed - Fx * R_eff + My
##
## Guaranteed Physical Properties:
## - Friction brakes are strictly passive dissipative devices: `brake_torque_magnitude_nm >= 0`
##   opposes existing rotation (`omega_w`) or opposes net active torque (`tau_drive - Fx * R_eff + My`)
##   when near standstill, and can NEVER accelerate a wheel in reverse through zero speed.

## Integrate one wheel's angular velocity `omega_rad_s` and visual `spin_angle_rad` over `dt_s`.
static func step_wheel_rotation(
	wheel: WheelState,
	wheel_inertia_kgm2: float,
	dt_s: float,
	standstill_omega_eps_rad_s: float = 0.05
) -> void:
	if wheel == null or dt_s <= 0.0 or wheel_inertia_kgm2 <= 0.0:
		return

	var iw: float = maxf(wheel_inertia_kgm2, 1e-4)
	var fx_n: float = wheel.tire_forces.fx_n if wheel.contact_state.in_contact else 0.0
	var my_nm: float = wheel.tire_forces.my_nm if wheel.contact_state.in_contact else 0.0
	var r_eff: float = maxf(wheel.effective_radius_m, 0.01)

	# Active non-brake torque acting on the wheel shaft:
	# Positive Fx (forward tractive road force) reacts against forward wheel spin (-Fx * R_eff).
	var active_torque_nm: float = wheel.drive_torque_nm - (fx_n * r_eff) + my_nm
	var max_brake_mag: float = maxf(0.0, wheel.brake_torque_magnitude_nm)

	# Semi-implicit tire longitudinal chord stiffness / slip damping:
	# Prevents explicit forward-Euler numerical instability (200 Hz Nyquist chatter) at low speeds and rest.
	var c_omega: float = 0.0
	if wheel.contact_state.in_contact:
		var fz: float = maxf(wheel.suspension_state.normal_load_fz_n, 100.0)
		var kx: float = 18.0 * fz
		if absf(wheel.slip_ratio_kappa) > 0.001 and absf(fx_n) > 1.0:
			kx = clampf(absf(fx_n) / absf(wheel.slip_ratio_kappa), 10.0 * fz, 30.0 * fz)
		var v_denom: float = maxf(absf(wheel.contact_velocity_wheel.x), maxf(absf(wheel.omega_rad_s * r_eff), 0.25))
		c_omega = (r_eff * r_eff) * (kx / v_denom)

	var iw_eff: float = iw + dt_s * c_omega

	# Momentum required to bring the current wheel spin to rest in one step dt_s:
	var stop_torque_needed_nm: float = (iw_eff * wheel.omega_rad_s / dt_s) + active_torque_nm

	var signed_brake_nm: float = 0.0
	if absf(wheel.omega_rad_s) <= standstill_omega_eps_rad_s and absf(stop_torque_needed_nm) <= max_brake_mag:
		# Standstill stiction / lockup: brake holds the wheel at exact zero spin without chattering
		signed_brake_nm = stop_torque_needed_nm
		wheel.signed_brake_torque_nm = signed_brake_nm
		wheel.net_wheel_torque_nm = 0.0
		wheel.angular_accel_rad_s2 = -wheel.omega_rad_s / dt_s
		wheel.omega_rad_s = 0.0
		return

	# Dynamic braking: opposes current rotation (or opposes active_torque if omega == 0)
	var spin_dir: float = signf(wheel.omega_rad_s)
	if spin_dir == 0.0:
		spin_dir = signf(active_torque_nm)

	signed_brake_nm = spin_dir * max_brake_mag
	var candidate_net_torque: float = active_torque_nm - signed_brake_nm
	var candidate_alpha: float = candidate_net_torque / iw_eff
	var candidate_omega: float = wheel.omega_rad_s + candidate_alpha * dt_s

	# Standstill coasting convergence: when unthrottled and wheel speed is negligible, converge to exact zero
	if absf(wheel.omega_rad_s) <= standstill_omega_eps_rad_s and absf(wheel.drive_torque_nm) <= 0.1:
		if absf(candidate_omega) <= standstill_omega_eps_rad_s * 0.5:
			candidate_omega = 0.0
			candidate_alpha = -wheel.omega_rad_s / dt_s

	# Prevent brake torque alone from crossing zero omega in a single step
	if wheel.omega_rad_s != 0.0 and signf(candidate_omega) != signf(wheel.omega_rad_s):
		if absf(active_torque_nm) <= max_brake_mag:
			wheel.signed_brake_torque_nm = stop_torque_needed_nm
			wheel.net_wheel_torque_nm = active_torque_nm - wheel.signed_brake_torque_nm
			wheel.angular_accel_rad_s2 = -wheel.omega_rad_s / dt_s
			wheel.omega_rad_s = 0.0
			return

	wheel.signed_brake_torque_nm = signed_brake_nm
	wheel.net_wheel_torque_nm = candidate_net_torque
	wheel.angular_accel_rad_s2 = candidate_alpha
	wheel.omega_rad_s = candidate_omega
	wheel.spin_angle_rad = fmod(wheel.spin_angle_rad + wheel.omega_rad_s * dt_s, TAU)
