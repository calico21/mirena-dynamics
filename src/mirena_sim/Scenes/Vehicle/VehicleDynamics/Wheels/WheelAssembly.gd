extends RefCounted
class_name WheelAssembly

## Encapsulates the per-corner wheel subsystem operations (`WheelState`, `WheelKinematics`, `WheelRotationalDynamics`).

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner

## Update local velocities and slip variables for this wheel corner.
func evaluate_kinematics(
	wheel: WheelState,
	v_com_vehicle: Vector3,
	omega_vehicle: Vector3,
	vx_eps_mps: float
) -> void:
	WheelKinematics.update_wheel_kinematics(wheel, v_com_vehicle, omega_vehicle, vx_eps_mps)

## Build a `TireInputs` structure from the current `WheelState` and `TireConfiguration`.
func build_tire_inputs(
	wheel: WheelState,
	tire_cfg: TireConfiguration,
	dt_s: float
) -> TireInputs:
	var inputs := TireInputs.new()
	inputs.corner = wheel.corner
	inputs.slip_ratio_kappa = wheel.slip_ratio_kappa
	inputs.slip_angle_alpha_rad = wheel.slip_angle_alpha_rad
	var eps: float = tire_cfg.vx_min_regularization_mps if tire_cfg != null else 0.25
	var roll_gate: float = clampf(absf(wheel.contact_velocity_wheel.x) / maxf(eps, 0.1), 0.0, 1.0)
	if dt_s > 0.0:
		var raw_rate: float = (wheel.slip_angle_alpha_rad - wheel.tire_state.prev_slip_angle_rad) / dt_s
		inputs.slip_angle_rate_rad_s = clampf(raw_rate * roll_gate, -10.0, 10.0)
	else:
		inputs.slip_angle_rate_rad_s = 0.0
	inputs.normal_load_fz_n = wheel.suspension_state.normal_load_fz_n if wheel.contact_state.in_contact else 0.0
	# Convert automotive setup camber (negative = top of wheel tilted inward toward vehicle centerline)
	# to ISO/TeR tire inclination angle `gamma_rad`:
	# - Left wheels (FL, RL): inward tilt (-camber_setup) leans toward -Y (right), which is +gamma_iso -> Fy0 < 0 (inward).
	# - Right wheels (FR, RR): inward tilt (-camber_setup) leans toward +Y (left), which is -gamma_iso -> Fy0 > 0 (inward).
	var camber_iso_sign: float = -1.0 if VehicleDynamicsTypes.is_left_corner(wheel.corner) else 1.0
	inputs.camber_gamma_rad = camber_iso_sign * wheel.camber_angle_rad
	inputs.vx_contact_mps = wheel.contact_velocity_wheel.x
	inputs.vy_contact_mps = wheel.contact_velocity_wheel.y
	inputs.wheel_omega_rad_s = wheel.omega_rad_s
	inputs.effective_radius_m = wheel.effective_radius_m
	if tire_cfg != null:
		inputs.inflation_pressure_kpa = tire_cfg.nominal_pressure_kpa
		inputs.tire_temperature_c = wheel.tire_state.tread_temp_c
	inputs.surface_mu_scale = wheel.contact_state.surface_friction_scale
	inputs.dt_s = dt_s
	return inputs

## Integrate rotational spin dynamics for this wheel corner.
func integrate_rotation(
	wheel: WheelState,
	wheel_inertia_kgm2: float,
	dt_s: float,
	standstill_omega_eps_rad_s: float = 0.05
) -> void:
	WheelRotationalDynamics.step_wheel_rotation(wheel, wheel_inertia_kgm2, dt_s, standstill_omega_eps_rad_s)
