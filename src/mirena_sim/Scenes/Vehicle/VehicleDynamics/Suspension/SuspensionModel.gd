extends RefCounted
class_name SuspensionModel

func evaluate_all_corners(
	wheels: Array[WheelState],
	body_roll_rad: float,
	aero_downforce_front_n: float,
	aero_downforce_rear_n: float,
	params: VehicleParameters,
	susp_cfg: SuspensionConfiguration,
	steer_cfg: SteeringConfiguration = null
) -> void:
	if wheels.size() != VehicleDynamicsTypes.WHEEL_COUNT or params == null or susp_cfg == null:
		return

	var static_loads: Array[float] = VehicleInertia.compute_static_corner_loads_n(params)

	var front_arb: Vector2 = AntiRollBarModel.compute_axle_arb_forces_n(
		wheels[VehicleDynamicsTypes.WheelCorner.FL].suspension_state.compression_m,
		wheels[VehicleDynamicsTypes.WheelCorner.FR].suspension_state.compression_m,
		VehicleDynamicsTypes.Axle.FRONT,
		susp_cfg
	)
	var rear_arb: Vector2 = AntiRollBarModel.compute_axle_arb_forces_n(
		wheels[VehicleDynamicsTypes.WheelCorner.RL].suspension_state.compression_m,
		wheels[VehicleDynamicsTypes.WheelCorner.RR].suspension_state.compression_m,
		VehicleDynamicsTypes.Axle.REAR,
		susp_cfg
	)
	var arb_forces: Array[float] = [front_arb.x, front_arb.y, rear_arb.x, rear_arb.y]

	var base_strut_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var w: WheelState = wheels[i]
		var s: SuspensionState = w.suspension_state
		var corner: VehicleDynamicsTypes.WheelCorner = w.corner
		var axle: VehicleDynamicsTypes.Axle = VehicleDynamicsTypes.corner_axle(corner)
		var is_front: bool = (axle == VehicleDynamicsTypes.Axle.FRONT)

		var preload_n: float = static_loads[i] if susp_cfg.auto_static_preload else (
			susp_cfg.manual_preload_front_n if is_front else susp_cfg.manual_preload_rear_n
		)
		s.static_preload_force_n = preload_n
		s.spring_force_n = SpringModel.compute_spring_force_n(s.compression_m, preload_n, axle, susp_cfg)
		s.bump_stop_force_n = SpringModel.compute_bump_stop_force_n(s.compression_m, susp_cfg)
		s.damper_force_n = DamperModel.compute_damper_force_n(s.compression_velocity_mps, axle, susp_cfg)
		if w.contact_state.in_contact:
			base_strut_forces[i] = maxf(0.0, s.spring_force_n + s.bump_stop_force_n + s.damper_force_n)
		else:
			base_strut_forces[i] = 0.0

	var front_arb_bounded: float = clampf(arb_forces[0], -base_strut_forces[0], base_strut_forces[1])
	if not wheels[0].contact_state.in_contact or not wheels[1].contact_state.in_contact:
		front_arb_bounded = clampf(front_arb_bounded, -base_strut_forces[0], base_strut_forces[1])
	arb_forces[0] = front_arb_bounded
	arb_forces[1] = -front_arb_bounded

	var rear_arb_bounded: float = clampf(arb_forces[2], -base_strut_forces[2], base_strut_forces[3])
	if not wheels[2].contact_state.in_contact or not wheels[3].contact_state.in_contact:
		rear_arb_bounded = clampf(rear_arb_bounded, -base_strut_forces[2], base_strut_forces[3])
	arb_forces[2] = rear_arb_bounded
	arb_forces[3] = -rear_arb_bounded

	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var w: WheelState = wheels[i]
		var s: SuspensionState = w.suspension_state
		var corner: VehicleDynamicsTypes.WheelCorner = w.corner
		var axle: VehicleDynamicsTypes.Axle = VehicleDynamicsTypes.corner_axle(corner)
		var is_front: bool = (axle == VehicleDynamicsTypes.Axle.FRONT)
		s.arb_force_n = arb_forces[i]

		var aero_corner_n: float = 0.5 * (aero_downforce_front_n if is_front else aero_downforce_rear_n)
		var raw_strut_force_n: float = base_strut_forces[i] + s.arb_force_n

		if w.contact_state.in_contact:
			s.total_suspension_force_n = maxf(0.0, raw_strut_force_n)
			s.normal_load_fz_n = maxf(0.0, raw_strut_force_n + aero_corner_n)
		else:
			s.total_suspension_force_n = 0.0
			s.normal_load_fz_n = 0.0

		var static_camber: float = susp_cfg.static_camber_front_rad if is_front else susp_cfg.static_camber_rear_rad
		var heave_gain: float = susp_cfg.camber_per_m_travel_front_radpm if is_front else susp_cfg.camber_per_m_travel_rear_radpm
		var roll_gain: float = susp_cfg.camber_gain_roll_front if is_front else susp_cfg.camber_gain_roll_rear
		var roll_sign: float = -1.0 if VehicleDynamicsTypes.is_left_corner(corner) else 1.0
		s.dynamic_camber_rad = static_camber + heave_gain * s.compression_m + roll_gain * body_roll_rad * roll_sign
		w.camber_angle_rad = s.dynamic_camber_rad

		if steer_cfg != null:
			var bs_coeff: float = steer_cfg.bump_steer_front_radpm if is_front else steer_cfg.bump_steer_rear_radpm
			var bs_sign: float = 1.0 if VehicleDynamicsTypes.is_left_corner(corner) else -1.0
			s.bump_steer_rad = bs_coeff * s.compression_m * bs_sign
		else:
			s.bump_steer_rad = 0.0
