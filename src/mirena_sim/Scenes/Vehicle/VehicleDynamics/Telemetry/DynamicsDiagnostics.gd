extends RefCounted
class_name DynamicsDiagnostics

var enabled: bool = true

var timestamp_s: float = 0.0
var chassis_u_mps: float = 0.0
var chassis_v_mps: float = 0.0
var chassis_w_mps: float = 0.0
var chassis_ax_mps2: float = 0.0
var chassis_ay_mps2: float = 0.0
var chassis_az_mps2: float = 0.0
var chassis_yaw_rate_rad_s: float = 0.0
var chassis_roll_rad: float = 0.0
var chassis_pitch_rad: float = 0.0
var driver_throttle_cmd: float = 0.0
var driver_brake_cmd: float = 0.0
var driver_steer_cmd_rad: float = 0.0
var chassis_beta_rad: float = 0.0
var chassis_psi_rad: float = 0.0
var chassis_x_m: float = 0.0
var chassis_y_m: float = 0.0

var wheel_in_contact: Array[bool] = [true, true, true, true]
var wheel_normal_load_fz_n: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_slip_ratio: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_slip_angle_rad: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_fx_n: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_fy_n: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_mz_nm: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_suspension_compression_m: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_omega_rad_s: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_rpm: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_drive_torque_nm: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_brake_torque_nm: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_steer_angle_rad: Array[float] = [0.0, 0.0, 0.0, 0.0]

var total_steps_recorded: int = 0
var non_finite_input_count: int = 0
var non_finite_state_count: int = 0
var non_finite_force_count: int = 0
var unimplemented_model_call_count: int = 0
var all_wheels_airborne_count: int = 0
var tire_saturation_count: int = 0
var last_fault_flags: int = VehicleDynamicsTypes.FaultFlag.NONE

func _init() -> void:
	reset()

func reset() -> void:
	timestamp_s = 0.0
	chassis_u_mps = 0.0
	chassis_v_mps = 0.0
	chassis_w_mps = 0.0
	chassis_ax_mps2 = 0.0
	chassis_ay_mps2 = 0.0
	chassis_az_mps2 = 0.0
	chassis_yaw_rate_rad_s = 0.0
	chassis_roll_rad = 0.0
	chassis_pitch_rad = 0.0

	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		wheel_in_contact[i] = true
		wheel_normal_load_fz_n[i] = 0.0
		wheel_slip_ratio[i] = 0.0
		wheel_slip_angle_rad[i] = 0.0
		wheel_fx_n[i] = 0.0
		wheel_fy_n[i] = 0.0
		wheel_mz_nm[i] = 0.0
		wheel_suspension_compression_m[i] = 0.0
		wheel_omega_rad_s[i] = 0.0
		wheel_rpm[i] = 0.0
		wheel_drive_torque_nm[i] = 0.0
		wheel_brake_torque_nm[i] = 0.0
		wheel_steer_angle_rad[i] = 0.0

	total_steps_recorded = 0
	non_finite_input_count = 0
	non_finite_state_count = 0
	non_finite_force_count = 0
	unimplemented_model_call_count = 0
	all_wheels_airborne_count = 0
	tire_saturation_count = 0
	last_fault_flags = VehicleDynamicsTypes.FaultFlag.NONE

func record_from_state(state: VehicleState) -> void:
	if not enabled or state == null:
		return

	total_steps_recorded += 1
	timestamp_s = state.sim_time_s
	chassis_u_mps = state.u_mps
	chassis_v_mps = state.v_mps
	chassis_w_mps = state.w_mps
	chassis_ax_mps2 = state.ax_mps2
	chassis_ay_mps2 = state.ay_mps2
	chassis_az_mps2 = state.az_mps2
	chassis_yaw_rate_rad_s = state.r_rad_s
	chassis_roll_rad = state.roll_rad
	chassis_pitch_rad = state.pitch_rad
	driver_throttle_cmd = state.throttle_cmd
	driver_brake_cmd = state.brake_cmd
	driver_steer_cmd_rad = state.steer_cmd_rad
	chassis_beta_rad = atan2(state.v_mps, maxf(absf(state.u_mps), 0.5))
	chassis_psi_rad = state.psi_odom_rad
	chassis_x_m = state.x_odom_m
	chassis_y_m = state.y_odom_m

	var any_contact: bool = false
	var step_flags: int = state.fault_flags
	var uncontacted_count: int = 0

	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var w: WheelState = state.wheels[i]
		wheel_in_contact[i] = w.contact_state.in_contact
		if w.contact_state.in_contact and w.suspension_state.normal_load_fz_n > 0.0:
			any_contact = true
		else:
			uncontacted_count += 1
		wheel_normal_load_fz_n[i] = w.suspension_state.normal_load_fz_n
		wheel_slip_ratio[i] = w.slip_ratio_kappa
		wheel_slip_angle_rad[i] = w.slip_angle_alpha_rad
		wheel_fx_n[i] = w.tire_forces.fx_n
		wheel_fy_n[i] = w.tire_forces.fy_n
		wheel_mz_nm[i] = w.tire_forces.mz_nm
		wheel_suspension_compression_m[i] = w.suspension_state.compression_m
		wheel_omega_rad_s[i] = w.omega_rad_s
		wheel_rpm[i] = w.get_rpm()
		wheel_drive_torque_nm[i] = w.drive_torque_nm
		wheel_brake_torque_nm[i] = w.signed_brake_torque_nm
		wheel_steer_angle_rad[i] = w.steering_angle_rad
		step_flags |= w.tire_forces.fault_flags

	if not any_contact:
		step_flags |= VehicleDynamicsTypes.FaultFlag.CONTACT_LOSS_ALL_WHEELS
	elif uncontacted_count > 0:
		step_flags |= VehicleDynamicsTypes.FaultFlag.WHEEL_LIFTOFF_SINGLE

	var left_uncontacted: bool = (not wheel_in_contact[0] or wheel_normal_load_fz_n[0] <= 0.0) and (not wheel_in_contact[2] or wheel_normal_load_fz_n[2] <= 0.0)
	var right_uncontacted: bool = (not wheel_in_contact[1] or wheel_normal_load_fz_n[1] <= 0.0) and (not wheel_in_contact[3] or wheel_normal_load_fz_n[3] <= 0.0)
	if left_uncontacted or right_uncontacted:
		step_flags |= VehicleDynamicsTypes.FaultFlag.WHEEL_LIFTOFF_SAME_SIDE

	var speed: float = sqrt(chassis_u_mps * chassis_u_mps + chassis_v_mps * chassis_v_mps)
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		if wheel_in_contact[i] and wheel_normal_load_fz_n[i] > 0.0:
			if wheel_normal_load_fz_n[i] > 1600.0 or (speed > 2.0 and wheel_normal_load_fz_n[i] < 150.0):
				step_flags |= VehicleDynamicsTypes.FaultFlag.TIRE_LOAD_OUT_OF_DOMAIN
			if absf(wheel_slip_angle_rad[i]) > 0.2618 or absf(wheel_slip_ratio[i]) > 0.25:
				step_flags |= VehicleDynamicsTypes.FaultFlag.TIRE_SLIP_OUT_OF_DOMAIN

	if speed < 0.20 and absf(state.throttle_cmd) <= 1e-3:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			if absf(wheel_fx_n[i]) > 300.0:
				step_flags |= VehicleDynamicsTypes.FaultFlag.LOW_SPEED_CHATTER
				break

	last_fault_flags = step_flags
	if (step_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_INPUT) != 0:
		non_finite_input_count += 1
	if (step_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_STATE) != 0:
		non_finite_state_count += 1
	if (step_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE) != 0:
		non_finite_force_count += 1
	if (step_flags & VehicleDynamicsTypes.FaultFlag.UNIMPLEMENTED_MODEL_CALLED) != 0:
		unimplemented_model_call_count += 1
	if (step_flags & VehicleDynamicsTypes.FaultFlag.CONTACT_LOSS_ALL_WHEELS) != 0:
		all_wheels_airborne_count += 1
	if (step_flags & VehicleDynamicsTypes.FaultFlag.TIRE_SATURATION) != 0:
		tire_saturation_count += 1

func to_dictionary() -> Dictionary:
	return {
		"timestamp_s": timestamp_s,
		"throttle_cmd": driver_throttle_cmd,
		"brake_cmd": driver_brake_cmd,
		"steer_cmd_rad": driver_steer_cmd_rad,
		"u_mps": chassis_u_mps,
		"v_mps": chassis_v_mps,
		"w_mps": chassis_w_mps,
		"ax_mps2": chassis_ax_mps2,
		"ay_mps2": chassis_ay_mps2,
		"az_mps2": chassis_az_mps2,
		"yaw_rate_rad_s": chassis_yaw_rate_rad_s,
		"roll_rad": chassis_roll_rad,
		"pitch_rad": chassis_pitch_rad,
		"beta_rad": chassis_beta_rad,
		"psi_rad": chassis_psi_rad,
		"x_m": chassis_x_m,
		"y_m": chassis_y_m,
		"wheel_steer_angle_rad": wheel_steer_angle_rad.duplicate(),
		"wheel_in_contact": wheel_in_contact.duplicate(),
		"wheel_fz_n": wheel_normal_load_fz_n.duplicate(),
		"wheel_kappa": wheel_slip_ratio.duplicate(),
		"wheel_alpha_rad": wheel_slip_angle_rad.duplicate(),
		"wheel_fx_n": wheel_fx_n.duplicate(),
		"wheel_fy_n": wheel_fy_n.duplicate(),
		"wheel_mz_nm": wheel_mz_nm.duplicate(),
		"wheel_compression_m": wheel_suspension_compression_m.duplicate(),
		"wheel_omega_rad_s": wheel_omega_rad_s.duplicate(),
		"wheel_rpm": wheel_rpm.duplicate(),
		"wheel_drive_torque_nm": wheel_drive_torque_nm.duplicate(),
		"wheel_brake_torque_nm": wheel_brake_torque_nm.duplicate(),
		"fault_flags": last_fault_flags,
	}
