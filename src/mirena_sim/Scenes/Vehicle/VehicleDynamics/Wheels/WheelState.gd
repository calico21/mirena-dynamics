extends RefCounted
class_name WheelState

var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL

var steering_angle_rad: float = 0.0

var camber_angle_rad: float = 0.0

var omega_rad_s: float = 0.0

var angular_accel_rad_s2: float = 0.0

var spin_angle_rad: float = 0.0

var effective_radius_m: float = 0.20

var hub_offset_from_com_vehicle: Vector3 = Vector3.ZERO

var contact_offset_from_com_vehicle: Vector3 = Vector3.ZERO

var hub_velocity_vehicle: Vector3 = Vector3.ZERO

var contact_velocity_wheel: Vector3 = Vector3.ZERO

var slip_ratio_kappa: float = 0.0

var slip_angle_alpha_rad: float = 0.0

var drive_torque_nm: float = 0.0

var brake_torque_magnitude_nm: float = 0.0

var signed_brake_torque_nm: float = 0.0

var net_wheel_torque_nm: float = 0.0

var force_vehicle: Vector3 = Vector3.ZERO

var moment_about_com_vehicle: Vector3 = Vector3.ZERO

var contact_state: ContactState
var suspension_state: SuspensionState
var tire_state: TireState
var tire_forces: TireForces

func _init(p_corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.WheelCorner.FL) -> void:
	corner = p_corner
	contact_state = ContactState.new(p_corner)
	suspension_state = SuspensionState.new(p_corner)
	tire_state = TireState.new(p_corner)
	tire_forces = TireForces.new()
	reset()

func reset(nominal_radius_m: float = 0.20, static_fz_n: float = 0.0, static_camber_rad: float = 0.0) -> void:
	steering_angle_rad = 0.0
	camber_angle_rad = static_camber_rad
	omega_rad_s = 0.0
	angular_accel_rad_s2 = 0.0
	spin_angle_rad = 0.0
	effective_radius_m = nominal_radius_m
	hub_velocity_vehicle = Vector3.ZERO
	contact_velocity_wheel = Vector3.ZERO
	slip_ratio_kappa = 0.0
	slip_angle_alpha_rad = 0.0
	drive_torque_nm = 0.0
	brake_torque_magnitude_nm = 0.0
	signed_brake_torque_nm = 0.0
	net_wheel_torque_nm = 0.0
	force_vehicle = Vector3.ZERO
	moment_about_com_vehicle = Vector3.ZERO
	contact_state.reset(nominal_radius_m)
	suspension_state.reset(static_fz_n, static_camber_rad)
	tire_state.reset()
	tire_forces.zero()

func get_rpm() -> float:
	return VehicleUnits.rad_s_to_rpm(omega_rad_s)

func is_valid() -> bool:
	return (
		VehicleUnits.is_finite_scalar(steering_angle_rad)
		and VehicleUnits.is_finite_scalar(camber_angle_rad)
		and VehicleUnits.is_finite_scalar(omega_rad_s)
		and VehicleUnits.is_finite_scalar(angular_accel_rad_s2)
		and VehicleUnits.is_finite_scalar(effective_radius_m)
		and effective_radius_m > 0.0
		and VehicleUnits.is_finite_vec3(hub_velocity_vehicle)
		and VehicleUnits.is_finite_vec3(contact_velocity_wheel)
		and VehicleUnits.is_finite_scalar(slip_ratio_kappa)
		and VehicleUnits.is_finite_scalar(slip_angle_alpha_rad)
		and VehicleUnits.is_finite_vec3(force_vehicle)
		and VehicleUnits.is_finite_vec3(moment_about_com_vehicle)
		and contact_state.is_valid()
		and suspension_state.is_valid()
		and tire_state.is_valid()
		and tire_forces.is_finite()
	)
