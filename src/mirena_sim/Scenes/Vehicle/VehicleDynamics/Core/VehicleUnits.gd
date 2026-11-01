extends RefCounted
class_name VehicleUnits

const STANDARD_GRAVITY_MPS2: float = 9.81
const STANDARD_AIR_DENSITY_KGPM3: float = 1.225

const RAD_PER_SEC_TO_RPM: float = 60.0 / TAU
const RPM_TO_RAD_PER_SEC: float = TAU / 60.0

const BAR_TO_KPA: float = 100.0
const KPA_TO_BAR: float = 0.01
const KPA_TO_PA: float = 1000.0
const PA_TO_KPA: float = 0.001

const MPS_TO_KMH: float = 3.6
const KMH_TO_MPS: float = 1.0 / 3.6

const EPSILON_SMALL: float = 1e-6
const EPSILON_STRICT: float = 1e-9

static func rad_s_to_rpm(omega_rad_s: float) -> float:
	return omega_rad_s * RAD_PER_SEC_TO_RPM

static func rpm_to_rad_s(rpm: float) -> float:
	return rpm * RPM_TO_RAD_PER_SEC

static func wheel_omega_to_linear_mps(omega_rad_s: float, effective_radius_m: float) -> float:
	return omega_rad_s * effective_radius_m

static func wheel_rpm_to_linear_mps(rpm: float, effective_radius_m: float) -> float:
	return (rpm * RPM_TO_RAD_PER_SEC) * effective_radius_m

static func quantize_rpm(rpm: float, step_rpm: float = 1.0) -> float:
	if step_rpm <= 0.0:
		return rpm
	return roundf(rpm / step_rpm) * step_rpm

static func bar_to_kpa(pressure_bar: float) -> float:
	return pressure_bar * BAR_TO_KPA

static func kpa_to_bar(pressure_kpa: float) -> float:
	return pressure_kpa * KPA_TO_BAR

static func deg_to_rad_scalar(angle_deg: float) -> float:
	return deg_to_rad(angle_deg)

static func rad_to_deg_scalar(angle_rad: float) -> float:
	return rad_to_deg(angle_rad)

static func celsius_to_kelvin(temp_c: float) -> float:
	return temp_c + 273.15

static func is_finite_scalar(val: float) -> bool:
	return not is_nan(val) and not is_inf(val)

static func is_finite_vec3(v: Vector3) -> bool:
	return is_finite_scalar(v.x) and is_finite_scalar(v.y) and is_finite_scalar(v.z)

static func is_finite_basis(b: Basis) -> bool:
	return is_finite_vec3(b.x) and is_finite_vec3(b.y) and is_finite_vec3(b.z)

static func is_finite_transform(t: Transform3D) -> bool:
	return is_finite_basis(t.basis) and is_finite_vec3(t.origin)
