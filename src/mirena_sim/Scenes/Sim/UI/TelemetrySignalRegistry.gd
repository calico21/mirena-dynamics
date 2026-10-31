extends RefCounted
class_name TelemetrySignalRegistry

## Central catalog and metadata registry of all telemetry channels available in MirenaSim.
## Populated from live vehicle snapshot (car.get_live_dynamics_snapshot()), DynamicsLogger,
## and modular vehicle state.

enum SignalCategory {
	VEHICLE_MOTION = 0,
	WHEELS = 1,
	TIRES = 2,
	SUSPENSION = 3,
	POWERTRAIN = 4,
	AERODYNAMICS = 5,
}

const CATEGORY_NAMES: Dictionary = {
	SignalCategory.VEHICLE_MOTION: "Vehicle Motion",
	SignalCategory.WHEELS: "Wheels",
	SignalCategory.TIRES: "Tires",
	SignalCategory.SUSPENSION: "Suspension",
	SignalCategory.POWERTRAIN: "Powertrain & Control",
	SignalCategory.AERODYNAMICS: "Aerodynamics",
}

class SignalDef:
	var id: String
	var name: String
	var category: int
	var unit: String
	var format_str: String
	var default_color: Color
	var default_min: float
	var default_max: float
	var modular_only: bool
	var tooltip: String

	func _init(
		p_id: String,
		p_name: String,
		p_cat: int,
		p_unit: String,
		p_fmt: String,
		p_col: Color,
		p_min: float,
		p_max: float,
		p_mod_only: bool,
		p_tooltip: String = ""
	) -> void:
		id = p_id
		name = p_name
		category = p_cat
		unit = p_unit
		format_str = p_fmt
		default_color = p_col
		default_min = p_min
		default_max = p_max
		modular_only = p_mod_only
		tooltip = p_tooltip

static var _signals: Dictionary = {}
static var _initialized: bool = false

static func _ensure_initialized() -> void:
	if _initialized:
		return
	_initialized = true
	_signals.clear()

	# --- 1. VEHICLE MOTION ---
	_add("chassis.speed_kmh", "Vehicle Speed", SignalCategory.VEHICLE_MOTION, "km/h", "%.1f", Color(0.35, 0.95, 0.90), 0.0, 140.0, false, "Total vehicle ground speed sqrt(u^2 + v^2)")
	_add("chassis.u", "Longitudinal Velocity u", SignalCategory.VEHICLE_MOTION, "m/s", "%+.2f", Color(0.40, 0.92, 0.60), -5.0, 40.0, false, "Forward velocity along vehicle body +X axis")
	_add("chassis.v", "Lateral Velocity v", SignalCategory.VEHICLE_MOTION, "m/s", "%+.2f", Color(0.95, 0.70, 0.30), -10.0, 10.0, false, "Lateral velocity along vehicle body +Y axis (+Y left)")
	_add("imu.ax", "Longitudinal Accel AX", SignalCategory.VEHICLE_MOTION, "m/s²", "%+.2f", Color(0.40, 0.92, 0.65), -15.0, 15.0, false, "Kinematic longitudinal acceleration (+X forward, negative braking)")
	_add("imu.ay", "Lateral Accel AY", SignalCategory.VEHICLE_MOTION, "m/s²", "%+.2f", Color(0.40, 0.88, 0.95), -20.0, 20.0, false, "Kinematic lateral acceleration (+Y leftward, negative rightward)")
	_add("imu.az", "Vertical Accel AZ", SignalCategory.VEHICLE_MOTION, "m/s²", "%+.2f", Color(0.75, 0.82, 0.90), -10.0, 10.0, false, "Kinematic vertical acceleration (+Z up, 0 at level static equilibrium)")
	_add("imu.gz", "Yaw Rate r", SignalCategory.VEHICLE_MOTION, "rad/s", "%+.3f", Color(0.85, 0.65, 0.95), -2.5, 2.5, false, "Chassis yaw angular velocity (+Z counter-clockwise / left turn)")
	_add("imu.gx", "Roll Rate p", SignalCategory.VEHICLE_MOTION, "rad/s", "%+.3f", Color(0.95, 0.55, 0.65), -2.0, 2.0, false, "Chassis roll angular velocity (+X roll right down)")
	_add("imu.gy", "Pitch Rate q", SignalCategory.VEHICLE_MOTION, "rad/s", "%+.3f", Color(0.95, 0.85, 0.45), -2.0, 2.0, false, "Chassis pitch angular velocity (+Y pitch nose down)")
	_add("chassis.beta_deg", "Sideslip Angle β", SignalCategory.VEHICLE_MOTION, "deg", "%+.1f", Color(0.92, 0.60, 0.35), -15.0, 15.0, false, "Vehicle centre-of-gravity sideslip angle atan2(v, u)")
	_add("chassis.roll_deg", "Chassis Roll Angle", SignalCategory.VEHICLE_MOTION, "deg", "%+.1f", Color(0.60, 0.80, 0.95), -8.0, 8.0, false, "Sprung mass roll angle (+ right side down)")
	_add("chassis.pitch_deg", "Chassis Pitch Angle", SignalCategory.VEHICLE_MOTION, "deg", "%+.1f", Color(0.80, 0.90, 0.60), -8.0, 8.0, false, "Sprung mass pitch angle (+ nose up)")

	# --- 2. WHEELS ---
	_add("wheel.fl_rpm", "FL Wheel Speed", SignalCategory.WHEELS, "RPM", "%.0f", Color(0.40, 0.85, 0.95), 0.0, 1600.0, false, "Front-Left wheel rotational angular speed in RPM")
	_add("wheel.fr_rpm", "FR Wheel Speed", SignalCategory.WHEELS, "RPM", "%.0f", Color(0.50, 0.95, 0.80), 0.0, 1600.0, false, "Front-Right wheel rotational angular speed in RPM")
	_add("wheel.rl_rpm", "RL Wheel Speed", SignalCategory.WHEELS, "RPM", "%.0f", Color(0.95, 0.75, 0.30), 0.0, 1600.0, false, "Rear-Left driven wheel rotational angular speed in RPM")
	_add("wheel.rr_rpm", "RR Wheel Speed", SignalCategory.WHEELS, "RPM", "%.0f", Color(0.95, 0.50, 0.40), 0.0, 1600.0, false, "Rear-Right driven wheel rotational angular speed in RPM")

	# --- 3. TIRES ---
	_add("tire.fz_fl", "FL Normal Load FZ", SignalCategory.TIRES, "N", "%.0f", Color(0.40, 0.85, 0.95), 0.0, 2000.0, true, "Front-Left tire vertical load FZ")
	_add("tire.fz_fr", "FR Normal Load FZ", SignalCategory.TIRES, "N", "%.0f", Color(0.50, 0.95, 0.80), 0.0, 2000.0, true, "Front-Right tire vertical load FZ")
	_add("tire.fz_rl", "RL Normal Load FZ", SignalCategory.TIRES, "N", "%.0f", Color(0.95, 0.75, 0.30), 0.0, 2000.0, true, "Rear-Left tire vertical load FZ")
	_add("tire.fz_rr", "RR Normal Load FZ", SignalCategory.TIRES, "N", "%.0f", Color(0.95, 0.50, 0.40), 0.0, 2000.0, true, "Rear-Right tire vertical load FZ")

	_add("tire.fy_fl", "FL Lateral Force FY", SignalCategory.TIRES, "N", "%+.0f", Color(0.40, 0.85, 0.95), -2500.0, 2500.0, true, "Front-Left tire lateral cornering force FY in wheel frame")
	_add("tire.fy_fr", "FR Lateral Force FY", SignalCategory.TIRES, "N", "%+.0f", Color(0.50, 0.95, 0.80), -2500.0, 2500.0, true, "Front-Right tire lateral cornering force FY in wheel frame")
	_add("tire.fy_rl", "RL Lateral Force FY", SignalCategory.TIRES, "N", "%+.0f", Color(0.95, 0.75, 0.30), -2500.0, 2500.0, true, "Rear-Left tire lateral cornering force FY in wheel frame")
	_add("tire.fy_rr", "RR Lateral Force FY", SignalCategory.TIRES, "N", "%+.0f", Color(0.95, 0.50, 0.40), -2500.0, 2500.0, true, "Rear-Right tire lateral cornering force FY in wheel frame")

	_add("tire.fx_fl", "FL Longitudinal Force FX", SignalCategory.TIRES, "N", "%+.0f", Color(0.40, 0.85, 0.95), -2000.0, 2000.0, true, "Front-Left tire braking force FX in wheel frame")
	_add("tire.fx_fr", "FR Longitudinal Force FX", SignalCategory.TIRES, "N", "%+.0f", Color(0.50, 0.95, 0.80), -2000.0, 2000.0, true, "Front-Right tire braking force FX in wheel frame")
	_add("tire.fx_rl", "RL Longitudinal Force FX", SignalCategory.TIRES, "N", "%+.0f", Color(0.95, 0.75, 0.30), -2500.0, 2500.0, true, "Rear-Left tire traction/braking force FX in wheel frame")
	_add("tire.fx_rr", "RR Longitudinal Force FX", SignalCategory.TIRES, "N", "%+.0f", Color(0.95, 0.50, 0.40), -2500.0, 2500.0, true, "Rear-Right tire traction/braking force FX in wheel frame")

	_add("tire.mz_fl", "FL Aligning Moment MZ", SignalCategory.TIRES, "Nm", "%+.1f", Color(0.40, 0.85, 0.95), -80.0, 80.0, true, "Front-Left self-aligning steering moment MZ")
	_add("tire.mz_fr", "FR Aligning Moment MZ", SignalCategory.TIRES, "Nm", "%+.1f", Color(0.50, 0.95, 0.80), -80.0, 80.0, true, "Front-Right self-aligning steering moment MZ")
	_add("tire.mz_rl", "RL Aligning Moment MZ", SignalCategory.TIRES, "Nm", "%+.1f", Color(0.95, 0.75, 0.30), -80.0, 80.0, true, "Rear-Left tire aligning moment MZ")
	_add("tire.mz_rr", "RR Aligning Moment MZ", SignalCategory.TIRES, "Nm", "%+.1f", Color(0.95, 0.50, 0.40), -80.0, 80.0, true, "Rear-Right tire aligning moment MZ")

	_add("tire.alpha_fl", "FL Slip Angle Alpha", SignalCategory.TIRES, "deg", "%+.2f", Color(0.40, 0.85, 0.95), -15.0, 15.0, true, "Front-Left tire lateral slip angle Alpha")
	_add("tire.alpha_fr", "FR Slip Angle Alpha", SignalCategory.TIRES, "deg", "%+.2f", Color(0.50, 0.95, 0.80), -15.0, 15.0, true, "Front-Right tire lateral slip angle Alpha")
	_add("tire.alpha_rl", "RL Slip Angle Alpha", SignalCategory.TIRES, "deg", "%+.2f", Color(0.95, 0.75, 0.30), -15.0, 15.0, true, "Rear-Left tire lateral slip angle Alpha")
	_add("tire.alpha_rr", "RR Slip Angle Alpha", SignalCategory.TIRES, "deg", "%+.2f", Color(0.95, 0.50, 0.40), -15.0, 15.0, true, "Rear-Right tire lateral slip angle Alpha")

	_add("tire.kappa_fl", "FL Slip Ratio Kappa", SignalCategory.TIRES, "[-]", "%+.3f", Color(0.40, 0.85, 0.95), -0.5, 0.5, true, "Front-Left tire longitudinal slip ratio Kappa")
	_add("tire.kappa_fr", "FR Slip Ratio Kappa", SignalCategory.TIRES, "[-]", "%+.3f", Color(0.50, 0.95, 0.80), -0.5, 0.5, true, "Front-Right tire longitudinal slip ratio Kappa")
	_add("tire.kappa_rl", "RL Slip Ratio Kappa", SignalCategory.TIRES, "[-]", "%+.3f", Color(0.95, 0.75, 0.30), -0.5, 0.5, true, "Rear-Left tire longitudinal slip ratio Kappa")
	_add("tire.kappa_rr", "RR Slip Ratio Kappa", SignalCategory.TIRES, "[-]", "%+.3f", Color(0.95, 0.50, 0.40), -0.5, 0.5, true, "Rear-Right tire longitudinal slip ratio Kappa")

	_add("tire.util_fl", "FL Combined Utilization", SignalCategory.TIRES, "[-]", "%.2f", Color(0.40, 0.85, 0.95), 0.0, 1.2, true, "Front-Left friction ellipse utilization (0 = free, 1.0 = peak grip)")
	_add("tire.util_fr", "FR Combined Utilization", SignalCategory.TIRES, "[-]", "%.2f", Color(0.50, 0.95, 0.80), 0.0, 1.2, true, "Front-Right friction ellipse utilization")
	_add("tire.util_rl", "RL Combined Utilization", SignalCategory.TIRES, "[-]", "%.2f", Color(0.95, 0.75, 0.30), 0.0, 1.2, true, "Rear-Left friction ellipse utilization")
	_add("tire.util_rr", "RR Combined Utilization", SignalCategory.TIRES, "[-]", "%.2f", Color(0.95, 0.50, 0.40), 0.0, 1.2, true, "Rear-Right friction ellipse utilization")

	# --- 4. SUSPENSION ---
	_add("susp.heave_fl", "FL Suspension Heave", SignalCategory.SUSPENSION, "mm", "%+.1f", Color(0.40, 0.85, 0.95), -40.0, 40.0, true, "Front-Left suspension strut displacement (+ compression, - droop)")
	_add("susp.heave_fr", "FR Suspension Heave", SignalCategory.SUSPENSION, "mm", "%+.1f", Color(0.50, 0.95, 0.80), -40.0, 40.0, true, "Front-Right suspension strut displacement")
	_add("susp.heave_rl", "RL Suspension Heave", SignalCategory.SUSPENSION, "mm", "%+.1f", Color(0.95, 0.75, 0.30), -40.0, 40.0, true, "Rear-Left suspension strut displacement")
	_add("susp.heave_rr", "RR Suspension Heave", SignalCategory.SUSPENSION, "mm", "%+.1f", Color(0.95, 0.50, 0.40), -40.0, 40.0, true, "Rear-Right suspension strut displacement")

	_add("susp.f_spring_fl", "FL Spring Force", SignalCategory.SUSPENSION, "N", "%.0f", Color(0.40, 0.85, 0.95), 0.0, 2500.0, true, "Front-Left suspension helical spring force")
	_add("susp.f_damper_fl", "FL Damper Force", SignalCategory.SUSPENSION, "N", "%+.0f", Color(0.95, 0.60, 0.35), -1500.0, 1500.0, true, "Front-Left hydraulic damper damping force")
	_add("susp.f_arb_fl", "FL Anti-Roll Bar Force", SignalCategory.SUSPENSION, "N", "%+.0f", Color(0.85, 0.65, 0.95), -1200.0, 1200.0, true, "Front anti-roll bar torsional transfer force on FL")

	# --- 5. POWERTRAIN & CONTROLS ---
	_add("control.gas", "Throttle / Gas Demand", SignalCategory.POWERTRAIN, "[-]", "%.2f", Color(0.30, 0.95, 0.60), 0.0, 1.0, false, "Driver normalized throttle input [0..1]")
	_add("control.brake", "Brake Demand", SignalCategory.POWERTRAIN, "[-]", "%.2f", Color(0.95, 0.40, 0.35), 0.0, 1.0, false, "Driver normalized service brake input [0..1]")
	_add("control.steer_deg", "Steering Angle", SignalCategory.POWERTRAIN, "deg", "%+.1f", Color(0.40, 0.85, 0.95), -25.0, 25.0, false, "Requested front road-wheel steering angle in degrees (+ left)")
	_add("powertrain.drive_torque_rl", "RL Drive Torque", SignalCategory.POWERTRAIN, "Nm", "%.1f", Color(0.95, 0.75, 0.30), 0.0, 400.0, true, "Rear-Left axle motor drive torque")
	_add("powertrain.drive_torque_rr", "RR Drive Torque", SignalCategory.POWERTRAIN, "Nm", "%.1f", Color(0.95, 0.50, 0.40), 0.0, 400.0, true, "Rear-Right axle motor drive torque")

	# --- 6. AERODYNAMICS ---
	_add("aero.drag_n", "Aero Drag Force", SignalCategory.AERODYNAMICS, "N", "%.0f", Color(0.95, 0.60, 0.30), 0.0, 1500.0, true, "Total aerodynamic drag resistance force on vehicle")
	_add("aero.downforce_front_n", "Front Downforce", SignalCategory.AERODYNAMICS, "N", "%.0f", Color(0.40, 0.80, 0.95), 0.0, 1500.0, true, "Front aerodynamic wing/body downforce")
	_add("aero.downforce_rear_n", "Rear Downforce", SignalCategory.AERODYNAMICS, "N", "%.0f", Color(0.30, 0.95, 0.70), 0.0, 2000.0, true, "Rear aerodynamic wing downforce")

static func _add(
	id: String,
	name: String,
	cat: int,
	unit: String,
	fmt: String,
	col: Color,
	min_v: float,
	max_v: float,
	mod_only: bool,
	tooltip: String
) -> void:
	_signals[id] = SignalDef.new(id, name, cat, unit, fmt, col, min_v, max_v, mod_only, tooltip)

static func get_signal(id: String) -> SignalDef:
	_ensure_initialized()
	return _signals.get(id, null)

static func get_all_signals() -> Array:
	_ensure_initialized()
	return _signals.values()

static func get_signals_in_category(category: int) -> Array[SignalDef]:
	_ensure_initialized()
	var res: Array[SignalDef] = []
	for sig in _signals.values():
		if sig.category == category:
			res.append(sig)
	return res

static func search_signals(query: String) -> Array[SignalDef]:
	_ensure_initialized()
	var res: Array[SignalDef] = []
	var q := query.to_lower().strip_edges()
	for sig in _signals.values():
		if q.is_empty() or sig.name.to_lower().contains(q) or sig.id.to_lower().contains(q) or sig.unit.to_lower().contains(q):
			res.append(sig)
	return res

## Extract numerical value for `channel_id` from live snapshot Dictionary
static func sample_value_from_snapshot(snap: Dictionary, channel_id: String) -> float:
	match channel_id:
		"chassis.speed_kmh":
			var u: float = float(snap.get("chassis.u", 0.0))
			var v: float = float(snap.get("chassis.v", 0.0))
			return sqrt(u * u + v * v) * 3.6
		"control.steer_deg":
			var st_rad: float = float(snap.get("control.steer_rad", 0.0))
			return rad_to_deg(st_rad)
		"susp.heave_fl":
			return float(snap.get("susp.heave_fl", 0.0)) * 1000.0 # m to mm
		"susp.heave_fr":
			return float(snap.get("susp.heave_fr", 0.0)) * 1000.0
		"susp.heave_rl":
			return float(snap.get("susp.heave_rl", 0.0)) * 1000.0
		"susp.heave_rr":
			return float(snap.get("susp.heave_rr", 0.0)) * 1000.0
		"tire.alpha_fl":
			return rad_to_deg(float(snap.get("tire.alpha_fl", 0.0)))
		"tire.alpha_fr":
			return rad_to_deg(float(snap.get("tire.alpha_fr", 0.0)))
		"tire.alpha_rl":
			return rad_to_deg(float(snap.get("tire.alpha_rl", 0.0)))
		"tire.alpha_rr":
			return rad_to_deg(float(snap.get("tire.alpha_rr", 0.0)))
		"tire.gamma_fl":
			return rad_to_deg(float(snap.get("tire.gamma_fl", 0.0)))
		"tire.gamma_fr":
			return rad_to_deg(float(snap.get("tire.gamma_fr", 0.0)))
		"tire.gamma_rl":
			return rad_to_deg(float(snap.get("tire.gamma_rl", 0.0)))
		"tire.gamma_rr":
			return rad_to_deg(float(snap.get("tire.gamma_rr", 0.0)))
		_:
			return float(snap.get(channel_id, 0.0))
