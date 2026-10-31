extends RefCounted
class_name AerodynamicModel

## Evaluates aerodynamic drag, front/rear downforce, and aerodynamic pitch moment.
##
## Coordinate Frame (ISO 8855 / ROS 2 Vehicle Body Frame: +X forward, +Y left, +Z up):
## - Dynamic pressure: `q = 0.5 * rho * v_air^2`
## - Drag force: opposes velocity vector (`F_drag_x <= 0` when moving forward `u > 0`).
## - Total downforce: `F_down = q * Cl * A` (positive scalar magnitude; acts along `-Z_vehicle` on the chassis
##   or is distributed onto front/rear axle normal loads according to `aero_split_front`).
##
## Double-Counting Prevention Rule:
## In `VehicleDynamicsModel`, aerodynamic downforce is distributed into `aero_downforce_front_n` and
## `aero_downforce_rear_n` so that `SuspensionModel` includes it in the tire vertical contact normal loads `Fz`,
## while `aero_force_vehicle` carries ONLY the translational aerodynamic drag (`F_aero_x, F_aero_y, 0`)
## plus the direct vertical downforce `-F_down` on the chassis (Wait: if downforce pushes the chassis down `-F_down`
## and the ground pushes the tires up `+F_down`, in standalone 6-DOF equilibrium they balance; when planar or
## when suspension normal load `Fz` is computed algebraically or dynamically, see `PHYSICAL_MODEL.md` for exact ownership).

## Compute aerodynamic forces and front/rear downforce split for a given body velocity vector `v_body_vehicle` (m/s).
## Returns Dictionary:
##   {
##     "drag_force_n": float,             # Scalar drag magnitude (N, >= 0)
##     "downforce_total_n": float,        # Total downward force magnitude (N, >= 0)
##     "downforce_front_n": float,        # Downforce allocated to front axle (N, >= 0)
##     "downforce_rear_n": float,         # Downforce allocated to rear axle (N, >= 0)
##     "aero_force_vehicle": Vector3,     # Force vector in Vehicle Body Frame (N)
##     "aero_moment_vehicle": Vector3,    # Moment vector about CoM in Vehicle Body Frame (N*m)
##   }
func evaluate_aerodynamics(
	v_body_vehicle: Vector3,
	config: AerodynamicConfiguration,
	params: VehicleParameters = null
) -> Dictionary:
	if config == null or not config.is_valid() or not VehicleUnits.is_finite_vec3(v_body_vehicle):
		return {
			"drag_force_n": 0.0,
			"downforce_total_n": 0.0,
			"downforce_front_n": 0.0,
			"downforce_rear_n": 0.0,
			"aero_force_vehicle": Vector3.ZERO,
			"aero_moment_vehicle": Vector3.ZERO,
		}

	var speed: float = v_body_vehicle.length()
	if speed <= config.min_airspeed_mps:
		return {
			"drag_force_n": 0.0,
			"downforce_total_n": 0.0,
			"downforce_front_n": 0.0,
			"downforce_rear_n": 0.0,
			"aero_force_vehicle": Vector3.ZERO,
			"aero_moment_vehicle": Vector3.ZERO,
		}

	var q_pa: float = 0.5 * config.air_density_kgpm3 * speed * speed
	var drag_mag_n: float = q_pa * config.get_cd_area_m2()
	var downforce_total_n: float = q_pa * config.get_cl_area_m2()

	var df_front_n: float = downforce_total_n * config.aero_split_front
	var df_rear_n: float = downforce_total_n * (1.0 - config.aero_split_front)

	# Drag opposes the 3D body velocity vector; downforce acts along -Z_vehicle
	var drag_vec_veh: Vector3 = -drag_mag_n * (v_body_vehicle / speed)
	var aero_force_veh: Vector3 = Vector3(drag_vec_veh.x, drag_vec_veh.y, drag_vec_veh.z - downforce_total_n)

	# Pitch moment about CoM (about +Y_vehicle: positive pitch is nose down, so front downforce creates +My)
	var pitch_moment_nm: float = 0.0
	if params != null:
		pitch_moment_nm = df_front_n * params.lf_m - df_rear_n * params.lr_m

	return {
		"drag_force_n": drag_mag_n,
		"downforce_total_n": downforce_total_n,
		"downforce_front_n": df_front_n,
		"downforce_rear_n": df_rear_n,
		"aero_force_vehicle": aero_force_veh,
		"aero_moment_vehicle": Vector3(0.0, pitch_moment_nm, 0.0),
	}
