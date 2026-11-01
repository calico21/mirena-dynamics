extends RefCounted
class_name AerodynamicModel

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

	var drag_vec_veh: Vector3 = -drag_mag_n * (v_body_vehicle / speed)
	var aero_force_veh: Vector3 = Vector3(drag_vec_veh.x, drag_vec_veh.y, drag_vec_veh.z - downforce_total_n)

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
