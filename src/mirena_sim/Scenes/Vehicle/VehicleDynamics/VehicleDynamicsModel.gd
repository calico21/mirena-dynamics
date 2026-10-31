extends RefCounted
class_name VehicleDynamicsModel

## Pure, scene-independent orchestrator of the 4-wheel physical vehicle dynamics model.
##
## Implements the central 13-step vehicle-dynamics contract:
## 1. Obtain current chassis and wheel states (`ChassisModel.update_kinematics_from_world`).
## 2. Evaluate contact geometry and wheel-ground conditions (`WheelContact.evaluate_wheel_contact`).
## 3. Calculate aerodynamic drag and downforce distribution (`AerodynamicModel.evaluate_aerodynamics`)
##    and suspension displacement + vertical normal load Fz (`SuspensionModel.evaluate_all_corners`).
## 4. Calculate individual wheel steering angles (`SteeringModel.update_steering`).
## 5. Calculate each wheel's local velocity (`WheelAssembly.evaluate_kinematics`).
## 6. Calculate tire slip variables (`kappa`, `alpha`, `alpha_dot`).
## 7. Obtain tire forces and moments from the configured tire model (`TireModel.evaluate_wheel_forces`).
## 8. Calculate available powertrain drive torque (`PowertrainModel.evaluate_drive_torques`)
##    and brake torque (`BrakeSystem.evaluate_brake_torques`).
## 9. Apply combined-slip and contact-load limits consistently (inside `TireModel` & `SuspensionModel`).
## 10. Assemble individual forces and moments about the Chassis CoM (`ChassisModel.assemble_forces_and_moments`).
## 11. Integrate wheel rotational dynamics (`WheelAssembly.integrate_rotation`) and, when running in
##     `STANDALONE_BENCH` mode, advance chassis rigid-body states (`RigidBodyDynamics.integrate_standalone_step`).
##     (In `CUSTOM_RIGID_BODY_3D` mode, chassis force application is delegated to `GodotPhysicsAdapter`.)
## 12. Update internal states and diagnostics (`DynamicsDiagnostics.record_from_state`).
## 13. Expose the updated `VehicleState` to `ROSVehicleAdapter` for publication.

var config: VehicleDynamicsConfig
var state: VehicleState
var contact_query: ContactQuery
var diagnostics: DynamicsDiagnostics

var chassis_model: ChassisModel
var contact_model: WheelContact
var suspension_model: SuspensionModel
var steering_model: SteeringModel
var wheel_assemblies: Array[WheelAssembly] = []
var tire_model: TireModel
var powertrain_model: PowertrainModel
var brake_system: BrakeSystem
var aerodynamic_model: AerodynamicModel

func _init(p_config: VehicleDynamicsConfig = null) -> void:
	config = p_config if p_config != null else VehicleDynamicsConfig.new()
	config.ensure_subresources()

	state = VehicleState.new()
	contact_query = ContactQuery.new()
	diagnostics = DynamicsDiagnostics.new()

	chassis_model = ChassisModel.new()
	contact_model = WheelContact.new()
	suspension_model = SuspensionModel.new()
	steering_model = SteeringModel.new()
	tire_model = TireModel.new(config.tire_config.model_type)
	powertrain_model = PowertrainModel.new()
	brake_system = BrakeSystem.new()
	aerodynamic_model = AerodynamicModel.new()

	wheel_assemblies.clear()
	for corner in VehicleDynamicsTypes.ALL_CORNERS:
		wheel_assemblies.append(WheelAssembly.new(corner))

	reset()

## Reset the entire vehicle model, sub-models, and diagnostics to a clean initial state at `origin_godot`.
func reset(origin_godot: Transform3D = Transform3D.IDENTITY) -> void:
	config.ensure_subresources()
	state.reset(origin_godot, config.vehicle_parameters)
	steering_model.reset()
	tire_model.set_model_type(config.tire_config.model_type)
	tire_model.reset_all(state.wheels, config.tire_config)
	diagnostics.enabled = config.enable_diagnostics
	diagnostics.reset()

## Set driver or ROS control commands on `state` prior to stepping.
## `gas_input` follows `mirena_common/msg/CarControl`: `[-1, 1]` where `> 0` is throttle and `< 0` is braking.
func set_control_commands(gas_input: float, steer_angle_rad: float, ebs_triggered: bool = false) -> void:
	if not VehicleUnits.is_finite_scalar(gas_input) or not VehicleUnits.is_finite_scalar(steer_angle_rad):
		state.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_INPUT
		state.gas_cmd = 0.0
		state.throttle_cmd = 0.0
		state.brake_cmd = 1.0
		state.steer_cmd_rad = 0.0
		state.ebs_active = true
		return

	state.gas_cmd = clampf(gas_input, -1.0, 1.0)
	state.throttle_cmd = maxf(state.gas_cmd, 0.0)
	state.brake_cmd = maxf(-state.gas_cmd, 0.0)
	state.steer_cmd_rad = clampf(
		steer_angle_rad,
		-config.steering_config.max_steer_angle_rad,
		config.steering_config.max_steer_angle_rad
	)
	state.ebs_active = ebs_triggered

## Execute one complete physics step of duration `dt_s` (supporting configured substeps).
func step(dt_s: float, integrate_chassis_standalone: bool = false, constrain_planar_standalone: bool = false) -> void:
	state.fault_flags = VehicleDynamicsTypes.FaultFlag.NONE

	if not VehicleUnits.is_finite_scalar(dt_s) or dt_s <= 0.0:
		state.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_INPUT
		diagnostics.record_from_state(state)
		return

	if not config.is_valid():
		state.fault_flags |= VehicleDynamicsTypes.FaultFlag.INVALID_CONFIG
		diagnostics.record_from_state(state)
		return

	var substeps: int = maxi(1, config.physics_substeps)
	var sub_dt: float = dt_s / float(substeps)

	for _sub in range(substeps):
		_step_single(sub_dt, integrate_chassis_standalone, constrain_planar_standalone)

	diagnostics.enabled = config.enable_diagnostics
	diagnostics.record_from_state(state)

func _step_single(dt_s: float, integrate_chassis_standalone: bool, constrain_planar_standalone: bool) -> void:
	var params: VehicleParameters = config.vehicle_parameters

	# Step 1: Synchronize chassis body-frame kinematics from world transform & velocity
	chassis_model.update_kinematics_from_world(state, params)
	if (state.fault_flags & VehicleDynamicsTypes.FaultFlag.NON_FINITE_STATE) != 0:
		return

	# Step 2: Evaluate wheel-ground contact geometry for all 4 wheels
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		contact_model.evaluate_wheel_contact(
			state.wheels[i],
			state.global_transform_godot,
			state.linear_velocity_world_godot,
			state.angular_velocity_world_godot,
			params,
			config.suspension_config,
			contact_query
		)

	# Step 3a: Evaluate aerodynamic drag and downforce distribution
	var aero_res: Dictionary = aerodynamic_model.evaluate_aerodynamics(
		state.get_body_velocity_vehicle(),
		config.aerodynamics_config,
		params
	)
	state.aero_drag_force_n = aero_res["drag_force_n"]
	state.aero_downforce_front_n = aero_res["downforce_front_n"]
	state.aero_downforce_rear_n = aero_res["downforce_rear_n"]
	state.aero_force_vehicle = aero_res["aero_force_vehicle"]
	state.aero_moment_vehicle = aero_res["aero_moment_vehicle"]

	# Step 3b: Calculate suspension displacement, forces, dynamic camber, and vertical normal load Fz
	# In 6-DOF mode, aerodynamic downforce on the chassis is transmitted to the tires naturally through
	# strut spring compression; in planar-constrained standalone mode (where heave is locked), downforce
	# is allocated directly to wheel normal loads so high-speed grip is maintained without double-counting.
	var df_front_eval: float = state.aero_downforce_front_n if constrain_planar_standalone else 0.0
	var df_rear_eval: float = state.aero_downforce_rear_n if constrain_planar_standalone else 0.0
	suspension_model.evaluate_all_corners(
		state.wheels,
		state.roll_rad,
		df_front_eval,
		df_rear_eval,
		params,
		config.suspension_config,
		config.steering_config
	)

	# Step 4: Calculate individual wheel steering angles (FL, FR, RL, RR)
	steering_model.update_steering(
		state.steer_cmd_rad,
		dt_s,
		state.wheels,
		params,
		config.steering_config
	)

	# Steps 5, 6 & 7: Calculate each wheel's local velocity, slip variables, and tire forces/moments
	var v_com_veh: Vector3 = state.get_body_velocity_vehicle()
	var omega_veh: Vector3 = state.get_body_angular_velocity_vehicle()
	var vx_eps: float = config.tire_config.vx_min_regularization_mps
	var wheel_omegas: Array[float] = [0.0, 0.0, 0.0, 0.0]

	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var w: WheelState = state.wheels[i]
		var assembly: WheelAssembly = wheel_assemblies[i]
		assembly.evaluate_kinematics(w, v_com_veh, omega_veh, vx_eps)
		var tire_inputs: TireInputs = assembly.build_tire_inputs(w, config.tire_config, dt_s)
		w.tire_forces = tire_model.evaluate_wheel_forces(tire_inputs, w.tire_state, config.tire_config)
		state.fault_flags |= w.tire_forces.fault_flags
		wheel_omegas[i] = w.omega_rad_s

	# Step 8: Calculate available powertrain drive torque and brake torque
	powertrain_model.evaluate_drive_torques(
		state.throttle_cmd,
		state.u_mps,
		wheel_omegas,
		config.powertrain_config,
		state.wheels
	)
	brake_system.evaluate_brake_torques(
		state.brake_cmd,
		state.ebs_active,
		state.brake_state,
		params,
		config.brake_config,
		state.wheels
	)

	# Step 10: Assemble per-wheel and aerodynamic forces and moments about the Chassis CoM
	chassis_model.assemble_forces_and_moments(state, params)

	# Step 11: Integrate wheel rotational dynamics and (if standalone) chassis rigid-body motion
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		wheel_assemblies[i].integrate_rotation(
			state.wheels[i],
			params.wheel_inertia_kgm2,
			dt_s,
			config.brake_config.standstill_omega_eps_rad_s
		)
		state.brake_state.applied_torque_nm[i] = state.wheels[i].signed_brake_torque_nm

	if integrate_chassis_standalone:
		RigidBodyDynamics.integrate_standalone_step(
			state, params, dt_s, constrain_planar_standalone
		)
		chassis_model.update_kinematics_from_world(state, params)

	state.sim_time_s += dt_s
