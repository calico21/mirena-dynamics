extends RefCounted
class_name ChassisModel

func update_kinematics_from_world(state: VehicleState, params: VehicleParameters) -> void:
	if state == null:
		return
	if not VehicleUnits.is_finite_transform(state.global_transform_godot) \
			or not VehicleUnits.is_finite_vec3(state.linear_velocity_world_godot) \
			or not VehicleUnits.is_finite_vec3(state.angular_velocity_world_godot):
		state.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_STATE
		return

	var odom_pose: Vector3 = VehicleCoordinates.compute_ros_odom_pose(
		state.origin_godot, state.global_transform_godot
	)
	state.x_odom_m = odom_pose.x
	state.y_odom_m = odom_pose.y
	state.psi_odom_rad = odom_pose.z

	var odom_tf: Transform3D = state.origin_godot.affine_inverse() * state.global_transform_godot
	var rpy: Vector3 = VehicleCoordinates.basis_godot_to_vehicle_rpy(odom_tf.basis)
	state.roll_rad = rpy.x
	state.pitch_rad = rpy.y
	state.yaw_rad = rpy.z

	var vel_veh: Vector3 = VehicleCoordinates.world_godot_to_vehicle_body(
		state.linear_velocity_world_godot, state.global_transform_godot.basis
	)
	state.u_mps = vel_veh.x
	state.v_mps = vel_veh.y
	state.w_mps = vel_veh.z

	var local_omega_godot: Vector3 = state.global_transform_godot.basis.inverse() * state.angular_velocity_world_godot
	var omega_veh: Vector3 = VehicleCoordinates.godot_to_vehicle_angular(local_omega_godot)
	state.p_rad_s = omega_veh.x
	state.q_rad_s = omega_veh.y
	state.r_rad_s = omega_veh.z

	if params != null:
		for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
			var corner: VehicleDynamicsTypes.WheelCorner = VehicleDynamicsTypes.ALL_CORNERS[i]
			var w: WheelState = state.wheels[i]
			w.hub_offset_from_com_vehicle = params.get_wheel_offset_from_com_vehicle(corner)
			w.contact_offset_from_com_vehicle = w.hub_offset_from_com_vehicle - Vector3(0.0, 0.0, w.effective_radius_m)

func assemble_forces_and_moments(state: VehicleState, params: VehicleParameters) -> void:
	if state == null or params == null:
		return

	var total_force_veh: Vector3 = Vector3.ZERO
	var total_moment_veh: Vector3 = Vector3.ZERO

	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		var w: WheelState = state.wheels[i]

		var wheel_planar_force: Vector3 = Vector3(w.tire_forces.fx_n, w.tire_forces.fy_n, 0.0)
		var wheel_moment: Vector3 = w.tire_forces.get_moment_vector_wheel()

		var assembled: Dictionary = RigidBodyDynamics.assemble_wheel_force_and_moment_vehicle(
			wheel_planar_force,
			wheel_moment,
			w.steering_angle_rad,
			w.contact_offset_from_com_vehicle
		)
		var f_veh: Vector3 = assembled["force_vehicle"]

		var h_rc_m: float = 0.04 if VehicleDynamicsTypes.is_front_corner(w.corner) else 0.06
		var m_veh_planar: Vector3 = assembled["moment_vehicle"]
		m_veh_planar.x -= f_veh.y * h_rc_m

		var fz_sprung: float = w.suspension_state.total_suspension_force_n if w.contact_state.in_contact else 0.0
		f_veh.z += fz_sprung

		var vert_moment_veh: Vector3 = w.hub_offset_from_com_vehicle.cross(Vector3(0.0, 0.0, fz_sprung))
		var m_veh: Vector3 = m_veh_planar + vert_moment_veh

		if not VehicleUnits.is_finite_vec3(f_veh) or not VehicleUnits.is_finite_vec3(m_veh):
			state.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE
			w.force_vehicle = Vector3.ZERO
			w.moment_about_com_vehicle = Vector3.ZERO
			continue

		w.force_vehicle = f_veh
		w.moment_about_com_vehicle = m_veh
		total_force_veh += f_veh
		total_moment_veh += m_veh

	var contacts_count: int = 0
	for i in range(VehicleDynamicsTypes.WHEEL_COUNT):
		if state.wheels[i].contact_state.in_contact:
			contacts_count += 1
	if contacts_count > 0 and state.throttle_cmd <= 1e-4:
		var planar_speed: float = sqrt(state.u_mps * state.u_mps + state.v_mps * state.v_mps)
		if planar_speed < 0.20:
			var stand_weight: float = (1.0 - planar_speed / 0.20) * (float(contacts_count) / 4.0)
			total_force_veh.x += -1200.0 * stand_weight * state.u_mps
			total_force_veh.y += -2400.0 * stand_weight * state.v_mps
			total_moment_veh.z += -450.0 * stand_weight * state.r_rad_s

	if VehicleUnits.is_finite_vec3(state.aero_force_vehicle) and VehicleUnits.is_finite_vec3(state.aero_moment_vehicle):
		total_force_veh += state.aero_force_vehicle
		total_moment_veh += state.aero_moment_vehicle
	else:
		state.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE

	state.total_force_vehicle = total_force_veh
	state.total_moment_vehicle = total_moment_veh

	state.total_force_world_godot = VehicleCoordinates.vehicle_body_to_world_godot(
		total_force_veh, state.global_transform_godot.basis
	)
	var moment_local_godot: Vector3 = VehicleCoordinates.vehicle_to_godot_angular(total_moment_veh)
	state.total_moment_world_godot = state.global_transform_godot.basis * moment_local_godot

	var accel_veh: Vector3 = RigidBodyDynamics.compute_linear_acceleration_vehicle(
		total_force_veh, params.total_mass_kg
	)
	var gravity_godot: Vector3 = Vector3(0.0, -params.gravity_mps2, 0.0)
	var gravity_veh: Vector3 = VehicleCoordinates.world_godot_to_vehicle_body(
		gravity_godot, state.global_transform_godot.basis
	)
	var kin_accel_veh: Vector3 = accel_veh + gravity_veh
	state.ax_mps2 = kin_accel_veh.x
	state.ay_mps2 = kin_accel_veh.y
	state.az_mps2 = kin_accel_veh.z
