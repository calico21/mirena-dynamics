extends Node
class_name VehicleDynamicsController

@export var config: VehicleDynamicsConfig = preload("res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_default.tres")
@export var host_body_path: NodePath
@export var managed_by_host_script: bool = false

var model: VehicleDynamicsModel
var scene_adapter: VehicleSceneAdapter
var ros_adapter: ROSVehicleAdapter
var logger: DynamicsLogger
var _host_body: RigidBody3D = null

func _ready() -> void:
	if config == null:
		config = VehicleDynamicsConfig.new()
	else:

		config = config.duplicate(true)
	config.ensure_subresources()

	model = VehicleDynamicsModel.new(config)
	scene_adapter = VehicleSceneAdapter.new()
	ros_adapter = ROSVehicleAdapter.new()
	logger = DynamicsLogger.new()

	if not host_body_path.is_empty():
		_host_body = get_node_or_null(host_body_path) as RigidBody3D
	elif get_parent() is RigidBody3D:
		_host_body = get_parent() as RigidBody3D

	if _host_body != null and config.integration_mode == VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D:
		scene_adapter.bind_scene_nodes(_host_body)
		GodotPhysicsAdapter.configure_rigid_body(_host_body, config.vehicle_parameters)
		model.contact_query.use_analytic_plane = false

func bind_to_host_vehicle(host_body: RigidBody3D, explicit_wheels: Array[Node3D] = []) -> void:
	_host_body = host_body
	if scene_adapter != null and _host_body != null:
		scene_adapter.bind_scene_nodes(_host_body, explicit_wheels)
	if _host_body != null and config != null:
		GodotPhysicsAdapter.configure_rigid_body(_host_body, config.vehicle_parameters)
	if model != null:
		model.contact_query.use_analytic_plane = false

func step_from_host(
	delta: float,
	gas_cmd: float,
	steer_angle_rad: float,
	ebs_active: bool,
	origin_godot: Transform3D
) -> void:
	if model == null or config == null or _host_body == null:
		return
	if config.integration_mode != VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D:
		return

	model.state.origin_godot = origin_godot
	model.set_control_commands(gas_cmd, steer_angle_rad, ebs_active)
	GodotPhysicsAdapter.sync_state_from_body(_host_body, model.state, model.contact_query)
	model.step(delta, false, false)
	GodotPhysicsAdapter.apply_forces_to_body(_host_body, model.state, config.integration_mode)
	scene_adapter.update_wheel_visuals(model.state, config.vehicle_parameters)
	logger.record_sample(model.diagnostics)

func _physics_process(delta: float) -> void:
	if managed_by_host_script:
		return
	if model == null or config == null:
		return

	match config.integration_mode:
		VehicleDynamicsTypes.IntegrationMode.INACTIVE_LEGACY_PASSTHROUGH:

			return

		VehicleDynamicsTypes.IntegrationMode.CUSTOM_RIGID_BODY_3D:
			if _host_body == null:
				return
			GodotPhysicsAdapter.sync_state_from_body(_host_body, model.state, model.contact_query)
			model.step(delta, false, false)
			GodotPhysicsAdapter.apply_forces_to_body(_host_body, model.state, config.integration_mode)
			scene_adapter.update_wheel_visuals(model.state, config.vehicle_parameters)
			ros_adapter.publish_step(model.state, float(Engine.physics_ticks_per_second))
			logger.record_sample(model.diagnostics)

		VehicleDynamicsTypes.IntegrationMode.STANDALONE_BENCH:
			model.step(delta, true, false)
			logger.record_sample(model.diagnostics)

func reset_vehicle(origin_godot: Transform3D = Transform3D.IDENTITY) -> void:
	if model != null:
		model.reset(origin_godot)
	if ros_adapter != null:
		ros_adapter.reset_encoder_filter()
	if scene_adapter != null:
		scene_adapter.reset_sensors()
	if logger != null:
		logger.clear()
