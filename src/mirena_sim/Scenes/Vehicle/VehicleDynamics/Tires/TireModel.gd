extends RefCounted
class_name TireModel

var _active_model: TireModelInterface
var _configured_type: VehicleDynamicsTypes.TireModelType = VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB

func _init(initial_type: VehicleDynamicsTypes.TireModelType = VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB) -> void:
	set_model_type(initial_type)

func set_model_type(model_type: VehicleDynamicsTypes.TireModelType) -> void:
	_configured_type = model_type
	_active_model = TireModelRegistry.create_tire_model(model_type)

func get_active_model() -> TireModelInterface:
	return _active_model

func get_active_model_name() -> String:
	return _active_model.get_model_name() if _active_model != null else "None"

func reset_all(wheels: Array[WheelState], config: TireConfiguration) -> void:
	if _active_model == null:
		return
	for w in wheels:
		if w != null and w.tire_state != null:
			_active_model.reset_state(w.tire_state, config)

func evaluate_wheel_forces(
	inputs: TireInputs,
	state: TireState,
	config: TireConfiguration
) -> TireForces:
	if config != null and config.model_type != _configured_type:
		set_model_type(config.model_type)
	if _active_model == null:
		_active_model = TireModelRegistry.create_tire_model(_configured_type)

	var forces: TireForces = _active_model.evaluate_forces(inputs, state, config)
	if not forces.is_finite():
		forces.zero()
		forces.valid = false
		forces.fault_flags |= VehicleDynamicsTypes.FaultFlag.NON_FINITE_FORCE
	return forces
