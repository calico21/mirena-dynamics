extends Resource
class_name VehicleDynamicsConfig

## Top-level composite configuration Resource for the MirenaSim VehicleDynamics framework.
##
## Aggregates all subsystem configurations so that a complete vehicle setup can be saved,
## inspected, and swapped as a single `.tres` Resource without modifying physics code.

@export_group("Integration & Execution")
## Integration mode governing force ownership. Defaults to INACTIVE_LEGACY_PASSTHROUGH in Phase 1
## so the legacy VehicleBody3D (`vehicleMotion.gd`) remains the sole active model until Phase 2+ switchover.
@export var integration_mode: VehicleDynamicsTypes.IntegrationMode = VehicleDynamicsTypes.IntegrationMode.INACTIVE_LEGACY_PASSTHROUGH
## Number of internal physics substeps per Godot `_physics_process` tick (default 1 at 400 Hz = 2.5 ms).
@export_range(1, 10) var physics_substeps: int = 1
## Enable per-tick diagnostic snapshot capture in `DynamicsDiagnostics`.
@export var enable_diagnostics: bool = true

@export_group("Subsystem Configurations")
@export var vehicle_parameters: VehicleParameters = VehicleParameters.new()
@export var tire_config: TireConfiguration = TireConfiguration.new()
@export var suspension_config: SuspensionConfiguration = SuspensionConfiguration.new()
@export var steering_config: SteeringConfiguration = SteeringConfiguration.new()
@export var powertrain_config: PowertrainConfiguration = PowertrainConfiguration.new()
@export var brake_config: BrakeConfiguration = BrakeConfiguration.new()
@export var aerodynamics_config: AerodynamicConfiguration = AerodynamicConfiguration.new()

## Ensure all sub-resources are instantiated if loaded from a partial resource.
func ensure_subresources() -> void:
	if vehicle_parameters == null:
		vehicle_parameters = VehicleParameters.new()
	if tire_config == null:
		tire_config = TireConfiguration.new()
	if suspension_config == null:
		suspension_config = SuspensionConfiguration.new()
	if steering_config == null:
		steering_config = SteeringConfiguration.new()
	if powertrain_config == null:
		powertrain_config = PowertrainConfiguration.new()
	if brake_config == null:
		brake_config = BrakeConfiguration.new()
	if aerodynamics_config == null:
		aerodynamics_config = AerodynamicConfiguration.new()

## Validate that all subsystem configurations are present and physically valid.
func is_valid() -> bool:
	if physics_substeps < 1:
		return false
	if vehicle_parameters == null or not vehicle_parameters.is_valid():
		return false
	if tire_config == null or not tire_config.is_valid():
		return false
	if suspension_config == null or not suspension_config.is_valid():
		return false
	if steering_config == null or not steering_config.is_valid():
		return false
	if powertrain_config == null or not powertrain_config.is_valid():
		return false
	if brake_config == null or not brake_config.is_valid():
		return false
	if aerodynamics_config == null or not aerodynamics_config.is_valid():
		return false
	return true
