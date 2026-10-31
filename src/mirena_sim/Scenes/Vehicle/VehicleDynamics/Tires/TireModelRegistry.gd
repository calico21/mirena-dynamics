extends RefCounted
class_name TireModelRegistry

## Registry and factory for instantiating `TireModelInterface` implementations from `TireConfiguration`.
##
## Ensures that switching between tire models (`UNIMPLEMENTED_CONTRACT`, `LINEAR_TEST_STUB`,
## `PACEJKA_MF62`, and `TER_GREYBOX_PINN`) is controlled solely by `TireConfiguration`
## without modifying `VehicleDynamicsController`, `ChassisModel`, `SuspensionModel`, or `ROSVehicleAdapter`.

static func create_tire_model(model_type: VehicleDynamicsTypes.TireModelType) -> TireModelInterface:
	match model_type:
		VehicleDynamicsTypes.TireModelType.LINEAR_TEST_STUB:
			return TireModelInterface.LinearTestStubTireModel.new()
		VehicleDynamicsTypes.TireModelType.PACEJKA_MF62:
			return PacejkaMF62TireModel.new()
		VehicleDynamicsTypes.TireModelType.TER_GREYBOX_PINN:
			return TeRTireGreyBoxModel.new()
		VehicleDynamicsTypes.TireModelType.UNIMPLEMENTED_CONTRACT, _:
			return TireModelInterface.new()
