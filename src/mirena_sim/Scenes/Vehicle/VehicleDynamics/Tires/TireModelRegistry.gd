extends RefCounted
class_name TireModelRegistry

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
