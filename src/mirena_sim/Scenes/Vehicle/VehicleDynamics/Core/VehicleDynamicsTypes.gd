extends RefCounted
class_name VehicleDynamicsTypes

enum WheelCorner {
	FL = 0,
	FR = 1,
	RL = 2,
	RR = 3,
}

const WHEEL_COUNT: int = 4
const ALL_CORNERS: Array[WheelCorner] = [
	WheelCorner.FL,
	WheelCorner.FR,
	WheelCorner.RL,
	WheelCorner.RR,
]

enum Axle {
	FRONT = 0,
	REAR = 1,
}

enum IntegrationMode {
	INACTIVE_LEGACY_PASSTHROUGH = 0,
	CUSTOM_RIGID_BODY_3D = 1,
	STANDALONE_BENCH = 2,
}

enum DrivetrainArchitecture {
	RWD_SINGLE_MOTOR = 0,
	AWD_HUB_MOTORS = 1,
}

enum TireModelType {
	UNIMPLEMENTED_CONTRACT = 0,
	LINEAR_TEST_STUB = 1,
	PACEJKA_MF62 = 2,
	TER_GREYBOX_PINN = 3,
}

enum FaultFlag {
	NONE = 0,
	NON_FINITE_INPUT = 1 << 0,
	NON_FINITE_STATE = 1 << 1,
	NON_FINITE_FORCE = 1 << 2,
	INVALID_CONFIG = 1 << 3,
	UNIMPLEMENTED_MODEL_CALLED = 1 << 4,
	CONTACT_LOSS_ALL_WHEELS = 1 << 5,
	TIRE_SATURATION = 1 << 6,
	NUMERICAL_CLAMP_APPLIED = 1 << 7,
	TIRE_LOAD_OUT_OF_DOMAIN = 1 << 8,
	TIRE_SLIP_OUT_OF_DOMAIN = 1 << 9,
	WHEEL_LIFTOFF_SINGLE = 1 << 10,
	WHEEL_LIFTOFF_SAME_SIDE = 1 << 11,
	LOW_SPEED_CHATTER = 1 << 12,
}

static func corner_name(corner: WheelCorner) -> String:
	match corner:
		WheelCorner.FL:
			return "FL"
		WheelCorner.FR:
			return "FR"
		WheelCorner.RL:
			return "RL"
		WheelCorner.RR:
			return "RR"
		_:
			return "UNKNOWN"

static func corner_axle(corner: WheelCorner) -> Axle:
	match corner:
		WheelCorner.FL, WheelCorner.FR:
			return Axle.FRONT
		_:
			return Axle.REAR

static func is_left_corner(corner: WheelCorner) -> bool:
	return corner == WheelCorner.FL or corner == WheelCorner.RL

static func is_front_corner(corner: WheelCorner) -> bool:
	return corner == WheelCorner.FL or corner == WheelCorner.FR
