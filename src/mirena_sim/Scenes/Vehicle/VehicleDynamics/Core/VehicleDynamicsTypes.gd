extends RefCounted
class_name VehicleDynamicsTypes

## Shared enumerations, corner indexing, and fault bitmasks for MirenaSim VehicleDynamics.

## Standard 4-wheel corner indexing used throughout all arrays of size 4.
enum WheelCorner {
	FL = 0, ## Front Left
	FR = 1, ## Front Right
	RL = 2, ## Rear Left
	RR = 3, ## Rear Right
}

const WHEEL_COUNT: int = 4
const ALL_CORNERS: Array[WheelCorner] = [
	WheelCorner.FL,
	WheelCorner.FR,
	WheelCorner.RL,
	WheelCorner.RR,
]

## Axle identifier.
enum Axle {
	FRONT = 0,
	REAR = 1,
}

## Operating mode for physics integration.
## Ensures the new dynamics model and legacy VehicleBody3D never apply forces simultaneously.
enum IntegrationMode {
	INACTIVE_LEGACY_PASSTHROUGH = 0, ## Legacy VehicleBody3D owns physics; new model is disabled.
	CUSTOM_RIGID_BODY_3D = 1,        ## New VehicleDynamicsModel owns all forces on a RigidBody3D chassis.
	STANDALONE_BENCH = 2,            ## Headless standalone numerical integration for unit/integration tests.
}

## Drivetrain layout selector.
enum DrivetrainArchitecture {
	RWD_SINGLE_MOTOR = 0, ## Rear-wheel drive with single motor + final drive (TeR26 baseline).
	AWD_HUB_MOTORS = 1,   ## Four-wheel drive with independent hub motors (TeR27 architecture).
}

## Pluggable tire model type selector.
enum TireModelType {
	UNIMPLEMENTED_CONTRACT = 0, ## Placeholder interface returning zero forces with explicit unimplemented flag.
	LINEAR_TEST_STUB = 1,       ## Analytic linear/saturated test stub for framework verification ONLY (uncalibrated).
	PACEJKA_MF62 = 2,           ## Reserved for Phase 2+ Pacejka Magic Formula 6.2 implementation.
	TER_GREYBOX_PINN = 3,       ## Reserved for Phase 2+ TeR-Tire Grey-Box (MF6.2 + residual MLP) implementation.
}

## Bitmask flags for numerical or physical fault detection during a step.
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
	TIRE_LOAD_OUT_OF_DOMAIN = 1 << 8,     ## Fz > 1600 N or < 150 N under dynamic loading (TTC envelope)
	TIRE_SLIP_OUT_OF_DOMAIN = 1 << 9,     ## Slip angle > 15 deg or slip ratio > 0.25
	WHEEL_LIFTOFF_SINGLE = 1 << 10,       ## One wheel has zero normal load
	WHEEL_LIFTOFF_SAME_SIDE = 1 << 11,    ## Both left or both right wheels simultaneously unloaded (impending roll)
	LOW_SPEED_CHATTER = 1 << 12,          ## Rapid force/velocity sign alternation near standstill
}

## Return human-readable short label for a wheel corner.
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

## Return the axle corresponding to a wheel corner.
static func corner_axle(corner: WheelCorner) -> Axle:
	match corner:
		WheelCorner.FL, WheelCorner.FR:
			return Axle.FRONT
		_:
			return Axle.REAR

## Return true if the corner is on the left (+Y in ISO/ROS body frame, -X in Godot body frame).
static func is_left_corner(corner: WheelCorner) -> bool:
	return corner == WheelCorner.FL or corner == WheelCorner.RL

## Return true if the corner is on the front axle (+X in ISO/ROS body frame, -Z in Godot body frame).
static func is_front_corner(corner: WheelCorner) -> bool:
	return corner == WheelCorner.FL or corner == WheelCorner.FR
