extends Resource
class_name SuspensionConfiguration

## Configuration resource for the 4-wheel suspension subsystem (springs, digressive dampers, ARBs, K&C).
##
## All parameters use strict SI units (N/m, N*s/m, m, rad).
## Note on legacy mirena_car.tscn values:
## Godot's VehicleWheel3D `suspension_stiffness` (35.03 front, 52.54 rear) is in N/mm per unit mass or
## internal Bullet/Godot scaling, which corresponds to 35030 N/m front and 52540 N/m rear in TeR26 (`ter26.py`).

@export_group("Springs & Ride Height")
## Front spring stiffness at the spring (N/m). Source: ter26.py (35030.0 N/m) / mirena_car.tscn (35.03 kN/m).
@export var spring_rate_front_npm: float = 35030.0
## Rear spring stiffness at the spring (N/m). Source: ter26.py (52540.0 N/m) / mirena_car.tscn (52.54 kN/m).
@export var spring_rate_rear_npm: float = 52540.0
## Front motion ratio (spring travel / wheel travel). Source: ter26.py K&C linear term (1.14).
@export var motion_ratio_front: float = 1.14
## Rear motion ratio (spring travel / wheel travel). Source: ter26.py K&C linear term (1.16).
@export var motion_ratio_rear: float = 1.16
## Nominal suspension rest length / travel from attachment to unloaded wheel center (m).
## Verified in mirena_car.tscn (wheel_rest_length = 0.05 m).
@export var rest_length_front_m: float = 0.05
@export var rest_length_rear_m: float = 0.05
## Maximum compression (bump) travel from static design height (m).
@export var max_compression_travel_m: float = 0.080
## Maximum extension (droop/rebound) travel from static design height (m).
@export var max_droop_travel_m: float = 0.060

@export_group("Bump Stops & Preload")
## Compression travel at which the bump stop engages (m). Source: ter26.py (0.025 m).
@export var bump_stop_engage_m: float = 0.025
## Bump stop secondary stiffness once engaged (N/m). Source: ter26.py (50000.0 N/m).
@export var bump_stop_rate_npm: float = 50000.0
## Damper gas charge preload force at the spring (N). Source: ter26.py (120.0 N).
@export var damper_gas_preload_front_n: float = 120.0
@export var damper_gas_preload_rear_n: float = 120.0
## Automatically compute static spring preload from corner static weight so compression=0 is static equilibrium.
@export var auto_static_preload: bool = true
## Manual static spring preload force at wheel (N), used only when auto_static_preload is false.
@export var manual_preload_front_n: float = 650.0
@export var manual_preload_rear_n: float = 800.0

@export_group("Digressive Dampers")
## Front low-speed bump damping coefficient at damper (N*s/m). Source: ter26.py (1800.0 N*s/m).
@export var damper_c_low_front_nspm: float = 1800.0
## Rear low-speed bump damping coefficient at damper (N*s/m). Source: ter26.py (1600.0 N*s/m).
@export var damper_c_low_rear_nspm: float = 1600.0
## Front high-speed bump damping coefficient at damper (N*s/m). Source: ter26.py (720.0 N*s/m).
@export var damper_c_high_front_nspm: float = 720.0
## Rear high-speed bump damping coefficient at damper (N*s/m). Source: ter26.py (640.0 N*s/m).
@export var damper_c_high_rear_nspm: float = 640.0
## Digressive knee velocity at damper (m/s). Source: ter26.py (0.10 m/s).
@export var damper_v_knee_front_mps: float = 0.10
@export var damper_v_knee_rear_mps: float = 0.10
## Rebound-to-bump damping ratio (dimensionless >= 1.0). Source: ter26.py (1.60).
@export var rebound_ratio_front: float = 1.60
@export var rebound_ratio_rear: float = 1.60

@export_group("Anti-Roll Bars (ARB)")
## Front anti-roll bar wheel-rate stiffness per differential wheel displacement (N/m).
## Tuned for Formula Student front-biased roll stiffness distribution (~58% front roll stiffness)
## so the rear inside tire remains loaded under high-g cornering and throttle application.
@export var arb_rate_front_npm: float = 16000.0
## Rear anti-roll bar wheel-rate stiffness per differential wheel displacement (N/m).
@export var arb_rate_rear_npm: float = 1500.0

@export_group("Kinematics & Compliance (K&C)")
## Front static camber angle (rad). Negative = top of tire leans inward toward chassis. Source: ter26.py (-2.0 deg).
@export var static_camber_front_rad: float = deg_to_rad(-2.0)
## Rear static camber angle (rad). Source: ter26.py (-1.5 deg).
@export var static_camber_rear_rad: float = deg_to_rad(-1.5)
## Camber change per metre of bump travel (rad/m). Source: ter26.py (-25.0 deg/m front, -20.0 deg/m rear).
@export var camber_per_m_travel_front_radpm: float = deg_to_rad(-25.0)
@export var camber_per_m_travel_rear_radpm: float = deg_to_rad(-20.0)
## Camber gain per radian of chassis roll (rad/rad). Source: ter26.py (-0.80 front, -0.65 rear).
@export var camber_gain_roll_front: float = -0.80
@export var camber_gain_roll_rear: float = -0.65
## Roll center height above ground at design ride height (m). Source: ter26.py (0.040 m front, 0.060 m rear).
@export var roll_center_height_front_m: float = 0.040
@export var roll_center_height_rear_m: float = 0.060

## Return effective wheel rate (N/m) for the given axle: k_wheel = k_spring / MR^2
## (where MR = spring_travel / wheel_travel, or k_spring * (d_spring/d_wheel)^2;
## here motion_ratio is defined as wheel_travel / spring_travel in K&C convention: k_wheel = k_spring / MR^2).
func get_effective_wheel_rate_npm(axle: VehicleDynamicsTypes.Axle) -> float:
	if axle == VehicleDynamicsTypes.Axle.FRONT:
		var mr: float = maxf(motion_ratio_front, 1e-4)
		return spring_rate_front_npm / (mr * mr)
	else:
		var mr: float = maxf(motion_ratio_rear, 1e-4)
		return spring_rate_rear_npm / (mr * mr)

## Validate physical consistency of suspension parameters.
func is_valid() -> bool:
	if spring_rate_front_npm <= 0.0 or spring_rate_rear_npm <= 0.0:
		return false
	if motion_ratio_front <= 0.0 or motion_ratio_rear <= 0.0:
		return false
	if damper_c_low_front_nspm < 0.0 or damper_c_low_rear_nspm < 0.0:
		return false
	if damper_v_knee_front_mps <= 0.0 or damper_v_knee_rear_mps <= 0.0:
		return false
	if arb_rate_front_npm < 0.0 or arb_rate_rear_npm < 0.0:
		return false
	return true
