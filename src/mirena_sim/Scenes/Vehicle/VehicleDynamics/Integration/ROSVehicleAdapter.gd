extends RefCounted
class_name ROSVehicleAdapter

## ROS 2 communication adapter for the MirenaSim vehicle dynamics framework.
##
## Isolates all `rclgd` publishers, subscribers, message allocations, encoder decimation/LPF,
## and quantization from the physical subsystems (`TireModel`, `SuspensionModel`, `PowertrainModel`, etc.).
##
## Preserves the exact MirenaSim ROS 2 topic and message contracts verified in `vehicleMotion.gd`:
## - Subscribes:
##   - `/control` (`mirena_common/msg/CarControl`)
##   - `/system/mission_status` (`mirena_common/msg/MissionInfo`)
##   - `/as_status` (`mirena_common/msg/AsInfo`)
## - Publishes:
##   - `/debug/state/car` (`mirena_common/msg/Car`, frame `"odom"`, child_frame `"car/cog"`)
##   - `/sensors/wheel_speeds` (`mirena_common/msg/WheelSpeeds`, `BEST_EFFORT` depth 5, window-averaged,
##     LPF-filtered with `wheel_speed_lpf_tau`, and quantized to whole RPM via `wheel_speed_quantise_rpm`)
##   - `/lap_status` (`mirena_common/msg/LapStatus`)
##   - `/inferred_control` (`mirena_common/msg/CarControl`)
##   - `/as_status` (`mirena_common/msg/AsInfo`)

var frame_id: String = "car/cog"
var odom_frame_id: String = "odom"
var wheel_speed_rate_hz: float = 100.0
var wheel_speed_lpf_tau_s: float = 0.05
var wheel_speed_quantise_rpm: float = 1.0

var latest_ros_gas: float = 0.0
var latest_ros_steer_rad: float = 0.0

var _wheel_decim_n: int = 0
var _wheel_rpm_acc: Array[float] = [0.0, 0.0] # [rl, rr]
var _wheel_rpm_lpf: Array[float] = [0.0, 0.0] # [rl, rr]

var _ros_node = null
var _state_pub = null
var _wheel_speeds_pub = null
var _lap_status_pub = null
var _inferred_control_pub = null
var _as_info_pub = null
var _control_sub = null
var _mission_info_sub = null
var _as_status_sub = null

var _car_state_msg = null
var _wheel_speeds_msg = null
var _lap_status_msg = null

## Reset encoder accumulator and low-pass filter state on vehicle respawn.
## Matches `vehicleMotion.gd` lines 523-527 so respawn teleports do not corrupt the first encoder window.
func reset_encoder_filter() -> void:
	_wheel_rpm_acc[0] = 0.0
	_wheel_rpm_acc[1] = 0.0
	_wheel_rpm_lpf[0] = 0.0
	_wheel_rpm_lpf[1] = 0.0
	_wheel_decim_n = 0

## Accumulate rear wheel speeds (`RL`, `RR`) once per physics tick and compute the decimated,
## low-pass-filtered, quantized RPM reading when the measurement window completes.
##
## Can be tested standalone in headless unit tests without an active `RosNode`.
## Returns a Dictionary:
##   {
##     "should_publish": bool,
##     "rl_rpm": float,
##     "rr_rpm": float,
##     "dt_window_s": float,
##   }
func step_wheel_speed_encoder(
	rl_instant_rpm: float,
	rr_instant_rpm: float,
	physics_hz: float = 400.0
) -> Dictionary:
	_wheel_rpm_acc[0] += rl_instant_rpm
	_wheel_rpm_acc[1] += rr_instant_rpm
	_wheel_decim_n += 1

	var window: int = maxi(1, int(round(physics_hz / maxf(wheel_speed_rate_hz, 1.0))))
	if _wheel_decim_n < window:
		return {
			"should_publish": false,
			"rl_rpm": _wheel_rpm_lpf[0],
			"rr_rpm": _wheel_rpm_lpf[1],
			"dt_window_s": 0.0,
		}

	var dt_s: float = float(_wheel_decim_n) / maxf(physics_hz, 1.0)
	var raw_rl: float = _wheel_rpm_acc[0] / float(_wheel_decim_n)
	var raw_rr: float = _wheel_rpm_acc[1] / float(_wheel_decim_n)
	_wheel_rpm_acc[0] = 0.0
	_wheel_rpm_acc[1] = 0.0
	_wheel_decim_n = 0

	var alpha: float = 1.0
	if wheel_speed_lpf_tau_s > 0.0:
		alpha = 1.0 - exp(-dt_s / maxf(wheel_speed_lpf_tau_s, 1e-6))

	_wheel_rpm_lpf[0] = lerpf(_wheel_rpm_lpf[0], raw_rl, alpha)
	_wheel_rpm_lpf[1] = lerpf(_wheel_rpm_lpf[1], raw_rr, alpha)

	var q_rl: float = VehicleUnits.quantize_rpm(_wheel_rpm_lpf[0], wheel_speed_quantise_rpm)
	var q_rr: float = VehicleUnits.quantize_rpm(_wheel_rpm_lpf[1], wheel_speed_quantise_rpm)

	return {
		"should_publish": true,
		"rl_rpm": q_rl,
		"rr_rpm": q_rr,
		"dt_window_s": dt_s,
	}

## Build a Dictionary representation of `mirena_common/msg/Car` from `VehicleState`.
## Directly verifiable in unit tests and used to populate `_car_state_msg` when ROS is active.
func build_car_state_payload(state: VehicleState) -> Dictionary:
	var cov: Array[float] = []
	cov.resize(36)
	cov.fill(0.0)
	return {
		"frame_id": odom_frame_id,
		"child_frame_id": frame_id,
		"x": state.x_odom_m,
		"y": state.y_odom_m,
		"psi": state.psi_odom_rad,
		"u": state.u_mps,
		"v": state.v_mps,
		"omega": state.r_rad_s,
		"covariance": cov,
	}

## Publish vehicle state and rear wheel speeds to ROS 2 if an active `RosNode` is initialized.
func publish_step(state: VehicleState, physics_hz: float = 400.0) -> void:
	if state == null:
		return

	var rl_rpm: float = state.wheels[VehicleDynamicsTypes.WheelCorner.RL].get_rpm()
	var rr_rpm: float = state.wheels[VehicleDynamicsTypes.WheelCorner.RR].get_rpm()
	var enc: Dictionary = step_wheel_speed_encoder(rl_rpm, rr_rpm, physics_hz)

	if _ros_node == null:
		return

	var now_stamp = _ros_node.now()
	if enc["should_publish"] and _wheel_speeds_pub != null and _wheel_speeds_msg != null:
		_wheel_speeds_msg.header.stamp = now_stamp
		_wheel_speeds_msg.header.frame_id = frame_id
		_wheel_speeds_msg.rl = enc["rl_rpm"]
		_wheel_speeds_msg.rr = enc["rr_rpm"]
		_wheel_speeds_pub.publish(_wheel_speeds_msg)

	if _state_pub != null and _car_state_msg != null:
		_car_state_msg.header.stamp = now_stamp
		_car_state_msg.header.frame_id = odom_frame_id
		_car_state_msg.child_frame_id = frame_id
		_car_state_msg.x = state.x_odom_m
		_car_state_msg.y = state.y_odom_m
		_car_state_msg.psi = state.psi_odom_rad
		_car_state_msg.u = state.u_mps
		_car_state_msg.v = state.v_mps
		_car_state_msg.omega = state.r_rad_s
		_state_pub.publish(_car_state_msg)
