extends Node3D
class_name RosIns

## Simulated VectorNav VN-300, and the driver that reads it.
##
## The sim counterpart of mirena_hardware's `mirena_imu` node. On the car there
## is one device: a single measurement carries the raw IMU, the onboard INS
## solution and the GNSS fix, and the driver turns each measurement into four
## messages. This node is that device and that driver together, which is why it
## replaces the separate RosImu and RosGps it was split into -- two nodes with
## two clocks and two independent noise draws could not express the one property
## that matters most about the real thing: /sensors/imu/odom and
## /sensors/gps/fix come from the *same receiver*, and a filter handed both must
## not treat them as independent observations of where the car is. The driver
## reads the fix from the GNSS binary group and the odometry from the INS group,
## so they are no longer literally the same field -- but the INS solution is
## built from that receiver, so they stay drawn from one error process here.
##
## Published, all from one measurement, mirroring ImuNode::publish_loop:
##   /sensors/imu        sensor_msgs/Imu       always, in the cog frame
##   /sensors/lidar/imu  sensor_msgs/Imu       always, in the lidar frame
##   /sensors/imu/odom   nav_msgs/Odometry     only while the INS is tracking
##   /sensors/gps/fix    sensor_msgs/NavSatFix as soon as the receiver has a fix
##
## NOTE on `linear_acceleration`, carried over verbatim from imu_pipeline.hpp
## because it is the single easiest thing to get wrong here: the Imu messages
## carry **gravity-free** acceleration, which sensor_msgs/Imu does not specify --
## that field is defined as the raw accelerometer reading. It is deliberate and
## load bearing: mirena_state's EKF mechanises from gravity-free body
## acceleration. A recorded run confirms it, reading ~0.07 m/s^2 on a level
## stationary car rather than 9.81.

# --- Configuration ---
@export_group("IMU Settings")
## White sensor noise floor, per base-rate sample. This is the accelerometer's
## own noise with the car still, which is small next to what vibration adds.
@export var accel_noise_std: float = 0.005
@export var gyro_noise_std: float = 0.0005
@export var bias_drift_std: float = 0.00001
@export var lpf_tau: float = 0.05 # Low-pass filter for physics jitter
## Road and drivetrain vibration, as a per-axis standard deviation of the
## *published* samples at `vibration_reference_speed`. Defaults are the measured
## on-car figures from mirena_imu.cpp's linear_acceleration_variance and
## angular_velocity_variance parameters, which imu_pipeline.hpp streams straight
## onto these messages -- and vibration dominates them by ~5x, exactly as that
## file says. This is what a real IMU on a real car reads; a bare sensor floor
## does not. Set vibration_scale to 0 for the smooth virtual-world IMU the sim
## used to have.
@export var accel_vibration_std: Vector3 = Vector3(0.173, 0.548, 0.141) # m/s^2
@export var gyro_vibration_std: Vector3 = Vector3(0.00806, 0.00361, 0.00316) # rad/s
@export var vibration_reference_speed: float = 15.0 # m/s
@export_range(0.0, 4.0) var vibration_scale: float = 1.0
## Scale on the published Imu covariances. 1.0 publishes the variance this model
## actually produces, which is the point: a covariance that does not describe the
## error being handed over is worse than no covariance. Set 0 only to reproduce
## the old all-zero behaviour.
@export var imu_covariance_scale: float = 1.0

@export_group("INS Settings")
## One rate for the whole device, because the car has one: every message below
## comes from the same measurement. The VN-300 streams at kBaseRateHz (400, see
## imu_wrapper.hpp) and mirena_imu divides down by an integer to reach this, so
## each output sample is the *mean* of a whole number of base-rate samples. The
## sim physics runs at that same 400 Hz, so this is modelled exactly rather than
## approximated.
@export var ins_rate: float = 100.0
## Duty cycle of the onboard filter having a usable solution. The driver
## publishes odometry only while the INS reports mode 2 (tracking) and says
## nothing otherwise, so this gates rather than throttles: a recorded run shows
## the IMU at 100 Hz and both INS topics at 76.6 Hz off the very same stream
## (20587 of 26888 measurements).
##
## It used to be applied as a per-measurement coin flip, which threw away 23% of
## the odometry at random intervals averaging 13 ms. Real INS outages are
## episodic -- mode 2 to mode 3 dead reckoning for seconds -- and a filter handed
## that sees its correction arrive at unpredictable times, which reads as jitter
## in the estimate even when the sensor is fine. It is now a two-state Markov
## process whose stationary duty cycle is this number and whose dwell times are
## exponential, so the average holds but the run lengths are realistic.
@export_range(0.0, 1.0) var ins_tracking_fraction: float = 0.766
## Mean time spent in mode 3 before the filter reacquires. Sets the scale of the
## outage runs; the tracking dwell follows from ins_tracking_fraction to keep the
## duty cycle where it is. A recorded run shows both INS topics at 76.6% over
## 26888 measurements, i.e. the aggregate, not the granularity.
@export var ins_dead_dwell_sec: float = 3.0
## Alignment delay: the hardware publishes no odometry at all before the onboard
## filter converges, because mirena_state seeds its EKF from the first odometry
## it sees and an unconverged solution would pin the run's origin on a position
## the sensor does not stand behind. Non-zero here for the same reason it is
## non-zero on the car -- at 0 the first odometry the fusion ever sees is trusted
## completely. Kept short because a sim run is short.
@export var ins_startup_delay_sec: float = 5.0
## Acquisition delay for the GNSS receiver itself, which is a different and much
## shorter job than the INS alignment above -- the driver publishes /sensors/gps/fix
## as soon as the receiver has a fix, without waiting for the filter. Kept small
## deliberately: the gap between this and ins_startup_delay_sec is the window in
## which the stack can see that GNSS is alive while the INS is still aligning.
@export var gnss_startup_delay_sec: float = 2.0
## Rate the receiver itself solves at, and therefore the rate /sensors/gps/fix
## goes out. mirena_imu only accepts 1 or 5; default 5. The fix was previously
## published once per IMU measurement, re-sending a single solution up to 20
## times -- evidence that does not exist, which imu_pipeline.hpp explicitly
## guards against by de-duplicating on GPS time-of-week.
@export_range(1, 5) var gnss_rate_hz: int = 5

## Reported position uncertainty wanders inside this band, in metres of standard
## deviation. Defaults are the range measured over a recorded run: posU put
## pose.covariance[0] between 5.36 and 11.07 m^2.
@export var ins_pos_sigma_min: float = 2.32
@export var ins_pos_sigma_max: float = 3.33
## Correlation time of the position error, in seconds. A real INS/GNSS position
## error wanders slowly; it does not resample itself every measurement. An
## estimator tuned against white noise is tuned against the wrong thing, which is
## why this is a random walk and not a per-sample draw.
@export var ins_pos_tau: float = 30.0
## Velocity uncertainty band, likewise measured: velU put twist.covariance[0]
## between 0.018 and 0.059 (m/s)^2.
@export var ins_vel_sigma_min: float = 0.135
@export var ins_vel_sigma_max: float = 0.243
@export var ins_vel_tau: float = 5.0
## How slowly the reported uncertainty itself drifts across its band.
@export var ins_sigma_tau: float = 45.0
## Attitude uncertainty, radians of standard deviation, matching the yprU
## figures the filter reports: roll 1e-6, pitch 2e-6, yaw 9.9e-5 rad^2.
@export var ins_roll_sigma: float = 0.001
@export var ins_pitch_sigma: float = 0.0014
@export var ins_yaw_sigma: float = 0.01
@export var ins_att_tau: float = 20.0
## Published on twist.covariance for the angular rates. The onboard filter does
## not stream a rate uncertainty, so the driver substitutes these constants --
## per axis, because the three are genuinely different numbers on this car and a
## single scalar either overstates one axis or understates another. Defaults are
## mirena_imu.cpp's angular_velocity_variance, which imu_pipeline.hpp writes
## straight onto twist.covariance[(i + 3) * 7].
@export var ins_gyro_var: Vector3 = Vector3(6.5e-5, 1.3e-5, 1.0e-5)

@export_group("Geodesy")
## Where the Godot world origin sits on the earth. The INS reports latitude and
## longitude; everything geodetic downstream is relative to the reference the
## driver latches from the first solution, so this only fixes the patch of earth
## the run happens on.
@export var origin_lat: float = 43.29454175
@export var origin_lon: float = -2.01119177
@export var origin_alt: float = 115.962

@export_group("ROS 2 Settings")
@export var ros_namespace: String = ""
## This node *is* the IMU, so this is its own mount frame -- the one the driver
## looks up car/cog <- car/imu and car/lidar <- car/imu against.
@export var frame_id: String = "car/imu"
@export var parent_frame_id: String = "car/cog"
@export var lidar_frame_id: String = "car/lidar"
@export var gnss_frame_id: String = "car/gnss"
@export var odom_frame_id: String = "odom"

# --- WGS84 Constants (mirena::GNSSConversion) ---
const WGS84_A = 6378137.0
const WGS84_E2 = 0.00669437999014

# --- Internal State ---
var is_initialized: bool = false

# Raw sensor state, in this node's own (IMU) axes.
var _last_imu_vel: Vector3 = Vector3.ZERO
var _lpf_accel: Vector3 = Vector3.ZERO
var _lpf_gyro: Vector3 = Vector3.ZERO
var _accel_bias: Vector3 = Vector3.ZERO
var _gyro_bias: Vector3 = Vector3.ZERO
var noisy_accel: Vector3 = Vector3.ZERO
var noisy_gyro: Vector3 = Vector3.ZERO

## Set after a respawn, and on the very first tick: the acceleration is a
## finite difference of velocity, so the first sample after a teleport would
## report the jump itself as acceleration. At 15 m/s over a 2.5 ms tick that is
## 6000 m/s^2, and it is a step the EKF cannot unsee -- it integrates straight
## into the pose and then has to be argued out of the covariance.
var _seed_pending: bool = true

# Decimation from the sensor's base rate down to ins_rate. Each output sample is
# the mean of `_decim_window()` base-rate samples, which is what dividing the
# real stream down does -- averaging, not holding.
var _decim_n: int = 0
var _decim_accel: Vector3 = Vector3.ZERO
var _decim_gyro: Vector3 = Vector3.ZERO

## Per-axis variance of the base-rate error, so the published covariance can be
## derived from the model instead of asserted. Averaging N independent draws
## divides the white part by N; the bias walk is common to the whole window and
## is not reduced by it.
var _accel_white_var: Vector3 = Vector3.ZERO
var _gyro_white_var: Vector3 = Vector3.ZERO

# INS solution error. One slow scalar in [0, 1] drives both the reported sigma
# and the size of the error, so the covariance a consumer reads always describes
# the error it is actually being handed -- which is the point of publishing one.
var _ins_quality: float = 0.5
var _ins_pos_sigma: float = 0.0
var _ins_vel_sigma: float = 0.0
var _ins_pos_err: Vector3 = Vector3.ZERO   # metres, NED
var _ins_vel_err: Vector3 = Vector3.ZERO   # m/s, sensor axes
var _att_err: Vector3 = Vector3.ZERO       # rad, (roll, pitch, yaw)
var _ins_elapsed: float = 0.0

## Onboard filter mode, as a dwell time rather than a per-sample coin flip.
var _ins_tracking_now: bool = true
var _ins_mode_remaining: float = 0.0

## Time since the last /sensors/gps/fix, so it goes out at the receiver's own
## solution rate instead of once per IMU measurement.
var _fix_elapsed: float = 0.0

## Where the odom frame's origin sits on the earth (mirena::IMU::GeodesicRef),
## latched from the first solution the driver trusts.
var _geodesic_ref: Vector3 = Vector3.ZERO
var _have_ref: bool = false
var _ref_car_origin: Transform3D = Transform3D.IDENTITY

# Mounting geometry, resolved once (ImuNode::ensure_mounting).
var _have_mounting: bool = false
var _cog_from_imu: Transform3D = Transform3D.IDENTITY
var _lidar_from_imu: Transform3D = Transform3D.IDENTITY

# --- ROS Components ---
var _node: RosNode
var _imu_in_cog_pub: RosPublisher
var _imu_in_lidar_pub: RosPublisher
var _odom_pub: RosPublisher
var _fix_pub: RosPublisher
var _cog_msg: RosSensorMsgsImu
var _lidar_msg: RosSensorMsgsImu
var _odom_msg: RosNavMsgsOdometry
var _fix_msg: RosSensorMsgsNavSatFix

## Mounts are broadcast centrally by the vehicle's RosTransformManager.
##
## Only car/imu, matching transforms.yaml. car/gnss is deliberately absent: the
## fix is stamped in it but the real car does not publish it either, and a frame
## that resolves in sim and dangles on the car is worse than one that dangles in
## both.
func get_static_transforms() -> Array:
	return [{"transform": transform, "frame_id": frame_id, "parent_frame_id": parent_frame_id}]

func _ready() -> void:
	_node = RosNode.new()
	_node.init(name.to_snake_case(), ros_namespace.to_snake_case())

	# Sensor-layer outputs only, exactly the four the driver owns. State fusion
	var qos := RosQoS.new()
	qos.reliability = RosQoS.RELIABLE
	qos.depth =	10
	# Sensor-layer outputs using the RosQoS object
	_imu_in_lidar_pub = _node.create_publisher("/sensors/lidar/imu", "sensor_msgs/msg/Imu", qos)
	_imu_in_cog_pub = _node.create_publisher("/sensors/imu", "sensor_msgs/msg/Imu", qos)
	_odom_pub = _node.create_publisher("/sensors/imu/odom", "nav_msgs/msg/Odometry", qos)
	_fix_pub = _node.create_publisher("/sensors/gps/fix", "sensor_msgs/msg/NavSatFix", qos)	
	
	_cog_msg = RosSensorMsgsImu.new()
	_lidar_msg = RosSensorMsgsImu.new()
	_odom_msg = RosNavMsgsOdometry.new()
	_fix_msg = RosSensorMsgsNavSatFix.new()
	_fill_imu_covariances()

	# No RosTimer here on purpose: publishing is driven off _physics_process so
	# it lands on the requested tick rather than on a render-frame boundary.
	# See _publish_output.
	_init_ins_mode()
	_warn_on_bad_rate()
	is_initialized = true

## The real driver refuses a rate that does not divide kBaseRateHz and streams at
## the nearest that does (mirena_imu.cpp::rate_divisor). Here the window is
## rounded, so the same thing happens silently -- except that a silent 99 Hz
## topic is exactly the sort of thing that gets blamed on the fusion later.
func _warn_on_bad_rate() -> void:
	var base := _base_rate_hz()
	var want := maxf(ins_rate, 1.0)
	var window := _decim_window()
	var actual := base / float(window)
	if absf(actual - want) / want > 0.01:
		push_warning("RosIns: ins_rate=%.1f does not divide the %.0f Hz base rate; "
			% [want, base]
			+ "streaming at %.1f Hz instead." % actual)

# --- Raw sensing -------------------------------------------------------------

## The sensor's base rate, which is also the physics rate. The real VN-300
## streams at kBaseRateHz = 400 and the driver divides that down by an integer
## (see imu_wrapper.hpp and mirena_imu.cpp::rate_divisor), so this is the rate
## the noise has to be drawn at: drawing it at ins_rate and holding would leave
## consecutive output samples correlated and fold the noise spectrum into the
## baseband at full amplitude, which is what made this topic look broken.
func _base_rate_hz() -> float:
	return float(Engine.physics_ticks_per_second)

## How many base-rate samples make up one output sample.
func _decim_window() -> int:
	return maxi(1, int(round(_base_rate_hz() / maxf(ins_rate, 1.0))))

func _physics_process(delta: float) -> void:
	if not is_initialized: return
	var body := get_parent() as RigidBody3D
	if not body: return

	# 1. Kinematic acceleration of *this point* on the rigid body, in the world.
	# Gravity is never added: the onboard filter removes it before the driver
	# ever sees a sample, so the reading is already gravity-free (see the note at
	# the top of this file).  This matches the real IMU, which publishes
	# gravity-free linear acceleration (verified: real static |g| ~ 0.74 m/s^2
	# bias only, not 9.81).
	var arm_world := global_position - body.global_position
	var imu_vel := body.linear_velocity + body.angular_velocity.cross(arm_world)
	var inv_basis := global_basis.inverse()

	# 2. Into the sensor's own axes.
	var local_gyro := inv_basis * body.angular_velocity
	var local_accel := Vector3.ZERO

	if _seed_pending:
		# First tick, or the first after a respawn. There is no previous velocity,
		# so there is no acceleration to report: contributing zero is the honest
		# answer, and it is what stops a teleport from being published as
		# 15 m/s over one 2.5 ms tick. Seeding _last_imu_vel here is what makes
		# the *next* tick's difference valid rather than a jump.
		#
		# The gyro is different -- it is measured directly, not differenced, so it
		# is seeded from the sample. Starting it at zero instead would put a
		# decaying transient on the yaw rate of every run.
		_seed_pending = false
		_last_imu_vel = imu_vel
		_lpf_accel = Vector3.ZERO
		_lpf_gyro = local_gyro
	else:
		local_accel = inv_basis * (imu_vel - _last_imu_vel) / delta
		_last_imu_vel = imu_vel

	# 3. Low-pass filter: physics engines create spikes that break factor graphs.
	# Proper exponential moving average: alpha = 1 - exp(-dt/tau).
	var alpha := 1.0 - exp(-delta / maxf(lpf_tau, 1e-6))
	_lpf_accel = _lpf_accel.lerp(local_accel, alpha)
	_lpf_gyro = _lpf_gyro.lerp(local_gyro, alpha)

	# 4. Random-walk biases. Drift must scale with sqrt(delta) to be consistent.
	var drift_scale := sqrt(delta)
	_accel_bias += _noise_vec(bias_drift_std * drift_scale)
	_gyro_bias += _noise_vec(bias_drift_std * drift_scale)

	# 5. Vibration, growing with speed: road and tyre buzz, and the drive
	#    train's own. This is the term that dominates a real IMU on a real car,
	#    and it is why the hardware's measured covariances are ~5x the
	#    accelerometer's own noise floor.
	#
	#    accel_vibration_std describes the *output* samples, because that is what
	#    the hardware's variance parameters describe and what a consumer reads
	#    off these messages. Drawing it at the base rate means scaling up by
	#    sqrt(window): averaging N independent draws divides their variance by N,
	#    so to land on a chosen output variance the base rate has to be sqrt(N)
	#    times larger. Doing it here rather than baking 2.0 into the defaults is
	#    what keeps the match intact if ins_rate ever changes.
	var speed := imu_vel.length()
	var vib_gain := vibration_scale * (speed / maxf(vibration_reference_speed, 1e-6))
	var scale_up := sqrt(float(_decim_window()))
	var vib_a_std := accel_vibration_std * vib_gain * scale_up
	var vib_g_std := gyro_vibration_std * vib_gain * scale_up

	# 6. Final base-rate readings.
	noisy_accel = _lpf_accel + _accel_bias + _noise_vec(accel_noise_std) + _noise_vec_v(vib_a_std)
	noisy_gyro = _lpf_gyro + _gyro_bias + _noise_vec(gyro_noise_std) + _noise_vec_v(vib_g_std)

	# 7. Track the variance actually being produced, so the published covariance
	#    describes this error rather than asserting a datasheet figure.
	var floor_a := Vector3.ONE * accel_noise_std
	var floor_g := Vector3.ONE * gyro_noise_std
	_accel_white_var = _vsq(floor_a) + _vsq(vib_a_std)
	_gyro_white_var = _vsq(floor_g) + _vsq(vib_g_std)

	# 8. Decimate down to ins_rate by averaging the base-rate samples. This is
	#    what the real driver does with `rate_divisor`, and it is the step that
	#    was missing: holding one sample per output period correlated 3 of every
	#    4 outputs with their predecessor and aliased the per-tick noise on top.
	_decim_accel += noisy_accel
	_decim_gyro += noisy_gyro
	_decim_n += 1
	var window := _decim_window()
	if _decim_n < window:
		return
	var n := float(_decim_n)
	noisy_accel = _decim_accel / n
	noisy_gyro = _decim_gyro / n
	_decim_accel = Vector3.ZERO
	_decim_gyro = Vector3.ZERO
	_decim_n = 0

	_publish_output(delta * n)

## Wipe the sensor's own state. Called when the car is respawned, and before the
## first tick. Everything that carries across a pose change has to go: the
## velocity the acceleration is differenced against, both filters, the bias
## random walks, and the INS solution's own error state -- a respawn is a new
## run, not a continuation of the last one.
func reset() -> void:
	_seed_pending = true
	_last_imu_vel = Vector3.ZERO
	_lpf_accel = Vector3.ZERO
	_lpf_gyro = Vector3.ZERO
	_accel_bias = Vector3.ZERO
	_gyro_bias = Vector3.ZERO
	_decim_accel = Vector3.ZERO
	_decim_gyro = Vector3.ZERO
	_decim_n = 0
	_accel_white_var = Vector3.ZERO
	_gyro_white_var = Vector3.ZERO
	_ins_quality = 0.5
	_ins_pos_err = Vector3.ZERO
	_ins_vel_err = Vector3.ZERO
	_att_err = Vector3.ZERO
	_ins_elapsed = 0.0
	_fix_elapsed = 0.0
	# The odom origin is latched from the first solution this device trusts, so
	# a new run has to latch a new one -- otherwise the pose origin would be
	# pinned to wherever the car happened to be on the previous respawn.
	_have_ref = false
	_init_ins_mode()

# --- The driver's publish loop ----------------------------------------------

## One output measurement, converted -- the sim's Pipeline::process.
##
## Driven from _physics_process once per decimation window rather than from a
## RosTimer: RosTimer callbacks reach Godot through call_deferred, so a timer
## nominally at ins_rate actually fires on a render-frame boundary, and with
## vsync off that frame rate floats with load. Arrival intervals jittered even
## though the stamps themselves came from /clock.
func _publish_output(dt: float) -> void:
	var body := get_parent() as RigidBody3D
	if not body: return
	_ensure_mounting()

	_ins_elapsed += dt
	_fix_elapsed += dt
	_advance_ins_error(dt)
	_advance_ins_mode(dt)

	var stamp = _node.now()

	# The attitude solution, shared by every message below exactly as the
	# driver's single `ned_attitude` is. Publishing an exact orientation on the
	# IMU and a perturbed one on the odometry would be two attitudes, which the
	# device does not have.
	var attitude := _noisy_attitude()

	# Always produced, both frames, from the one sample. The covariances cover both
	# messages, so they are filled first.
	_fill_imu_covariances()
	_fill_imu(_cog_msg, stamp, parent_frame_id, _cog_from_imu, attitude)
	_imu_in_cog_pub.publish(_cog_msg)
	_fill_imu(_lidar_msg, stamp, lidar_frame_id, _lidar_from_imu, attitude)
	_imu_in_lidar_pub.publish(_lidar_msg)

	# One position error, two messages -- still. The driver now reads the fix
	# from the GNSS binary group rather than from posLla, so the two are no
	# longer literally the same field, but on the device the INS solution is
	# built from this receiver: they stay drawn from one error process here,
	# because a filter handed both must not treat them as independent.
	var lla := _ins_solution()

	# The receiver has a fix long before the INS converges, and the driver
	# publishes it from that moment. Before this change the whole GNSS side of
	# the stack was silent until the INS came up, which is precisely when
	# someone debugging a car that will not converge needs to see it.
	#
	# At the receiver's own solution rate, though. This used to go out once per
	# IMU measurement -- the same fix repeated up to ins_rate/gnss_rate times,
	# which is the "evidence that does not exist" imu_pipeline.hpp guards
	# against by keying on GPS time-of-week.
	if _gnss_has_fix() and _fix_elapsed >= 1.0 / float(maxi(1, gnss_rate_hz)):
		_fix_elapsed = 0.0
		_publish_fix(stamp, lla)

	if not _ins_tracking():
		return

	_publish_odom(body, stamp, lla, attitude)

## Whether the GNSS receiver itself has a position fix.
##
## Separate from, and earlier than, _ins_tracking(): acquiring satellites is a
## receiver job that finishes in seconds, while the INS alignment behind it can
## take a minute or more.
func _gnss_has_fix() -> bool:
	return _ins_elapsed >= gnss_startup_delay_sec

## VectorNav INS mode 2 is "tracking"; 0, 1 and 3 are no solution, aligning, and
## dead reckoning after GPS loss. None of them may publish odometry.
##
## Episodic, not per-sample: see ins_tracking_fraction for why a coin flip per
## measurement was wrong.
func _ins_tracking() -> bool:
	if _ins_elapsed < ins_startup_delay_sec:
		return false
	return _ins_tracking_now

## Start the filter in mode 2 with a dwell drawn from the tracking distribution,
## so the first outage lands at a random point rather than immediately.
func _init_ins_mode() -> void:
	_ins_tracking_now = true
	_ins_mode_remaining = _draw_dwell(_tracking_dwell_sec())

## Mean dwell in mode 2, derived so that the stationary duty cycle is
## ins_tracking_fraction whatever the outage length is set to.
func _tracking_dwell_sec() -> float:
	var dead := maxf(ins_dead_dwell_sec, 1e-6)
	var stay := clampf(ins_tracking_fraction, 0.0, 1.0)
	return dead * stay / maxf(1.0 - stay, 1e-6)

## Exponential dwell. A constant dwell would make the outage runs evenly spaced,
## which is its own kind of tell; the exponential is the correct distribution
## for a two-state continuous-time Markov chain and is what makes the average
## dwell exactly the value passed in.
func _draw_dwell(mean: float) -> float:
	if mean <= 0.0:
		return INF
	return -log(maxf(randf(), 1e-12)) * mean

## Advance the filter mode, flipping it when the current dwell expires.
func _advance_ins_mode(dt: float) -> void:
	_ins_mode_remaining -= dt
	if _ins_mode_remaining > 0.0:
		return
	_ins_tracking_now = not _ins_tracking_now
	_ins_mode_remaining = _draw_dwell(
		_tracking_dwell_sec() if _ins_tracking_now else ins_dead_dwell_sec)

## Fill one Imu message for the frame `frame_from_imu` targets.
##
## Mirrors Pipeline::imu_message: rotate the sensor's reading into the target
## frame, then transfer it along the mounting arm. Angular acceleration is
## neglected in the transfer there, so it is neglected here too -- the sim
## carries the driver's approximation rather than papering over it.
func _fill_imu(msg: RosSensorMsgsImu, stamp, frame: String,
		frame_from_imu: Transform3D, attitude: Basis) -> void:
	var rot := frame_from_imu.basis
	var ang := rot * noisy_gyro
	# transfer_acceleration: a - w x (w x arm), arm being where the IMU sits in
	# the target frame.
	var acc := (rot * noisy_accel) - ang.cross(ang.cross(frame_from_imu.origin))
	# orientation_of: world <- frame = (world <- imu)(imu <- frame)
	var orientation := _to_ros_quat(Quaternion(attitude * rot.inverse().orthonormalized()))

	msg.header.stamp = stamp
	msg.header.frame_id = _node.resolve_frame(frame)
	_set_vec(msg.angular_velocity, ang)
	_set_vec(msg.linear_acceleration, acc)
	_set_quat(msg.orientation, orientation)

## The INS position solution, as latitude/longitude/altitude -- what posLla
## carries on the real device, and the one place the solution's position error
## enters. Everything geodetic downstream is derived from this, so the fix and
## the odometry cannot drift apart.
func _ins_solution() -> Vector3:
	# Ground truth of this sensor, in metres north/east/down of the world origin.
	var p := global_position
	var ned := Vector3(-p.z, p.x, -p.y) + _ins_pos_err

	var phi := deg_to_rad(origin_lat)
	var den := sqrt(1.0 - WGS84_E2 * sin(phi) * sin(phi))
	var m_rad := WGS84_A * (1.0 - WGS84_E2) / (den * den * den)
	var n_rad := WGS84_A / den
	return Vector3(
		origin_lat + rad_to_deg(ned.x / m_rad),
		origin_lon + rad_to_deg(ned.y / (n_rad * cos(phi))),
		origin_alt - ned.z)

func _publish_odom(body: RigidBody3D, stamp, lla: Vector3, attitude: Basis) -> void:
	# The odom frame is pinned to wherever the car was when the INS first had a
	# solution worth trusting, latched here so there is exactly one place it can
	# happen. Re-latched when the car is respawned, which the real driver has no
	# equivalent of -- without it a sim reset would teleport the whole stack.
	var car_origin: Transform3D = body.origin if "origin" in body else Transform3D.IDENTITY
	if not _have_ref or not car_origin.is_equal_approx(_ref_car_origin):
		_geodesic_ref = lla
		_ref_car_origin = car_origin
		_have_ref = true

	var ned := _lla_to_ned(lla, _geodesic_ref)
	# flip_x: NED -> the NWU world the stack publishes in.
	var imu_position := Vector3(ned.x, -ned.y, -ned.z)

	var cog_orientation := _to_ros_quat(Quaternion(
		attitude * _cog_from_imu.basis.inverse().orthonormalized()))

	# The IMU is not the car. Shift the reported position along the arm from the
	# IMU to the cog, rotated into the world by the car's own attitude -- the arm
	# is a body-frame quantity, and adding it unrotated is a heading-dependent
	# error the size of the arm.
	var cog_position := imu_position - (cog_orientation * _to_ros_vec(_cog_from_imu.origin))

	# Velocity and rate stay in the body frame, which is what nav_msgs/Odometry
	# asks for and what the EKF consumes as u, v and omega.
	# velBody: the solution's velocity in the sensor's own axes, error included.
	var imu_vel := global_basis.inverse() * _last_imu_vel
	var cog_ang := _cog_from_imu.basis * noisy_gyro
	# transfer_velocity: v - w x arm.
	var cog_vel := (_cog_from_imu.basis * (imu_vel + _ins_vel_err)) \
		- cog_ang.cross(_cog_from_imu.origin)

	_odom_msg.header.stamp = stamp
	_odom_msg.header.frame_id = _node.resolve_frame(odom_frame_id)
	_odom_msg.child_frame_id = _node.resolve_frame(parent_frame_id)

	_odom_msg.pose.pose.position.x = cog_position.x
	_odom_msg.pose.pose.position.y = cog_position.y
	_odom_msg.pose.pose.position.z = cog_position.z
	_set_quat(_odom_msg.pose.pose.orientation, cog_orientation)
	_set_vec(_odom_msg.twist.twist.linear, cog_vel)
	_set_vec(_odom_msg.twist.twist.angular, cog_ang)

	_fill_odom_covariances()
	_odom_pub.publish(_odom_msg)

func _publish_fix(stamp, lla: Vector3) -> void:
	_fix_msg.header.stamp = stamp
	_fix_msg.header.frame_id = _node.resolve_frame(gnss_frame_id)
	_fix_msg.status.status = 0    # STATUS_FIX
	_fix_msg.status.service = 1   # SERVICE_GPS
	_fix_msg.latitude = lla.x
	_fix_msg.longitude = lla.y
	_fix_msg.altitude = lla.z

	# The GNSS group reports a real per-axis uncertainty, unlike the INS's one
	# isotropic sigma, so this is DIAGONAL_KNOWN rather than APPROXIMATED. In
	# ENU order, as NavSatFix specifies -- the driver has to swap north and east
	# coming off the device, which reports NED.
	#
	# Vertical is worse than horizontal on any GNSS receiver, because the
	# satellites are all on one side of the antenna; 1.5x is the usual rule of
	# thumb for VDOP against HDOP.
	var var_h := pow(_ins_pos_sigma, 2)
	var var_v := pow(_ins_pos_sigma * 1.5, 2)
	_fix_msg.position_covariance[0] = var_h   # east
	_fix_msg.position_covariance[4] = var_h   # north
	_fix_msg.position_covariance[8] = var_v   # up
	_fix_msg.position_covariance_type = 2   # COVARIANCE_TYPE_DIAGONAL_KNOWN

	_fix_pub.publish(_fix_msg)

# --- Error model -------------------------------------------------------------

## Walk the INS solution's error forward one measurement.
func _advance_ins_error(dt: float) -> void:
	# Solution quality wanders slowly and bounded; sigma follows it and so does
	# the error, so a degrading fix widens the covariance and drifts further out
	# together rather than one without the other.
	_ins_quality = clampf(_ou_step(_ins_quality - 0.5, 0.25, ins_sigma_tau, dt) + 0.5, 0.0, 1.0)
	_ins_pos_sigma = lerpf(ins_pos_sigma_min, ins_pos_sigma_max, _ins_quality)
	_ins_vel_sigma = lerpf(ins_vel_sigma_min, ins_vel_sigma_max, _ins_quality)

	_ins_pos_err = _ou_vec(_ins_pos_err, _ins_pos_sigma, ins_pos_tau, dt)
	_ins_vel_err = _ou_vec(_ins_vel_err, _ins_vel_sigma, ins_vel_tau, dt)
	_att_err = Vector3(
		_ou_step(_att_err.x, ins_roll_sigma, ins_att_tau, dt),
		_ou_step(_att_err.y, ins_pitch_sigma, ins_att_tau, dt),
		_ou_step(_att_err.z, ins_yaw_sigma, ins_att_tau, dt))

## Ground-truth attitude perturbed by the solution's attitude error.
##
## The ROS axes map onto Godot's as x = -z, y = -x, z = +y, so a ROS roll is a
## rotation about *negative* Godot z and a ROS pitch about negative Godot x --
## only yaw keeps its sign. Zero-mean noise hides the difference statistically,
## which is exactly why it is worth writing down rather than leaving to be
## rediscovered. Applied on the left, so the perturbation is about world axes the
## way an attitude error is.
func _noisy_attitude() -> Basis:
	return Basis.from_euler(Vector3(-_att_err.y, _att_err.z, -_att_err.x)) * global_basis

## One step of an Ornstein-Uhlenbeck process: exponentially correlated noise of
## time constant `tau` whose stationary standard deviation is `sigma`. The exact
## discretisation, so the result does not depend on the rate it is stepped at.
func _ou_step(value: float, sigma: float, tau: float, dt: float) -> float:
	if tau <= 0.0:
		return randfn(0.0, sigma)
	var decay := exp(-dt / tau)
	return value * decay + randfn(0.0, sigma * sqrt(1.0 - decay * decay))

func _ou_vec(value: Vector3, sigma: float, tau: float, dt: float) -> Vector3:
	return Vector3(
		_ou_step(value.x, sigma, tau, dt),
		_ou_step(value.y, sigma, tau, dt),
		_ou_step(value.z, sigma, tau, dt))

func _noise_vec(std: float) -> Vector3:
	return Vector3(randfn(0, std), randfn(0, std), randfn(0, std))

## Noise with a different standard deviation per axis, so the vibration term can
## follow the measured per-axis spread rather than pretending it is isotropic.
func _noise_vec_v(std: Vector3) -> Vector3:
	return Vector3(randfn(0, std.x), randfn(0, std.y), randfn(0, std.z))

## Component-wise square, i.e. the per-axis variance of a deviation vector.
##
## Written out by hand rather than as `v * v` on purpose: `Vector3.square()` was
## assumed here and does not exist in Godot 4, and because a parse error takes the
## whole script down with it, that mistake silently removed all four INS
## publishers. Vector3 has no component-wise square in any Godot 4 release, and
## spelling it out means there is nothing left to misremember.
func _vsq(v: Vector3) -> Vector3:
	return Vector3(v.x * v.x, v.y * v.y, v.z * v.z)

# --- Covariances -------------------------------------------------------------

## Covariances of the decimation *output*, refilled every sample because the
## vibration term scales with speed and the window's length is what decides how
## much of the white part survives the averaging.
##
## The white terms are divided by the window because averaging N independent
## draws divides their variance by N; the bias walk is common to every sample in
## the window, so it is not reduced at all. Reporting the white part unreduced
## would overstate the error by the window length; reporting the bias reduced
## would understate it.
func _fill_imu_covariances() -> void:
	var n := float(maxi(1, _decim_window()))
	var a_var := (_accel_white_var / n + _vsq(_accel_bias)) * imu_covariance_scale
	var g_var := (_gyro_white_var / n + _vsq(_gyro_bias)) * imu_covariance_scale
	for i in range(3):
		_cog_msg.linear_acceleration_covariance[i * 4] = a_var[i]
		_cog_msg.angular_velocity_covariance[i * 4] = g_var[i]
		_lidar_msg.linear_acceleration_covariance[i * 4] = a_var[i]
		_lidar_msg.angular_velocity_covariance[i * 4] = g_var[i]
	if imu_covariance_scale > 0.0:
		var s := imu_covariance_scale
		for msg in [_cog_msg, _lidar_msg]:
			msg.orientation_covariance[0] = pow(ins_roll_sigma, 2) * s
			msg.orientation_covariance[4] = pow(ins_pitch_sigma, 2) * s
			msg.orientation_covariance[8] = pow(ins_yaw_sigma, 2) * s

## Refilled every solution, not once at startup: on the car these are the onboard
## filter's own 1-sigma uncertainties and they move as the fix gets better or
## worse. Row-major 6x6 over [x, y, z, roll, pitch, yaw] / [vx, vy, vz, wx, wy,
## wz]. Position is isotropic because the filter reports one posU figure, not a
## horizontal/vertical split.
func _fill_odom_covariances() -> void:
	var pos_var := pow(_ins_pos_sigma, 2)
	var vel_var := pow(_ins_vel_sigma, 2)

	var pose_cov := PackedFloat64Array()
	pose_cov.resize(36)
	var twist_cov := PackedFloat64Array()
	twist_cov.resize(36)
	for i in range(3):
		pose_cov[i * 7] = pos_var
		twist_cov[i * 7] = vel_var
		twist_cov[(i + 3) * 7] = ins_gyro_var[i]
	pose_cov[21] = pow(ins_roll_sigma, 2)
	pose_cov[28] = pow(ins_pitch_sigma, 2)
	pose_cov[35] = pow(ins_yaw_sigma, 2)
	_odom_msg.pose.covariance = pose_cov
	_odom_msg.twist.covariance = twist_cov

# --- Geometry and conversions ------------------------------------------------

## Latch the mounting geometry once (ImuNode::ensure_mounting). The scene is the
## sim's TF: this node's own transform is car/cog <- car/imu, and the lidar's
## gives car/lidar <- car/imu. They are bolts, not motion, so re-deriving them
## per measurement would buy nothing and cost a failure mode.
func _ensure_mounting() -> void:
	if _have_mounting:
		return
	_have_mounting = true
	_cog_from_imu = transform

	var found := false
	var root := get_parent()
	if root:
		for child in root.get_children():
			if child != self and child is Node3D and "frame_id" in child \
					and child.frame_id == lidar_frame_id:
				_lidar_from_imu = (child as Node3D).transform.affine_inverse() * transform
				found = true
				break
	if not found:
		# Left at identity, which would make /sensors/lidar/imu a copy of
		# /sensors/imu -- plausible-looking and wrong, so say so out loud.
		push_warning("RosIns: no sibling mounted at '%s'; the lidar IMU stream will "
			% lidar_frame_id + "duplicate the cog one.")
	print("RosIns: mounting latched, IMU sits at %s in %s." % [_cog_from_imu.origin, parent_frame_id])

## Latitude/longitude/altitude to metres north, east and down of a reference --
## mirena::GNSSConversion::latLonAltToNED, ported so the odometry is derived from
## the fix by the same arithmetic the driver uses.
func _lla_to_ned(lla: Vector3, ref: Vector3) -> Vector3:
	var p := _geodetic_to_ecef(lla)
	var p_ref := _geodetic_to_ecef(ref)
	var d := p - p_ref

	var ref_lat := deg_to_rad(ref.x)
	var ref_lon := deg_to_rad(ref.y)
	var sin_lat := sin(ref_lat)
	var cos_lat := cos(ref_lat)
	var sin_lon := sin(ref_lon)
	var cos_lon := cos(ref_lon)

	return Vector3(
		-sin_lat * cos_lon * d.x - sin_lat * sin_lon * d.y + cos_lat * d.z,
		-sin_lon * d.x + cos_lon * d.y,
		-cos_lat * cos_lon * d.x - cos_lat * sin_lon * d.y - sin_lat * d.z)

func _geodetic_to_ecef(lla: Vector3) -> Vector3:
	var lat := deg_to_rad(lla.x)
	var lon := deg_to_rad(lla.y)
	var sin_lat := sin(lat)
	var cos_lat := cos(lat)
	var n_rad := WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
	return Vector3(
		(n_rad + lla.z) * cos_lat * cos(lon),
		(n_rad + lla.z) * cos_lat * sin(lon),
		(n_rad * (1.0 - WGS84_E2) + lla.z) * sin_lat)

## Godot -> ROS. Godot is +x right, +y up, -z forward; ROS is +x forward, +y
## left, +z up. Right-handed both ways, so this is a change of basis and nothing
## more -- and applying it to the world gives north = -z, west = -x, up = +y.
func _to_ros_vec(v: Vector3) -> Vector3:
	return Vector3(-v.z, -v.x, v.y)

func _to_ros_quat(q: Quaternion) -> Quaternion:
	return Quaternion(-q.z, -q.x, q.y, q.w)

## Writes a Godot-axis vector into a ROS message field, converting on the way.
func _set_vec(out, v: Vector3) -> void:
	var r := _to_ros_vec(v)
	out.x = r.x
	out.y = r.y
	out.z = r.z

## Writes a quaternion that is *already* in ROS axes. Unlike _set_vec this
## converts nothing -- callers hold ROS orientations because they compose them
## with ROS positions.
func _set_quat(out, q: Quaternion) -> void:
	out.x = q.x
	out.y = q.y
	out.z = q.z
	out.w = q.w
