# `mirena_state` node

State estimator for the Mirena car. It fuses the sensor layer into one
`/state/car` estimate and owns the `odom -> car/cog` transform.

Source:

- `src/mirena_state.cpp` — ROS node, sensor callbacks, INS anchoring, publishing
- `src/ekf.cpp` / `src/ekf.hpp` — the 7-state EKF (6 kinematic states + IMU `a_x` bias)
- `src/fast_lio.cpp` — fully commented-out LiDAR alternative; not built

Build target is `mirena_state` (`CMakeLists.txt`). See the note at the bottom
about launch files referencing `mirena_state_ekf`.

---

## 1. Interfaces

### Subscriptions

| Topic | Type | Source | Notes |
|---|---|---|---|
| `/sensors/imu` | `sensor_msgs/Imu` | VN-300 driver, COG frame, gravity removed | 100 Hz |
| `/sensors/imu/odom` | `nav_msgs/Odometry` | VN-300 onboard INS | 100 Hz, **only while the INS is tracking** |
| `/sensors/wheel_speeds` | `mirena_common/WheelSpeeds` | CAN `WheelInfo` (rear-left/right **RPM**) | CAN rate, optional (`use_encoders`) |
| `/control/can` | `mirena_common/CarControl` | CAN steering sensor, republished by `mirena_can` | CAN rate, when `use_bicycle_model` |

Topics are defined at `mirena_state.cpp:30-33`; parameters at
`mirena_state.cpp:149-215`.

### Publications

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/state/car` | `mirena_common/Car` | IMU-clocked, ~100 Hz | `x, y, psi, u, v, omega` + row-major 6x6 covariance |
| `/health` | `mirena_common/Heartbeat` | 10 Hz | JSON status (mode, ages, re-anchors, resets) |
| TF `odom -> car/cog` | `geometry_msgs/TransformStamped` | same as `/state/car` | planar pose (`z = 0`) |

`/state/car` and TF are published **only from the IMU callback**
(`mirena_state.cpp:419, 695-721`). INS and encoder updates do not trigger a
publication, and a state whose stamp did not advance is suppressed.

Threading: the three filter subscriptions share one `MutuallyExclusive`
callback group and the filter is additionally guarded by `_filter_mutex`
(`mirena_state.cpp:213-214, 417`). The health timer runs in a separate group.
The executor is `MultiThreadedExecutor`.

---

## 2. State vector and frames

\[
x = [x,\;y,\;\psi,\;u,\;v,\;\omega,\;b_{ax}]^T
\]

- `x, y, psi` — COG pose in the local `odom` frame.
- `u, v` — **body-frame** longitudinal / lateral velocity at the COG
  (`dbm_model.py` conventions).
- `omega` — yaw rate.
- `b_ax` — bias of the IMU longitudinal acceleration, m/s², modeled as a
  random walk. The INS/GPS does not measure it; it is observed indirectly
  through the encoder `u` update.
- `P` — 7x7 covariance. `/state/car` still publishes the leading 6x6 block
  because the `Car` message has no bias field; `/health` reports `imu_ax_bias`
  and `imu_ax_bias_var`.
- Vehicle geometry used: COG-to-rear-axle `L_R = 0.744 m`
  (`cog_to_rear_axle_m`), wheel radius `0.20 m`, track width `1.2 m`.

The INS has its own odom frame, unrelated to the EKF's. The transform
`T_odom <- ins` is latched on the first INS fix and is the subject of section 5.

---

## 3. Prediction model

`Ekf::predict(stamp, ax, ay, have_steering, steering)` — `ekf.cpp:57-151`.

Called on every IMU sample, and as a "time-sync" step at the start of the
encoder and INS callbacks so that innovations are not taken against a pose up
to one IMU period stale.

Two models are available:

- **Kinematic bicycle** (default, `use_bicycle_model=true`)
- **Body-frame Euler** (fallback: disabled model, missing/stale steering, or
  speed below `bicycle_min_speed`)

Only one model runs per prediction step; the choice is made in
`predict_at()` / `steering_at()` in the node.

### 3.1 Kinematic bicycle (default)

Measured steering from `/control/can` drives a single-track kinematic model
expressed at the COG:

\[
\kappa = \frac{\tan\delta}{L_F+L_R},\qquad
\omega_{kin} = u\,\kappa,\qquad
v_{kin} = L_R\,\omega_{kin}
\]

Instead of overwriting `v` and `omega` (which would make the state singular and
throw away the gyro and the INS), they follow the kinematic relation through a
first-order lag:

\[
\alpha = 1 - e^{-K\,dt}
\]

\[
\begin{aligned}
x^+ &= x + (u\cos\psi - v\sin\psi)\,dt\\
y^+ &= y + (u\sin\psi + v\cos\psi)\,dt\\
\psi^+ &= \operatorname{wrap}(\psi + \omega\,dt)\\
u^+ &= u + (a_x - b_{ax} + v\,\omega)\,dt\\
v^+ &= v + \alpha\,(L_R\,\omega - v)\\
\omega^+ &= \omega + \alpha\,(\omega_{kin} - \omega)\\
b_{ax}^+ &= b_{ax}
\end{aligned}
\]

with `K = bicycle_relaxation = 20 1/s` by default, matching `K_RELAX` in
`mirena_control/src/mpc/codegen/dbm_model.py`. `alpha` is the exact
discretisation, so a large `dt` cannot overshoot.

Properties and caveats:

- `a_y` is **not used** in this model; the lateral motion comes from steering.
  `a_x - b_ax` drives `u`, and the bias state absorbs a slowly varying
  longitudinal accelerometer offset.
- `v` and `omega` are corrected every step by the gyro (4.1), encoder (4.2) and
  INS (4.3) updates, so the model is a prediction, not a hard constraint. The
  encoder no-slip update is now partly redundant; it can be disabled with
  `no_slip_var <= 0` if it is seen fighting the model.
- Below `bicycle_min_speed = 0.5 m/s` the Euler model is used instead. The
  kinematic relation says `omega = 0` at rest, which is wrong for a spin
  (doughnut, handbrake turn) where the gyro is the only truthful source.
- The steering reading is clamped to `max_steering_rad = 0.6` (~34 deg) before
  it reaches the model.
- If the steering sensor is not calibrated yet, lower `bicycle_relaxation`
  and/or raise `steering_noise_rad`. With a bad offset the model pulls `omega`
  every step, and the gyro's NIS gate can start rejecting the very measurement
  that would correct it.

Jacobian `F = I + A dt`:

```text
F(0,2) = (-u s - v c) dt        F(0,3) =  c dt        F(0,4) = -s dt
F(1,2) = ( u c - v s) dt        F(1,3) =  s dt        F(1,4) =  c dt
F(2,5) = dt
F(3,4) =  omega dt               F(3,5) =  v dt        F(3,6) = -dt
F(4,4) =  1 - alpha              F(4,5) =  alpha L_R
F(5,3) =  alpha kappa            F(5,5) =  1 - alpha
F(6,6) =  1  (random walk de b_ax)
```

`v` relaxes towards `L_R * omega` (the *actual* yaw rate, not the steering
target) so that the model and the encoder no-slip update agree during the
transient; `omega` is what steering drives.

Steering-angle uncertainty is injected as input noise rather than as a state:

\[
G = \begin{bmatrix}0&0&0&0&0&
\alpha\,\frac{u}{L\cos^2\delta}\end{bmatrix}^T
\]

\[
P \leftarrow FPF^T + \operatorname{diag}(Q)\,dt
      + \sigma_\delta^2\,GG^T
\]

with `sigma_delta = steering_noise_rad = 0.02 rad` (1.1 deg) by default.

### 3.2 Body-frame Euler (fallback)

Used when `use_bicycle_model=false`, no fresh steering, or `|u|` below
`bicycle_min_speed`:

\[
\begin{aligned}
x^+ &= x + (u\cos\psi - v\sin\psi)\,dt\\
y^+ &= y + (u\sin\psi + v\cos\psi)\,dt\\
\psi^+ &= \operatorname{wrap}(\psi + \omega\,dt)\\
u^+ &= u + (a_x + v\,\omega)\,dt\\
v^+ &= v + (a_y - u\,\omega)\,dt\\
\omega^+ &= \omega
\end{aligned}
\]

Jacobian `F = I + A dt`:

```text
F(0,2) = (-u sin - v cos) dt    F(0,3) =  cos dt    F(0,4) = -sin dt
F(1,2) = ( u cos - v sin) dt    F(1,3) =  sin dt    F(1,4) =  cos dt
F(2,5) = dt
F(3,4) =  omega dt              F(3,5) =  v dt    F(3,6) = -dt
F(4,3) = -omega dt              F(4,5) = -u dt
```

In this mode both `a_x` and `a_y` are the gravity-compensated IMU
accelerations rotated to the COG. `a_x - b_ax` drives `u`; `b_ax` is a
random-walk state observed indirectly through the encoder `u` update, so a
constant longitudinal accelerometer bias is absorbed instead of integrated
into position. The gyro is not a control input in either model: heading is
propagated through the `omega` state and the gyro corrects it as a measurement.

### 3.3 Common to both models

\[
P \leftarrow FPF^T + \operatorname{diag}(Q)\,dt
\]

- `Q = process_noise_diag = [0.01, 0.01, 0.005, 0.5, 0.5, 0.5, 1e-6]`
  (continuous-time). The last entry is the random-walk intensity of `b_ax`.
- `dt = clamp(t - stamp_, 1e-6, min(max_predict_dt_sec, 0.5))`, with
  `max_predict_dt_sec = 0.1` by default. The hard 0.5 s cap is a safety net
  against a bad parameter or a clock jump.
- `stamp_` is set to the **full** `t` before the clamp. A gap longer than
  100 ms is therefore **skipped, not integrated**: after a 1 s dropout the
  filter advances only 0.1 s of motion. This prevents huge covariance jumps but
  loses distance travelled during an INS outage.
- `elapsed <= 0` returns `Ignored` (out-of-order or duplicate sample).

### Commit / divergence reset

Every operation ends in `commit()` (`ekf.cpp:32-43`):

- If any state or covariance element is non-finite, the filter is reset and
  `divergences_` is incremented. The next INS fix re-initializes it.
- Diagonal variances are floored at `kMinVariance = 1e-12`.
- `symmetrize()` restores `P = (P + P^T)/2`; it does not guarantee positive
  definiteness.

---

## 4. Measurement updates

General form: `y = z - Hx`, `S = HPH^T + R`, `K = PH^T S^-1`,
`x += K y`. All updates are sequential on the same 7-state filter. Because
`K` is a full 7-vector, a scalar measurement also corrects every state it is
correlated with through `P`; the INS/GPS `H` selects only the first six states
and leaves `b_ax` to be observed through the encoder velocity update.

### 4.1 IMU gyro yaw rate -> `omega`

- Callback: `imu_callback`, `mirena_state.cpp:415-430`
- Update: `update_gyro(omega_z, gyro_meas_var)` — `ekf.cpp:154-179`
- `H = [0,0,0,0,0,1,0]`, `R = gyro_meas_var = 1e-5 rad^2/s^2`
- **NIS gate**: if `y^2 / S > 6.63` (chi-square, 99 %, 1 dof) the sample is
  silently dropped. Logged as `Rejected` once every 2 s.
- Joseph-form covariance update:
  `P = (I-KH)P(I-KH)^T + K R K^T`
- Also nudges `psi` and `v` through cross-covariances.

### 4.2 Wheel encoders -> `u`, `omega`, and lateral no-slip

Callback: `encoder_callback`, `mirena_state.cpp:432-514`.

Conversion (`mirena_state.cpp:465-468`):

\[
v_l = rl\frac{2\pi}{60}r,\qquad v_r = rr\frac{2\pi}{60}r,\qquad
u_{enc} = \frac{v_l+v_r}{2},\qquad \omega_{enc} = \frac{v_r-v_l}{b}
\]

with `r = wheel_radius = 0.20 m`, `b = track_width = 1.2 m`.

Rejection guards:

- null (`0,0`) header stamp
- non-finite `rl` / `rr`
- `|u| > 15 m/s` (outlier)
- `t < ekf->stamp()` (out-of-order)
- if `t > ekf->stamp()`, a zero-acceleration predict advances the filter first

Both of the last two apply only while `sensor_time_sync_predict` is true. Set
it to `false` for bags where the IMU driver's latency leaves the IMU header
stamp a few milliseconds behind the encoder and INS: otherwise the encoder
advances the filter past every IMU and the prediction is never integrated.
With `false`, the encoder/INS measurement is applied at the current filter
stamp (at most one IMU period old) and the IMU remains the clock. The recorded
`rosbag2_2026_10_06-18_02_11` bag needs this: its IMU stamp starts about
67 ms behind the encoder and the offset decays to about 1 ms after 160 s.

#### (a) Longitudinal speed

`update_velocity_u(u, var_u)` — `ekf.cpp:253-282`

- `H = [0,0,0,1,0,0,0]`, `S = P33 + var_u`, `K = P.col(3)/S`
- `P = (I-KH)P` (not Joseph form), then symmetrized
- Dynamic measurement variance:

\[
var_u = \min\bigl(0.01 + 0.10\,(v_l-v_r)^2,\; 1.0\bigr)
\]

Wheel disagreement (slip, cornering) inflates `R`, so the encoder does not
fight the car at speed.

#### (b) Differential-wheel yaw rate

`update_gyro(omega_enc, var_w)` — same scalar update as 4.1.

\[
var_w = \min\left(\frac{2\cdot0.01}{b^2} + 0.10\,(v_l-v_r)^2,\; 0.5\right)
\]

Caveat: the second term has units of `(m/s)^2` but is added to a yaw-rate
variance; dimensionally it should be divided by `b^2`. It is capped at `0.5`
so it only reads as very conservative. See `mirena_state.cpp:480-482`.

#### (c) Rear-axle no-slip pseudo-measurement

`update_nonholonomic(L_R, var_v)` — `ekf.cpp:297-323`

This is not a sensor: it encodes the kinematic constraint that the rear axle
does not slip. In the state at the COG, the rear-axle lateral velocity is
`v - L_R * omega` (same quantity as `alpha_r = (v - L_R omega)/u` in
`dbm_model.py`).

\[
H = [0,0,0,0,1,-L_R,0],\qquad z = 0,\qquad y = -(x_4 - L_R x_5)
\]

\[
var_v = \min\bigl(0.01 + 0.004\,(u^2+v^2),\; 1.0\bigr)
\]

- Speed-dependent variance: tight while parked, loose in a fast corner.
- Joseph-form covariance update.
- This is the **lateral half** of the encoder update. `u` pins longitudinal
  motion; without this, nothing observes `v` when the INS drops out and `y`
  random-walks.
- Disable by setting `no_slip_var <= 0`.

### 4.3 INS odometry -> all six states

Callback: `ins_odom_callback`, `mirena_state.cpp:578-690`
Update: `update_odom(z, r)` — `ekf.cpp:181-199`

- `H = I_6`: the INS is a direct measurement of the full state.
- Measurement vector:
  - position and yaw from the INS pose, mapped through the anchor
    (`ins_to_odom`, `mirena_state.cpp:569-576`)
  - `u, v, omega` from `msg->twist.twist.*` raw (already body/COG frame)
- Measurement covariance from the message (diagonal only):

```text
R = [ pose.covariance[0],  pose.covariance[7],  pose.covariance[35],
      twist.covariance[0], twist.covariance[7], twist.covariance[35] ]
```

  each floored at `odom_meas_floor_diag = 0.01` (`mirena_state.cpp:670-677`).
  The INS's own position variance (metres-scale, `posU`) therefore dominates
  its weight in position.
- Gain `K = P (P+R)^-1`, then `P = (I-K)P`. **No NIS gate** here.
- Yaw innovation is wrapped to `[-pi, pi]`.
- If the filter is not initialized, `init(z, r, t)` is called instead.

---

## 5. INS anchoring and the teleport guard

The INS odom frame is arbitrary. On the first fix the node latches
`T_odom <- ins` such that the INS solution lands exactly on the pose the
filter currently carries (`latch_anchor`, `mirena_state.cpp:523-567`). The
state is not moved.

### First anchor

- `dpsi = wrap(x(2) - ins_psi)` — the odom heading is **defined** by this fix.
- `t = x_pos - R(dpsi) * ins_pos`.
- Reported at INFO level.

### Re-anchor

If the INS returns after a dropout and disagrees with the estimate by more
than `ins_reanchor_m = 12 m`, the measurement is **not applied to the pose**.
Instead:

- `_ins_reanchors++`
- the translation part of the anchor is re-latched so the solution maps onto
  the current pose
- the **yaw offset is deliberately kept**, because the estimate's heading is
  gyro-integrated and drifts across a dropout (measured 27.7 deg over 23 s vs
  0.7 deg from the INS). Re-deriving it would freeze that error into the
  anchor permanently.
- the measurement is re-mapped and then applied normally
- reported at WARN level

Rationale: an odom frame that never moves is worth more to the rest of the
stack than the lost absolute position. The recorded run this was measured on
teleported 12.9 m and then 13.4 m on the first messages after dropouts;
routine disagreement was p50 1.5 m / p99 8.5 m / p99.9 9.2 m.

Set `ins_reanchor_m <= 0` to disable the gate.

---

## 6. Initialization

Two paths:

1. **INS arrives first** (`mirena_state.cpp:588-603`)
   - anchor is identity, `_ins_anchored = true`
   - filter initializes from the first INS `z, r`

2. **IMU arrives first** (`init_at_origin`, `mirena_state.cpp:392-413`)
   - pose `(0,0,0)`, velocity variance `0.0025` if stationary
     (`|a| < 0.5 m/s^2` and `|omega| < 0.05 rad/s`), else `25`
   - runs dead-reckoning (`NO_INS_YET`) until the first INS fix anchors it

`init` sets `x = z` and `P = diag(max(r, init_cov_floor_diag))`.
With
`init_cov_floor_diag = [0.01, 0.01, 0.001, 0.01, 0.01, 0.001, 0.04]`, the
"stationary" 0.0025 is effectively raised to 0.01. The 6-element `init`
overload used by the INS initializes `b_ax = 0` with variance `0.04`.

After a divergence reset the filter has already been anchored, so it
re-initializes from the next INS fix rather than from the origin.

---

## 7. Health report

`report_callback` (`mirena_state.cpp:744-823`) runs at 10 Hz on `/health` and
publishes JSON:

- `mode`: `INIT`, `NO_INS_YET`, `STALE`, `DEAD_RECKONING`, `NO_IMU`, `FUSED`
  (derived from IMU / INS / encoder ages against their timeout parameters)
- `ins_anchored`, `ins_reanchors`, `ins_reanchor_m`
- `encoders_enabled`
- `odom_stale`, `imu_stale`, `encoder_stale`, and the corresponding ages
- `motion_model` (`kinematic_bicycle` or `euler`, the configured model)
- `prediction_active` (`bicycle` or `euler`, what the last step actually used,
  including steering freshness and `bicycle_min_speed`)
- `steering_seen`, `steering_stale`, `steering_rad`, `steering_age_s`
  (when the bicycle model is enabled)
- `ekf_resets` (divergence counter)
- current `car_pos_x`, `car_pos_y`, `car_pos_yaw`, `car_vel_u`

---

## 8. Parameters

Declared at `mirena_state.cpp:149-215`. The mission launch files pass the core
values shown here (e.g. `mirena_supervisor/launch/missions/autocross.launch.py:59-89`);
the bicycle parameters are node defaults and can be added to the same
`parameters=[...]` block when calibrating.

| Parameter | Default | Meaning |
|---|---|---|
| `odom_timeout_sec` | 0.5 | INS age before it is not trusted (health only) |
| `imu_timeout_sec` | 0.2 | IMU age before it is not trusted (health only) |
| `encoder_timeout_sec` | 0.2 | encoder age before it is not trusted (health only) |
| `max_predict_dt_sec` | 0.1 | longest gap the filter will integrate |
| `process_noise_diag` | `[0.01,0.01,0.005,0.5,0.5,0.5,1e-6]` | continuous `Q` diagonal; last entry is `b_ax` random walk |
| `odom_meas_floor_diag` | `[1e-2,...,1e-2]` | floor on INS `R` diagonal |
| `init_cov_floor_diag` | `[0.01,0.01,0.001,0.01,0.01,0.001,0.04]` | floor on `P` at init; last entry is `b_ax` |
| `gyro_meas_var` | `1e-5` | IMU yaw-rate variance, `rad^2/s^2` |
| `wheel_radius` | 0.2 | m |
| `track_width` | 1.2 | m |
| `encoder_var` | 0.01 | base `(m/s)^2` for the encoder update |
| `use_encoders` | true | subscribe to `/sensors/wheel_speeds` |
| `ins_reanchor_m` | 12.0 | innovation that triggers a re-anchor; `<= 0` disables |
| `encoder_yaw_var_max` | 0.5 | cap on the differential-wheel yaw variance |
| `cog_to_rear_axle_m` | 0.744 | `L_R`, no-slip constraint and bicycle geometry |
| `no_slip_var` | 0.01 | no-slip variance at standstill; `<= 0` disables |
| `use_bicycle_model` | true | kinematic bicycle prediction; false = Euler |
| `steering_topic` | `/control/can` | measured steering source |
| `steering_degrees` | true | CAN signal is in degrees, not radians |
| `steering_sign` | 1.0 | flip if the sensor sign is opposite the model |
| `steering_ratio` | 1.0 | steering-wheel : road-wheel ratio, if applicable |
| `steering_offset_rad` | 0.0 | sensor zero offset, rad (applied after ratio) |
| `steering_timeout_sec` | 0.2 | older than this falls back to Euler |
| `max_steering_rad` | 0.6 | clamp on the converted road-wheel angle |
| `steering_noise_rad` | 0.02 | 1 sigma steering uncertainty injected into `P` |
| `cog_to_front_axle_m` | 0.806 | `L_F`; wheelbase = `L_F + L_R` |
| `bicycle_relaxation` | 20.0 | `1/s`, lag towards the kinematic target (`K_RELAX`) |
| `bicycle_min_speed` | 0.5 | `m/s`, below this Euler is used |
| `sensor_time_sync_predict` | true | encoder/INS advance the filter clock before their update; `false` keeps the IMU as the only clock (see below) |

The launch files cap the IMU driver at 100 Hz specifically because that is the
rate `predict` expects (`mirena_supervisor/launch/hardware.launch.py:88-91`),
and the driver's z angular variance `1.0e-5` is kept equal to
`gyro_meas_var` (`hardware.launch.py:150-160`).

The bicycle parameters are declared by the node with the defaults above. The
mission launch files set the car-specific calibration explicitly; a launch that
omits them falls back to the nominal `0.2 m` / `1.0` values. In the simulator
nothing publishes `/control/can`, so the node falls back to Euler
automatically; to exercise the bicycle model in sim, remap `steering_topic` to
`/control` and set `steering_degrees=false`.

### Current steering and wheel calibration

The mission launch files set the car-specific values explicitly:

| Parameter | Value | Source |
|---|---:|---|
| `wheel_radius` | `0.191 m` | encoder `u` vs INS `u`, low-acceleration samples |
| `steering_ratio` | `1.35` | gyro yaw rate vs `u*tan(delta)/L`, low lateral acceleration |
| `steering_offset_rad` | `0.030 rad` | same fit |

They were measured on the `2026-10-06` bag with
`utilities/calibrate_wheel_steering.py`:

```bash
python3 src/mirena_state/utilities/calibrate_wheel_steering.py \
    rosbag2_2026_10_06-18_02_11
```

The utility prints `wheel_radius`, `steering_ratio` and
`steering_offset_rad`; it also prints the steering residual before and after
the fit. With the values above, the yaw-rate residual fell from about
`0.23 rad/s` to `0.10 rad/s` on the low-slip subset. These are effective model
parameters, not necessarily the physical steering-wheel ratio or unloaded
tire radius; re-run the utility after changing tires or steering geometry.

A/B result on the same bag: `bags/bici_bias` (uncalibrated) produced three
re-anchors (`55.0`, `25.9`, `37.2 m`), while `bags/bici_calib` produced none.
The INS innovation in `bici_calib` stayed below the `12 m` gate, with a
maximum of `11.77 m` and an RMS of `2.55 m` over the INS-covered interval.

---

## 9. What the filter does not estimate

- No gyro bias, wheel-radius or scale states. The longitudinal accelerometer
  bias `b_ax` is estimated as a random-walk state; the lateral accelerometer
  bias is not modeled. `Q` is fixed and ignores the IMU's reported
  covariances.
- Steering is not a state either: the measured angle is treated as a known
  input, with its uncertainty injected as `G sigma_delta^2 G^T`. A steering
  offset is therefore not estimated online; set `steering_offset_rad`.
- `/control/can` is the raw CAN signal, in **degrees**, spanning about
  +-327 deg. If it is a steering-wheel sensor and not a rack sensor,
  `steering_ratio` must be set, or `max_steering_rad` will saturate every
  reading and the model will think the wheels are at full lock.
- No lever-arm handling: `/sensors/imu` and the INS odom are assumed already
  rotated and shifted to the COG (the hardware driver does that,
  `mirena_hardware/src/imu/imu_pipeline.hpp:370-404`).
- `update_pose`, `update_velocity`, `update_zupt` exist in the EKF but the
  active node never calls them (only the commented-out `fast_lio.cpp` did).
  There is no zero-velocity update in this node.
- `update_odom` treats the INS as six independent scalars (diagonal `R`,
  cross-covariances ignored) and uses `(I-K)P`, as does `update_velocity_u`,
  rather than Joseph form. Symmetry is restored by `symmetrize()` but positive
  definiteness can degrade over long runs.
- Out-of-order samples: the encoders have an explicit stamp guard; IMU and INS
  updates do not check the stamp before the update. A late IMU gyro sample or
  a late INS sample is still applied, even if `predict` ignored its stamp.

---

## 10. Executable name

The mission launch files used to start `executable='mirena_state_ekf'`, while
`CMakeLists.txt` builds `mirena_state`. The `mirena_state_ekf` binary still in
`install/` was from an older build, so a mission would have run stale code
instead of this source. The launch files now start `mirena_state`
(`autocross`, `skidpad`, `trackdrive`, `acceleration`, `brake_test`).
