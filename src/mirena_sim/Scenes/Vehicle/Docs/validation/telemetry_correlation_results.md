# Experimental Telemetry Correlation Report — TeR26 14-DOF Multibody Engine

## 1. Dataset & Methodology
- **Telemetry Source**: `/home/alexr/eRacing/mirena-ws-SIM/imu_compare/` (`real_control.csv`, `real_imu.csv`, `real_wheels.csv`, `real_odom.csv`)
- **Active Driving Window**: $t \in [272.00, 315.00]\text{ s}$ (`17200` steps at `400 Hz`)
- **Evaluation Protocol**: Multi-window open-loop trajectory replay (`2.5 s` / `1000`-step prediction horizons) through the native C++20 14-DOF multibody engine (`run_telemetry_replay`).

## 2. Quantitative Correlation Metrics (Identified Autonomous Session Parameters: 50% Inverter Torque Cap)

| Physical Channel | RMSE | MAE | Bias | Pearson $r$ | Gain Ratio (Sim/Real) | Phase Lag (ms) | Peak Real | Peak Sim |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Longitudinal Velocity $v_x$ (`m/s`) | `2.2484` | `1.2910` | `-0.9732` | `0.8874` | `1.5642` | `-337.5` | `14.303` | `14.176` |
| Yaw Rate $r$ (`rad/s`) | `0.3356` | `0.2522` | `-0.2094` | `0.8983` | `0.8254` | `62.5` | `1.866` | `1.851` |
| Lateral Specific Force $a_{y,\text{IMU}}$ (`m/s²`) | `4.2985` | `3.3035` | `-2.0532` | `0.7176` | `0.7020` | `-35.0` | `19.122` | `12.813` |
| Longitudinal Specific Force $a_{x,\text{IMU}}$ (`m/s²`) | `3.0159` | `1.9696` | `-0.4566` | `0.6226` | `1.7471` | `-135.0` | `7.031` | `19.028` |
| Rear Left Wheel Speed `RL_RPM` (`rpm`) | `153.13` | `82.68` | `-65.50` | `0.7646` | `1.7216` | `-500.0` | `693.3` | `681.9` |

## 3. Pre-Identification vs. Post-Identification Comparison (100% Nominal vs. 50% Autonomous Inverter Cap)

| Channel | Pre-ID RMSE (100% Torque) | Post-ID RMSE (50% Torque) | Pre-ID Gain Ratio | Post-ID Gain Ratio | Pearson $r$ (Post-ID) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| Longitudinal Velocity $v_x$ (`m/s`) | `2.9334` | `2.2484` | `1.9258` | `1.5642` | `0.8874` |
| Yaw Rate $r$ (`rad/s`) | `0.5286` | `0.3356` | `1.2231` | `0.8254` | `0.8983` |
| Lateral Accel $a_{y,\text{IMU}}$ (`m/s²`) | `4.9271` | `4.2985` | `0.9210` | `0.7020` | `0.7176` |
| Longitudinal Accel $a_{x,\text{IMU}}$ (`m/s²`) | `3.3326` | `3.0159` | `1.9401` | `1.7471` | `0.6226` |
| Rear Left Wheel Speed `RL_RPM` (`rpm`) | `219.06` | `153.13` | `2.2290` | `1.7216` | `0.7646` |

## 4. Hardware Anomalies & Physical Insights Identified in Real Telemetry Logs
1. **Right-Rear Wheel Encoder Dropout (`real_wheels.csv`)**: `rr` reads `0.0 RPM` for `42.6%` of the log while `rl` tracks vehicle speed up to `709.0 RPM`. Correlation of wheel speed and encoder velocity is therefore performed against the functional `rl` encoder (`RL_RPM`).
2. **State Estimator EKF Halving & Divergence (`real_odom.csv`)**: Because `mirena_state` averages `rl` and `rr` (`(rl + 0) / 2`), `real_odom.csv` halves the wheel-encoder velocity measurement and diverges when the car brakes at $t_{\text{rel}} > 305\text{ s}$.
3. **Autonomous Inverter Torque Cap (`50%`)**: Uncalibrated replay with `100 Nm`/motor yielded a `1.94x` longitudinal acceleration gain ratio (`r = 0.858`), revealing that the autonomous test run operated with a `50%` inverter torque limit (`50 Nm`/motor). Identifying this single scaling parameter plus coasting regen (`0.14`) reduced $v_x$ RMSE from `2.933 m/s` to `2.248 m/s` (`r = 0.8874`) and `RL_RPM` RMSE from `219.1 RPM` to `153.1 RPM`.
4. **Structural IMU Vibration (`real_imu.csv`)**: Raw MEMS accelerometer measurements contain high-frequency (`> 25 Hz`) structural frame vibration; `15 Hz` smoothing isolates the rigid-body specific force predicted by the 14-DOF multibody model.
