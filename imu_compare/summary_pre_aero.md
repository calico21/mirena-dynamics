# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu.csv` (46618 rows, 181.21 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 151015 rows, onset at t_rel=128.194s
- compared window: tau = 0..281.0s, 28103 samples at 100 Hz

## Static bias (pre-onset mean)

| axis | real | sim | difference |
|---|---:|---:|---:|
| ax | +0.0897 | +0.0002 | -0.0895 |
| ay | -0.0056 | -0.0003 | +0.0053 |
| az | -0.7315 | -0.0000 | +0.7315 |
| gx | -0.0001 | +0.0001 | +0.0002 |
| gy | -0.0000 | +0.0001 | +0.0002 |
| gz | +0.0001 | -0.0001 | -0.0002 |

## Dynamic metrics (main drive, tau >= 20 s)

| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |
|---|---:|---:|---:|---:|---:|---:|
| ax | 2.1230 | 2.7908 | 2.5752 | 0.478 | 0.628 | -0.050 |
| ay | 7.0029 | 1.6129 | 6.5647 | 0.378 | 0.087 | -0.010 |
| az | 3.2635 | 0.7014 | 3.3257 | 0.018 | 0.004 | +2.320 |
| gx | 0.2250 | 0.0551 | 0.2341 | -0.045 | -0.011 | -24.870 |
| gy | 0.0607 | 0.0639 | 0.0872 | 0.021 | 0.022 | -120.430 |
| gz | 0.5271 | 0.2686 | 0.3817 | 0.721 | 0.368 | +0.140 |

## Integrated quantities, tau >= 0

- yaw change: real -0.236 rad, sim -27.513 rad (ratio 116.383)
- longitudinal Δv: real -43.494 m/s, sim +25.032 m/s (ratio -0.576)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
