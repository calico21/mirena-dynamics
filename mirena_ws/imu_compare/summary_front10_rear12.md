# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu.csv` (46920 rows, 133.97 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 151015 rows, onset at t_rel=127.614s
- compared window: tau = 0..281.0s, 28103 samples at 100 Hz

## Static bias (pre-onset mean)

| axis | real | sim | difference |
|---|---:|---:|---:|
| ax | +0.0897 | +0.0000 | -0.0897 |
| ay | -0.0056 | -0.0000 | +0.0056 |
| az | -0.7315 | +0.0001 | +0.7316 |
| gx | -0.0001 | -0.0001 | +0.0000 |
| gy | -0.0000 | -0.0000 | +0.0000 |
| gz | +0.0001 | -0.0000 | -0.0001 |

## Dynamic metrics (main drive, tau >= 20 s)

| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |
|---|---:|---:|---:|---:|---:|---:|
| ax | 2.1230 | 2.4970 | 2.5088 | 0.420 | 0.493 | -0.070 |
| ay | 7.0029 | 1.2282 | 6.7153 | 0.317 | 0.056 | -0.030 |
| az | 3.2635 | 0.5919 | 3.3104 | 0.011 | 0.002 | -111.520 |
| gx | 0.2250 | 0.1174 | 0.2530 | 0.007 | 0.004 | +8.640 |
| gy | 0.0607 | 0.0925 | 0.1099 | 0.015 | 0.022 | -119.880 |
| gz | 0.5271 | 0.2085 | 0.4379 | 0.589 | 0.233 | +0.130 |

## Integrated quantities, tau >= 0

- yaw change: real -0.236 rad, sim -19.851 rad (ratio 83.973)
- longitudinal Δv: real -43.494 m/s, sim +15.450 m/s (ratio -0.355)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
