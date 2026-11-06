# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu.csv` (46692 rows, 124.26 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 151015 rows, onset at t_rel=128.334s
- compared window: tau = 0..281.0s, 28103 samples at 100 Hz

## Static bias (pre-onset mean)

| axis | real | sim | difference |
|---|---:|---:|---:|
| ax | +0.0897 | -0.0003 | -0.0900 |
| ay | -0.0056 | -0.0001 | +0.0055 |
| az | -0.7315 | +0.0001 | +0.7317 |
| gx | -0.0001 | +0.0000 | +0.0001 |
| gy | -0.0000 | +0.0000 | +0.0001 |
| gz | +0.0001 | +0.0001 | -0.0000 |

## Dynamic metrics (main drive, tau >= 20 s)

| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |
|---|---:|---:|---:|---:|---:|---:|
| ax | 2.1230 | 0.8859 | 2.1742 | 0.150 | 0.063 | +0.000 |
| ay | 7.0029 | 0.2752 | 7.0064 | 0.007 | 0.000 | -161.880 |
| az | 3.2635 | 0.2506 | 3.2708 | 0.009 | 0.001 | -213.460 |
| gx | 0.2250 | 0.0354 | 0.2279 | -0.003 | -0.000 | -121.590 |
| gy | 0.0607 | 0.0799 | 0.0999 | 0.010 | 0.013 | -136.700 |
| gz | 0.5271 | 0.0452 | 0.5276 | 0.030 | 0.003 | -96.080 |

## Integrated quantities, tau >= 0

- yaw change: real -0.236 rad, sim -4.337 rad (ratio 18.344)
- longitudinal Δv: real -43.494 m/s, sim +3.188 m/s (ratio -0.073)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
