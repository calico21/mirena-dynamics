# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu_modular_baseline.csv` (28100 rows, 100.00 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 28100 rows, onset at t_rel=123.345s
- compared window: tau = 0..281.0s, 28099 samples at 100 Hz

## Static bias (pre-onset mean)

| axis | real | sim | difference |
|---|---:|---:|---:|
| ax | +0.0897 | +0.0000 | -0.0897 |
| ay | -0.0056 | +0.0000 | +0.0056 |
| az | -0.7315 | -0.1717 | +0.5599 |
| gx | -0.0001 | +0.0000 | +0.0001 |
| gy | -0.0000 | -0.0001 | -0.0001 |
| gz | +0.0001 | +0.0000 | -0.0001 |

## Dynamic metrics (main drive, tau >= 20 s)

| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |
|---|---:|---:|---:|---:|---:|---:|
| ax | 2.1232 | 1.4062 | 1.4566 | 0.731 | 0.484 | -0.020 |
| ay | 7.0035 | 4.3334 | 6.2121 | 0.482 | 0.298 | -0.030 |
| az | 3.2638 | 0.0490 | 3.2634 | 0.015 | 0.000 | +4.810 |
| gx | 0.2250 | 0.0103 | 0.2250 | 0.025 | 0.001 | +0.570 |
| gy | 0.0607 | 0.0068 | 0.0599 | 0.168 | 0.019 | -0.020 |
| gz | 0.5271 | 0.5603 | 0.1755 | 0.950 | 1.009 | +0.010 |

## Integrated quantities, tau >= 0

- yaw change: real -0.236 rad, sim -40.877 rad (ratio 172.899)
- longitudinal Δv: real -43.493 m/s, sim -52.040 m/s (ratio 1.197)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
