# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu_modular_calibrated.csv` (28100 rows, 100.00 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 28100 rows, onset at t_rel=123.345s
- compared window: tau = 0..281.0s, 28099 samples at 100 Hz

## Static bias (pre-onset mean)

| axis | real | sim | difference |
|---|---:|---:|---:|
| ax | +0.0897 | +0.0000 | -0.0897 |
| ay | -0.0056 | -0.0000 | +0.0056 |
| az | -0.7315 | -0.2482 | +0.4834 |
| gx | -0.0001 | +0.0000 | +0.0001 |
| gy | -0.0000 | -0.0001 | -0.0001 |
| gz | +0.0001 | +0.0000 | -0.0001 |

## Dynamic metrics (main drive, tau >= 20 s)

| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |
|---|---:|---:|---:|---:|---:|---:|
| ax | 2.1232 | 1.4237 | 1.4590 | 0.729 | 0.489 | -0.050 |
| ay | 7.0035 | 4.8735 | 6.3346 | 0.479 | 0.333 | -0.030 |
| az | 3.2638 | 0.0647 | 3.2635 | 0.014 | 0.000 | +4.800 |
| gx | 0.2250 | 0.0125 | 0.2250 | 0.031 | 0.002 | +0.550 |
| gy | 0.0607 | 0.0081 | 0.0599 | 0.160 | 0.021 | -0.030 |
| gz | 0.5271 | 0.5928 | 0.1914 | 0.948 | 1.066 | -0.010 |

## Integrated quantities, tau >= 0

- yaw change: real -0.236 rad, sim -41.776 rad (ratio 176.704)
- longitudinal Δv: real -43.493 m/s, sim -53.909 m/s (ratio 1.240)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
