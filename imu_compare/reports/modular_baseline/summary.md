# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu_modular_baseline.csv` (9500 rows, 100.00 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 9500 rows, onset at t_rel=123.345s
- compared window: tau = 0..95.0s, 9499 samples at 100 Hz

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
| ax | 0.9742 | 0.6337 | 0.5132 | 0.881 | 0.573 | -0.010 |
| ay | 0.7189 | 0.3125 | 0.7731 | 0.037 | 0.016 | -0.830 |
| az | 0.5013 | 0.0104 | 0.4997 | 0.162 | 0.003 | +0.460 |
| gx | 0.0292 | 0.0008 | 0.0293 | -0.140 | -0.004 | +0.600 |
| gy | 0.0160 | 0.0032 | 0.0146 | 0.509 | 0.100 | +0.000 |
| gz | 0.0330 | 0.0821 | 0.0785 | 0.310 | 0.771 | -11.580 |

## Integrated quantities, tau >= 0

- yaw change: real +0.259 rad, sim -2.713 rad (ratio -10.471)
- longitudinal Δv: real +13.112 m/s, sim +8.006 m/s (ratio 0.611)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
