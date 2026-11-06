# IMU comparison: real bag vs simulator

- real IMU: `real_imu.csv` (40344 rows, 99.95 Hz, frame `car/cog`)
- sim IMU: `sim_imu.csv` (46901 rows, 129.09 Hz, frame `car/cog`)
- real control: 151015 rows, onset at t_rel=123.345s
- sim control: 151015 rows, onset at t_rel=127.787s
- compared window: tau = 0..281.0s, 28103 samples at 100 Hz

## Static bias (pre-onset mean)

| axis | real | sim | difference |
|---|---:|---:|---:|
| ax | +0.0897 | +0.0001 | -0.0896 |
| ay | -0.0056 | -0.0002 | +0.0054 |
| az | -0.7315 | +0.0001 | +0.7316 |
| gx | -0.0001 | +0.0000 | +0.0001 |
| gy | -0.0000 | -0.0001 | -0.0001 |
| gz | +0.0001 | -0.0001 | -0.0002 |

## Dynamic metrics (main drive, tau >= 20 s)

| signal | real std | sim std | RMSE | corr | gain sim/real | lag [s] |
|---|---:|---:|---:|---:|---:|---:|
| ax | 2.1230 | 2.7928 | 2.5928 | 0.471 | 0.619 | -0.060 |
| ay | 7.0029 | 1.3288 | 6.6273 | 0.370 | 0.070 | -0.020 |
| az | 3.2635 | 0.5987 | 3.3116 | 0.011 | 0.002 | -111.460 |
| gx | 0.2250 | 0.1195 | 0.2537 | 0.011 | 0.006 | +8.700 |
| gy | 0.0607 | 0.0921 | 0.1094 | 0.019 | 0.029 | -119.850 |
| gz | 0.5271 | 0.2294 | 0.4013 | 0.700 | 0.305 | +0.160 |

## Integrated quantities, tau >= 0

- yaw change: real -0.236 rad, sim -24.424 rad (ratio 103.316)
- longitudinal Δv: real -43.494 m/s, sim +13.965 m/s (ratio -0.321)

Plots: `plots/accel_vs_time.png`, `plots/gyro_vs_time.png`, `plots/accel_psd.png`, `plots/gyro_psd.png`, `plots/yaw_integral.png`
