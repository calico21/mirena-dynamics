# Speed and yaw-gain analysis

## Real car

- drive onset (bag header): 1791309858.32 s
- speed source: rear wheel RPM x 2*pi/60 x 0.2032, calibrated k=0.928 against odom
- speed p50/p95/max: 6.85 / 12.11 / 13.89 m/s
- time above  5 m/s:  60.9 %
- time above 10 m/s:  23.2 %
- time above 15 m/s:   0.0 %
- time above 20 m/s:   0.0 %
- time above 25 m/s:   0.0 %

### Real

| speed bin [m/s] | samples | steer rms [rad] | gz std [rad/s] | gain [1/s] | R2 |
|---|---:|---:|---:|---:|---:|
| 0-2 | 8245 | 0.123 | 0.055 | 0.42 | 0.80 |
| 2-5 | 2662 | 0.362 | 0.765 | 2.39 | 0.93 |
| 5-8 | 5229 | 0.246 | 0.730 | 2.87 | 0.93 |
| 8-12 | 10351 | 0.110 | 0.469 | 4.51 | 0.94 |
| 12-16 | 1473 | 0.038 | 0.188 | 7.58 | 0.67 |

Overall drive gain: **2.60** 1/s (R2 0.80, n=28011)

## Simulator

- wall/sim time slope: 1.008840 (1.0 = real-time)
- drive onset (wall): 128.33 s
- speed p50/p95/max: 0.00 / 2.68 / 7.70 m/s
- time above  5 m/s:   2.3 %
- time above 10 m/s:   0.0 %
- time above 15 m/s:   0.0 %
- time above 20 m/s:   0.0 %
- time above 25 m/s:   0.0 %

### Simulator

| speed bin [m/s] | samples | steer rms [rad] | gz std [rad/s] | gain [1/s] | R2 |
|---|---:|---:|---:|---:|---:|
| 0-2 | 31523 | 0.169 | 0.049 | 0.09 | 0.09 |
| 2-5 | 1572 | 0.119 | 0.129 | 1.22 | 0.89 |
| 5-8 | 796 | 0.032 | 0.084 | 3.71 | 0.91 |

Overall drive gain: **0.11** 1/s (R2 0.09, n=33891)

