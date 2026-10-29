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

- wall/sim time slope: 1.000028 (1.0 = real-time)
- drive onset (wall): 127.61 s
- speed p50/p95/max: 0.04 / 7.57 / 9.71 m/s
- time above  5 m/s:  24.4 %
- time above 10 m/s:   0.0 %
- time above 15 m/s:   0.0 %
- time above 20 m/s:   0.0 %
- time above 25 m/s:   0.0 %

### Simulator

| speed bin [m/s] | samples | steer rms [rad] | gz std [rad/s] | gain [1/s] | R2 |
|---|---:|---:|---:|---:|---:|
| 0-2 | 21024 | 0.181 | 0.067 | 0.20 | 0.28 |
| 2-5 | 4807 | 0.195 | 0.300 | 1.55 | 0.84 |
| 5-8 | 7253 | 0.080 | 0.274 | 3.64 | 0.93 |
| 8-12 | 1077 | 0.037 | 0.130 | 5.04 | 0.86 |

Overall drive gain: **0.61** 1/s (R2 0.26, n=34161)

