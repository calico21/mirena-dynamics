# Validation & Experimental Calibration Plan (`VALIDATION_PLAN.md`)

## 1. Multi-Tier Verification Strategy

Validation of the `VehicleDynamics` framework proceeds across three tiers:
1. **Tier 1 — Headless Subsystem Unit Tests (Implemented & Active in Phase 1)**
2. **Tier 2 — Open-Loop Physical Manoeuvres (Phase 2–3)**
3. **Tier 3 — Experimental Bag Replay & Closed-Loop Autonomy Validation (Phase 4–5)**

---

## 2. Tier 1: Headless Unit & Integration Tests (`Tests/`)

Executed via:
```bash
godot --headless --path src/mirena_sim -s res://Scenes/Vehicle/VehicleDynamics/Tests/test_vehicle_dynamics.gd
```

| Test Suite | Verified Physical & Software Invariants |
| :--- | :--- |
| `test_tire_model.gd` | • `LINEAR_TEST_STUB` explicitly reports `is_calibrated() == false`<br>• Unoverridden `TireModelInterface` sets `FaultFlag.TIRE_MODEL_UNIMPLEMENTED` and zeroes forces<br>• Unilateral contact loss ($F_z = 0 \implies F_x = F_y = M_z = 0$)<br>• Sign conventions ($\kappa > 0 \implies F_x > 0$; $\alpha > 0 \implies F_y < 0, M_z > 0$)<br>• Combined-slip friction ellipse saturation ($\sqrt{(F_x/\mu_x F_z)^2 + (F_y/\mu_y F_z)^2} \le 1$)<br>• First-order transient slip relaxation convergence<br>• `NaN`/`Inf` rejection with `FaultFlag.NON_FINITE_FORCE` |
| `test_wheel_kinematics.gd` | • `VehicleCoordinates` Godot $\leftrightarrow$ ISO 8855/ROS vector and angular velocity round-trip<br>• Body $\leftrightarrow$ steered wheel planar rotation orthogonality<br>• Chassis yaw rate $r > 0$ decreases inside (FL) and increases outside (FR) longitudinal hub velocity<br>• Slip ratio $\kappa = +0.10$ under traction and $-0.10$ under braking<br>• Standstill ($u = 0, \omega = 0$) singularity protection<br>• `VehicleUnits` rad/s $\leftrightarrow$ RPM conversion and encoder quantization |
| `test_suspension_model.gd` | • Static wheel loads recovered at zero compression and velocity<br>• Progressive bump-stop engagement above `max_compression_m`<br>• 4-way digressive damper bump/rebound asymmetry<br>• Anti-roll bar zero net heave contribution ($F_{\text{ARB,L}} + F_{\text{ARB,R}} = 0$)<br>• Wheel liftoff clamping ($F_z \ge 0$, never tensile) |
| `test_powertrain.gd` | • Low-speed peak torque limit ($100\text{ N}\cdot\text{m} \times 5.0 \times 0.92 = 460\text{ N}\cdot\text{m}$ rear axle)<br>• Formula Student $80\text{ kW}$ hyperbolic power ceiling at high speed<br>• Forward-only lockout ($u_{\text{thr}} < 0 \implies T_{\text{drive}} = 0$)<br>• `BrakeSystem` 60/40 front/rear bias and $1177.2\text{ N}$ total force budget<br>• `WheelRotationalDynamics` passive brake lockup at $\omega_w = 0$ without reverse spin |
| `test_vehicle_dynamics.gd` | • `Config/vehicle_default.tres` resource loading and zero-error validation<br>• Static axle weight distribution ($\sum F_{z,\text{stat}} = m g = 2943.0\text{ N}$)<br>• Multi-step straight-line traction + aerodynamic downforce/drag integration<br>• Left-steering command ($+\delta \implies +M_z$ yaw moment)<br>• `ROSVehicleAdapter` encoder LPF + window averaging + quantization and telemetry schema<br>• `DynamicsLogger` 28-column CSV schema<br>• Deterministic `reset()` clearing all state and fault counters |

---

## 3. Tier 2: Standard Vehicle-Dynamics Benchmark Manoeuvres (Phases 2–3)

Once the calibrated Pacejka MF6.2 / `TeR_Tire` grey-box tire model is registered in Phase 2, the following synthetic manoeuvres will be executed headlessly using `VehicleDynamicsModel` and logged via `DynamicsLogger`:

1. **Static Settle & Weight Transfer Check**:
   - Zero control inputs at $u = 0$: verify pitch/roll equilibrium and $\sum F_{z,i} = m g$.
   - Steady-state longitudinal acceleration $a_x$: verify front-to-rear load transfer $\Delta F_z = \pm \frac{m a_x h_{\text{CoM}}}{L}$.
   - Steady-state lateral acceleration $a_y$: verify left-to-right load transfer $\Delta F_{z,f} + \Delta F_{z,r} = \frac{m a_y h_{\text{CoM}}}{t}$.
2. **Constant-Radius Skidpad (FS Skidpad $R = 9.125\text{ m}$)**:
   - Sweep forward speed $u \in [3, 12]\text{ m/s}$, measure understeer gradient $K_{\text{us}} = \frac{\partial \delta}{\partial a_y} - \frac{L}{R g}$, roll angle $\phi(a_y)$, and maximum sustainable lateral acceleration $a_{y,\max}$.
3. **Step-Steer & Sine-with-Dwell Transient Response**:
   - Step steering input $\delta = 0 \to 0.15\text{ rad}$ at $u = 10\text{ m/s}$: verify yaw-rate rise time, overshoot, and tire relaxation phase lag against `TeR_TWIN` reference trajectories.
4. **Straight-Line 0–75 m Acceleration & Full Braking**:
   - Verify 80 kW power-limited acceleration curve and braking deceleration without unphysical wheel-speed oscillation.

---

## 4. Tier 3: Experimental ROS 2 Bag Replay & Telemetry Validation (Phases 4–5)

The repository already contains the experimental validation pipeline in the root directory:
- `run_sim_replay.sh`: Replays recorded `/mirena/control/car` commands from `rosbag2_2026_03_28-18_09_18/` (38.5 s track session, 4591 control messages) into MirenaSim and records simulated IMU, GPS, and `/mirena/wheel_speeds`.
- `compare_imu.py` & `imu_compare/`: Aligns simulated vs. real telemetry by cross-correlating yaw rate $\omega_z$ and evaluates RMSE, MAE, Pearson correlation $r$, and 95th-percentile error.

### Quantitative Acceptance Targets (vs. Legacy Baseline in `imu_compare/summary_front10_rear12.md`)

| Metric | Legacy `VehicleBody3D` Baseline (`front10_rear12`) | Phase 4–5 Calibrated Target |
| :--- | :---: | :---: |
| Yaw Rate $\omega_z$ RMSE ($\text{rad/s}$) | `0.1541` | $\le 0.10$ |
| Yaw Rate $\omega_z$ Correlation $r$ | `0.8190` | $\ge 0.90$ |
| Lateral Acceleration $a_y$ RMSE ($\text{m/s}^2$) | `1.8832` | $\le 1.25$ |
| Lateral Acceleration $a_y$ Correlation $r$ | `0.8193` | $\ge 0.88$ |
| Longitudinal Acceleration $a_x$ RMSE ($\text{m/s}^2$) | `1.1965` | $\le 0.95$ |
| Longitudinal Acceleration $a_x$ Correlation $r$ | `0.7037` | $\ge 0.80$ |
| Wheel Speed $\text{RPM}$ RMSE (FL / FR / RL / RR) | `54.7 / 60.4 / 64.9 / 71.0` | $\le 40.0\text{ RPM}$ across all 4 corners |
| Braking Wheel Lockup Spikes (`0 RPM` glitches) | Frequent during moderate braking | Eliminated except under genuine physical lockup |
