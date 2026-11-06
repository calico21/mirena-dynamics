# Parameter Provenance, Missing Data Registry, and Observability Analysis — TeR26

## 1. Parameter Provenance Classification

Every parameter in `Data/TeR26/*.json` carries an explicit `"provenance"` tag from the following controlled vocabulary:
1. `MEASURED`: Directly weighed or measured on TeR26 hardware (e.g., wheel radius, gearbox reduction ratio, motor peak torque, 80 kW FS rule cap).
2. `CAD_DERIVED`: Exported directly from TeR26 CAD / MirenaSim scene geometry (`wheelbase = 1.593 m`, `track = 1.174 m`, `h_cg = 0.330 m`).
3. `CAD_DERIVED_APPROXIMATE`: Present in `FS_Driver_Setup_Optimizer/config/vehicles/ter26.py` as rounded CAD values (e.g., the 9 primary suspension hardpoints per corner).
4. `GEOMETRICALLY_DERIVED`: Solved to satisfy documented TeR26 kinematic targets (`MR = 1.14` front / `1.16` rear, static toe, Ackermann percentage, spring/damper installation axis) where raw CAD point exports were omitted from `ter26.py`.
5. `TTC_FITTED`: Calibrated directly against Calspan TTC Round 9 raw data (`Hoosier 16.0x7.5-10 R20` Runs 2–9 and `16.0x6.0-10 R20` Runs 10–15).
6. `TTC_R20_COMPOUND_SCALED`: Longitudinal MF6.2 parameters transferred from 10" Hoosier R20 Drive/Brake runs (`Runs 68–73`) and scaled by load/radius because Round 9 did not test the 16" tires on the Drive/Brake machine.
7. `ESTIMATED`: Engineering estimate with documented uncertainty bounds $\pm \sigma$.
8. `PENDING_IDENTIFICATION`: Parameter awaiting experimental identification from vehicle telemetry or K&C rig testing.

---

## 2. Missing TeR26 Data Registry & Impact Analysis

| Missing / Incomplete Item | Current Status in Repo | Provisional Resolution | Required Experimental / CAD Source |
| :--- | :--- | :--- | :--- |
| **Exact TeR26 CAD coords for `tie_in`, `tie_out`, `rocker_axis`, `rocker_rod`, `rocker_coi`, `antiroll_in/out`** | `ter26.py` has 9 hardpoints/corner; `TeR27_OptimumK.xlsx` has all 16 points for TeR27. | Derived geometrically in `Data/TeR26/hardpoints/*.json` (`GEOMETRICALLY_DERIVED`) to match TeR26's exact motion ratio (`1.14` F / `1.16` R) and steering ratio; `TeR27` 16-point CAD fixture provided separately in `Tests/fixtures/`. | Export full 16-point table from TeR26 SolidWorks / OptimumKinematics assembly. |
| **TeR26 Chassis & Upright 3x3 Inertia Tensors (CAD / Trifilar Pendulum)** | `Ixx=45`, `Iyy=85`, `Izz=120-150 kg·m²` (`ESTIMATED` / `CALIBRATED` in `active_vehicle_config.json`). | Stored with `±15%` uncertainty bounds; identifiable from yaw-rate transient response (`real_imu.csv`). | SolidWorks Mass Properties export of complete TeR26 assembly + upright subassemblies. |
| **Hoosier 16.0x7.5-10 & 16.0x6.0-10 R20 Longitudinal TTC Runs** | Round 9 `DriveBrake` folder only includes `18.0x6.0-10 R20` (`Runs 68–73`). | Longitudinal MF6.2 scaled from `18.0x6.0-10 R20` (`TTC_R20_COMPOUND_SCALED`) + calibrated against TeR26 braking/acceleration telemetry. | On-car straight-line launch & braking telemetry identification (`Tools/identification/`). |
| **ANSYS 6x6 Compliance Matrix $\mathbf{C}_{6\times 6}$ for TeR26 Uprights/Wishbones** | Diagonal compliance estimates in `ter26.py` (`k_camber = 1.2 deg/kN`). | Populated diagonal + cross-coupled $\mathbf{C}_{6\times 6}$ in `Data/TeR26/suspension/*.json` (`ESTIMATED`). | Static unit-wrench FEA load cases in ANSYS Mechanical at wheel center. |

---

## 3. Structural Collinearity & Observability Analysis

Given trajectory sensitivity matrix $J_{\text{traj}} = \frac{\partial \mathbf{y}_{1:N}}{\partial \boldsymbol{\theta}} \in \mathbb{R}^{N_y \times N_\theta}$ and measurement covariance $\Sigma_y$, the **Fisher Information Matrix (FIM)** is:

$$
\mathcal{I}(\boldsymbol{\theta}) = J_{\text{traj}}^T \Sigma_y^{-1} J_{\text{traj}}
$$

### 3.1 Known Collinear Parameter Pairs (Must Not Be Fitted Simultaneously Without Dedicated Excitation)
1. **Overall Yaw Inertia $I_{zz}$ vs. Rear Cornering Stiffness $\lambda_{Ky\alpha,r}$:** Collinear in steady-state cornering; separable only during transient step-steer / slalom maneuvers where $\dot{r}$ is large.
2. **CoG Height $h_{\text{cg}}$ vs. Roll Stiffness Distribution ($k_{\text{ARB},f} / k_{\text{ARB},r}$):** Without suspension potentiometers (LVDTs), total lateral load transfer $\Delta F_z$ is determined by $h_{\text{cg}}$, while roll angle $\phi$ depends on $h_{\text{cg}} / (k_{\phi,f} + k_{\phi,r})$.
3. **Drivetrain Efficiency $\eta_g$ vs. Tire Longitudinal Friction Scaling $\lambda_{\mu x}$:** In unsaturated acceleration (below traction limit), $\eta_g$ and rolling resistance $q_{sy1}$ are collinear; $\lambda_{\mu x}$ is only observable when $|\kappa|$ enters the nonlinear saturation peak.
4. **Aerodynamic $C_L A$ vs. High-Speed Tire Load Sensitivity $p_{Dy2}$:** Observable only across wide speed sweeps ($v_x \in [5, 28]\text{ m/s}$).
