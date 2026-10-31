# Vehicle Dynamics Baseline Configuration Report

**Date**: 2026-10-30  
**Repository Branch**: `main`  
**Git Commit**: `a0d0415d654c6198f2ebcb79a4ec296f8c7e8689`  
**Archive Location**: `imu_compare/baselines/2026-10-30_pre_calibration/`  
**Modular Baseline Config Location**: `src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Config/baselines/modular_baseline/`

---

## 1. Baseline Configurations Overview

To ensure total reproducibility and prevent overwriting baseline results, three separate configurations are formally defined and maintained:

1. **Original Legacy Baseline (`LEGACY_GODOT_VEHICLE_BODY`)**:
   - Implementation: Built-in Godot `VehicleBody3D` + `VehicleWheel3D` with manual raycasts and script `vehicleMotion.gd`.
   - Configurations recorded:
     - `grip10`: `wheel_friction_slip = 1.0` (all 4 wheels). Evaluated in `active_grip10.md` / `summary_grip10.md`. HUD Correlation: **27.8%** (`r(ay)=0.026`, `r(gz)=0.283`, `r(ax)=0.816`).
     - `front10_rear12`: `wheel_friction_slip = 1.0` (front), `1.2` (rear). Evaluated in `active_front10_rear12.md`. HUD Correlation: **32.4%** / **34.8%** (`r(ay)=0.071`, `r(gz)=0.320`, `r(ax)=0.812`).

2. **Current Modular Baseline (`MODULAR_VEHICLE_DYNAMICS` Baseline)**:
   - Resource Path: `src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Config/baselines/modular_baseline/vehicle_default.tres`
   - Sub-Resources:
     - `suspension_default.tres`
     - `tire_default.tres`
     - `powertrain_default.tres`
   - Active Physics Mode: `CUSTOM_RIGID_BODY_3D` (or standalone planar bench `STANDALONE_BENCH`).
   - Active Tire Model: `TeRTireGreyBoxModel` (`model_type = 3`, MF6.2 + 7D PINN residual).
   - Pre-calibration state: Tested with 49 headless unit tests passing; exhibits 200 Hz Nyquist wheel spin limit cycle at rest, aerodynamic downforce double-counting, and excessive lateral load transfer / two-wheel lift during extreme transients.

3. **New Calibrated Modular Candidate (`vehicle_ter26_calibrated.tres`)**:
   - Resource Path: `src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Config/vehicle_ter26_calibrated.tres`
   - Goal: Fully defensible physical calibration against real TeR26 VN-300 + CAN bag recordings (`imu_compare/real_*.csv`), with unconditional low-speed wheel rotation stability, corrected roll moment arm, unified aerodynamic normal load balance, and validated real-world correlation.

---

## 2. Parameter Comparison Table

| Subsystem / Parameter | Unit | Legacy Baseline (`front10_rear12`) | Modular Baseline (`vehicle_default.tres`) | Source / Verification Status |
|---|---|---:|---:|---|
| **Chassis Total Mass** | kg | 300.0 | 300.0 | TeR26 Technical Spec Sheet |
| **Front / Rear Weight Split** | % | 45.0 / 55.0 | 45.0 / 55.0 | Measured corner scales |
| **Center of Mass (Godot)** | m | `(0, 0.33, -0.117)` | `(0, 0.33, -0.117)` | ISO: $h_{{cg}}=0.33\\text{{ m}}, a=0.876\\text{{ m}}, b=0.717\\text{{ m}}$ |
| **Wheelbase** ($L$) | m | 1.593 | 1.593 | Measured chassis CAD |
| **Track Width (F / R)** | m | 1.174 / 1.174 | 1.174 / 1.174 | Measured rim centerlines |
| **Yaw Inertia** ($I_{{zz}}$) | kg m² | (Godot default ~100) | 120.0 | Sensitivity range [110, 150] kg m² |
| **Roll Inertia** ($I_{{xx}}$) | kg m² | (Godot default) | 45.0 | CAD estimate |
| **Pitch Inertia** ($I_{{yy}}$) | kg m² | (Godot default) | 85.0 | CAD estimate |
| **Wheel Unloaded Radius** | m | 0.20 | 0.2032 | 16x7.5-10 Hoosier R25B tire spec |
| **Wheel Rotational Inertia** | kg m² | 0.32 | 0.32 | Rim + tire + hub assembly |
| **Steering Ratio / Max Angle** | rad | 0.523 (30°) | 0.35 (20.0°) | Pinion-rack mechanical limit |
| **Ackermann Factor** | - | 0.0 (parallel) | 0.60 | Anti-Ackermann / Pro-Ackermann tie rod geometry |
| **Static Toe (F / R)** | rad | 0.0 / 0.0 | +0.0015 / -0.0020 | Setup sheet |
| **Front Spring Rate** | N/m | 30000.0 | 35030.0 | Measured spring rate |
| **Rear Spring Rate** | N/m | 35000.0 | 52540.0 | Measured spring rate |
| **Front Motion Ratio** | - | 1.00 | 1.14 | Pushrod suspension linkage |
| **Rear Motion Ratio** | - | 1.00 | 1.16 | Pushrod suspension linkage |
| **Front ARB Rate** | N/m | 0.0 | 16000.0 | Front blade ARB |
| **Rear ARB Rate** | N/m | 0.0 | 1500.0 | Rear blade ARB (soft) |
| **Roll Center Height (F / R)** | m | 0.00 / 0.00 | 0.04 / 0.06 | Double wishbone kinematics |
| **Aero Drag Area** ($C_d A$) | m² | 0.0 | 2.50 × 1.10 = 2.75 | Wind tunnel / CFD correlation |
| **Aero Downforce Area** ($C_l A$) | m² | 0.0 | 4.14 × 1.10 = 4.55 | Wind tunnel / CFD correlation |
| **Tire Model** | - | Godot Pacejka proxy | `TeR_Tire` MF6.2 + 7D PINN | Calibrated from FSAE TTC Round 9 |
| **Tire Peak Friction** | - | 1.0 / 1.2 | ~2.49 (TTC belt) / 1.45 (companion Fx) | Calibrated model |
| **Motor Peak Torque** | Nm | 230.0 (wheel) | 100.0 (motor) × 5.0 (gear) = 500.0 Nm | Emrax 208 motor spec |
| **Power Limit** | kW | 80.0 | 80.0 | Formula Student rules (80 kW limit) |
| **Differential Type** | - | Open / Spool | Torque-sensing bias (0.65) + viscous (40 Nm s/rad) | Drexler FS LSD |
