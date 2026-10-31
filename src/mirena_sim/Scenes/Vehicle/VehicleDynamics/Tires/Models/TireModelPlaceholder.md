# Tire Model Implementations — Phase 2+ Placeholder

## Purpose

This directory (`src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Tires/Models/`) hosts concrete implementations of [`TireModelInterface.gd`](TireModelInterface.gd).

In **Phase 1**, the foundational contract (`TireModelInterface`, `TireInputs`, `TireState`, `TireForces`, `TireConfiguration`, and `TireModelRegistry`) is implemented and verified using:

1. `TireModelInterface` (`TireModelType.UNIMPLEMENTED_CONTRACT`): Base contract verifier that enforces input validation and contact-loss semantics (`normal_load_fz_n <= 0 -> F = 0`) and flags `FaultFlag.UNIMPLEMENTED_MODEL_CALLED` when invoked with positive normal load.
2. `TireModelInterface.LinearTestStubTireModel` (`TireModelType.LINEAR_TEST_STUB`): Uncalibrated analytic linear + friction-ellipse test stub (`is_calibrated() == false`) used exclusively to test sign conventions, combined-slip bounding, relaxation filtering, and chassis force/moment assembly.

## Planned Phase 2+ Tire Models

### 1. `PacejkaMF62TireModel.gd` (`TireModelType.PACEJKA_MF62`)
- **Formulation**: Pacejka Magic Formula 6.2 pure and combined longitudinal/lateral slip (`Fx`, `Fy`, `Mz`) with load sensitivity (`dfz`), inflation pressure sensitivity (`dpi`), and camber effects (`gamma`).
- **Existing Verified Reference**: `/home/alexr/eRacing/TeR_Tire/physics/pacejka_baseline.py` and fitted Hoosier 16.0x7.5-10 R20 parameters in `/home/alexr/eRacing/TeR_Tire/config/pacejka_mf62_params.yaml` (`Fz0 = 700 N`, `p0 = 82.7 kPa`, `R0 = 0.2032 m`).
- **Status in Phase 1**: Interface contract established; equations and parameter loader scheduled for Phase 2.

### 2. `TeRGreyBoxTireModel.gd` (`TireModelType.TER_GREYBOX_PINN`)
- **Formulation**: Hybrid Grey-Box model combining `PacejkaMF62` baseline with the 7-input physics-informed residual MLP (`alpha, alpha_dot, gamma, fz, pressure, temp_avg, delta_temp`) from `/home/alexr/eRacing/TeR_Tire/models/grey_box_tire.py` and weights in `/home/alexr/eRacing/TeR_Tire/export/weights/`.
- **Status in Phase 1**: Interface contract (`TireInputs.slip_angle_rate_rad_s`, `TireState.tread_temp_c`, `TireState.carcass_temp_c`) established; GDScript/GDExtension inference bridge scheduled for Phase 2+.
