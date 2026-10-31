# Telemetry & Diagnostics Subsystem (`Telemetry/`)

## Overview

The `Telemetry/` directory provides configurable, zero-production-overhead diagnostic hooks for inspecting internal physical variables and comparing simulated runs against recorded real-car telemetry (`imu_compare/`).

## Modules

- [`DynamicsDiagnostics.gd`](DynamicsDiagnostics.gd): Captures per-step snapshots of chassis states (`u, v, w, ax, ay, az, yaw_rate`), per-wheel states (`Fz, kappa, alpha, Fx, Fy, Mz, compression, omega, RPM, drive_torque, brake_torque, in_contact`), and cumulative numerical fault counters (`non_finite_input_count`, `non_finite_state_count`, `non_finite_force_count`, `unimplemented_model_call_count`, `all_wheels_airborne_count`, `tire_saturation_count`).
- [`DynamicsLogger.gd`](DynamicsLogger.gd): Optional in-memory ring/linear buffer that exports CSV traces at the end of a test or replay run without introducing per-frame file I/O latency on the 400 Hz physics thread.
