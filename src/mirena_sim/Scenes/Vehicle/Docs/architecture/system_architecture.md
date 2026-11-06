# System Architecture — TeR26 Multibody Vehicle Dynamics Engine

## 1. Architectural Principles

1. **Single Physical Authority:** The C++20 multibody dynamics engine (`ter26::VehicleDynamicsEngine`) is the sole integrator of the 14-DOF mechanical state $q, v$ and internal states $z$ (tire relaxation, thermal, actuator, compliance). Godot 4.7 never integrates chassis or wheel dynamics independently when the native engine is active.
2. **Strict Separation of Physics Core and Godot Adapter:**
   - `Native/include/ter26/` and `Native/src/core/` are 100% independent of Godot headers and types. They compile into a standalone static/shared library (`libter26_core.a`) testable from C++20 unit tests and Python/JAX bindings.
   - `Native/src/godot/` provides the thin GDExtension binding (`Ter26VehicleServer`, `libter26_vehicle.so`) that bridges `Godot::Dictionary`/`PackedFloat64Array`/`Transform3D` to `ter26::VehicleDynamicsEngine`.
   - `vehicleMotion.gd` (`MirenaCar`) and `Godot/*.gd` synchronize the calculated 3D poses and velocities to the Godot scene tree (`MirenaCar`, `$FL_WHEEL`, `$FR_WHEEL`, `$RL_WHEEL`, `$RR_WHEEL`) and sensors (`$INS`, `$Lidar`, `$Camera`, `$MirenaTransform`).
3. **Deterministic Functional Stepping:** Every simulation step is a pure function $(x_{k+1}, y_k, \text{diag}_k) = \mathcal{F}(x_k, u_k, w_k, \theta, \Delta t)$ with zero hidden global mutable state.

---

## 2. Architecture Diagram

```mermaid
flowchart TD
    subgraph ROS2["ROS 2 Driverless Stack"]
        CTRL["/control (CarControl)"]
        SENS["/sensors/imu, /sensors/wheel_speeds, /debug/state/car"]
    end

    subgraph Godot["Godot 4.7 Scene (MirenaSim @ 400 Hz)"]
        MC["MirenaCar (vehicleMotion.gd)"]
        WV["Wheel & Suspension Visualizers"]
        CB["Collision Bridge (Cones & Obstacles)"]
        INS["RosIns / RosLidar / RosCamera"]
    end

    subgraph GDExt["GDExtension Layer (libter26_vehicle.so)"]
        BIND["Ter26VehicleBridge (vehicle_model_binding.cpp)"]
        CONV["ISO 8855 <-> Godot 3D Frame Conversions"]
    end

    subgraph Core["C++20 Physics Core (libter26_core.a)"]
        ENG["VehicleDynamicsEngine::step_vehicle_dynamics()"]
        KIN["3D Suspension Kinematics & 6x6 Joint Wrench Solver"]
        MBD["14-DOF Multibody EoM: M(q)dv/dt + C(q,v) = Q"]
        HPITM["HPITM Tire Model (MF6.2 + Bounded Neural + Relaxation + Bristle Static)"]
        PT["Powertrain (TeR26 RWD Dual-Motor + Planetary Gearbox FBD)"]
        AERO["Ride-Height Sensitive Aerodynamics & Hydraulic/EBS Brakes"]
    end

    CTRL --> MC
    MC --> BIND
    CB --> BIND
    BIND --> CONV
    CONV --> ENG
    ENG --> KIN
    ENG --> MBD
    ENG --> HPITM
    ENG --> PT
    ENG --> AERO
    ENG --> CONV
    CONV --> MC
    MC --> WV
    MC --> INS
    INS --> SENS
```

---

## 3. Directory Structure

- `Legacy/`: Original `vehicleMotion_legacy.gd` preserved for A/B comparison and instant rollback.
- `Data/`: Centralized JSON engineering database (`TeR26/`, `Tires/`, `Calibration/`, `Schemas/`).
- `Native/`: C++20 core headers (`include/ter26/`), core implementation (`src/core/`), SymPy/SymForce generated C++ kernels (`generated/symforce/`), GDExtension bindings (`src/godot/`), and C++20 unit/numerical/benchmark tests (`tests/`).
- `Godot/`: Modular GDScript adapters (`vehicle_adapter.gd`, `wheel_visualizer.gd`, `suspension_visualizer.gd`, `vehicle_debug_draw.gd`, `vehicle_collision_bridge.gd`).
- `Tools/`: Python 3.12 scientific pipelines (`symforce/`, `tire_fitting/`, `identification/`, `parameter_optimization/`, `validation/`, `visualization/`).
- `Tests/`: Automated test runners and reference fixtures (`unit/`, `numerical/`, `regression/`, `integration/`, `fixtures/`).
- `Docs/`: Complete engineering documentation suite (`architecture/`, `coordinate_systems/`, `equations/`, `interfaces/`, `identification/`, `validation/`).
