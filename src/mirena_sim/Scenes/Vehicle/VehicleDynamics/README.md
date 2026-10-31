# MirenaSim Modular Vehicle-Dynamics Framework (`VehicleDynamics/`)

This directory contains the modular, physically rigorous 10-DOF vehicle-dynamics architecture for **MirenaSim** (6-DOF sprung chassis rigid body + 4 independent wheel rotational DOFs), integrated directly into the drivable `MirenaCar` (`../mirena_car.tscn` / `../vehicleMotion.gd`) alongside a dedicated, resizable **Vehicle Dynamics & Correlation Dashboard Window** (`UI/VehicleDynamicsDashboardWindow.gd`).

## Key Capabilities

1. **Live Drivable Integration & Single-Authority Runtime Switching (`F6`)**:
   - `MirenaCar` boots in **`MODULAR_VEHICLE_DYNAMICS`** mode by default.
   - Pressing **`F6`** (or using the top-center HUD button / `Tab` Vehicle Settings menu / Dashboard Window selector) toggles at runtime between:
     - **`MODULAR_VEHICLE_DYNAMICS` (`1`)**: The 4 `VehicleWheel3D` nodes are reparented under `ModularWheelHolder` so `VehicleBody3D` has **0 direct `VehicleWheel3D` children** and applies zero native raycast wheel/suspension forces. `VehicleDynamicsController` is the sole authority for tire, suspension, aerodynamic, powertrain, and braking forces/moments (`GodotPhysicsAdapter.apply_forces_to_body`) and visual wheel transforms (`VehicleSceneAdapter.update_wheel_visuals`).
     - **`LEGACY_GODOT_VEHICLE_BODY` (`0`)**: The 4 `VehicleWheel3D` nodes are reparented back as direct children of `MirenaCar` at their exact rest transforms, and `VehicleDynamicsController` switches to `INACTIVE_LEGACY_PASSTHROUGH`.
2. **Local `TeR_Tire` Grey-Box PINN & Pacejka MF6.2 Integration (`Tires/Models/`)**:
   - Directly integrates `/home/alexr/eRacing/mirena_ws-SIM/src/mirena_sim/Scenes/Vehicle/VehicleDynamics/Tires/TeR_Tire`:
     - **`PacejkaMF62TireModel.gd`**: Exact GDScript port of `TeR_Tire/models/pacejka_mf62.py` using the 27 calibrated Hoosier 16.0x7.5-10 (R20, 8" rim, 12 psi) coefficients from `TeR_Tire/config/pacejka_mf62_params.yaml`.
     - **`TeRTireGreyBoxModel.gd`**: Exact GDScript port of `TeR_Tire/models/grey_box_tire.py` and `TeR_Tire/models/network.py` (`ResidualTireMLP`: `7 -> 64 -> 64 -> 2` SiLU MLP with bounded `tanh` scaling and zero-slip physics gate) using embedded checkpoint weights (`TeRTireWeightsData.gd`) extracted from `TeR_Tire/export/weights/pinn_residual_best.pth`.
     - **Explicitly Separated Longitudinal & Combined-Slip Companion**: Because `TeR_Tire` only calibrates pure lateral force $F_{y0}$ and aligning moment $M_{z0}$, longitudinal force $F_x$ and friction-ellipse combined-slip coupling ($G_{yx}$) are computed in a clearly labeled companion method (`_evaluate_uncalibrated_longitudinal_companion`) and reported distinctly in provenance metadata.
3. **Separate Resizable Diagnostics & Real-vs-Sim Correlation Window (`F7`)**:
   - Pressing **`F7`** (or clicking **Diagnostics & Correlation (F7)** in the HUD or Vehicle Settings menu) opens `VehicleDynamicsDashboardWindow`, an independent native Godot `Window` with two clearly separated modes:
     - **Mode A — Live Simulation Monitoring**: Real-time multi-trace oscilloscope plots (`SignalPlotCanvas.gd`), friction-circle / XY trajectory scatter plots, structured telemetry cards across all **8 vehicle-dynamics categories**, and live parameter sliders (`TeR Lateral Grip Scale`, `PINN Residual Enable`, `Tire Model Selector`, `Front/Rear Spring Stiffness`, `Front/Rear ARB`, `Max Motor Torque`, `Brake Bias`, `Aero Cl/Cd`).
     - **Mode B — Real-vs-Simulation Correlation Analysis**: Loads real vehicle logs (`imu_compare/real_*.csv`, single CSV files, or ROS 2 bag directories) and simulation recordings (`imu_compare/sim_*.csv` or live-recorded modular runs), aligns timestamps transparently (auto-detected maneuver onset + manual time-shift slider), renders overlaid time-series + residual error ($y_{\text{sim}} - y_{\text{real}}$) + XY correlation scatter plots, and computes per-channel metrics (`RMSE`, `MAE`, `Bias`, `Peak Error`, `Pearson r`, `NRMSE`, `Gain Ratio`, `Estimated Lag`) with explicit status badges (`Evaluated`, `Validation warning`, `Awaiting reference data`, `Insufficient data`, `Not evaluated`).

## Quick-Start: Launching MirenaSim & Running Automated Verification

### 1. Run the 43-Test Headless Verification Suite

```bash
/home/alexr/mirena_ws-main/mirena_ws/build/rclgd/godot-engine/Godot_v4.7.1-stable_linux.x86_64 \
  --headless --path /home/alexr/eRacing/mirena_ws-SIM/src/mirena_sim \
  -s res://Scenes/Vehicle/VehicleDynamics/Tests/test_vehicle_dynamics.gd
```

### 2. Launch the Full 3D Simulator

```bash
/home/alexr/mirena_ws-main/mirena_ws/build/rclgd/godot-engine/Godot_v4.7.1-stable_linux.x86_64 \
  --path /home/alexr/eRacing/mirena_ws-SIM/src/mirena_sim
```

- Press **`WASD`** to drive manually (or select `PilotMode` in the **`Tab`** menu).
- Press **`F6`** at any time to switch between **Modular 6-DOF (`TeR_Tire`)** and **Legacy (`VehicleBody3D`)**.
- Press **`F7`** at any time to open the resizable **Vehicle Dynamics & Correlation Window**.

## Directory Map

| Directory / File | Responsibility |
| :--- | :--- |
| `VehicleDynamicsController.gd` | Top-level Godot `Node` entry point coordinating scene adapters, substepped physics execution, ROS I/O, and telemetry logging. |
| `VehicleDynamicsModel.gd` | Pure-physics 13-step vehicle-dynamics orchestrator (no scene-tree or ROS dependencies). |
| `VehicleDynamicsConfig.gd` | Composite Godot `Resource` aggregating all subsystem configurations. |
| `Core/` | Canonical SI units (`VehicleUnits`), coordinate transformations (`VehicleCoordinates`), wheel/fault enums (`VehicleDynamicsTypes`), static parameters (`VehicleParameters`), and runtime state (`VehicleState`). |
| `Chassis/` | Inertia tensor construction (`VehicleInertia`), 6-DOF wrench summation about CoM (`RigidBodyDynamics`), and specific-force accelerometer kinematics (`ChassisModel`). |
| `Contact/` | Raycast/analytic ground queries (`ContactQuery`), contact patch kinematics (`WheelContact`), and unilateral contact state (`ContactState`). |
| `Suspension/` | Springs + bump stops (`SpringModel`), 4-way digressive dampers (`DamperModel`), anti-roll bars (`AntiRollBarModel`), and corner normal-load resolution (`SuspensionModel`). |
| `Steering/` | Actuator rate/bandwidth filtering (`SteeringModel`) and Ackermann + static-toe + bump-steer wheel angles (`SteeringGeometry`). |
| `Wheels/` | Per-wheel state (`WheelState`), hub velocity & slip kinematics (`WheelKinematics`), 1-DOF spin integration with passive brake lockup (`WheelRotationalDynamics`), and 4-wheel container (`WheelAssembly`). |
| `Tires/` | Pluggable tire contract (`Models/TireModelInterface.gd`), calibrated `TeR_Tire` Pacejka MF6.2 (`Models/PacejkaMF62TireModel.gd`), `TeR_Tire` 7D PINN Grey-Box (`Models/TeRTireGreyBoxModel.gd`, `Models/TeRTireWeightsData.gd`), registry (`TireModelRegistry.gd`), and explicit I/O structs (`TireInputs`, `TireState`, `TireForces`). |
| `Powertrain/` | Formula Student 80 kW / 100 Nm motor envelope (`MotorModel`), gear reduction & open/viscous differential (`DrivetrainModel`), and orchestrator (`PowertrainModel`). |
| `Brakes/` | Hydraulic 60/40 front/rear brake split, actuator lag, and Emergency Brake System (`BrakeSystem`, `BrakeState`, `BrakeConfiguration`). |
| `Aerodynamics/` | Quadratic aerodynamic drag, front/rear downforce split, and drag-induced pitch moment (`AerodynamicModel`, `AerodynamicConfiguration`). |
| `Integration/` | Single-authority `RigidBody3D` wrench injector (`GodotPhysicsAdapter`), visual wheel bridge (`VehicleSceneAdapter`), and `rclgd` ROS 2 bridge with encoder conditioning (`ROSVehicleAdapter`). |
| `Telemetry/` | Real-time diagnostic monitor (`DynamicsDiagnostics`), 30-column CSV logger (`DynamicsLogger`), canonical recording schema (`TelemetryChannel`, `TelemetryRecording`), multi-format importer (`TelemetryImporter`), and quantitative correlation engine (`CorrelationEngine`). |
| `UI/` | Custom multi-trace time-series & XY scatter canvas (`SignalPlotCanvas.gd`) and separate resizable diagnostics & correlation window (`VehicleDynamicsDashboardWindow.gd`). |
| `Config/` | Default `.tres` resources (`vehicle_default.tres`, `tire_default.tres`, `suspension_default.tres`, `powertrain_default.tres`). |
| `Tests/` | Standalone headless unit and integration test suites (`test_*.gd` — 43 assertions). |
| `Documentation/` | Detailed engineering specifications (`ARCHITECTURE.md`, `PHYSICAL_MODEL.md`, `COORDINATE_CONVENTIONS.md`, `PARAMETER_PROVENANCE.md`, `INTEGRATION_PLAN.md`, `VALIDATION_PLAN.md`). |
