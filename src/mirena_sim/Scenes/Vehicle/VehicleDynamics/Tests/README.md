# VehicleDynamics Unit & Integration Tests (`Tests/`)

This directory contains headless Godot 4 unit and integration tests for the `VehicleDynamics` framework. All tests run without opening the graphical editor, spawning ROS 2 nodes, or loading the full track scene.

## Running the Complete Suite

From the workspace root (`/home/alexr/eRacing/mirena_ws-SIM`), execute:

```bash
godot --headless --path src/mirena_sim -s res://Scenes/Vehicle/VehicleDynamics/Tests/test_vehicle_dynamics.gd
```

*(If using the workspace-built Godot 4.7.1 binary with `rclgd`, point `LD_LIBRARY_PATH` to the `rclgd` library directory if needed, though the unit tests themselves do not instantiate GDExtension nodes.)*

## Running Individual Subsystem Suites

Each subsystem test file extends `SceneTree` and can also be run individually:

| Test Script | Tested Modules & Invariants |
| :--- | :--- |
| `test_tire_model.gd` | `TireModelInterface`, `TireModelRegistry`, `LINEAR_TEST_STUB` (`is_calibrated() == false`), `UNIMPLEMENTED_CONTRACT` fault flag, `F_z = 0` contact loss, force/moment sign conventions, combined-slip friction ellipse, transient slip relaxation, and `NaN`/`Inf` rejection. |
| `test_wheel_kinematics.gd` | `VehicleCoordinates` (Godot $\leftrightarrow$ ISO 8855/ROS linear & angular mappings, body $\leftrightarrow$ steered wheel frame orthogonality), `WheelKinematics` (chassis yaw-rate hub velocity transfer, slip ratio $\kappa$ and slip angle $\alpha$ signs, standstill $u = 0$ regularization), and `VehicleUnits`. |
| `test_suspension_model.gd` | `SpringModel` (static preload, linear rate, bump stop), `DamperModel` (4-way digressive bump/rebound asymmetry), `AntiRollBarModel` ($F_{\text{ARB},L} + F_{\text{ARB},R} = 0$), and `SuspensionModel` ($F_z \ge 0$ unilateral contact constraint). |
| `test_powertrain.gd` | `PowertrainModel`, `MotorModel` (100 Nm peak torque, 80 kW hyperbolic power ceiling, forward-only lockout), `DrivetrainModel` (5.0:1 reduction, 0.92 efficiency), `BrakeSystem` (60/40 bias, EBS), and `WheelRotationalDynamics` (passive brake lockup at $\omega_w = 0$). |
| `test_vehicle_dynamics.gd` | Master runner + full-vehicle integration tests: loading & validating `Config/vehicle_default.tres`, static axle load distribution, 13-step longitudinal traction + aerodynamic downforce/drag cycle, left-turn yaw moment sign, `ROSVehicleAdapter` wheel encoder filtering/quantization, `DynamicsLogger` 28-column CSV schema, and deterministic `reset()`. |
