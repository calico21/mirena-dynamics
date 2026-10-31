# VehicleDynamics Configuration Resources (`Config/`)

This directory stores the Godot 4 `Resource` (`.tres`) files that parameterize the modular vehicle-dynamics framework. Every physical constant used at runtime originates from these resources—no physical numbers are hard-coded inside subsystem scripts.

## Resource Hierarchy

```text
vehicle_default.tres (VehicleDynamicsConfig)
├── vehicle_parameters      (SubResource: VehicleParameters)
├── suspension_config       -> suspension_default.tres (SuspensionConfiguration)
├── steering_config         (SubResource: SteeringConfiguration)
├── powertrain_config       -> powertrain_default.tres (PowertrainConfiguration)
├── brake_config            (SubResource: BrakeConfiguration)
├── aero_config             (SubResource: AerodynamicConfiguration)
├── tire_config_front       -> tire_default.tres (TireConfiguration)
└── tire_config_rear        -> tire_default.tres (TireConfiguration)
```

## Files in This Directory

| File | Resource Script | Purpose |
| :--- | :--- | :--- |
| `vehicle_default.tres` | `VehicleDynamicsConfig.gd` | Top-level composite vehicle configuration (`TER26_Baseline`) matching the verified MirenaSim `mirena_car.tscn` geometry and mass properties. |
| `tire_default.tres` | `Tires/TireConfiguration.gd` | Hoosier 16.0x7.5-10 R20 nominal geometry and vertical stiffness (`R_0 = 0.2032 m`, `F_z0 = 700 N`, `C_z = 110,000 N/m`). Selects `"LINEAR_TEST_STUB"` in Phase 1; switch `model_id` to `"PACEJKA_MF62"` or `"TER_TIRE_GREYBOX"` in Phase 2. |
| `suspension_default.tres` | `Suspension/SuspensionConfiguration.gd` | Front/rear spring rates, 4-way digressive damping coefficients, anti-roll bar stiffnesses, travel limits, and camber/bump-steer gains. |
| `powertrain_default.tres` | `Powertrain/PowertrainConfiguration.gd` | Formula Student 80 kW power limit, 100 Nm peak motor torque, 5.0:1 single-stage reduction, 0.92 drivetrain efficiency, and forward-only lockout. |

## Creating Setup Variants

To create an alternate setup (for example, `vehicle_ter27.tres`, a wet-track setup, or a high-downforce setup):
1. Duplicate `vehicle_default.tres` or any sub-resource in the Godot Editor Inspector (or via text copy).
2. Modify the `@export` fields in the new `.tres` file.
3. Assign the new `VehicleDynamicsConfig` resource to `VehicleDynamicsController.config` or pass it to `VehicleDynamicsModel.configure(cfg)`.
4. See `../Documentation/PARAMETER_PROVENANCE.md` for the physical source and verification status of every parameter.
