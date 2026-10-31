# VehicleDynamics Software Architecture

## 1. Architectural Overview & Design Principles

The `VehicleDynamics` framework replaces the monolithic `vehicleMotion.gd` script (which coupled Godot `VehicleBody3D` raycast-wheel internals, powertrain curves, aerodynamic forces, visual steering meshes, camera cycling, and ROS 2 publishing in a single class) with a strictly layered architecture governed by four principles:

1. **Strict Separation of Physical Concerns**: Each physical domain (Contact, Suspension, Steering, Wheel Kinematics, Tire Force Generation, Powertrain, Brakes, Aerodynamics, Chassis Rigid-Body Wrench Summation) resides in an isolated module with explicit typed inputs and outputs.
2. **Pure-Physics Core vs. Engine/Middleware Adapters**: `VehicleDynamicsModel.gd` and all physics modules (`Core/`, `Chassis/`, `Contact/`, `Suspension/`, `Steering/`, `Wheels/`, `Tires/`, `Powertrain/`, `Brakes/`, `Aerodynamics/`) have **zero dependencies** on the Godot scene tree (`VehicleWheel3D`, `Camera3D`, `NodePath`) or ROS 2 (`RosNode`, `rclgd`). All Godot physics engine I/O, scene visual updates, and ROS 2 communications are confined to `Integration/`.
3. **Immutable Configuration vs. Mutable Runtime State**: Physical parameters live exclusively in `Resource` classes (`*Configuration.gd`, `VehicleParameters.gd`) serialized in `Config/*.tres`. Runtime dynamic variables live in `RefCounted` state containers (`VehicleState`, `WheelState`, `SuspensionState`, `ContactState`, `TireState`, `BrakeState`) with deterministic `.reset()` methods.
4. **Single Force-Application Authority**: To eliminate double-counted tire, gravity, or suspension forces, only `Integration/GodotPhysicsAdapter.gd` applies forces and torques to the chassis `RigidBody3D`.

---

## 2. Module Dependency Graph

Dependencies are strictly acyclic (DAG) flowing downward from `VehicleDynamicsController` through `VehicleDynamicsModel` and `Integration/` to individual domain modules and `Core/`:

```mermaid
flowchart TD
    Controller["VehicleDynamicsController.gd<br/>(Godot Node Entry Point)"]
    Model["VehicleDynamicsModel.gd<br/>(Pure-Physics 13-Step Orchestrator)"]
    Config["VehicleDynamicsConfig.gd<br/>(Composite Resource)"]

    subgraph Integration ["Integration/ (Godot & ROS Adapters)"]
        GodotAdapter["GodotPhysicsAdapter.gd"]
        SceneAdapter["VehicleSceneAdapter.gd"]
        ROSAdapter["ROSVehicleAdapter.gd"]
    end

    subgraph Subsystems ["Physics Subsystems (Pure Math / RefCounted)"]
        Contact["Contact/<br/>WheelContact, ContactQuery"]
        Suspension["Suspension/<br/>SuspensionModel, Spring, Damper, ARB"]
        Steering["Steering/<br/>SteeringModel, SteeringGeometry"]
        Wheels["Wheels/<br/>WheelAssembly, Kinematics, RotationalDynamics"]
        Tires["Tires/<br/>TireModel, TireModelRegistry, TireModelInterface"]
        Powertrain["Powertrain/<br/>PowertrainModel, MotorModel, DrivetrainModel"]
        Brakes["Brakes/<br/>BrakeSystem"]
        Aero["Aerodynamics/<br/>AerodynamicModel"]
        Chassis["Chassis/<br/>ChassisModel, RigidBodyDynamics, VehicleInertia"]
    end

    subgraph Telemetry ["Telemetry/"]
        Diag["DynamicsDiagnostics.gd"]
        Logger["DynamicsLogger.gd"]
    end

    Core["Core/<br/>VehicleCoordinates, VehicleUnits, VehicleParameters, VehicleState, VehicleDynamicsTypes"]

    Controller --> Model
    Controller --> Config
    Controller --> Integration
    Controller --> Logger

    Model --> Config
    Model --> Subsystems
    Model --> Diag
    Subsystems --> Core
    Integration --> Core
    Diag --> Core
```

---

## 3. Per-Step Data-Flow Pipeline (13-Step Substepped Execution)

Within each Godot `_physics_process(delta)` tick, `VehicleDynamicsController` queries the chassis pose/velocity via `GodotPhysicsAdapter` and ground contact geometry via `ContactQuery`, then executes `N` (default `4`) deterministic substeps of `VehicleDynamicsModel.step(dt_sub, ...)`:

```mermaid
sequenceDiagram
    participant Godot as Godot PhysicsServer3D
    participant Ctrl as VehicleDynamicsController
    participant Model as VehicleDynamicsModel
    participant Steer as SteeringModel
    participant Susp as SuspensionModel
    participant Kin as WheelKinematics
    participant Tire as TireModel (Interface)
    participant PT as Powertrain & Brakes
    participant Rot as WheelRotationalDynamics
    participant Chas as ChassisModel
    participant ROS as ROSVehicleAdapter

    Godot->>Ctrl: _physics_process(delta)
    Ctrl->>Godot: Read chassis Transform3D, v_body, omega_body & 4-corner raycasts
    loop N substeps (dt_sub = delta / N)
        Ctrl->>Model: step(dt_sub, throttle, brake, steer, ...)
        Model->>Steer: 1. Compute wheel steer angles delta_i & static/bump toe
        Model->>Model: 2. Compute effective loaded radii r_eff,i
        Model->>Susp: 3. Resolve F_spring, F_damper, F_ARB & normal loads Fz_i >= 0
        Model->>Kin: 4. Project chassis velocity to hubs -> slip ratio kappa_i & slip angle alpha_i
        Model->>Tire: 5. Evaluate TireModelInterface -> (Fx_w, Fy_w, Mz_w)_i
        Model->>Model: 6. Rotate tire forces by delta_i into vehicle body frame
        Model->>PT: 7-8. Compute drive torques T_drive,i (80 kW / 100 Nm) & brake torques T_brake,i
        Model->>Rot: 9. Integrate J_w * d(omega_i)/dt = T_drive - T_brake - r_eff * Fx_w - T_drag
        Model->>Chas: 10-12. Sum Aero + Suspension + Tire wrenches about CoM & compute IMU specific force
    end
    Ctrl->>Godot: Apply net chassis force & torque (single authority)
    Ctrl->>ROS: Publish /mirena/car & conditioned /mirena/wheel_speeds
```

---

## 4. Architectural Decisions & File Organization Notes

All files specified in the target architecture have been created with explicit responsibilities. Rather than leaving empty stubs, every subsystem script implements its complete data contract, coordinate convention, and validation logic:
- **Tire Model Separation**: `Tires/TireModel.gd` manages per-wheel state and first-order transient relaxation lengths ($\sigma_\kappa, \sigma_\alpha$), while delegating steady-state force generation `(F_x, F_y, M_z)` to `Tires/Models/TireModelInterface.gd` resolved through `Tires/TireModelRegistry.gd`. This guarantees that Phase 2 tire implementations (Pacejka MF6.2 from `TeR_Tire/config/pacejka_mf62_params.yaml` or the `TeR_Tire` Physics-Informed Grey-Box residual model) drop in without modifying a single line of `TireModel.gd`, `WheelAssembly.gd`, or `VehicleDynamicsModel.gd`.
