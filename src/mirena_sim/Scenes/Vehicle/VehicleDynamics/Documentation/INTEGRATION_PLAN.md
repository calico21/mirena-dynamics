# Godot & ROS 2 Integration Plan (`INTEGRATION_PLAN.md`)

## 1. Technical Analysis: `VehicleBody3D` / `VehicleWheel3D` vs. `RigidBody3D`

### 1.1 How the Legacy Implementation Works (`vehicleMotion.gd` + `mirena_car.tscn`)
In `mirena_car.tscn`, the root node `mirena_car` is a `VehicleBody3D` (which inherits from `RigidBody3D`) with four `VehicleWheel3D` children (`Wheel0`, `Wheel1`, `Wheel2`, `Wheel3`). Inside Godot's C++ `GodotBody3D` / `VehicleBody3D` solver:
- Every physics tick, `VehicleBody3D` automatically casts internal rays from each `VehicleWheel3D`, computes its own internal spring/damper impulse, and applies its own heuristic friction-slip impulse (`wheel_friction_slip = 10.0` front, `12.0` rear) plus `engine_force` and `brake` impulses directly to the chassis body.
- In `vehicleMotion.gd`, `apply_central_force(...)` was simultaneously called to inject aerodynamic drag and downforce.
- **Why `VehicleBody3D` cannot remain active alongside a custom tire/suspension model**:
  1. **Double Force Application**: If `VehicleWheel3D` nodes remain active children of a `VehicleBody3D`, Godot's internal C++ solver continues applying its own normal, suspension, and lateral/longitudinal tire impulses on top of any forces applied by `VehicleDynamicsController`.
  2. **Opaque Internal Wheel Spin**: `VehicleWheel3D.get_rpm()` is driven by Godot's internal raycast kinematic heuristic rather than a true rotational differential equation $J_w \dot{\omega}_w = T_{\text{drive}} - T_{\text{brake}} - r_{\text{eff}} F_x$, causing unrealistic wheel-speed spikes during braking and cornering (documented in `imu_compare/summary_front10_rear12.md`).
  3. **No Access to Combined-Slip or Camber Physics**: `VehicleWheel3D` exposes only a single scalar `wheel_friction_slip` parameter per wheel.

### 1.2 Chosen Replacement Strategy: `RigidBody3D` Chassis + Visual-Only Wheel Nodes
When transitioning `mirena_car.tscn` in Phase 4:
1. The root node type of `mirena_car.tscn` changes from `VehicleBody3D` to `RigidBody3D` (or `VehicleWheel3D` nodes are replaced by `Node3D` visual wheel anchors `Wheel0`..`Wheel3` at the exact same transforms).
2. **Zero Double-Counting Guarantee**:
   - `Integration/GodotPhysicsAdapter.gd` is the **sole authority** that calls `chassis_body.apply_central_force(force_world)` and `chassis_body.apply_torque(torque_world)`.
   - Gravity continues to be applied once by Godot's `RigidBody3D` gravity integrator (while `SuspensionModel` generates the upward ground reaction $F_z$, matching physical reality at equilibrium $4 \bar{F}_z = m g$).
   - The `CollisionShape3D` (`CollisionShape3D2` box in `mirena_car.tscn`, layer 2, mask 3) remains intact on the chassis `RigidBody3D` so **cone collisions (`Cone/cone.gd`)** and **lap-timing gate triggers (`Gate/gate.gd`, which check `body.is_in_group("car_body")`)** work identically without any change.

---

## 2. Compatibility Preservation Matrix

Every external interface required by `Sim.tscn`, `sim.gd`, `UI/vehicle_menu.gd`, `UI/stats.gd`, `Track/Gate/gate.gd`, `addons/rclgd-sensors/*`, `mirena_state`, and `mirena_control` is preserved by `VehicleDynamicsController.gd`, `VehicleSceneAdapter.gd`, and `ROSVehicleAdapter.gd`:

| Simulator / ROS Interface | Legacy Mechanism (`vehicleMotion.gd`) | New Mechanism (`VehicleDynamicsController.gd`) | Status |
| :--- | :--- | :--- | :--- |
| Node group `"car_body"` | Set in `mirena_car.tscn` | Preserved on root `RigidBody3D` | Compatible |
| `use_ros_control: bool` | Property toggled by `vehicle_menu.gd:59` | `@export var use_ros_control: bool` on `VehicleDynamicsController` | Compatible |
| `get_speed_kph() -> float` | Called by `UI/stats.gd:26` | Implemented on `VehicleDynamicsController.get_speed_kph()` | Compatible |
| `next_camera()` | Called by `sim.gd:48` on `change_camera` input | Delegated to `VehicleSceneAdapter.next_camera()` | Compatible |
| Reset (`R` key / UI button) | Zeroes `linear_velocity`, `angular_velocity`, resets `global_position = (0, 0.5, 0)` | `VehicleDynamicsController.reset_vehicle()` zeroes `RigidBody3D` + calls `model.reset()` + `ros_adapter.reset()` | Compatible |
| `/mirena/control/car` (`CarControl.msg`) | Subscribed on `/mirena/control/car` (`gas`, `brake`, `steer_angle`, `ebs`) | Subscribed in `ROSVehicleAdapter` with identical topic & field names | Compatible |
| `/mirena/car` (`Car.msg`) | Published at physics rate (`u, v, w, yaw, omega, ax, ay, az`) | Published via `ROSVehicleAdapter.build_car_telemetry_dict()` | Compatible |
| `/mirena/wheel_speeds` (`WheelSpeeds.msg`) | Published at 125 Hz (`fl, fr, rl, rr` in RPM) | Published at 125 Hz via `ROSVehicleAdapter.compute_conditioned_wheel_speeds_rpm()` | Compatible |
| Onboard Sensors (`Sensors/INS`, `Camera`, `Lidar`, `Transforms`) | Child nodes of `mirena_car` reading parent `RigidBody3D` state | Remain child nodes of `mirena_car`; `ros_ins.gd` reads `RigidBody3D.linear_velocity` and `angular_velocity` directly | Compatible |

---

## 3. Staged Migration Roadmap (Phases 2–5)

- **Phase 1 (Current — Complete)**: Full modular framework, data contracts, configuration `.tres` resources, adapters, diagnostics, and headless unit/integration test suite created under `Scenes/Vehicle/VehicleDynamics/`. `mirena_car.tscn` and `vehicleMotion.gd` remain untouched.
- **Phase 2 (Tire & Contact Calibration)**: Implement `PacejkaMF62TireModel.gd` (loading `TeR_Tire/config/pacejka_mf62_params.yaml`) and `TeRTireGreyBoxModel.gd` (implementing `TireModelInterface.gd`), register them in `TireModelRegistry.gd`, and verify against Hoosier 16.0x7.5-10 R20 TTC curves.
- **Phase 3 (Suspension, Powertrain & Encoder Tuning)**: Calibrate spring/damper/ARB rates and wheel rotational inertia $J_w$ so transient load transfer and braking wheel slip match `imu_compare/` telemetry without unphysical lockup spikes.
- **Phase 4 (Scene Cutover)**: Create a toggleable or replacement `mirena_car.tscn` binding `VehicleDynamicsController.gd` to the `mirena_car` `RigidBody3D` and visual wheel nodes while archiving `vehicleMotion.gd`.
- **Phase 5 (Closed-Loop Bag Replay & MPC Validation)**: Execute `run_sim_replay.sh` + `compare_imu.py` and closed-loop MPC (`mirena_control` + `mirena_state`) lap verification.
