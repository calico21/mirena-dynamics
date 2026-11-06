# Initial Project Audit — TeR26 Multibody Vehicle Dynamics Engine

**Date:** 2026-11-06  
**Target Workspace:** `/home/alexr/eRacing/mirena-ws-SIM/src/mirena_sim/Scenes/Vehicle`  
**Target Vehicle:** Formula Student **TeR26** (2026 season, Rear-Wheel Drive dual-motor architecture)

---

## 1. Current Simulation Architecture (`MirenaSim`)

### 1.1 Godot Engine & Physics Configuration
- **Godot Version:** `v4.7.1.stable.official.a13da4feb` (`/home/alexr/.local/bin/godot`), configured via `project.godot` with feature tag `"4.7"`.
- **Physics Rate:** `physics/common/physics_ticks_per_second = 400` ($\Delta t = 2.5\text{ ms}$), `physics/common/max_physics_steps_per_frame = 20`.
- **Threading:** `physics/3d/run_on_separate_thread = true`.
- **Autoloads:**
  - `rclgd_init`: Initializes the `librclgd.so` ROS 2 Jazzy GDExtension (`addons/rclgd/rclgd.gdextension`).
  - `Sim`: Global simulation manager (`res://Scenes/Sim/sim.gd`).

### 1.2 Scene Hierarchy (`mirena_car.tscn`)
- **Root Node:** `MirenaCar` (`VehicleBody3D`, script `res://Scenes/Vehicle/vehicleMotion.gd`, `class_name MirenaCar`, groups `["vehicle", "Car"]`).
  - `mass = 300.0` kg, `center_of_mass_mode = 1` (custom), `center_of_mass = Vector3(0, 0.33, -0.117)`.
  - `collision_layer = 2` (Vehicle layer), `collision_mask = 5` (World/Track layer 1 + Cones/Obstacles layer 4).
  - `contact_monitor = true`, `max_contacts_reported = 1`.
- **Child Nodes:**
  - `CAR_MESH` (`Node3D`): Embedded visual meshes of the formula car chassis and bodywork.
  - `CAR_COLLIDER` (`CollisionShape3D`): `BoxShape3D` (`size = Vector3(1.132, 0.858, 2.753)`) at `Vector3(0, 0.453, -0.095)`.
  - `FL_WHEEL`, `FR_WHEEL`, `RL_WHEEL`, `RR_WHEEL` (`VehicleWheel3D`):
    - Front positions: `(-0.587, 0.208, -0.993)` and `(0.587, 0.208, -0.993)`.
    - Rear positions: `(-0.587, 0.208, 0.600)` and `(0.587, 0.208, 0.600)`.
    - Derived wheelbase: $L = 0.993 + 0.600 = 1.593\text{ m}$ ($l_f = 0.993 - 0.117 = 0.876\text{ m}$, $l_r = 0.600 + 0.117 = 0.717\text{ m}$).
    - Derived track width: $t_f = t_r = 2 \times 0.587 = 1.174\text{ m}$.
    - Nominal wheel radius: $r_w = 0.208\text{ m}$.
  - `FPCam`, `TPCam` (`Camera3D`): First-person and third-person viewport cameras.
  - `Lidar` (`RosLidar`, `frame_id = "car/lidar"`): GPU depth-map raycaster publishing `sensor_msgs/msg/PointCloud2` on `/sensors/lidar/points` and `sensor_msgs/msg/Imu` on `/sensors/lidar/imu`.
  - `Camera` (`RosCamera`, `frame_id = "car/camera"`): SubViewport sensor publishing `/sensors/camera/image` and `/sensors/camera/camera_info`.
  - `INS` (`RosIns`, `frame_id = "car/imu"`): Reads parent `RigidBody3D` state (`global_position`, `global_basis`, `linear_velocity`, `angular_velocity`, `origin`) at 200 Hz and publishes `/sensors/imu`, `/sensors/imu/odom`, and `/sensors/gps/fix`.
  - `MirenaTransform` (`RosTransformManager`): Broadcasts static TF tree (`car/cog` $\to$ `car/imu`, `car/lidar`, `car/camera`) on `/tf_static`.

### 1.3 Limitations of the Legacy Vehicle Physics (`vehicleMotion.gd`)
1. **Dual-Authority / Semi-Analytical Override:** The legacy `vehicleMotion.gd` sets `freeze = false` on `VehicleBody3D` for collision detection, zeros wheel friction slips (`wheel_friction_slip = 0.0`), and manually overwrites `linear_velocity` ($v_x, v_z$) and `angular_velocity.y` ($r$) every tick using a planar 3-DOF (`vx`, `vy`, `r`) model plus quasi-static load transfer (`Fz = Fz_static +/- dFz_long +/- dFz_lat`). Meanwhile, Godot's Jolt/GodotPhysics solver simultaneously integrates vertical, roll, and pitch dynamics via `VehicleWheel3D` raycasts.
2. **Absence of 3D Multibody Suspension Kinematics:** No 3D hardpoint closure, no wishbone/pushrod/rocker geometry, no camber/toe/scrub/trail curves vs. jounce and steer, no anti-dive/anti-squat geometry, and no joint reaction wrenches for structural validation.
3. **Simplified Powertrain & Wheel Dynamics:** No planetary or gear reduction multibody reaction torques, simplified slip calculation without true 14-DOF inertial coupling ($M(q)\ddot{q} + C(q,\dot{q}) = Q$), and artificial velocity clamping (`if speed < 0.03: vx = 0`) instead of physical bristle/static friction holding.
4. **Simplified Tire Physics:** Pure-slip 4-parameter Pacejka (`PacejkaTireModel.gd`) without MF6.2 camber/load/combined-slip coupling, overturning/rolling/aligning moments ($M_x, M_y, M_z$), or bounded HPITM neural correction.

---

## 2. External Interface Contract (Driverless, Sensors & UI)

Every external consumer in `MirenaSim` and the ROS 2 Driverless stack depends on the following contract, which **must be preserved 100% intact**:

| Consumer | Interface Type | Symbol / Topic / Node Path | Details & Units |
| :--- | :--- | :--- | :--- |
| `sim.gd`, `vehicle_menu.gd`, `stats.gd` | GDScript Class & Groups | `class_name MirenaCar`, groups `"vehicle"`, `"Car"` | Must inherit `VehicleBody3D` (or `RigidBody3D`) so `RosIns` (`get_parent() as RigidBody3D`) works without modifying sensor scripts. |
| `sim.gd`, `vehicle_menu.gd` | Enum & Properties | `PilotMode { NO_PILOT, MANUAL, ROS, TRACK_RAIL }`, `pilot`, `path`, `origin`, `gas`, `steering`, `brake`, `_mission_info`, `_as_info`, `_as_info_pub`, `enable_ros_node`, `frame_id`, `tire_diagnostics` | `gas` $\in [-1, 1]$, `steering` in rad (CCW+), `brake` $\in [0, 1]$. |
| `sim.gd`, `track.gd` | Public Methods | `set_origin(transform, reset_vel=false)`, `reset_position()`, `cone_collision_set(enable: bool)`, `get_cones_in_sight(max_dist=10.0)` | Used for track spawning, lap resets, and perception ground-truth debugging. |
| `sim.gd` | Signal | `signal cone_hit(count: int)` | Emitted when colliding with a node in group `"cone"`. |
| `RosIns` (`ros_ins.gd`) | Parent Node Properties | `global_position`, `global_basis`, `linear_velocity`, `angular_velocity`, `origin` | Read every physics tick; requires smooth, noise-free world-space velocities so numeric differentiation `(v - prev_v)/dt` produces accurate IMU specific force. |
| `RosLidar`, `RosCamera`, `RosTransformManager` | Child Nodes | `$Lidar`, `$Camera` (`_camera`), `$INS` (`.reset()`), `$MirenaTransform` | Must remain at their exact node paths and transforms relative to `MirenaCar`. |
| ROS 2 Subscribers | Topics | `/control` (`mirena_common/msg/CarControl`), `/system/mission_status` (`MissionInfo`), `/as_status` (`AsInfo`) | `CarControl.gas` $\in [-1, 1]$, `CarControl.steer_angle` in rad (`+` = left). |
| ROS 2 Publishers | Topics | `/as_status` (`AsInfo`), `/debug/state/car` (`mirena_common/msg/Car`), `/debug/perception` (`EntityList`), `/debug/slam` (`EntityList`), `/inferred_control` (`CarControl`), `/lap_status` (`LapStatus`), `/sensors/wheel_speeds` (`WheelSpeeds`, 100 Hz `BEST_EFFORT`) | `WheelSpeeds` publishes `rl` and `rr` in RPM; `fl` and `fr` are populated in full multibody mode. |

---

## 3. Audit of Available Engineering Data Across Workspace

We audited `/home/alexr/eRacing/mirena-ws-SIM` and all sibling repositories (`/home/alexr/eRacing/TeR_TWIN`, `/home/alexr/eRacing/TeR_Tire`, `/home/alexr/eRacing/FS_Driver_Setup_Optimizer`, `/home/alexr/eRacing/TeR_ECU-main`, `/home/alexr/eRacing/mirena_ws-SIM_02-11`).

### 3.1 Critical Vehicle Discrepancy Identified: TeR26 vs. TeR27
During inspection of `/home/alexr/eRacing/FS_Driver_Setup_Optimizer/config/vehicles/`, two distinct vehicle configurations were found:
- **`ter26.py` + `active_vehicle_config.json` + `TeR_ECU-main` (TeR26 — 2026 Vehicle, Target of this Project):**
  - **Drivetrain:** Rear-Wheel Drive (RWD), dual rear electric motors (`100 Nm` peak per motor, `5.0:1` gear ratio in `TeR_ECU` / `mirena_sim`, `80 kW` total power limit, mechanical/viscous differential + rear torque vectoring).
  - **Mass:** `298.0 - 300.0 kg` with driver (`223 - 225 kg` car + `75 kg` driver).
  - **Suspension:** Front pushrod (`MR = 1.14`), Rear pullrod/pushrod (`MR = 1.16`), springs `35.03 N/mm` (front) / `52.54 N/mm` (rear), Horstman digressive dampers (`c_low = 1800/1600 N·s/m`, `c_high = 720/640 N·s/m`, `v_knee = 0.10 m/s`, `rebound_ratio = 1.60`, `gas_preload = 120 N`).
- **`ter27.py` + `TeR27_OptimumK.xlsx` + `Ter27_Chassis_Loads_Report.xlsx` (TeR27 — 2027 Vehicle):**
  - **Drivetrain:** 4WD in-hub planetary (`10:1` or `14:1` reduction, 4 motors).
  - **Hardpoints:** Contains complete 16-point OptimumKinematics CAD exports (`TeR27_OptimumK.xlsx`) and ANSYS load cases (`Ter27_Chassis_Loads_Report.xlsx`).

**Resolution (Rule 2.1 & 2.3):**
1. All baseline parameters in `Data/TeR26/` represent **TeR26** (RWD, `300 kg`, `1.593 m` wheelbase, `35.03 / 52.54 N/mm` springs, `5.0:1` gear reduction).
2. In `ter26.py`, only 9 suspension hardpoints per corner were exported (`lca_in_f`, `lca_in_r`, `lbj`, `uca_in_f`, `uca_in_r`, `ubj`, `pushrod_out`, `rocker_piv`, `spring_in`), lacking explicit CAD coordinates for `tie_in`, `tie_out`, `rocker_axis`, `rocker_rod`, `rocker_coi`, `antiroll_in`, and `antiroll_out`. In `Data/TeR26/hardpoints/front.json` and `rear.json`, we store the 9 TeR26 points as `CAD_DERIVED_APPROXIMATE` and complete the remaining kinematic points with `GEOMETRICALLY_DERIVED` values tuned to match TeR26's documented motion ratio (`1.14` F / `1.16` R), static toe, and Ackermann geometry, while explicitly flagging their provenance. We also provide the exact 16-point OptimumKinematics fixture (`Tests/fixtures/ter27_optimumk_hardpoints.json`) to validate the 3D kinematics and 6x6 reaction load solver against the `Ter27_Chassis_Loads_Report.xlsx` reference.
3. Our C++20 powertrain subsystem implements **both** TeR26's actual RWD dual-motor drivetrain and the full 3-element planetary gearbox FBD (sun, planet carrier, ring, stator housing reaction torque on upright vs. wheel) so both architectures are supported without compromise.

### 3.2 Calspan TTC Round 9 Tire Data (`TeR_Tire`)
- **Available Cornering Runs (`RunData_Cornering_ASCII_SI_Round9/`):**
  - `Hoosier 16.0x7.5-10 R20 C2000`: Runs 2–4 (`7.0"` rim), Runs 5–9 (`8.0"` rim; note Runs 5–6 have unlabeled headers).
  - `Hoosier 16.0x6.0-10 R20 C2000`: Runs 10–12 (`6.0"` rim), Runs 13–15 (`7.0"` rim).
- **Available Drive/Brake Runs (`RunData_DriveBrake_ASCII_SI_Round9/`):**
  - Runs 68–73 (`Hoosier 18.0x6.0-10 R20`, 10" rim longitudinal + combined slip). Note: Calspan Round 9 did not test `16.0x7.5-10` or `16.0x6.0-10` on the longitudinal machine; therefore longitudinal MF6.2 parameters for the 16" tires are transferred from the 10" R20 compound family (`Runs 68–73`) and scaled by tire load/radius, with provenance explicitly marked `TTC_R20_COMPOUND_SCALED`.

### 3.3 Real TeR26 Telemetry (`imu_compare/`)
- `/home/alexr/eRacing/mirena-ws-SIM/imu_compare/` contains synchronized real-car logs (`real_control.csv`, `real_imu.csv`, `real_odom.csv`, `real_wheels.csv`) spanning $t \in [0, 77]\text{ s}$ with dynamic driving over $t \in [38, 76]\text{ s}$. This dataset is used by our correlation pipeline (`Tools/validation/correlate_telemetry.py`).

---

## 4. Toolchain & Build Environment Audit

| Component | Status | Path / Version | Notes |
| :--- | :--- | :--- | :--- |
| **Godot 4.7** | Available | `/home/alexr/.local/bin/godot` (`v4.7.1.stable`) | Supports headless execution (`--headless`). |
| **C++20 Compiler** | Available | `/usr/bin/g++` (`GCC 13.3.0`), `/usr/bin/cmake` (`3.28.3`) | Full `-std=c++20 -O3 -march=native` support. |
| **Eigen3** | Available | `/usr/include/eigen3` (`Eigen 3.4.0`) | Used for dense/small-matrix linear algebra and decompositions. |
| **godot-cpp** | Available | `/home/alexr/eRacing/TeR_TWIN/cpp/external/godot-cpp` | Prebuilt `libgodot-cpp.linux.template_debug.x86_64.a` available for fast CMake linking. |
| **Python 3.12 Env** | Available | `/home/alexr/project_gp_env/bin/python` | Includes `numpy 2.4.3`, `scipy 1.15.3`, `jax 0.9.0.1`, `flax 0.12.4`, `optax 0.2.6`, `torch 2.10.0`, `sympy 1.14.0`, `casadi 3.7.2`, `pandas 2.3.3`, `matplotlib 3.10.8`. |
| **Symbolic Codegen** | Available via SymPy/JAX | `Tools/symforce/generate_kernels.py` | Generates CSE-optimized C++20 analytical kernels into `Native/generated/symforce/`. |
