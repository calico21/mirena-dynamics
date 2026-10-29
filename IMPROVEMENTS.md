# Mirena Architecture Review — Sim-to-Real Improvements

*Generated 2026-06-12. All P0/P1/P2/P3 items applied 2026-06-12.*  
*Goal: make switching between MirenaSim and the real car (Jetson + sensors + CAN) a one-flag operation instead of a code-editing exercise.*

## 1. Current architecture (as observed)

The workspace is a classic perception → state estimation → planning → control pipeline,
with a Godot-based simulator (`mirena_sim` on top of `rclgd`) on one side and real
drivers (`mirena_hardware`, `mirena_can_bus`) on the other:

```
                 SIM (mirena_sim/rclgd)                REAL (mirena_hardware, mirena_can_bus)
                 ──────────────────────                ──────────────────────────────────────
 sensors         /car/cloud, /car/image_raw,           /mirena_lidar/cloud, /mirena_camera/image,
                 /car/camera_info, IMU                 /mirena_camera/camera_info, vectornav IMU,
                                                       gss_velocity, CAN (vcan0)
 ground truth    car/debug_state, car/debug_perception,
                 car/debug_slam, track_manager/track

 pipeline        mirena_perception  → /perception/cones
                 mirena_slam        → slam/map          (subs: car/debug_state, car/debug_perception)
                 mirena_consensus   → /consensus/car_state (subs: GPS/IMU/GSS)
                 mirena_planning    → planner/track     (subs: car/debug_slam)
                 mirena_control     → control output    (subs: track_manager/track, car/debug_state)
                 mirena_supervisor  → lifecycle mgmt, /health heartbeats, mission status
```

Support: `mirena_common` (msgs), `mirena_rviz2_plugins`, `mirena_utilities` (RemoCon).

### The core problem

**The pipeline is not actually chained, and each stage is wired to a different
world.** Every node was developed against the simulator's ground-truth debug topics:

- `mirena_slam` consumes `car/debug_state` + `car/debug_perception` (sim ground truth), **not** `/perception/cones`.
- `mirena_planning` consumes `car/debug_slam` (sim ground truth), **not** `slam/map`.
- `mirena_control` consumes `track_manager/track` (the sim's ground-truth track) and `car/debug_state`, **not** `planner/track` or any estimated state.
- `mirena_perception` subscribes to `/mirena_lidar/cloud` / `/mirena_camera/image` (hardware names), while the sim publishes `/car/cloud` / `/car/image_raw` — so perception only works against real hardware, while everything downstream only works against the sim.

None of these topics ever existed on the real car. Switching to hardware today means
editing hardcoded constants in five packages. Everything below follows from fixing this.

---

## 2. Recommendations (priority order)

### P0 — Define one canonical topic contract

Pick a single set of interface topics that *both* the sim and the hardware drivers must
satisfy, and make the autonomy nodes know only those names. Suggested contract:

| Interface topic | Type | Sim provides | Real provides |
|---|---|---|---|
| `/sensors/lidar/points` | `PointCloud2` | GPU lidar | `mirena_lidar` |
| `/sensors/camera/image` + `/camera_info` | `Image`/`CameraInfo` | sim camera | `mirena_camera` |
| `/sensors/imu` | `Imu` | sim IMU | vectornav |
| `/sensors/gss` | `TwistStamped` | sim | `gss_node` |
| `/perception/cones` | `EntityList` | `mirena_perception` (same node both worlds) | same |
| `/state/pose` | `Car` (or `Odometry`) | `mirena_consensus`/`mirena_slam` | same |
| `/slam/map` | `EntityList` | `mirena_slam` | same |
| `/planning/track` | `Track` | `mirena_planning` | same |
| `/car/control` | `CarControl` | sim vehicle consumes | `mirena_can_bus` consumes |

Keep the sim's `debug_*` ground-truth topics — they're genuinely useful — but treat them
as *substitutes injected by remapping*, never as the names the pipeline subscribes to.

Practical steps:
- Move all `*_TOPIC` constants (e.g. [planner_node.py:11](src/mirena_planning/mirena_planning/planner_node.py#L11), [mirena_slam.cpp:27-29](src/mirena_slam/src/mirena_slam.cpp#L27-L29), [mirena_perception.py:26-33](src/mirena_perception/mirena_perception/mirena_perception.py#L26-L33), [mirena_consensus.py:25-30](src/mirena_consensus/mirena_consensus/mirena_consensus.py#L25-L30)) to the canonical names, and rely on launch-time remapping for everything else. Hardcoded constants are fine *if* they are the contract names — ROS remapping then handles the rest without any parameters.
- Document the contract in one place (e.g. `docs/topics.md` or `mirena_common/README.md`) and make it the review checklist for new nodes.

### P0 — Create a `mirena_bringup` package with launch files

There are currently **zero launch files** in the workspace (only the vendored vectornav
one). Every run is a stack of `ros2 run` commands, which is exactly why topic wiring
drifted. Create one package that owns composition:

```
mirena_bringup/
  launch/
    autonomy.launch.py      # perception+slam+consensus+planning+control+supervisor (world-agnostic)
    sim.launch.py           # mirena_sim + autonomy.launch.py (use_sim_time:=true)
    real.launch.py          # hardware drivers + can_bus + autonomy.launch.py
  config/
    sim.yaml                # per-node params for sim
    real.yaml               # per-node params for the car (TeR.yaml lives here too)
```

Then "switch from simulation to real hardware" becomes:

```bash
ros2 launch mirena_bringup sim.launch.py mission:=trackdrive
ros2 launch mirena_bringup real.launch.py mission:=trackdrive
```

Add launch arguments for stage bypassing during development, implemented as remaps of
sim ground truth onto the contract topics, e.g.
`ground_truth_slam:=true` → remap `slam/map` ⇒ `car/debug_slam` for the planner. This
preserves the current "develop each stage against ground truth" workflow without baking
it into source code.

### P1 — Parameterize hardware bindings

- `mirena_can_bus` hardcodes `"vcan0"` ([mirena_can_bus.py:32](src/mirena_can_bus/mirena_can_bus/mirena_can_bus.py#L32)). Make the CAN interface a declared parameter (`can_interface: vcan0|can0`) set from the bringup config. Same review for serial/device paths in the camera, lidar, GSS and IMU drivers — anything that names a `/dev/*`, IP, or interface should be a parameter with the real-car value in `real.yaml`.
- The hardware drivers' good habit of `~/topic` private names + node name (`mirena_camera`, `mirena_lidar`) should be kept; just remap them onto the contract names in `real.launch.py`.

### P1 — Use sim time consistently

`rclgd` already publishes `/clock` when `use_sim_time` is set ([rclgd.cpp:91](src/rclgd/src/rclgd/rclgd.cpp#L91)), but the only configs in the repo set `use_sim_time: false` ([config.yaml](src/mirena_control/config/config.yaml)). In sim, **every** node must run with `use_sim_time: true` or timestamp-sensitive code (TF lookups, heartbeat timeouts, MPC horizons) behaves differently than on the car — the most common source of "works in sim, breaks on hardware" bugs. This is a one-liner once the bringup launch files exist (`SetParameter(name='use_sim_time', value=LaunchConfiguration('sim'))`).

### P1 — One source of truth for the TF tree / sensor extrinsics

The frame tree is currently defined twice: the sim broadcasts its own TF, and the real
car has `mirena_hardware/config/transforms.yaml` + a transform node. Define the frame
names (`map → odom → base_link → {lidar, camera, imu, gss}`) once — ideally a URDF/xacro
or a single static-transform YAML in `mirena_common`/`mirena_bringup` — and make both the
sim and the hardware transform publisher consume it. Perception's camera-lidar fusion
should read extrinsics from TF, not from a separate calibration file, so the same
perception binary is correct in both worlds.

### P2 — Finish the lifecycle/supervisor story

The supervisor concept (lifecycle sequencing + heartbeats + mission configs) is the
right architecture for FS rules compliance, but it's half-wired:

- `MAX_REPORT_SLACK_SEC = 10000000` ([mirena_supervisor.py:22](src/mirena_supervisor/mirena_supervisor/mirena_supervisor.py#L22)) disables heartbeat timeouts — restore a real value (and make it a parameter).
- Some nodes are `LifecycleNode` (can_bus, camera, lidar, imu), the Python pipeline nodes mostly aren't. Converting perception/slam/planning/control to lifecycle nodes lets the supervisor own startup ordering and the AS state machine cleanly, in *both* sim and real.
- The mission YAMLs ([mission_config/](src/mirena_supervisor/mission_config/)) are a good seed for the bringup configs — consider folding them into `mirena_bringup/config/missions/`.

### P2 — QoS and message hygiene

- Sensor topics should use `SensorDataQoS` (best-effort) consistently on both publisher and subscriber sides; perception already does this for the camera, but verify the sim's lidar/camera publishers match, otherwise topics silently fail to connect on the real network even though they worked in intraprocess sim.
- Consider standard types where tooling matters: `nav_msgs/Odometry` instead of (or alongside) `mirena_common/Car` for state would give you rviz, robot_localization, and rosbag tooling for free. Keep `EntityList`/`Track` — those are genuinely domain-specific.
- `mirena_consensus` publishes `/consensus/car_state` that nothing consumes, and subscribes a `"/void"` GSS topic — decide whether consensus or SLAM is the state estimator and wire the loser out.

### P3 — Repo/workspace hygiene

- `src/mirena_utilities/install/` looks like a committed colcon artifact — remove it and gitignore.
- `mirena_planning`, `mirena_perception`, `mirena_supervisor` are separate git repos/submodules with uncommitted drift (`m` in git status). With the topic contract being cross-cutting, consider whether the submodule split still pays for itself versus a monorepo; at minimum, pin submodule revisions that are known to satisfy the contract together.
- Record a canonical rosbag per mission from the sim (sensor topics only) and use it for offline regression of perception/SLAM — the cheapest "third world" between sim and real.

---

## 3. Suggested order of attack

1. Agree on the topic contract table (§2-P0) — one meeting, write `docs/topics.md`.
2. Rename topics in nodes to contract names; fix the perception sim/real split.
3. Build `mirena_bringup` with `sim.launch.py` / `real.launch.py` + per-world YAMLs.
4. `use_sim_time` everywhere in sim; parameterize `vcan0` and device paths.
5. Unify TF/extrinsics source; then lifecycle conversion + supervisor timeouts.

After steps 1–4, sim↔real switching is a launch-file choice and nothing else.
