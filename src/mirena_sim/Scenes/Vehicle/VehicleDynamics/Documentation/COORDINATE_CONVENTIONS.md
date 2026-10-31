# Coordinate Conventions & Sign Definitions

## 1. Important Repository Audit Finding: `src/mirena_sim/README.md` vs. Active Code

> [!WARNING]
> The legacy `src/mirena_sim/README.md` (Section "Coordinate Systems") states:
> `ROS X (Forward) = Godot +Z`, `ROS Y (Left) = Godot +X`, `ROS Z (Up) = Godot +Y`.
> **This statement in `src/mirena_sim/README.md` is incorrect** (and defines a left-handed frame where $\hat{X}_{\text{ROS}} \times \hat{Y}_{\text{ROS}} = -\hat{Z}_{\text{ROS}}$).

Direct inspection of the active codebase proves that **MirenaSim actually uses standard right-handed Godot $-Z$ forward, $-X$ left, $+Y$ up**:

1. **`mirena_car.tscn`**:
   - Front wheels (`Wheel0` FL, `Wheel1` FR) are placed at `z = -0.992727` ($-Z$ is forward).
   - Rear wheels (`Wheel2` RL, `Wheel3` RR) are placed at `z = +0.599941` ($+Z$ is rearward).
   - Left wheels (`Wheel0` FL, `Wheel2` RL) are placed at `x = -0.586657` ($-X$ is left).
   - Right wheels (`Wheel1` FR, `Wheel3` RR) are placed at `x = +0.586657` ($+X$ is right).
2. **`vehicleMotion.gd`** (lines 273–280, 384–387, 441):
   - Forward velocity `u = -local_vel.z`.
   - Leftward velocity `v = -local_vel.x`.
   - Upward velocity `w = +local_vel.y`.
   - Yaw angle `yaw = global_rotation.y`, yaw rate `omega = angular_velocity.y`.
3. **`Scenes/Track/track.gd`** (lines 83–85, 96–98):
   - Track spawns `Vector3(-point.y, 0, -point.x)` with comments `# ROS X -> Godot -Z` and `# ROS Y -> Godot -X`.
4. **`addons/rclgd-sensors/ros_ins/ros_ins.gd`** (lines 562, 586, 666–673):
   - `imu_msg.angular_velocity = Vector3(-local_ang_vel.z, -local_ang_vel.x, local_ang_vel.y)`.
   - `imu_msg.linear_acceleration = Vector3(-specific_force_local.z, -specific_force_local.x, specific_force_local.y)`.

All transformations in `Core/VehicleCoordinates.gd` implement this verified right-handed mapping and are covered by unit tests in `Tests/test_wheel_kinematics.gd`.

---

## 2. The Five Canonical Reference Frames

### Frame 1 & 2: Godot World (`W_G`) and Godot Local Chassis (`B_G`)
Right-handed Y-up engine frames:
- $+X_G$: Right
- $+Y_G$: Up
- $+Z_G$: Rearward (so $-Z_G$ is Forward)

### Frame 3: Vehicle Body / ROS `base_link` (`B_V`, ISO 8855 / REP-103)
Right-handed Z-up vehicle frame with origin at the sprung-mass Center of Mass (CoM):
- $+X_V$: Forward longitudinal axis
- $+Y_V$: Leftward lateral axis
- $+Z_V$: Upward vertical axis

#### Exact Transformation Equations (`Core/VehicleCoordinates.gd`)

For any linear vector (position offset $\mathbf{r}$, velocity $\mathbf{v}$, acceleration $\mathbf{a}$, or force $\mathbf{F}$):

$$
\begin{bmatrix} x_V \\ y_V \\ z_V \end{bmatrix} =
\begin{bmatrix} 0 & 0 & -1 \\ -1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}
\begin{bmatrix} x_G \\ y_G \\ z_G \end{bmatrix}
\quad \Longleftrightarrow \quad
\begin{bmatrix} x_G \\ y_G \\ z_G \end{bmatrix} =
\begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & 1 \\ -1 & 0 & 0 \end{bmatrix}
\begin{bmatrix} x_V \\ y_V \\ z_V \end{bmatrix}
$$

Because $\det(T) = +1$, angular velocity $\boldsymbol{\omega} = [p, q, r]^T$ and moment $\mathbf{M} = [M_x, M_y, M_z]^T$ transform identically:
- **Roll rate** $p = +\omega_{x,V} = -\omega_{z,G}$ (positive = right side of car moves down)
- **Pitch rate** $q = +\omega_{y,V} = -\omega_{x,G}$ (positive = nose moves down)
- **Yaw rate** $r = +\omega_{z,V} = +\omega_{y,G}$ (positive = counter-clockwise viewed from above / left turn)

### Frame 4: Individual Steered Wheel / Contact-Patch Frame (`W_i`)
Rotated from `B_V` about $+Z_V$ by wheel steering angle $\delta_i$ (`+` = left turn):
- $+X_{W,i}$: Along wheel intersection plane with ground (forward rolling direction)
- $+Y_{W,i}$: Leftward perpendicular to wheel plane
- $+Z_{W,i}$: Upward along contact normal

Planar horizontal transformation between `B_V` and `W_i`:

$$
\begin{bmatrix} v_{wx,i} \\ v_{wy,i} \end{bmatrix} =
\begin{bmatrix} \cos\delta_i & \sin\delta_i \\ -\sin\delta_i & \cos\delta_i \end{bmatrix}
\begin{bmatrix} v_{bx,i} \\ v_{by,i} \end{bmatrix},
\qquad
\begin{bmatrix} F_{bx,i} \\ F_{by,i} \end{bmatrix} =
\begin{bmatrix} \cos\delta_i & -\sin\delta_i \\ \sin\delta_i & \cos\delta_i \end{bmatrix}
\begin{bmatrix} F_{wx,i} \\ F_{wy,i} \end{bmatrix}
$$

---

## 3. Wheel Indexing Convention

Every array of size 4 across the entire framework (`VehicleDynamicsTypes.WheelIndex`) uses the ordering:

| Index | Enum | Corner | Body Coordinates $(x_i, y_i)$ in `B_V` |ROS `WheelSpeeds.msg` Field |
| :---: | :--- | :--- | :--- | :--- |
| `0` | `FL` | Front-Left | $(+l_f, +t_f/2)$ | `fl` |
| `1` | `FR` | Front-Right | $(+l_f, -t_f/2)$ | `fr` |
| `2` | `RL` | Rear-Left | $(-l_r, +t_r/2)$ | `rl` |
| `3` | `RR` | Rear-Right | $(-l_r, -t_r/2)$ | `rr` |

---

## 4. Summary of Physical Sign Conventions

| Quantity | Symbol | Unit | Positive (`+`) Meaning |
| :--- | :---: | :---: | :--- |
| Steering command / angle | $\delta$ | $\text{rad}$ | Left turn (CCW viewed from above) |
| Wheel angular velocity | $\omega_i$ | $\text{rad/s}$ | Forward rolling |
| Suspension deflection | $z_{c,i}$ | $\text{m}$ | Compression (bump / jounce upward into chassis) |
| Suspension velocity | $\dot{z}_{c,i}$ | $\text{m/s}$ | Compressing (bump velocity $> 0$, rebound $< 0$) |
| Normal tire load | $F_{z,i}$ | $\text{N}$ | Upward ground support on tire ($F_{z,i} \ge 0$, never tensile) |
| Longitudinal slip ratio | $\kappa_i$ | $-$ | Traction ($\omega_i r_{\text{eff},i} > v_{wx,i} \implies \kappa_i > 0, F_{wx,i} > 0$) |
| Lateral slip angle | $\alpha_i$ | $\text{rad}$ | $\arctan2(v_{wy,i}, |v_{wx,i}|) > 0$ produces $-F_{wy,i}$ restoring force |
| Camber / inclination angle | $\gamma_i$ | $\text{rad}$ | Negative when top of wheel tilts inward toward vehicle centerline |
