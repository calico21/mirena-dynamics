# Physical Model Specification (`PHYSICAL_MODEL.md`)

## 1. Degrees of Freedom (10-DOF Vehicle Formulation)

The vehicle model tracks **10 dynamical degrees of freedom** plus internal first-order actuator and tire-relaxation states:
1. **Sprung-Mass / Vehicle Rigid Body (6 DOF)**:
   - Translational velocity $\mathbf{v}_B = [u, v, w]^T$ in Vehicle Body Frame `B_V` ($\text{m/s}$).
   - Rotational angular velocity $\boldsymbol{\omega}_B = [p, q, r]^T$ (roll, pitch, yaw rates) in `B_V` ($\text{rad/s}$).
2. **Wheel Rotational Spin (4 DOF)**:
   - Independent angular velocities $\omega_i \in \{\omega_{\text{FL}}, \omega_{\text{FR}}, \omega_{\text{RL}}, \omega_{\text{RR}}\}$ ($\text{rad/s}$).
3. **Internal First-Order States**:
   - Steering rack angle $\delta_{\text{rack}}$ ($\tau_{\text{steer}} = 0.025\text{ s}$).
   - Motor torque state $T_m$ ($\tau_{\text{motor}} = 0.020\text{ s}$).
   - Hydraulic brake force state $F_{\text{brk}}$ ($\tau_{\text{brake}} = 0.030\text{ s}$).
   - Per-wheel relaxed longitudinal slip $\kappa_{\text{rel},i}$ ($\sigma_\kappa = 0.12\text{ m}$) and lateral slip $\alpha_{\text{rel},i}$ ($\sigma_\alpha = 0.15\text{ m}$).

---

## 2. Subsystem Equations

### 2.1 Steering Geometry (`Steering/`)
Given commanded steering angle $\delta_{\text{cmd}}$, the rack angle evolves via rate and bandwidth limits, and individual front wheel angles $\delta_{\text{FL}}, \delta_{\text{FR}}$ combine fractional Ackermann geometry with static toe and suspension bump steer:

$$
\delta_{\text{Ack,FL/FR}} = \arctan\!\left(\frac{L \tan\delta_{\text{rack}}}{L \mp \frac{t_f}{2}\tan\delta_{\text{rack}}}\right), \qquad
\delta_{i} = (1 - f_{\text{Ack}})\delta_{\text{rack}} + f_{\text{Ack}}\delta_{\text{Ack},i} + \delta_{\text{toe},i} + k_{\text{bs},i} z_{c,i}
$$

### 2.2 Suspension & Vertical Normal Load (`Suspension/`)
Static corner weights are determined from CoM longitudinal distances $l_f, l_r$ and wheelbase $L = l_f + l_r$:

$$
F_{z,\text{stat,FL/FR}} = \frac{1}{2} m g \frac{l_r}{L}, \qquad F_{z,\text{stat,RL/RR}} = \frac{1}{2} m g \frac{l_f}{L}
$$

For each corner $i$ with compression $z_{c,i}$ and compression rate $\dot{z}_{c,i}$:
1. **Spring + Bump Stop (`SpringModel.gd`)**:
   $$F_{\text{spring},i} = F_{z,\text{stat},i} + k_{s,i} \text{MR}_i^2 z_{c,i} + k_{\text{bs}} \max(0, z_{c,i} - z_{c,\max})$$
2. **4-Way Digressive Damper (`DamperModel.gd`)**:
   Piecewise-linear low-speed ($|\dot{z}| \le v_{\text{knee}}$) and high-speed ($|\dot{z}| > v_{\text{knee}}$) slopes with independent bump ($\dot{z}_{c,i} > 0$) and rebound ($\dot{z}_{c,i} < 0$) coefficients.
3. **Anti-Roll Bar (`AntiRollBarModel.gd`)**:
   $$F_{\text{ARB,L}} = +k_{\text{ARB}}(z_{c,L} - z_{c,R}), \qquad F_{\text{ARB,R}} = -k_{\text{ARB}}(z_{c,L} - z_{c,R})$$
   satisfying $F_{\text{ARB,L}} + F_{\text{ARB,R}} \equiv 0$.
4. **Unilateral Ground Normal Load (`SuspensionModel.gd`)**:
   $$F_{z,i} = \begin{cases} \max\!\left(0,\; F_{\text{spring},i} + F_{\text{damper},i} + F_{\text{ARB},i}\right) & \text{if wheel } i \text{ in contact} \\ 0 & \text{otherwise} \end{cases}$$

### 2.3 Wheel Kinematics & Slip Regularization (`Wheels/WheelKinematics.gd`)
The hub velocity in `B_V` accounts for full 3D chassis velocity and angular velocity at corner offset $\mathbf{r}_i = [x_i, y_i, z_i]^T$:

$$\mathbf{v}_{\text{hub},i} = \mathbf{v}_B + \boldsymbol{\omega}_B \times \mathbf{r}_i$$

After rotating horizontal components into the steered wheel frame `W_i` by $\delta_i$ to obtain $(v_{wx,i}, v_{wy,i})$, slip ratio $\kappa_i$ and slip angle $\alpha_i$ are computed with low-speed regularization ($v_{\varepsilon,\kappa} = 0.5\text{ m/s}$, $v_{\varepsilon,\alpha} = 0.25\text{ m/s}$):

$$
\kappa_i = \text{clamp}\!\left(\frac{\omega_i r_{\text{eff},i} - v_{wx,i}}{\max(|v_{wx,i}|,\; |\omega_i r_{\text{eff},i}|,\; v_{\varepsilon,\kappa})},\; -1,\; +1\right), \qquad
\alpha_i = \arctan2\!\left(v_{wy,i},\; \sqrt{v_{wx,i}^2 + v_{\varepsilon,\alpha}^2}\right)
$$

### 2.4 Tire Relaxation & Pluggable Force Interface (`Tires/`)
First-order transient tire relaxation (`TireModel.gd`) integrates:

$$
\frac{d\kappa_{\text{rel},i}}{dt} = \frac{\max(|v_{wx,i}|, v_{\min})}{\sigma_\kappa}(\kappa_i - \kappa_{\text{rel},i}), \qquad
\frac{d\alpha_{\text{rel},i}}{dt} = \frac{\max(|v_{wx,i}|, v_{\min})}{\sigma_\alpha}(\alpha_i - \alpha_{\text{rel},i})
$$

using an unconditionally stable exponential decay step $\exp(-\Delta t \cdot v / \sigma)$. The resulting `TireInputs` are passed to `TireModelInterface.compute_forces()`, which returns $(F_{wx,i}, F_{wy,i}, M_{wz,i})$ in `W_i`.

### 2.5 Powertrain, Brakes & Wheel Rotational ODE (`Powertrain/`, `Brakes/`, `Wheels/`)
- **Motor Envelope (`MotorModel.gd`)**:
  $$T_{m,\max}(\omega_m) = \min\!\left(T_{\text{peak}},\; \frac{P_{\max}}{\max(\omega_m, 1\text{ rad/s})}\right)$$
  with linear derating near $\text{RPM}_{\max}$ and forward-only lockout ($u_{\text{thr}} \ge 0$).
- **Wheel Spin ODE (`WheelRotationalDynamics.gd`)**:
  $$J_{w,i}\frac{d\omega_i}{dt} = T_{\text{drive},i} - r_{\text{eff},i} F_{wx,i} - b_{\text{drag}}\omega_i + T_{\text{brake,applied},i}$$
  where passive brake torque $T_{\text{brake},i}$ opposes motion and clamps $\omega_i = 0$ whenever braking exceeds the net accelerating impulse over $\Delta t$, preventing numerical chattering or backward wheel spin at rest.

### 2.6 Aerodynamics (`Aerodynamics/AerodynamicModel.gd`)
At forward airspeed $u_+ = \max(0, u)$ with dynamic pressure $q_\infty = \frac{1}{2}\rho A u_+^2$:

$$
F_{\text{drag}} = q_\infty C_d, \qquad F_{\text{down}} = q_\infty C_l, \qquad
F_{z,\text{aero},f} = s_f F_{\text{down}}, \qquad F_{z,\text{aero},r} = (1 - s_f) F_{\text{down}}
$$

### 2.7 Chassis Wrench Summation & Specific Force (`Chassis/`)
`RigidBodyDynamics.gd` sums all four corner tire/suspension forces and aerodynamic wrenches about the CoM:

$$
\mathbf{F}_{\text{net},B} = \mathbf{F}_{\text{aero},B} + \sum_{i=0}^{3} \mathbf{F}_{\text{corner},i}, \qquad
\mathbf{M}_{\text{net},B} = \mathbf{M}_{\text{aero},B} + \sum_{i=0}^{3} \left(\mathbf{r}_i \times \mathbf{F}_{\text{corner},i} + [0, 0, M_{wz,i}]^T\right)
$$

`ChassisModel.gd` computes both kinematic CoM acceleration $\mathbf{a}_{\text{kin},B}$ and true IMU accelerometer specific force $\mathbf{f}_{\text{imu},B} = \mathbf{a}_{\text{kin},B} - \mathbf{g}_B$.
