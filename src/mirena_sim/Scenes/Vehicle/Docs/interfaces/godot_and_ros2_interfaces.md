# Godot 4.7 Integration, ROS 2 Interface, and External Collision Bridge — TeR26

## 1. Single Physical Authority in `MirenaCar` (`vehicleMotion.gd`)

When `use_native_multibody = true` (default), `MirenaCar` delegates 100% of vehicle dynamics integration to the C++20 `Ter26VehicleBridge` GDExtension:
1. **Godot Physics De-confliction:**
   - All four `VehicleWheel3D` nodes (`FL_WHEEL`, `FR_WHEEL`, `RL_WHEEL`, `RR_WHEEL`) have their Godot spring stiffness, damping, and friction slip zeroed (`wheel_friction_slip = 0.0`, `suspension_stiffness = 0.0`, `damping_compression = 0.0`, `damping_relaxation = 0.0`) so Godot's internal raycast vehicle never applies parasitic forces.
   - During each `_physics_process(delta)` at 400 Hz:
     1. Ground plane height $z_{\text{ground},i}$ and unit normal $\hat{\mathbf{n}}_{\text{ground},i}$ under each wheel are queried via Godot `PhysicsDirectSpaceState3D.intersect_ray` (masking Layer 1 = Track/Ground) and converted from Godot 3D ($\mathcal{F}_G$) to ISO 8855 ($\mathcal{F}_W$).
     2. Any external collision impulses detected on `MirenaCar` (such as cones on Layer 4 or barriers) are converted via the impulse-momentum bridge and passed to `Ter26VehicleBridge.step(delta, gas, brake, steering, ground_planes, ext_wrench)`.
     3. The integrated 6-DOF chassis pose $(\mathbf{p}_B^W, R_B^W)$ and 6-DOF velocity $(\mathbf{v}_B^W, \boldsymbol{\omega}_B^W)$ are converted from ISO 8855 ($\mathcal{F}_W$) to Godot 3D ($\mathcal{F}_G$) and written directly to `global_transform`, `linear_velocity`, and `angular_velocity` of `MirenaCar`.
     4. The 3D wheel poses $(\mathbf{p}_{W,i}^B, R_{W,i}^B)$ computed by the 3D suspension kinematics + wheel spin $\phi_{w,i}$ are applied directly to `$FL_WHEEL`, `$FR_WHEEL`, `$RL_WHEEL`, and `$RR_WHEEL`.

---

## 2. External Collision & Cone Impulse-Momentum Bridge

When `MirenaCar` contacts an external rigid body $B_{\text{ext}}$ (e.g., a cone `Cone` of mass $m_c = 0.2\text{ kg}$ at contact point $\mathbf{p}_c^W$ with normal $\hat{\mathbf{n}}_c^W$ and relative closing velocity $v_{\text{rel},n} = (\mathbf{v}_{B,\text{contact}}^W - \mathbf{v}_{c,\text{contact}}^W) \cdot \hat{\mathbf{n}}_c^W < 0$):
1. Let $J_c(q) \in \mathbb{R}^{3 \times 14}$ be the contact point Jacobian mapping generalized velocities $v$ to the world velocity of the contact point on the vehicle.
2. The normal impulse magnitude $j_n$ with coefficient of restitution $e \in [0, 1]$ is:
   $$
   j_n = \frac{-(1 + e) v_{\text{rel},n}}{\hat{\mathbf{n}}_c^T J_c M(q)^{-1} J_c^T \hat{\mathbf{n}}_c + m_{\text{ext}}^{-1} + (\mathbf{r}_{\text{ext}} \times \hat{\mathbf{n}}_c)^T \mathbf{I}_{\text{ext}}^{-1} (\mathbf{r}_{\text{ext}} \times \hat{\mathbf{n}}_c)}
   $$
3. Tangential friction impulse $\mathbf{j}_t$ is bounded by Coulomb's cone $\|\mathbf{j}_t\| \le \mu_c j_n$.
4. The total impulse $\boldsymbol{\mathcal{J}} = j_n \hat{\mathbf{n}}_c + \mathbf{j}_t$ produces an exact instantaneous generalized velocity jump on the multibody state:
   $$
   \Delta v = M(q)^{-1} J_c(q)^T \boldsymbol{\mathcal{J}}
   $$
   while applying $-\boldsymbol{\mathcal{J}}$ to the Godot rigid body (`cone.apply_impulse(...)`) and emitting `cone_hit.emit(count)`.
   - **Conservation Guarantee:** $\Delta \mathbf{P}_{\text{vehicle}} + \Delta \mathbf{P}_{\text{ext}} = \boldsymbol{\mathcal{J}} - \boldsymbol{\mathcal{J}} = \mathbf{0}$.
   - **Mass-Ratio Scaling:** For a $0.2\text{ kg}$ cone vs. a $300\text{ kg}$ TeR26 car, $\|\Delta \mathbf{v}_B\| \approx \frac{0.2}{300.2} (1+e) |v_{\text{rel},n}| \approx 0.07\% \text{ of } v_{\text{rel},n}$ (negligible deceleration), whereas hitting a heavy barrier ($m_{\text{ext}} \to \infty$) arrests the car and excites realistic pitch/roll/heave via $J_c(q)^T$.
