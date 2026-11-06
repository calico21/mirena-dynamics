# Numerical Integration and Sensitivity Propagation — TeR26

## 1. State-Space Structure

The full state vector $x \in \mathbb{R}^{38}$ is partitioned into:
- **Mechanical configuration $q$ (14 DOFs):** Chassis position $\mathbf{p}_B \in \mathbb{R}^3$, chassis rotation $R_B \in SO(3)$ (represented as unit quaternion $\mathbf{q}_B \in S^3$ or rotation matrix $R_B$ updated via the $SO(3)$ exponential map), rocker angles $\boldsymbol{\theta}_r \in \mathbb{R}^4$, wheel spin angles $\boldsymbol{\phi}_w \in \mathbb{R}^4$.
- **Mechanical generalized velocities $v \in \mathbb{R}^{14}$:** $[\mathbf{v}_B^B; \, \boldsymbol{\omega}_B^B; \, \dot{\boldsymbol{\theta}}_r; \, \boldsymbol{\omega}_w]$.
- **Internal dynamical states $z \in \mathbb{R}^{10+}$:**
  - Tire transient slip states: $\kappa_{\text{dyn},1..4}, \alpha_{\text{dyn},1..4} \in \mathbb{R}^8$.
  - Tire bristle static deflection states: $\boldsymbol{\xi}_{1..4} \in \mathbb{R}^8$.
  - Tire surface/core thermal states: $T_{\text{surf},1..4} \in \mathbb{R}^4$.
  - Steering actuator state: $\delta_{\text{act}}, \dot{\delta}_{\text{act}} \in \mathbb{R}^2$.

---

## 2. Lie-Group Structure-Preserving Integration

Rather than integrating Euler angles (which suffer from gimbal lock) or unnormalized quaternions, chassis orientation $R_B \in SO(3)$ is updated on the Lie group $SO(3)$ using the exponential map:

$$
R_{B,k+1} = R_{B,k} \exp\left( [\Delta \boldsymbol{\psi}_B]_\times \right), \qquad \exp([\boldsymbol{\psi}]_\times) = I + \frac{\sin\|\boldsymbol{\psi}\|}{\|\boldsymbol{\psi}\|} [\boldsymbol{\psi}]_\times + \frac{1 - \cos\|\boldsymbol{\psi}\|}{\|\boldsymbol{\psi}\|^2} [\boldsymbol{\psi}]_\times^2
$$

This guarantees $R_{B,k}^T R_{B,k} = I$ and $\det(R_{B,k}) = +1$ to machine precision across millions of timesteps.

### 2.1 Available Integration Schemes in `ter26::VehicleDynamicsEngine`
1. **Lie-Group Semi-Implicit Euler with Exponential Relaxation Substepping (`IntegratorType::SEMI_IMPLICIT_SUBSTEP`):**
   - Evaluates acceleration $\dot{v}_k = M(q_k)^{-1} (Q(q_k, v_k, z_k, u_k) - C(q_k, v_k))$.
   - Updates velocity $v_{k+1} = v_k + \Delta t \, \dot{v}_k$ and configuration $q_{k+1} = q_k \oplus (\Delta t \, v_{k+1})$.
   - Updates stiff tire relaxation and bristle ODEs using exact exponential Euler integration over each substep $\Delta \tau = \Delta t / N_{\text{sub}}$:
     $$
     \alpha_{\text{dyn},k+1} = e^{-\lambda_\alpha \Delta \tau} \alpha_{\text{dyn},k} + \left(1 - e^{-\lambda_\alpha \Delta \tau}\right) \alpha_{\text{ss},k}, \qquad \lambda_\alpha = \frac{\sqrt{v_{x,C}^2 + v_{\min}^2}}{\sigma_y}
     $$
     which is **unconditionally stable** for any $\Delta \tau > 0$ even as $\sigma_y \to 0$ or $v_x \to \infty$.
2. **4th-Order Lie-Group Runge-Kutta (`IntegratorType::RK4_LIE`):**
   - 4-stage Crouch-Grossman / Munthe-Kaas RK4 on $SE(3) \times \mathbb{R}^8 \times \mathbb{R}^{14}$ with exact exponential updates for internal stiff states.
   - At $\Delta t = 2.5\text{ ms}$ (400 Hz) with $N_{\text{sub}} = 2$, executes in $< 0.15\text{ ms}$ in C++20 (over $16\times$ faster than the $2.5\text{ ms}$ real-time budget).

---

## 3. Differentiability and Sensitivity Propagation

To support gradient-based parameter identification and setup optimization:
1. **Smooth Surrogates for Non-Smooth Physics:**
   - Unilateral tire liftoff: $F_z = k_z \, \text{softplus}_\beta(\delta_z)$ in differentiable mode ($\beta = 500\text{ m}^{-1}$) and $\max(0, \delta_z)$ in hard real-time mode.
   - Sign functions in MF6.2 ($E_x, E_y$) and rolling resistance ($M_y$): replaced by $\tanh(10 s)$.
   - Damper knee transition: $C^1$ smooth hyperbolic-tangent knee transition between $c_{\text{low}}$ and $c_{\text{high}}$.
2. **State and Parameter Jacobians:**
   $$
   A_k = \frac{\partial x_{k+1}}{\partial x_k}, \qquad B_k = \frac{\partial x_{k+1}}{\partial u_k}, \qquad S_k = \frac{\partial x_{k+1}}{\partial \theta}
   $$
   Propagated along trajectories via forward sensitivity recursion:
   $$
   \frac{d x_{k+1}}{d \theta} = A_k \frac{d x_k}{d \theta} + S_k
   $$
