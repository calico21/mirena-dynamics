# Multibody Vehicle Dynamics Equations — TeR26 (14-DOF Coupled Formulation)

## 1. Generalized Coordinates and Velocities

The mechanical configuration lives on the configuration manifold $\mathcal{Q} = SE(3) \times \mathbb{R}^4 \times \mathbb{R}^4$:

$$
q = \left( \mathbf{p}_B \in \mathbb{R}^3, \; R_B \in SO(3), \; \boldsymbol{\theta}_r \in \mathbb{R}^4, \; \boldsymbol{\phi}_w \in \mathbb{R}^4 \right)
$$

where:
- $\mathbf{p}_B \in \mathbb{R}^3$: Chassis reference point position in the inertial frame $\mathcal{F}_W$.
- $R_B \in SO(3)$: Rotation matrix mapping body-fixed vectors in $\mathcal{F}_B$ to $\mathcal{F}_W$.
- $\boldsymbol{\theta}_r = [\theta_{r,FL}, \theta_{r,FR}, \theta_{r,RL}, \theta_{r,RR}]^T \in \mathbb{R}^4$: Suspension rocker angles (or generalized vertical jounce coordinates $z_{s,i}$ bijectively mapped via rocker closure $\theta_{r,i}(z_{s,i})$).
- $\boldsymbol{\phi}_w = [\phi_{w,FL}, \phi_{w,FR}, \phi_{w,RL}, \phi_{w,RR}]^T \in \mathbb{R}^4$: Wheel spin angles about their respective spindle axes.

The generalized velocity vector $v \in \mathbb{R}^{14}$ in the tangent space $T_q \mathcal{Q} \cong \mathbb{R}^{14}$ is:

$$
v = \begin{bmatrix} \mathbf{v}_B^B \\ \boldsymbol{\omega}_B^B \\ \dot{\boldsymbol{\theta}}_r \\ \boldsymbol{\omega}_w \end{bmatrix} \in \mathbb{R}^{14}
$$

where $\mathbf{v}_B^B = R_B^T \dot{\mathbf{p}}_B \in \mathbb{R}^3$ and $\boldsymbol{\omega}_B^B = (R_B^T \dot{R}_B)^\vee \in \mathbb{R}^3$ are the body-frame linear and angular velocities of the chassis.

---

## 2. Kinematic Reconstruction of All Rigid Bodies

For each rigid body $k \in \{\text{chassis}, \, \text{upright}_{1..4}, \, \text{wheel}_{1..4}, \, \text{rocker}_{1..4}, \, \text{wishbones}_{1..4}\}$, its spatial twist $\mathbf{V}_k = [\mathbf{v}_k^T, \boldsymbol{\omega}_k^T]^T \in \mathbb{R}^6$ (expressed in the body-fixed frame $\mathcal{F}_k$) is linear in the generalized velocities $v$:

$$
\mathbf{V}_k(q, v, \delta) = J_k(q, \delta) v + \mathbf{V}_{k,\delta}(q, \delta) \dot{\delta}
$$

where $J_k(q, \delta) \in \mathbb{R}^{6 \times 14}$ is the configuration-dependent body Jacobian derived from the 3D suspension constraint solver:

$$
\Phi(x_{u,i}, \theta_{r,i}, \delta) = 0 \implies \frac{\partial x_{u,i}}{\partial \theta_{r,i}} = -\Phi_{x,i}^{-1} \Phi_{\theta_r,i}, \quad \frac{\partial x_{u,i}}{\partial \delta} = -\Phi_{x,i}^{-1} \Phi_{\delta,i}
$$

Specifically, for corner $i$:
- The upright position $\mathbf{r}_{U,i}^B(\theta_{r,i}, \delta)$ and orientation $R_{U,i}^B(\theta_{r,i}, \delta)$ relative to the chassis have partial translational and rotational Jacobians $\mathbf{j}_{p,i}(\theta_{r,i}, \delta) = \frac{\partial \mathbf{r}_{U,i}^B}{\partial \theta_{r,i}} \in \mathbb{R}^3$ and $\mathbf{j}_{\omega,i}(\theta_{r,i}, \delta) = \left( R_{U,i}^{B,T} \frac{\partial R_{U,i}^B}{\partial \theta_{r,i}} \right)^\vee \in \mathbb{R}^3$.
- The upright velocity in $\mathcal{F}_B$ is:
  $$
  \mathbf{v}_{U,i}^B = \mathbf{v}_B^B + \boldsymbol{\omega}_B^B \times \mathbf{r}_{U,i}^B + \mathbf{j}_{p,i} \dot{\theta}_{r,i} + \mathbf{j}_{p,\delta,i} \dot{\delta}
  $$
  $$
  \boldsymbol{\omega}_{U,i}^B = \boldsymbol{\omega}_B^B + \mathbf{j}_{\omega,i} \dot{\theta}_{r,i} + \mathbf{j}_{\omega,\delta,i} \dot{\delta}
  $$
- The wheel angular velocity adds the spin rate $\omega_{w,i}$ about the unit spindle axis $\hat{\mathbf{s}}_i(\theta_{r,i}, \delta)$:
  $$
  \boldsymbol{\omega}_{W,i}^B = \boldsymbol{\omega}_{U,i}^B + \hat{\mathbf{s}}_i \omega_{w,i}
  $$

---

## 3. Coupled Equations of Motion (Kane / Euler-Lagrange on Lie Groups)

Let $\mathcal{M}_k = \text{diag}(m_k I_{3\times 3}, \, \mathbf{I}_k) \in \mathbb{R}^{6 \times 6}$ be the spatial inertia matrix of body $k$ about its center of mass. Summing the virtual work of inertial and applied wrenches across all bodies yields the coupled 14-DOF equations of motion:

$$
M(q, \delta) \dot{v} + C(q, v, \delta, \dot{\delta}) = Q_{\text{grav}}(q) + Q_{\text{susp}}(q, v) + Q_{\text{tire}}(q, v, z) + Q_{\text{pt}}(q, v, u) + Q_{\text{brake}}(v, u) + Q_{\text{aero}}(q, v) + Q_{\text{ext}}
$$

where:
1. **Symmetric Positive-Definite Generalized Mass Matrix ($M(q, \delta) \in \mathbb{R}^{14 \times 14}$):**
   $$
   M(q, \delta) = \sum_{k} J_k(q, \delta)^T \, \mathcal{M}_k \, J_k(q, \delta)
   $$
   Notice that $M(q, \delta)$ explicitly captures:
   - Inertia coupling between chassis pitch/roll/heave and suspension jounce ($\mathbf{r}_{U,i}^B \times m_{u,i} \mathbf{j}_{p,i}$).
   - Gyroscopic and reaction-torque coupling between wheel spin $\omega_{w,i}$, upright camber/toe rates $\mathbf{j}_{\omega,i}$, and chassis angular acceleration.
   - Reflected drivetrain and planetary carrier/sun inertias.

2. **Convective, Coriolis, Centrifugal, and Gyroscopic Vector ($C(q, v, \delta, \dot{\delta}) \in \mathbb{R}^{14}$):**
   $$
   C(q, v, \delta, \dot{\delta}) = \sum_{k} J_k(q, \delta)^T \left( \mathcal{M}_k \dot{\mathbf{V}}_{k,\text{conv}}(q, v, \delta, \dot{\delta}) + \begin{bmatrix} \boldsymbol{\omega}_k \times (m_k \mathbf{v}_k) \\ \boldsymbol{\omega}_k \times (\mathbf{I}_k \boldsymbol{\omega}_k) \end{bmatrix} \right)
   $$
   where $\dot{\mathbf{V}}_{k,\text{conv}} = \dot{J}_k v$ contains the convective accelerations $\boldsymbol{\omega}_B^B \times (\boldsymbol{\omega}_B^B \times \mathbf{r}_{U,i}^B) + 2 \boldsymbol{\omega}_B^B \times (\mathbf{j}_{p,i} \dot{\theta}_{r,i}) + \frac{\partial \mathbf{j}_{p,i}}{\partial \theta_{r,i}} \dot{\theta}_{r,i}^2$ and the wheel gyroscopic precession torque $\boldsymbol{\omega}_{U,i} \times (\mathbf{I}_{w,i} \hat{\mathbf{s}}_i \omega_{w,i})$.

3. **Generalized Applied Forces ($Q \in \mathbb{R}^{14}$):**
   Every external or internal wrench $\mathcal{W}_k = [\mathbf{F}_k^T, \mathbf{M}_k^T]^T$ acting on body $k$ projects onto the generalized coordinates via virtual work:
   $$
   Q_k = J_k(q, \delta)^T \mathcal{W}_k
   $$
   This automatically generates physical anti-dive, anti-squat, and jacking forces in the suspension without ad-hoc pitch/roll moment hacks!
