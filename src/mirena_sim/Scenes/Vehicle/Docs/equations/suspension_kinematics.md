# 3D Suspension Kinematics and Joint Reaction Loads — TeR26

## 1. Hardpoint Topology per Corner

Each corner $i \in \{FL, FR, RL, RR\}$ consists of 16 3D hardpoints defined in the chassis reference frame $\mathcal{F}_B$:
1. **Lower Wishbone (LCA):** Chassis fore pivot $\mathbf{p}_{LF}^B$, chassis aft pivot $\mathbf{p}_{LA}^B$, upright lower ball joint $\mathbf{p}_{LBJ}^U$ (initial $\mathbf{p}_{LBJ,0}^B$).
2. **Upper Wishbone (UCA):** Chassis fore pivot $\mathbf{p}_{UF}^B$, chassis aft pivot $\mathbf{p}_{UA}^B$, upright upper ball joint $\mathbf{p}_{UBJ}^U$ (initial $\mathbf{p}_{UBJ,0}^B$).
3. **Tie-Rod (Steering Link):** Steering rack inner ball joint $\mathbf{p}_{TI}^B(\delta) = \mathbf{p}_{TI,0}^B + [0, y_{\text{rack}}(\delta), 0]^T$, upright steering arm ball joint $\mathbf{p}_{TO}^U$ (initial $\mathbf{p}_{TO,0}^B$).
4. **Pushrod / Pullrod:** Upright/wishbone attachment $\mathbf{p}_{PO}^U$, rocker attachment $\mathbf{p}_{PR}^B(\theta_r)$.
5. **Rocker (Bellcrank):** Chassis pivot $\mathbf{p}_{RP}^B$, rotation axis unit vector $\hat{\mathbf{u}}_R^B$, pushrod pickup $\mathbf{p}_{PR,0}^B$, coilover pickup $\mathbf{p}_{RC,0}^B$, ARB link pickup $\mathbf{p}_{RA,0}^B$.
6. **Coilover (Spring/Damper):** Chassis fixed pickup $\mathbf{p}_{CI}^B$, rocker pickup $\mathbf{p}_{RC}^B(\theta_r)$.
7. **Anti-Roll Bar (ARB):** Chassis torsion bar bearing $\mathbf{p}_{AI}^B$, droplink attachment $\mathbf{p}_{AO}^B(\theta_r)$.
8. **Wheel Spindle:** Wheel center $\mathbf{p}_{WC}^U$ and spindle unit axis $\hat{\mathbf{s}}^U$.

---

## 2. Nonlinear Constraint Closure Equations $\Phi(x_u, \theta_r, \delta; p) = 0$

Let the upright pose in $\mathcal{F}_B$ be parameterized by $x_u = [\mathbf{p}_{LBJ}^B; \, \boldsymbol{\psi}_u] \in \mathbb{R}^6$, where $\mathbf{p}_{LBJ}^B \in \mathbb{R}^3$ is the position of the lower ball joint and $\boldsymbol{\psi}_u \in \mathfrak{so}(3) \cong \mathbb{R}^3$ is the exponential map rotation vector such that $R_u(\boldsymbol{\psi}_u) = R_{u,0} \exp([\boldsymbol{\psi}_u]_\times)$.

Any point $\mathbf{p}_k^U$ fixed on the upright moves in $\mathcal{F}_B$ as:
$$
\mathbf{p}_k^B(x_u) = \mathbf{p}_{LBJ}^B + R_u(\boldsymbol{\psi}_u) \left( \mathbf{p}_{k,0}^B - \mathbf{p}_{LBJ,0}^B \right)
$$

For a given rocker angle $\theta_r$ (or vertical wheel jounce $z_w$) and steering rack displacement $y_{\text{rack}}(\delta)$, the rocker pushrod pickup is rotated via Rodrigues' formula about $(\mathbf{p}_{RP}^B, \hat{\mathbf{u}}_R^B)$:
$$
\mathbf{p}_{PR}^B(\theta_r) = \mathbf{p}_{RP}^B + \exp\left(\theta_r [\hat{\mathbf{u}}_R^B]_\times\right) \left( \mathbf{p}_{PR,0}^B - \mathbf{p}_{RP}^B \right)
$$

The 6 scalar distance-squared residual equations $\Phi(x_u, \theta_r, \delta; p) \in \mathbb{R}^6$ enforce the rigid link lengths $L_m = \|\mathbf{p}_{m,1,0}^B - \mathbf{p}_{m,2,0}^B\|$:

$$
\Phi(x_u, \theta_r, \delta; p) = \frac{1}{2} \begin{bmatrix}
\|\mathbf{p}_{LBJ}^B - \mathbf{p}_{LF}^B\|^2 - L_{LF}^2 \\
\|\mathbf{p}_{LBJ}^B - \mathbf{p}_{LA}^B\|^2 - L_{LA}^2 \\
\|\mathbf{p}_{UBJ}^B(x_u) - \mathbf{p}_{UF}^B\|^2 - L_{UF}^2 \\
\|\mathbf{p}_{UBJ}^B(x_u) - \mathbf{p}_{UA}^B\|^2 - L_{UA}^2 \\
\|\mathbf{p}_{TO}^B(x_u) - \mathbf{p}_{TI}^B(\delta)\|^2 - L_{TR}^2 \\
\|\mathbf{p}_{PO}^B(x_u) - \mathbf{p}_{PR}^B(\theta_r)\|^2 - L_{PR}^2
\end{bmatrix} = \mathbf{0}
$$

### 2.1 Analytical Jacobian $\Phi_x = \frac{\partial \Phi}{\partial x_u} \in \mathbb{R}^{6 \times 6}$
For any link connecting upright point $\mathbf{p}_k^B(x_u)$ (with lever arm $\mathbf{d}_k^B = R_u (\mathbf{p}_{k,0}^B - \mathbf{p}_{LBJ,0}^B)$) to chassis/rocker point $\mathbf{p}_c^B$, let $\boldsymbol{\Delta}_k = \mathbf{p}_k^B(x_u) - \mathbf{p}_c^B$. The row of $\Phi_x$ is exact:

$$
\frac{\partial \Phi_k}{\partial \mathbf{p}_{LBJ}^B} = \boldsymbol{\Delta}_k^T, \qquad \frac{\partial \Phi_k}{\partial \boldsymbol{\psi}_u} = \boldsymbol{\Delta}_k^T \left( -[\mathbf{d}_k^B]_\times \, J_r(\boldsymbol{\psi}_u) \right)
$$

We solve $\Phi(x_u, \theta_r, \delta; p) = 0$ via damped Newton-Raphson with backtracking line search and warm-starting from the previous timestep, achieving $\|\Phi\|_\infty < 10^{-12}\text{ m}^2$ in 1–2 iterations during real-time stepping.

---

## 3. Implicit Velocity, Acceleration, and Parameter Sensitivities

Differentiating $\Phi(x_u, q_c; p) = 0$ where $q_c = [\theta_r, \delta]^T$:
1. **First-order kinematic sensitivity (Motion Ratio, Steer Gain, Upright Twist Jacobian):**
   $$
   \Phi_x \frac{\partial x_u}{\partial q_c} = -\Phi_{q_c} \implies J_{u,q_c} = -\Phi_x^{-1} \Phi_{q_c} \in \mathbb{R}^{6 \times 2}
   $$
2. **Second-order convective acceleration:**
   $$
   \Phi_x \ddot{x}_u = -\left( \dot{\Phi}_x \dot{x}_u + \dot{\Phi}_{q_c} \dot{q}_c + \Phi_{q_c} \ddot{q}_c \right)
   $$
3. **Hardpoint sensitivity for design optimization:**
   $$
   \frac{\partial x_u}{\partial p} = -\Phi_x^{-1} \frac{\partial \Phi}{\partial p}
   $$

---

## 4. Internal Suspension Member Forces & 6x6 Upright Equilibrium (ANSYS Correlation)

At any state, the upright is in dynamic equilibrium under the external tire/hub/gravity/inertial wrench $\mathcal{W}_{\text{ext}}^B = [\mathbf{F}_{\text{ext}}^B; \, \mathbf{M}_{\text{ext, WC}}^B] \in \mathbb{R}^6$ and the 6 axial link forces $\mathbf{f}_{\text{links}} = [f_{UF}, f_{UA}, f_{LF}, f_{LA}, f_{TR}, f_{PR}]^T \in \mathbb{R}^6$ along unit link vectors $\hat{\mathbf{e}}_m = \frac{\mathbf{p}_{m,\text{in}}^B - \mathbf{p}_{m,\text{out}}^B}{\|\mathbf{p}_{m,\text{in}}^B - \mathbf{p}_{m,\text{out}}^B\|}$:

$$
\underbrace{\begin{bmatrix}
\hat{\mathbf{e}}_{UF} & \hat{\mathbf{e}}_{UA} & \hat{\mathbf{e}}_{LF} & \hat{\mathbf{e}}_{LA} & \hat{\mathbf{e}}_{TR} & \hat{\mathbf{e}}_{PR} \\
\mathbf{r}_{UF} \times \hat{\mathbf{e}}_{UF} & \mathbf{r}_{UA} \times \hat{\mathbf{e}}_{UA} & \mathbf{r}_{LF} \times \hat{\mathbf{e}}_{LF} & \mathbf{r}_{LA} \times \hat{\mathbf{e}}_{LA} & \mathbf{r}_{TR} \times \hat{\mathbf{e}}_{TR} & \mathbf{r}_{PO} \times \hat{\mathbf{e}}_{PR}
\end{bmatrix}}_{A_{\text{stat}}(x_u, \theta_r, \delta) \in \mathbb{R}^{6 \times 6}} \mathbf{f}_{\text{links}} + \mathcal{W}_{\text{ext}}^B = \mathbf{0}
$$

where $\mathbf{r}_m = \mathbf{p}_{m,\text{out}}^B - \mathbf{p}_{WC}^B$. Solving $A_{\text{stat}} \mathbf{f}_{\text{links}} = -\mathcal{W}_{\text{ext}}^B$ yields the exact axial forces in every wishbone leg, tie-rod, and pushrod/pullrod. Propagating $f_{PR}$ onto the rocker yields the damper force $F_{\text{damper}}$, ARB link force $F_{\text{ARB}}$, and 3D rocker pivot bearing reaction $\mathbf{F}_{RP}^B$ for direct export to ANSYS FEA.
