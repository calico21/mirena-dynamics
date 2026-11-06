# Structural Compliance Formulation — TeR26

## 1. Quasi-Static Reduced Compliance Matrix

Suspension links, wishbone brackets, upright spindles, and rocker mounts deform elastically under tire and inertial loads. Promoting full 3D finite element DOFs into the real-time state would introduce kHz-frequency eigenmodes requiring $\mu\text{s}$ timesteps. Instead, we implement a **Guyan / Craig-Bampton static condensation interface** at each wheel center $i \in \{FL, FR, RL, RR\}$.

Let $\mathcal{W}_{\text{hub},i}^U = [\mathbf{F}_{\text{hub},i}^U; \, \mathbf{M}_{\text{hub},i}^U] \in \mathbb{R}^6$ be the total wrench acting on wheel hub $i$ in the upright frame $\mathcal{F}_{U,i}$. The elastic deflection $\delta \mathbf{x}_{c,i} = [\delta \mathbf{p}_{c,i}; \, \delta \boldsymbol{\theta}_{c,i}] \in \mathbb{R}^6$ satisfies either:
1. **Quasi-static compliance:**
   $$
   \delta \mathbf{x}_{c,i} = \mathbf{C}_{6\times 6,i} \, \mathcal{W}_{\text{hub},i}^U
   $$
2. **First-order filtered compliance (preventing algebraic loop instability):**
   $$
   \tau_c \delta \dot{\mathbf{x}}_{c,i} + \delta \mathbf{x}_{c,i} = \mathbf{C}_{6\times 6,i} \, \mathcal{W}_{\text{hub},i}^U
   $$

where $\mathbf{C}_{6\times 6,i} = \mathbf{C}_{6\times 6,i}^T \succeq 0$ is a symmetric positive-semidefinite $6\times 6$ compliance matrix verified by Cholesky / eigenvalue decomposition ($\lambda_{\min}(\mathbf{C}_{6\times 6,i}) \ge 0$).

## 2. Elastic Strain Energy and Compliance Camber/Toe Contributions

The stored elastic strain energy is strictly non-negative:
$$
V_{\text{comp}} = \frac{1}{2} \sum_{i=1}^4 \left(\mathcal{W}_{\text{hub},i}^U\right)^T \mathbf{C}_{6\times 6,i} \, \mathcal{W}_{\text{hub},i}^U \ge 0
$$

The effective wheel orientation seen by the tire-ground contact model includes the kinematic orientation plus the compliance deflection:
$$
\gamma_{\text{eff},i} = \gamma_{\text{kin},i}(\theta_{r,i}, \delta) + \delta \theta_{c,x,i}, \qquad \delta_{\text{toe,eff},i} = \delta_{\text{toe,kin},i}(\theta_{r,i}, \delta) + \delta \theta_{c,z,i}
$$

Key diagonal and cross-coupling terms parameterized in `Data/TeR26/suspension/front.json` and `rear.json`:
- $C_{\gamma, F_y}$ ($\text{rad/N}$): Camber compliance under lateral force $F_y$.
- $C_{\gamma, M_x}$ ($\text{rad/(N}\cdot\text{m)}$): Camber compliance under overturning moment $M_x$.
- $C_{\delta, F_y}$ ($\text{rad/N}$): Lateral-force steer (understeer/oversteer compliance) under $F_y$.
- $C_{\delta, M_z}$ ($\text{rad/(N}\cdot\text{m)}$): Aligning-torque steer compliance under $M_z$.
- $C_{\delta, F_x}$ ($\text{rad/N}$): Brake/drive toe compliance under $F_x$.
