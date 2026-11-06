# Hybrid Physics-Informed Tire Model (HPITM) — TeR26

## 1. Model Structure

The TeR26 Hybrid Physics-Informed Tire Model (HPITM) combines:
1. **Nominal Physics Layer (Pacejka Magic Formula 6.2):** Full empirical-physical structure for $F_x, F_y, F_z, M_x, M_y, M_z$ under pure and combined slip, capturing load sensitivity ($df_z = \frac{F_z - F_{z0}}{F_{z0}}$), camber inclination ($\gamma$), inflation pressure ($dp = \frac{p - p_0}{p_0}$), and effective rolling radius $r_{\text{eff}}(F_z)$.
2. **Bounded Neural Parameter Correction Layer:** Instead of an unconstrained black-box force adder that can violate sign symmetry or friction limits outside the training hull, the neural correction modulates bounded physical scaling factors:
   $$
   \Delta \boldsymbol{\lambda}(\kappa, \alpha, df_z, \gamma, T_{\text{tire}}) = \boldsymbol{\delta}_{\max} \odot \tanh\left( W_2 \, \phi_{\text{SiLU}}(W_1 \mathbf{x}_{\text{norm}} + \mathbf{b}_1) + \mathbf{b}_2 \right)
   $$
   with strict symmetry enforcement: $\Delta \lambda_{\mu,y}(\alpha, \gamma) = \frac{1}{2}\left[\mathcal{N}(\alpha, \gamma) + \mathcal{N}(-\alpha, -\gamma)\right]$ (even in $(\alpha, \gamma)$) and $|\Delta \lambda| \le 0.15$, guaranteeing $F_y(0, 0) = 0$ and bounded friction ellipse compliance.
3. **First-Order Relaxation-Length Transient Dynamics with Low-Speed $C^\infty$ Regularization:**
   $$
   \sigma_x(\kappa, F_z) \dot{\kappa}_{\text{dyn}} + \sqrt{v_{x,C}^2 + v_{\text{min}}^2} \, \kappa_{\text{dyn}} = v_{s,x} = r_{\text{eff}} \omega_w - v_{x,C}
   $$
   $$
   \sigma_y(\alpha, F_z) \dot{\alpha}_{\text{dyn}} + \sqrt{v_{x,C}^2 + v_{\text{min}}^2} \, \tan(\alpha_{\text{dyn}}) = -v_{y,C}
   $$
4. **Bristle / Dahl Elastoplastic Static Friction at Standstill:**
   To hold the vehicle stationary on an inclined slope without artificial velocity clamping (`if v < eps: v = 0`), each contact patch maintains 2D bristle deflection states $\boldsymbol{\xi}_i = [\xi_{x,i}, \xi_{y,i}]^T \in \mathbb{R}^2$:
   $$
   \dot{\boldsymbol{\xi}}_i = \mathbf{v}_{\text{slip},i} - \frac{\|\mathbf{v}_{\text{slip},i}\|}{\xi_{\max}(F_{z,i})} \boldsymbol{\xi}_i, \qquad \xi_{\max}(F_{z,i}) = \frac{\mu_s F_{z,i}}{k_{\text{bristle}}}
   $$
   $$
   \mathbf{F}_{\text{bristle},i} = k_{\text{bristle}} \boldsymbol{\xi}_i + c_{\text{bristle}} \dot{\boldsymbol{\xi}}_i
   $$
   A smooth $C^\infty$ blending function $w_{\text{roll}}(v) = \frac{1}{2}\left(1 + \tanh\left(\frac{\|\mathbf{v}_C\| - v_{\text{blend}}}{\Delta v_{\text{blend}}}\right)\right)$ transitions seamlessly between the elastoplastic bristle static regime at $v \to 0$ and the MF6.2 + neural rolling regime above $v_{\text{blend}} = 0.25\text{ m/s}$.

---

## 2. Magic Formula 6.2 Equations

### 2.1 Vertical Force and Effective Radius
$$
F_z = \max\left(0, \; k_z(\gamma) \delta_z + k_{z,\text{nl}} \delta_z^2 + c_z \, \psi_+(\delta_z) \, \dot{\delta}_z\right), \qquad \psi_+(\delta_z) = \frac{\delta_z}{\delta_z + \delta_{z,\text{char}}}
$$
$$
r_{\text{eff}}(F_z) = R_0 - \delta_{z0} \left[ D_{\text{reff}} \arctan(B_{\text{reff}} \, \bar{F}_z) + F_{\text{reff}} \, \bar{F}_z \right]
$$

### 2.2 Pure Longitudinal Force $F_{x0}(\kappa, F_z, \gamma)$
$$
\kappa_x = \kappa + S_{Hx}, \quad S_{Hx} = (p_{Hx1} + p_{Hx2} df_z) \lambda_{Hx}
$$
$$
D_x = \mu_x F_z, \quad \mu_x = (p_{Dx1} + p_{Dx2} df_z)(1 + p_{px3} dp + p_{px4} dp^2)(1 - p_{Dx3} \gamma^2) \lambda_{\mu x} (1 + \Delta \lambda_{\mu x})
$$
$$
C_x = p_{Cx1} \lambda_{Cx}, \quad K_{xk} = F_z (p_{Kx1} + p_{Kx2} df_z) \exp(p_{Kx3} df_z) \lambda_{Kxk}, \quad B_x = \frac{K_{xk}}{C_x D_x + \epsilon}
$$
$$
E_x = (p_{Ex1} + p_{Ex2} df_z + p_{Ex3} df_z^2)\left(1 - p_{Ex4} \tanh(10 \kappa_x)\right) \lambda_{Ex}
$$
$$
F_{x0} = D_x \sin\left[ C_x \arctan\left( B_x \kappa_x - E_x (B_x \kappa_x - \arctan(B_x \kappa_x)) \right) \right] + S_{Vx}
$$

### 2.3 Pure Lateral Force $F_{y0}(\alpha, F_z, \gamma)$
$$
\alpha_y = \tan\alpha + S_{Hy}, \quad \mu_y = (p_{Dy1} + p_{Dy2} df_z)(1 + p_{py3} dp + p_{py4} dp^2)(1 - p_{Dy3} \gamma^2) \lambda_{\mu y} (1 + \Delta \lambda_{\mu y})
$$
$$
D_y = \mu_y F_z, \quad C_y = p_{Cy1} \lambda_{Cy}, \quad K_{y\alpha} = p_{Ky1} F_{z0} \sin\left[ p_{Ky4} \arctan\left( \frac{F_z / F_{z0}}{(p_{Ky2} + p_{Ky5} \gamma^2)} \right) \right] (1 - p_{Ky3} |\gamma|) \lambda_{Ky\alpha}
$$
$$
B_y = \frac{K_{y\alpha}}{C_y D_y + \epsilon}, \quad E_y = (p_{Ey1} + p_{Ey2} df_z)\left[ 1 + p_{Ey5}\gamma^2 - (p_{Ey3} + p_{Ey4}\gamma)\tanh(10\alpha_y) \right] \lambda_{Ey}
$$
$$
F_{y0} = -D_y \sin\left[ C_y \arctan\left( B_y \alpha_y - E_y (B_y \alpha_y - \arctan(B_y \alpha_y)) \right) \right] + S_{Vy}
$$

### 2.4 Combined-Slip Weighting ($G_{x\alpha}, G_{y\kappa}$) and Moments ($M_x, M_y, M_z$)
$$
G_{x\alpha} = \frac{\cos\left[ C_{x\alpha} \arctan\left( B_{x\alpha} \alpha_s - E_{x\alpha}(B_{x\alpha}\alpha_s - \arctan(B_{x\alpha}\alpha_s)) \right) \right]}{\cos\left[ C_{x\alpha} \arctan\left( B_{x\alpha} S_{Hx\alpha} - E_{x\alpha}(B_{x\alpha} S_{Hx\alpha} - \arctan(B_{x\alpha} S_{Hx\alpha})) \right) \right]}
$$
$$
F_x = G_{x\alpha} F_{x0}, \qquad F_y = G_{y\kappa} F_{y0} + S_{Vy\kappa}
$$
- **Overturning Moment $M_x$:**
  $$
  M_x = R_0 F_z \left[ q_{sx1} - q_{sx2} \gamma + q_{sx3} \frac{F_y}{F_{z0}} \right] \lambda_{Mx}
  $$
- **Rolling Resistance Moment $M_y$:**
  $$
  M_y = -R_0 F_z \left[ q_{sy1} + q_{sy2} \frac{|F_x|}{F_{z0}} + q_{sy3} \left|\frac{v_x}{v_{\text{ref}}}\right| + q_{sy4} \left(\frac{v_x}{v_{\text{ref}}}\right)^4 \right] \tanh\left(\frac{v_x}{v_{\text{char}}}\right) \lambda_{My}
  $$
- **Self-Aligning Moment $M_z$:**
  $$
  M_z = -t(\alpha_t, \kappa) F_y + M_{zr}(\alpha_r, \kappa) + s(\alpha, F_z) F_x
  $$
  where $t(\alpha_t, \kappa) = D_t \cos\left[ C_t \arctan\left( B_t \alpha_{t,\text{eq}} - E_t(B_t \alpha_{t,\text{eq}} - \arctan(B_t \alpha_{t,\text{eq}})) \right) \right]$ is the pneumatic trail.
