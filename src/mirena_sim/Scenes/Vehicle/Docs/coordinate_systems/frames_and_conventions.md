# Coordinate Systems and Frame Transformations — TeR26

## 1. Primary Reference Frames

| Symbol | Name | Convention | Origin | Axes Definition |
| :--- | :--- | :--- | :--- | :--- |
| $\mathcal{F}_W$ | Inertial World Frame (ISO 8855) | Right-handed, SI | Track spawn origin | $+X_W$ East/Forward, $+Y_W$ North/Left, $+Z_W$ Up (opposite gravity) |
| $\mathcal{F}_G$ | Godot 3D World Frame | Right-handed, Y-Up | Scene origin | $-Z_G$ Forward, $-X_G$ Left ($+X_G$ Right), $+Y_G$ Up |
| $\mathcal{F}_B$ | Chassis Reference Frame | ISO 8855 | Sprung CoG (or Mid-Front-Axle CAD datum) | $+x_B$ Forward, $+y_B$ Left, $+z_B$ Up |
| $\mathcal{F}_{S,i}$ | Suspension Corner Frame ($i \in \{FL, FR, RL, RR\}$) | ISO 8855 | Axle centerline datum | $+x_S$ Forward, $+y_S$ Left (outboard on left, inboard on right), $+z_S$ Up |
| $\mathcal{F}_{U,i}$ | Upright Frame | Right-handed | Wheel center $W_i$ | Moves with 6-DOF upright pose $(p_{U,i}, R_{U,i})$ |
| $\mathcal{F}_{C,i}$ | Tire Contact Patch Frame (ISO 8855 / Adapted SAE) | Right-handed | Ground projection $p_{C,i}$ | $\hat{e}_z = \hat{n}_{\text{ground}}$, $\hat{e}_x = \frac{(\hat{u}_{\text{spin}} \times \hat{n}_{\text{ground}})}{\|\hat{u}_{\text{spin}} \times \hat{n}_{\text{ground}}\|}$, $\hat{e}_y = \hat{e}_z \times \hat{e}_x$ |

---

## 2. Exact Isomorphism Between ISO 8855 ($\mathcal{F}_W / \mathcal{F}_B$) and Godot 3D ($\mathcal{F}_G$)

Let $P_{G \leftarrow \text{ISO}}$ be the orthogonal permutation matrix ($\det(P_{G \leftarrow \text{ISO}}) = +1$, $P_{G \leftarrow \text{ISO}}^{-1} = P_{G \leftarrow \text{ISO}}^T$):

$$
P_{G \leftarrow \text{ISO}} = \begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & 1 \\ -1 & 0 & 0 \end{bmatrix}, \qquad P_{\text{ISO} \leftarrow G} = P_{G \leftarrow \text{ISO}}^T = \begin{bmatrix} 0 & 0 & -1 \\ -1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}
$$

### 2.1 Vectors (Positions, Linear Velocities, Forces)
$$
\mathbf{v}_G = \begin{bmatrix} -v_{y,\text{ISO}} \\ v_{z,\text{ISO}} \\ -v_{x,\text{ISO}} \end{bmatrix}, \qquad \mathbf{v}_{\text{ISO}} = \begin{bmatrix} -v_{z,G} \\ -v_{x,G} \\ v_{y,G} \end{bmatrix}
$$

### 2.2 Pseudovectors (Angular Velocities, Moments, Torques)
Because $\det(P_{G \leftarrow \text{ISO}}) = +1$ (proper rotation in $SO(3)$), axial vectors (angular velocity $\boldsymbol{\omega}$, torque $\boldsymbol{\tau}$) transform identically to polar vectors:

$$
\boldsymbol{\omega}_G = P_{G \leftarrow \text{ISO}} \boldsymbol{\omega}_{\text{ISO}}, \qquad \boldsymbol{\tau}_G = P_{G \leftarrow \text{ISO}} \boldsymbol{\tau}_{\text{ISO}}
$$

### 2.3 Rotation Matrices and Inertia Tensors
$$
R_G = P_{G \leftarrow \text{ISO}} \, R_{\text{ISO}} \, P_{\text{ISO} \leftarrow G}, \qquad \mathbf{I}_G = P_{G \leftarrow \text{ISO}} \, \mathbf{I}_{\text{ISO}} \, P_{\text{ISO} \leftarrow G}
$$

---

## 3. Sign Conventions for Angles and Tire Slip

1. **Steering Angle ($\delta$):** Positive counter-clockwise viewed from above ($+z_B$), i.e., **Left Turn = $+$**.
2. **Camber Angle ($\gamma_i$):** Inclination of the wheel plane relative to the ground normal. **Negative camber** ($\gamma_i < 0$) means the top of the tire tilts **inboard** toward the vehicle centerline on both left and right corners:
   $$
   \gamma_i = \text{sign}_{\text{side},i} \cdot \arcsin(\hat{u}_{\text{spin},i} \cdot \hat{n}_{\text{ground},i}), \quad \text{sign}_{\text{side},FL/RL} = +1, \; \text{sign}_{\text{side},FR/RR} = -1
   $$
3. **Toe Angle ($\delta_{\text{toe},i}$):** **Toe-in is positive ($+$)**, **toe-out is negative ($-$)**.
4. **Slip Angle ($\alpha_i$) & Longitudinal Slip ($\kappa_i$):**
   $$
   \alpha_i = \arctan2\left(v_{y,C,i}, \, |v_{x,C,i}| + \epsilon_v\right), \qquad \kappa_i = \frac{r_{\text{eff},i} \omega_{w,i} - v_{x,C,i}}{\max(|v_{x,C,i}|, \, v_{\text{char}})}
   $$
   where $\kappa_i > 0$ under driving torque ($F_{x,i} > 0$) and $\alpha_i < 0$ produces positive lateral force ($F_{y,i} > 0$) in ISO 8855.
