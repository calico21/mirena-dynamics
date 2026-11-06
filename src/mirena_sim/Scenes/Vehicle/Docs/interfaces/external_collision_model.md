# External Collision Model — TeR26

See also [godot_and_ros2_interfaces.md](file:///home/alexr/eRacing/mirena-ws-SIM/src/mirena_sim/Scenes/Vehicle/Docs/interfaces/godot_and_ros2_interfaces.md) for the full impulse-momentum transfer formulation:

$$
\Delta v = M(q)^{-1} J_c(q)^T \boldsymbol{\mathcal{J}}, \qquad \boldsymbol{\mathcal{J}} = j_n \hat{\mathbf{n}}_c + \mathbf{j}_t
$$

where $J_c(q) \in \mathbb{R}^{3 \times 14}$ maps generalized velocities $v$ to the world velocity at contact point $\mathbf{p}_c^W$:
- For chassis contact at $\mathbf{r}_c^B = R_B^T (\mathbf{p}_c^W - \mathbf{p}_B^W)$:
  $$
  J_{c,\text{chassis}}(q) = \begin{bmatrix} R_B & -R_B [\mathbf{r}_c^B]_\times & \mathbf{0}_{3\times 4} & \mathbf{0}_{3\times 4} \end{bmatrix}
  $$
- For wheel/upright contact on corner $i$: $J_c(q)$ includes the suspension column $\frac{\partial \mathbf{p}_c^W}{\partial \theta_{r,i}}$ so kerb strikes or wheel impacts directly excite suspension jounce $\dot{\theta}_{r,i}$ as well as chassis heave/roll/pitch.
