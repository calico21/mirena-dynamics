# Electric Powertrain and Planetary Gearbox Dynamics — TeR26

## 1. Dual-Mode Powertrain Architecture

To satisfy Rule 2.1 (represent the actual TeR26 RWD drivetrain faithfully) while also providing the full planetary gearbox formulation requested in §12:
- **Mode A (`TER26_RWD_DUAL_MOTOR` — Default for TeR26):** Dual rear electric motors driving the rear wheels (`RL`, `RR`) through a reduction ratio $i_g = 5.0$, with motor torque/speed/power limits (`100 Nm`/motor, `80 kW` total electrical/mechanical cap, `6000 RPM`), rotor inertia $I_m$, viscous/spool differential coupling, and electronic torque vectoring ($\Delta \tau_{\text{TV}}$). Front wheels (`FL`, `FR`) are unpowered ($T_{\text{drive},FL} = T_{\text{drive},FR} = 0$).
- **Mode B (`PLANETARY_IN_HUB_4WD` — Explicit 3-Element Planetary Gearbox):** Full 4-wheel or 2-wheel planetary epicyclic reduction with explicit sun, planet carrier, ring/housing, and upright reaction torques.

---

## 2. Explicit Planetary Gearbox Equations of Motion

Consider a single-stage planetary reduction in corner $i$ consisting of:
- **Sun gear ($S$):** Connected to the electric motor rotor, spin angle $\phi_{s,i}$, angular velocity $\omega_{s,i}$ relative to the upright housing, inertia $I_s$, pitch radius $R_s$ (teeth $Z_s$).
- **Planet gears ($P$, count $N_p$):** Mounted on the planet carrier, mass $m_p$, spin inertia $I_p$ about own axis, pitch radius $R_p = \frac{R_r - R_s}{2}$ (teeth $Z_p$).
- **Ring gear ($R$):** Fixed to the upright/motor stator housing ($\omega_{r,\text{rel},i} = 0$), pitch radius $R_r$ (teeth $Z_r$).
- **Planet Carrier ($C$):** Connected to the wheel hub, spin angle $\phi_{c,i} = \phi_{w,i}$, angular velocity $\omega_{c,i} = \omega_{w,i}$ relative to the upright, inertia $I_c$.

### 2.1 Fundamental Kinematic Willis Equations
With the ring gear fixed to the upright housing, define the stationary gear ratio $\rho = \frac{Z_r}{Z_s} = \frac{R_r}{R_s} > 1$:
1. **Sun-to-Carrier transmission ratio ($i_p$):**
   $$
   \omega_{s,i} = (1 + \rho) \omega_{w,i} = i_p \omega_{w,i}, \qquad i_p = 1 + \frac{Z_r}{Z_s}
   $$
2. **Planet gear orbital center velocity and self-spin velocity ($\omega_{p,i}$):**
   $$
   v_{\text{center},p,i} = R_c \omega_{w,i} = \frac{R_s + R_r}{2} \omega_{w,i}, \qquad \omega_{p,i} = -\frac{\rho - 1}{2} \omega_{w,i} = -\frac{R_r}{R_r - R_s} \left(1 - \frac{R_s}{R_r}\right) \omega_{w,i}
   $$

### 2.2 Free-Body Diagrams and Internal Mesh Tangential Forces
Let $F_{sp}$ be the total tangential mesh force exerted by the sun gear on the $N_p$ planets at radius $R_s$, and $F_{rp}$ be the total tangential mesh force exerted by the fixed ring gear on the $N_p$ planets at radius $R_r$:
1. **Sun + Rotor FBD:**
   $$
   (I_m + I_s) \dot{\omega}_{s,i} = \tau_{m,i} - \tau_{\text{loss},s} - R_s F_{sp}
   $$
2. **Planet Spin FBD (summed over $N_p$ planets):**
   $$
   N_p I_p \dot{\omega}_{p,i} = R_p (F_{sp} - F_{rp})
   $$
3. **Carrier + Wheel + Planet Translation FBD:**
   $$
   \left( I_w + I_c + N_p m_p R_c^2 \right) \dot{\omega}_{w,i} = R_c (F_{sp} + F_{rp}) + \tau_{\text{tire/brake},i}
   $$

### 2.3 Exact Equivalent Reflected Inertia $I_{\text{eq}}$ at the Wheel Spindle
Eliminating the internal mesh forces $F_{sp}$ and $F_{rp}$ using the kinematic constraints $\dot{\omega}_{s,i} = i_p \dot{\omega}_{w,i}$ and $\dot{\omega}_{p,i} = i_{p,\text{self}} \dot{\omega}_{w,i}$ where $i_{p,\text{self}} = -\frac{R_r}{2 R_p} \left(1 - \frac{1}{i_p}\right) = -\frac{\rho^2 - 1}{2(\rho - 1)} = -\frac{\rho + 1}{2} \frac{R_s}{R_p}$:

$$
I_{\text{eq}} \dot{\omega}_{w,i} = \eta_g \, i_p \, \tau_{m,i} + \tau_{\text{tire/brake},i}
$$

where the exact reflected rotational inertia at the wheel spindle is:

$$
I_{\text{eq}} = I_w + I_c + N_p m_p \left(\frac{R_s + R_r}{2}\right)^2 + N_p I_p \left(\frac{R_r}{R_r - R_s}\right)^2 + (I_m + I_s) \left(1 + \frac{Z_r}{Z_s}\right)^2
$$

### 2.4 Stator & Ring Housing Reaction Torque on the Upright vs. Chassis
Crucially, where the motor stator and ring gear are mounted determines the suspension pitch/anti-squat reaction:
- **In-Hub / Upright-Mounted Motor + Ring Gear:**
  - The motor stator experiences reaction torque $-\tau_{m,i}$ on the upright.
  - The fixed ring gear experiences mesh torque $-R_r F_{rp} = -(\eta_g i_p - 1)\tau_{m,i} + \text{inertial terms}$ on the upright.
  - **Total torque applied to the upright about $\hat{\mathbf{s}}_i$:**
    $$
    \tau_{\text{upright},i}^{\text{drive}} = -\tau_{\text{wheel,drive},i} = -\eta_g i_p \tau_{m,i} + I_{\text{internal}} \dot{\omega}_{w,i}
    $$
    Therefore, for an in-hub motor/gearbox fixed to the upright, the entire amplified wheel drive torque $\eta_g i_p \tau_{m,i}$ reacts directly against the **upright** (generating wishbone/pushrod anti-squat/anti-lift forces), just like an outboard brake caliper!
- **Inboard Chassis-Mounted Motor + Gearbox (TeR26 RWD Half-Shaft Architecture):**
  - The motor stator and gearbox housing are bolted directly to the **chassis** ($\mathcal{F}_B$).
  - The half-shaft with CV joints transmits pure torque $\tau_{\text{drive},i} = \eta_g i_g \tau_{m,i}$ directly to the wheel hub without applying a pitch reaction torque to the upright! The reaction torque $-\tau_{\text{drive},i}$ acts directly on the **chassis** pitch/roll equations, while only the tire longitudinal force $F_{x,i}$ at ground level reacts through the upright ball joints.
  - Our C++20 engine explicitly distinguishes `MountingLocation::CHASSIS_INBOARD_CV` (TeR26 RWD) from `MountingLocation::UPRIGHT_IN_HUB_PLANETARY`, routing the reaction wrench to $\mathcal{F}_B$ or $\mathcal{F}_{U,i}$ accordingly.
