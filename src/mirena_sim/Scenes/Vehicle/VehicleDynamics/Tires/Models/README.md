# Tire Models Subdirectory (`Tires/Models/`)

All tire model implementations in this directory implement the contract defined in [`TireModelInterface.gd`](TireModelInterface.gd) and are instantiated through [`../TireModelRegistry.gd`](../TireModelRegistry.gd).

## Contract Summary

- **Inputs**: [`TireInputs`](../TireInputs.gd) (`slip_ratio_kappa`, `slip_angle_alpha_rad`, `slip_angle_rate_rad_s`, `normal_load_fz_n`, `camber_gamma_rad`, `vx_contact_mps`, `vy_contact_mps`, `wheel_omega_rad_s`, `effective_radius_m`, `inflation_pressure_kpa`, `tire_temperature_c`, `surface_mu_scale`, `dt_s`).
- **Mutable State**: [`TireState`](../TireState.gd) (`relaxed_slip_ratio`, `relaxed_slip_angle_rad`, `deflection_x_m`, `deflection_y_m`, `tread_temp_c`, `carcass_temp_c`, `is_saturated`).
- **Configuration**: [`TireConfiguration`](../TireConfiguration.gd).
- **Outputs**: [`TireForces`](../TireForces.gd) (`fx_n`, `fy_n`, `fz_n`, `mx_nm`, `my_nm`, `mz_nm`, `valid`, `fault_flags`) in the ISO 8855 Steered Wheel Contact Frame (`+X` forward, `+Y` left, `+Z` up).

See [`TireModelPlaceholder.md`](TireModelPlaceholder.md) for the roadmap of the calibrated Hoosier 16.0x7.5-10 R20 Pacejka MF6.2 and TeR-Tire Grey-Box models.
