# Parameter Provenance & Discrepancy Audit (`PARAMETER_PROVENANCE.md`)

Every physical parameter in `Config/vehicle_default.tres`, `tire_default.tres`, `suspension_default.tres`, and `powertrain_default.tres` has been audited across the MirenaSim repository and the sibling Formula Student repositories in `/home/alexr/eRacing/` (`TeR_Tire`, `FS_Driver_Setup_Optimizer`, `TeR_TWIN`).

## 1. Provenance Classification Legend

- **Verified from repository**: Directly present and active in `mirena_sim` (`vehicleMotion.gd`, `mirena_car.tscn`) or `mirena_state` / `mirena_hardware`.
- **Calculated from geometry**: Derived mathematically from verified scene coordinates in `mirena_car.tscn`.
- **Calibrated from telemetry / sibling repo**: Derived from experimental telemetry (`imu_compare/`, `TeR_Tire` TTC fits, or `FS_Driver_Setup_Optimizer/config/vehicles/ter26.py`).
- **Assumed (requires measurement)**: Plausible Formula Student value configured as an explicit `@export` resource field awaiting physical measurement on the real car.
- **Unavailable (pending)**: External calibration file or hardware measurement required in Phase 2+.

---

## 2. Master Parameter Provenance Table

| Parameter | Default Value | Unit | Config Location | Provenance Status | Primary Source & Cross-Repo Audit |
| :--- | :---: | :---: | :--- | :--- | :--- |
| `total_mass_kg` | `300.0` | $\text{kg}$ | `VehicleParameters` | **Verified from repository** | `vehicleMotion.gd:15` (`MASS = 300.0`), `mirena_car.tscn:22` (`mass = 300.0`), `ter26.py` (`total_mass = 300.0`, 230 kg car + 70 kg driver). *Note: `dbm_model.py` uses dry mass `230.0 kg`.* |
| `lf_front_axle_to_com_m` | `0.876` | $\text{m}$ | `VehicleParameters` | **Calculated from geometry** | `mirena_car.tscn`: CoM $z = -0.117\text{ m}$, Front wheels $z = -0.992727\text{ m}$ $\implies l_f = 0.8757\text{ m}$. (`ter26.py`: $0.8525\text{ m}$; `dbm_model.py`: $0.806\text{ m}$). |
| `lr_rear_axle_to_com_m` | `0.717` | $\text{m}$ | `VehicleParameters` | **Calculated from geometry** | `mirena_car.tscn`: CoM $z = -0.117\text{ m}$, Rear wheels $z = +0.599941\text{ m}$ $\implies l_r = 0.7169\text{ m}$, $L = 1.593\text{ m}$. (`ter26.py`: $0.6975\text{ m}$, $L = 1.550\text{ m}$; `dbm_model.py`: $0.744\text{ m}$). |
| `com_height_m` | `0.33` | $\text{m}$ | `VehicleParameters` | **Verified from repository** | `mirena_car.tscn:23` (`center_of_mass = Vector3(0, 0.33, -0.117)`). (`ter26.py`: `0.285 m`). |
| `track_width_front_m`, `rear_m` | `1.174` | $\text{m}$ | `VehicleParameters` | **Calculated from geometry** | `mirena_car.tscn`: wheel $|x| = 0.586657\text{ m} \implies t = 1.1733\text{ m}$. (`mirena_state.cpp:25`: `1.2 m`; `ter26.py`: `1.20 m` front / `1.18 m` rear). |
| `wheel_radius_front_m`, `rear_m` | `0.20` | $\text{m}$ | `VehicleParameters` | **Verified from repository** | `vehicleMotion.gd:16` (`WHEEL_RADIUS = 0.20`), `mirena_car.tscn` (`wheel_radius = 0.2`), `mirena_state.cpp:24` (`wheel_radius = 0.20`). *Note: Hoosier 16.0x7.5-10 unloaded radius is `0.2032 m` (`tire_default.tres`).* |
| `izz_yaw_kg_m2` | `120.0` | $\text{kg}\cdot\text{m}^2$ | `VehicleParameters` | **Calibrated / sibling repo** | `FS_Driver_Setup_Optimizer/config/vehicles/ter26.py` (`I_z = 110–120 kg m^2`; `dbm_model.py` uses `90.0 kg m^2` for 230 kg dry mass). |
| `ixx_roll_kg_m2`, `iyy_pitch_kg_m2` | `45.0`, `115.0` | $\text{kg}\cdot\text{m}^2$ | `VehicleParameters` | **Assumed (requires measurement)** | Typical Formula Student tubular-frame roll/pitch inertia; requires CAD inertia export or bifilar pendulum test. |
| `unsprung_mass_*_kg` | `11.5` (F), `12.5` (R) | $\text{kg}$ | `VehicleParameters` | **Calibrated / sibling repo** | Consistent with 10-inch Hoosier + OZ magnesium rim + upright/hub assembly (`ter26.py`). |
| `wheel_rotational_inertia_*` | `0.38` (F), `0.42` (R) | $\text{kg}\cdot\text{m}^2$ | `VehicleParameters` | **Assumed (requires measurement)** | Tire + rim + rotor + half-shaft rotational inertia about spin axis. |
| `power_limit_w` | `80000.0` | $\text{W}$ | `PowertrainConfiguration` | **Verified from repository** | `vehicleMotion.gd:26` (`POWER_LIM = 80000.0`), FS rules 80 kW limit. |
| `motor_peak_torque_nm` | `100.0` | $\text{N}\cdot\text{m}$ | `PowertrainConfiguration` | **Verified from repository** | `vehicleMotion.gd:27` (`MOTOR_PEAK_TRQ = 100.0`). |
| `gear_ratio` | `5.0` | $-$ | `PowertrainConfiguration` | **Verified from repository** | `vehicleMotion.gd:29` (`GEAR_RATIO = 5.0`; `ter26.py` uses `4.5`). |
| `drivetrain_efficiency` | `0.92` | $-$ | `PowertrainConfiguration` | **Verified from repository** | `vehicleMotion.gd:30` (`DRIVETRAIN_EFF = 0.92`). |
| `max_total_braking_force_n` | `1177.2` | $\text{N}$ | `BrakeConfiguration` | **Calibrated from telemetry** | `vehicleMotion.gd:34` (`BRAKE_MU * MASS * G = 0.40 * 300 * 9.81 = 1177.2 N`), calibrated against recorded bag peak braking deceleration (`3.93 m/s^2`). |
| `front_brake_bias` | `0.60` | $-$ | `BrakeConfiguration` | **Verified from repository** | `vehicleMotion.gd:35` (`BRAKE_BIAS_F = 0.60`). |
| `air_density_kg_m3` | `1.225` | $\text{kg/m}^3$ | `AerodynamicConfiguration` | **Verified from repository** | `vehicleMotion.gd:19` (`RHO_AIR = 1.225`). |
| `frontal_area_m2` | `1.10` | $\text{m}^2$ | `AerodynamicConfiguration` | **Verified from repository** | `vehicleMotion.gd:20` (`A_ref = 1.10`), `ter26.py` (`frontal_area = 1.1`). |
| `cd_drag_coefficient` | `2.50` | $-$ | `AerodynamicConfiguration` | **Verified from repository** | `vehicleMotion.gd:21` (`Cd_ref = 2.50`), `ter26.py` (`Cd = 2.5`). |
| `cl_downforce_coefficient` | `4.14` | $-$ | `AerodynamicConfiguration` | **Verified from repository** | `vehicleMotion.gd:22` (`Cl_ref = 4.14`), `ter26.py` (`Cl = 4.14`). |
| `aero_balance_front` | `0.45` | $-$ | `AerodynamicConfiguration` | **Calibrated / sibling repo** | `FS_Driver_Setup_Optimizer/config/vehicles/ter26.py` (`aero_split_f = 0.45`). *(Note: `vehicleMotion.gd` applied downforce at CoM without front/rear pitch moment).* |
| `nominal_load_fz0_n` | `700.0` | $\text{N}$ | `TireConfiguration` | **Calibrated / sibling repo** | `TeR_Tire/config/tire_specs.yaml` (`Fz0 = 700.0 N` for Hoosier 16.0x7.5-10 R20). |
| `nominal_pressure_p0_kpa` | `82.7` | $\text{kPa}$ | `TireConfiguration` | **Calibrated / sibling repo** | `TeR_Tire/config/tire_specs.yaml` (`p0 = 82.7 kPa` / 12 psi). |
| `unloaded_radius_m` | `0.2032` | $\text{m}$ | `TireConfiguration` | **Calibrated / sibling repo** | `TeR_Tire/config/tire_specs.yaml` (`R0 = 0.2032 m`, 16.0 in / 2). |
| `vertical_stiffness_cz_n_per_m` | `110000.0` | $\text{N/m}$ | `TireConfiguration` | **Calibrated / sibling repo** | `TeR_Tire/config/tire_specs.yaml` (`Cz = 110000.0 N/m`). |
| `spring_rate_front/rear_n_per_m` | `45000` / `50000` | $\text{N/m}$ | `SuspensionConfiguration` | **Assumed (requires measurement)** | `mirena_car.tscn` had `suspension_stiffness = 70.0` (Godot internal mass-normalized units $\approx 21\text{ kN/m}$); `45–50 kN/m` reflects realistic FS ground-effect wheel rates. |
| `max_steering_angle_rad` | `0.50` | $\text{rad}$ | `SteeringConfiguration` | **Verified from repository** | `vehicleMotion.gd:55` (`MAX_STEER_RAD = 0.50`). |

---

## 3. Critical Cross-Package Discrepancies to Reconcile in Phase 2/3

1. **Mass & Wheelbase Mismatch between `mirena_sim` and `mirena_control` (`dbm_model.py`)**:
   - `mirena_sim` uses `m = 300.0 kg`, $l_f = 0.876\text{ m}$, $l_r = 0.717\text{ m}$ ($L = 1.593\text{ m}$).
   - `mirena_control/src/mpc/codegen/dbm_model.py` uses `m = 230.0 kg`, $l_f = 0.806\text{ m}$, $l_r = 0.744\text{ m}$ ($L = 1.550\text{ m}$).
   - Both configurations can be represented via separate `.tres` presets (`vehicle_default.tres` vs `vehicle_ter26_mpc.tres`).
2. **Wheel Radius Discrepancy (`0.200 m` vs `0.2032 m`)**:
   - `mirena_state.cpp` converts `/mirena/wheel_speeds` RPM to linear velocity using `wheel_radius = 0.20 m`.
   - As documented in `imu_compare/speed_analysis_front10_rear12.md`, under ~735 N static load and $C_z = 110\text{ kN/m}$, the Hoosier $R_0 = 0.2032\text{ m}$ tire deflects by $\Delta z \approx 6.7\text{ mm}$, yielding a loaded effective rolling radius $r_{\text{eff}} \approx 0.197–0.200\text{ m}$, which naturally bridges the two numbers when vertical tire deflection is enabled.
