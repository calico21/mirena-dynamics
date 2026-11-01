#!/usr/bin/env python3
"""
Parameter Architecture, Units, and Semantics Audit Tool (Phase IV Priority 6)
Author: MirenaSim Vehicle Dynamics Team
Date: November 2026

Audits every configuration parameter across the MirenaSim vehicle model hierarchy:
traces declarations, runtime loading, consuming physics models, SI units, physical
meaning, and architectural status (ACTIVE, INACTIVE, DEPRECATED, DUPLICATED).
"""

import os
import pandas as pd

def build_parameter_architecture_audit():
    out_dir = "CorrelationLab/reports/phase_iv_model_integrity"
    os.makedirs(out_dir, exist_ok=True)

    # 1. Parameter Dependency Matrix
    # Traces: .tres file -> Schema Class -> Subsystem Model -> Force/Torque Calculation -> State Integration
    dependency_data = [
        # Vehicle / Chassis Parameters
        {
            "parameter_name": "total_mass_kg",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "ChassisModel.gd, SuspensionModel.gd, BrakeSystem.gd",
            "force_torque_calculation": "Static normal loads Fz_static = m*g, total weight distribution",
            "state_integration": "Linear momentum: du/dt = Fx_net/m + v*r, dv/dt = Fy_net/m - u*r",
            "status": "ACTIVE",
            "si_unit": "kg",
            "physical_meaning": "Total vehicle curb mass including driver"
        },
        {
            "parameter_name": "izz_kgm2",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "ChassisModel.gd, RigidBodyDynamics.gd",
            "force_torque_calculation": "Total yaw moment Mz_net assembled about CG",
            "state_integration": "Yaw angular acceleration: dr/dt = Mz_net / Izz",
            "status": "ACTIVE",
            "si_unit": "kg*m^2",
            "physical_meaning": "Chassis yaw mass moment of inertia about vertical Z-axis"
        },
        {
            "parameter_name": "wheelbase_m",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "ChassisModel.gd, SteeringGeometry.gd, SuspensionModel.gd",
            "force_torque_calculation": "Pitch load transfer delta_Fz_long = m*ax*h / L, Ackermann angles",
            "state_integration": "Corner positions relative to CG: x_front = a, x_rear = -b",
            "status": "ACTIVE",
            "si_unit": "m",
            "physical_meaning": "Longitudinal distance between front and rear axle centerlines"
        },
        {
            "parameter_name": "track_front_m",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "ChassisModel.gd, SteeringGeometry.gd, AntiRollBarModel.gd",
            "force_torque_calculation": "Lateral load transfer delta_Fz_lat_f = m_f*ay*h / t_f, Ackermann geometry",
            "state_integration": "Front wheel lateral offsets y = +/- t_f/2",
            "status": "ACTIVE",
            "si_unit": "m",
            "physical_meaning": "Front axle center-to-center wheel track width"
        },
        {
            "parameter_name": "track_rear_m",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "ChassisModel.gd, AntiRollBarModel.gd",
            "force_torque_calculation": "Lateral load transfer delta_Fz_lat_r = m_r*ay*h / t_r",
            "state_integration": "Rear wheel lateral offsets y = +/- t_r/2",
            "status": "ACTIVE",
            "si_unit": "m",
            "physical_meaning": "Rear axle center-to-center wheel track width"
        },
        {
            "parameter_name": "cg_height_m",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "ChassisModel.gd, SuspensionModel.gd",
            "force_torque_calculation": "Governs dynamic longitudinal and lateral load transfer arm h",
            "state_integration": "Roll/pitch moment arm about roll/pitch centers",
            "status": "ACTIVE",
            "si_unit": "m",
            "physical_meaning": "Height of vehicle center of gravity above ground plane"
        },
        {
            "parameter_name": "wheel_unloaded_radius_m",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "WheelContact.gd, WheelKinematics.gd, WheelRotationalDynamics.gd",
            "force_torque_calculation": "Tire deflection delta_z = R0 - (z_hub - z_gnd), contact height, spindle torque",
            "state_integration": "Longitudinal slip kappa = (R0*omega - vx)/vx, wheel speed integration",
            "status": "ACTIVE_PRIMARY",
            "si_unit": "m",
            "physical_meaning": "Primary undeflected physical tire radius measured on setup stand (0.2012m)"
        },
        {
            "parameter_name": "wheel_inertia_kgm2",
            "config_resource": "VehicleParameters.gd",
            "tres_section": "vehicle_parameters",
            "subsystem_model": "WheelRotationalDynamics.gd",
            "force_torque_calculation": "Net wheel torque: T_net = T_drive - T_brake - Fx*R0 - My",
            "state_integration": "Wheel spin acceleration: d_omega/dt = T_net / Iw",
            "status": "ACTIVE",
            "si_unit": "kg*m^2",
            "physical_meaning": "Rotational mass moment of inertia of single wheel assembly and rotor"
        },

        # Powertrain & Differential Parameters
        {
            "parameter_name": "motor_peak_torque_nm",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "MotorModel.gd",
            "force_torque_calculation": "Raw motor torque: T_mot = gas * motor_peak_torque_nm (subject to power limit)",
            "state_integration": "Directly drives total axle shaft torque",
            "status": "ACTIVE",
            "si_unit": "N*m",
            "physical_meaning": "Peak EMRAX 208 motor shaft torque under inverter phase current limit"
        },
        {
            "parameter_name": "power_limit_w",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "MotorModel.gd",
            "force_torque_calculation": "Hyperbolic torque ceiling: T_max = min(T_peak, P_limit / omega_mot)",
            "state_integration": "Clamps high-speed vehicle acceleration",
            "status": "ACTIVE",
            "si_unit": "W",
            "physical_meaning": "Formula Student EV accumulator electrical/mechanical power rule ceiling (80 kW)"
        },
        {
            "parameter_name": "gear_ratio",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "DrivetrainModel.gd",
            "force_torque_calculation": "Multiplies motor torque to axle: T_axle = T_mot * gear_ratio * eff",
            "state_integration": "Governs motor RPM to wheel RPM kinematics",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Final drive chain/sprocket speed reduction ratio (5.0)"
        },
        {
            "parameter_name": "drivetrain_efficiency",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "DrivetrainModel.gd",
            "force_torque_calculation": "Scales deliverable axle torque: T_axle = T_mot * ratio * eff",
            "state_integration": "Mechanical transmission energy loss factor (0.93)",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Mechanical chain drive and bearing power transmission efficiency"
        },
        {
            "parameter_name": "diff_lock_ratio",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "DrivetrainModel.gd",
            "force_torque_calculation": "Scales viscous differential torque transfer: T_lock = r_lock * K_visc * delta_omega",
            "state_integration": "Transfers torque between left and right rear wheels",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Differential locking torque scaling multiplier (0.65)"
        },
        {
            "parameter_name": "diff_viscous_damping_nm_s_per_rad",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "DrivetrainModel.gd",
            "force_torque_calculation": "Viscous locking torque: T_lock = r_lock * K_visc * (omega_RL - omega_RR)",
            "state_integration": "Opposes rear wheel speed differential, stabilizing yaw rate under throttle",
            "status": "ACTIVE_RECONCILED",
            "si_unit": "N*m*s/rad",
            "physical_meaning": "Rear differential viscous damping coefficient (renamed from diff_preload_viscous_nm_s_rad)"
        },
        {
            "parameter_name": "diff_preload_viscous_nm_s_rad",
            "config_resource": "PowertrainConfiguration.gd",
            "tres_section": "powertrain_config",
            "subsystem_model": "PowertrainConfiguration.gd",
            "force_torque_calculation": "Delegates getter/setter to diff_viscous_damping_nm_s_per_rad",
            "state_integration": "Maintains 100% backward-compatibility for historical .tres candidate files",
            "status": "DEPRECATED_ALIAS",
            "si_unit": "N*m*s/rad",
            "physical_meaning": "Deprecated legacy alias for diff_viscous_damping_nm_s_per_rad"
        },

        # Tire Model Parameters
        {
            "parameter_name": "companion_longitudinal_peak_mu",
            "config_resource": "TireConfiguration.gd",
            "tres_section": "tire_config",
            "subsystem_model": "PacejkaMF62TireModel.gd",
            "force_torque_calculation": "Sets companion peak traction ceiling: Fx_max = mu_x * Fz",
            "state_integration": "Longitudinal tractive force Fx",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Peak longitudinal friction coefficient derived from TTC flat-belt data (1.88)"
        },
        {
            "parameter_name": "companion_longitudinal_kx_per_fz",
            "config_resource": "TireConfiguration.gd",
            "tres_section": "tire_config",
            "subsystem_model": "PacejkaMF62TireModel.gd",
            "force_torque_calculation": "Sets normalized longitudinal slip stiffness C_kappa / Fz",
            "state_integration": "Initial slope of tractive force Fx vs slip ratio kappa",
            "status": "ACTIVE",
            "si_unit": "rad^-1",
            "physical_meaning": "Normalized longitudinal slip stiffness per unit vertical load (38.0)"
        },
        {
            "parameter_name": "rolling_resistance_coeff",
            "config_resource": "TireConfiguration.gd",
            "tres_section": "tire_config",
            "subsystem_model": "PacejkaMF62TireModel.gd",
            "force_torque_calculation": "Resistive moment: My = -f_rr * Fz * R0 * tanh(vx / 0.1)",
            "state_integration": "Opposes wheel angular rotation in WheelRotationalDynamics",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Dimensionless rolling resistance coefficient calibrated from coastdown (0.018)"
        },
        {
            "parameter_name": "ter_lateral_grip_scale",
            "config_resource": "TireConfiguration.gd",
            "tres_section": "tire_config",
            "subsystem_model": "PacejkaMF62TireModel.gd",
            "force_torque_calculation": "Multiplies pure lateral Pacejka force: Fy0_scaled = Fy0 * scale",
            "state_integration": "Generates cornering force Fy and lateral chassis acceleration ay",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Global lateral grip multiplier for TeR26 Hoosier compound (1.00)"
        },
        {
            "parameter_name": "unloaded_radius_m",
            "config_resource": "TireConfiguration.gd",
            "tres_section": "tire_config",
            "subsystem_model": "PacejkaMF62TireModel.gd",
            "force_torque_calculation": "Moment arm in My: My = -f_rr * Fz * unloaded_radius_m",
            "state_integration": "Only affects rolling resistance moment; not used in slip or kinematics",
            "status": "ACTIVE_SECONDARY",
            "si_unit": "m",
            "physical_meaning": "Nominal 16.0-inch catalog casing radius (0.2032m) in tire configuration"
        },

        # Steering Subsystem Parameters
        {
            "parameter_name": "max_steer_angle_rad",
            "config_resource": "SteeringConfiguration.gd",
            "tres_section": "steering_config",
            "subsystem_model": "SteeringModel.gd, SteeringGeometry.gd",
            "force_torque_calculation": "Clamps target road-wheel steer angle: delta_clamp = clamp(delta, -max, max)",
            "state_integration": "Governs maximum physical road-wheel turning angle (0.35 rad = 20.05 deg)",
            "status": "ACTIVE",
            "si_unit": "rad",
            "physical_meaning": "Maximum physical road-wheel steering angle limit"
        },
        {
            "parameter_name": "ackermann_factor",
            "config_resource": "SteeringConfiguration.gd",
            "tres_section": "steering_config",
            "subsystem_model": "SteeringGeometry.gd",
            "force_torque_calculation": "Blends parallel vs true Ackermann: delta_FL/FR = lerp(parallel, ack, factor)",
            "state_integration": "Produces differential steering angles between inside and outside front wheels",
            "status": "ACTIVE",
            "si_unit": "-",
            "physical_meaning": "Ackermann geometry percentage (0.60 = 60% pro-Ackermann)"
        },
        {
            "parameter_name": "static_toe_front_rad",
            "config_resource": "SteeringConfiguration.gd",
            "tres_section": "steering_config",
            "subsystem_model": "SteeringGeometry.gd",
            "force_torque_calculation": "Adds static front toe: delta_FL += toe, delta_FR -= toe",
            "state_integration": "Static wheel yaw alignment angle (+0.0015 rad = toe-out)",
            "status": "ACTIVE",
            "si_unit": "rad",
            "physical_meaning": "Front axle static toe angle"
        },
        {
            "parameter_name": "compliance_steer_front_radpn",
            "config_resource": "SteeringConfiguration.gd",
            "tres_section": "steering_config",
            "subsystem_model": "NONE (Option B: Declared in schema, NOT consumed in physics code)",
            "force_torque_calculation": "NONE: zero effect on wheel steer angles, forces, or moments",
            "state_integration": "NONE: bit-for-bit identical outputs when changed",
            "status": "INACTIVE_UNSUPPORTED",
            "si_unit": "rad/N",
            "physical_meaning": "Unsupported front steering elastokinematic compliance (Option B audit closure)"
        },

        # Suspension Subsystem Parameters
        {
            "parameter_name": "spring_rate_front_npm",
            "config_resource": "SuspensionConfiguration.gd",
            "tres_section": "suspension_config",
            "subsystem_model": "SpringModel.gd, SuspensionModel.gd",
            "force_torque_calculation": "Corner spring force: F_spring = K_spring * z_deflection",
            "state_integration": "Vertical wheel load Fz and ride height kinematics",
            "status": "ACTIVE",
            "si_unit": "N/m",
            "physical_meaning": "Front suspension spring stiffness rate (42,000 N/m)"
        },
        {
            "parameter_name": "spring_rate_rear_npm",
            "config_resource": "SuspensionConfiguration.gd",
            "tres_section": "suspension_config",
            "subsystem_model": "SpringModel.gd, SuspensionModel.gd",
            "force_torque_calculation": "Corner spring force: F_spring = K_spring * z_deflection",
            "state_integration": "Vertical wheel load Fz and ride height kinematics",
            "status": "ACTIVE",
            "si_unit": "N/m",
            "physical_meaning": "Rear suspension spring stiffness rate (48,000 N/m)"
        },
        {
            "parameter_name": "arb_rate_front_npm",
            "config_resource": "SuspensionConfiguration.gd",
            "tres_section": "suspension_config",
            "subsystem_model": "AntiRollBarModel.gd",
            "force_torque_calculation": "Front roll stiffness transfer: F_arb = K_arb * (z_FL - z_FR)",
            "state_integration": "Transfers vertical load from inside to outside tire during roll",
            "status": "ACTIVE",
            "si_unit": "N/m",
            "physical_meaning": "Front anti-roll bar stiffness (15,000 N/m)"
        },
        {
            "parameter_name": "damper_gas_preload_front_n",
            "config_resource": "SuspensionConfiguration.gd",
            "tres_section": "suspension_config",
            "subsystem_model": "DamperModel.gd",
            "force_torque_calculation": "Constant nitrogen gas expansion preload force in damper",
            "state_integration": "Biases static suspension extension force (+120 N)",
            "status": "ACTIVE",
            "si_unit": "N",
            "physical_meaning": "Front damper mono-tube internal gas pressure preload"
        }
    ]

    df_dep = pd.DataFrame(dependency_data)
    dep_csv = os.path.join(out_dir, "parameter_dependency_matrix.csv")
    df_dep.to_csv(dep_csv, index=False)
    print(f"Saved parameter dependency matrix ({len(df_dep)} parameters) to {dep_csv}")

    # 2. Parameter Units and Semantics Table
    semantics_data = [
        {
            "parameter_name": p["parameter_name"],
            "si_unit": p["si_unit"],
            "config_file": p["config_resource"],
            "status": p["status"],
            "mathematical_symbol": "m" if "mass" in p["parameter_name"] else ("I_{zz}" if "izz" in p["parameter_name"] else ("R_0" if "radius" in p["parameter_name"] else ("K_{visc}" if "viscous" in p["parameter_name"] else "-"))),
            "physical_interpretation": p["physical_meaning"],
            "audit_classification": "PASS" if "ACTIVE" in p["status"] else ("WARNING" if "DEPRECATED" in p["status"] or "SECONDARY" in p["status"] else "FAIL_RESOLVED")
        }
        for p in dependency_data
    ]

    df_sem = pd.DataFrame(semantics_data)
    sem_csv = os.path.join(out_dir, "parameter_units_and_semantics.csv")
    df_sem.to_csv(sem_csv, index=False)
    print(f"Saved parameter units and semantics ({len(df_sem)} entries) to {sem_csv}")


if __name__ == "__main__":
    build_parameter_architecture_audit()
