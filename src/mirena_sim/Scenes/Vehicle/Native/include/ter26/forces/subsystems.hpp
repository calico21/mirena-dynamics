#pragma once
/**
 * @file subsystems.hpp
 * @brief Suspension Force Elements (Coilover Spring, Horstman Digressive Damper, Bump Stop,
 *        Coupled U-Bar ARB, 6x6 Structural Compliance), Electric Powertrain + Planetary Gearbox,
 *        Hydraulic/EBS Braking, and Ride-Height Sensitive Aerodynamics for TeR26.
 */

#include "ter26/state/vehicle_state.hpp"
#include "ter26/kinematics/suspension_kinematics.hpp"

namespace ter26 {

/// Suspension force and compliance parameters for one axle (Front or Rear)
struct AxleSuspensionParams {
    double k_spring_N_m{35030.0};
    double preload_force_N{420.0};
    double c_low_bump_N_s_m{1800.0};
    double c_high_bump_N_s_m{720.0};
    double v_knee_bump_m_s{0.10};
    double rebound_ratio{1.60};
    double gas_force_N{120.0};
    double bump_stop_jounce_m{0.025};
    double bump_stop_k_lin_N_m{50000.0};
    double bump_stop_k_quad_N_m2{1500000.0};
    double bump_stop_c_N_s_m{1200.0};
    double arb_wheel_rate_N_m{4500.0};

    // 6x6 Compliance coefficients at wheel center
    double camber_per_Fy_rad_N{2.094e-5};
    double camber_per_Mx_rad_Nm{8.50e-5};
    double toe_per_Fy_rad_N{-3.49e-6};
    double toe_per_Mz_rad_Nm{4.20e-5};
    double toe_per_Fx_rad_N{1.75e-6};
    double compliance_tau_s{0.005};
};

/// Powertrain & Planetary Gearbox Parameters
struct PowertrainParams {
    enum class Mode {
        TER26_RWD_DUAL_MOTOR = 0,       ///< TeR26 actual configuration: 2 rear motors, 5:1 reduction
        PLANETARY_IN_HUB_4WD = 1        ///< 4-wheel in-hub epicyclic planetary gearbox configuration
    };

    enum class MountingLocation {
        CHASSIS_INBOARD_CV = 0,         ///< Stator & gearbox mounted to chassis; CV half-shafts to hub (TeR26 RWD)
        UPRIGHT_IN_HUB_PLANETARY = 1    ///< Stator & fixed ring gear mounted to upright housing
    };

    Mode mode{Mode::TER26_RWD_DUAL_MOTOR};
    MountingLocation mounting{MountingLocation::CHASSIS_INBOARD_CV};

    // Motor & Inverter
    double peak_torque_per_motor_Nm{100.0};
    double gear_ratio{5.0};
    double transmission_efficiency{0.92};
    double max_motor_rpm{6000.0};
    double field_weakening_rpm{4800.0};
    double fs_power_limit_W{80000.0};
    double rotor_inertia_kg_m2{0.0032};
    double torque_time_constant_s{0.005};

    // Rear Differential & Torque Vectoring
    double diff_viscous_coeff_Nm_s_rad{18.0};
    double diff_preload_Nm{15.0};
    double tv_yaw_gain_Nm_per_rad_s{120.0};
    double tv_max_delta_motor_Nm{35.0};

    // Explicit 3-Element Planetary Gearbox Parameters (Sun S, N_p Planets P, Fixed Ring R, Carrier C)
    int sun_teeth_Zs{18};
    int planet_teeth_Zp{27};
    int ring_teeth_Zr{72};
    int num_planets_Np{3};
    double sun_pitch_radius_m{0.0135};
    double planet_pitch_radius_m{0.02025};
    double ring_pitch_radius_m{0.0540};
    double sun_inertia_kg_m2{0.00018};
    double planet_mass_kg{0.145};
    double planet_spin_inertia_kg_m2{0.000042};
    double carrier_inertia_kg_m2{0.0028};
    double planetary_viscous_loss_Nm_s_rad{0.0015};
    double planetary_coulomb_seal_Nm{0.25};

    /// Willis kinematic transmission ratio i_p = 1 + Z_r / Z_s (when ring is fixed to upright housing)
    double planetary_ratio_ip() const noexcept {
        return 1.0 + static_cast<double>(ring_teeth_Zr) / static_cast<double>(sun_teeth_Zs);
    }

    /// Exact reflected inertia I_eq at the wheel spindle for the planetary assembly (excluding wheel I_w)
    double compute_planetary_reflected_inertia() const noexcept {
        const double ip = planetary_ratio_ip();
        const double Rc = 0.5 * (sun_pitch_radius_m + ring_pitch_radius_m);
        const double planet_spin_ratio = ring_pitch_radius_m / (ring_pitch_radius_m - sun_pitch_radius_m);
        const double I_carrier_planets_trans = carrier_inertia_kg_m2 + num_planets_Np * planet_mass_kg * (Rc * Rc);
        const double I_planets_spin = num_planets_Np * planet_spin_inertia_kg_m2 * (planet_spin_ratio * planet_spin_ratio);
        const double I_sun_rotor = (rotor_inertia_kg_m2 + sun_inertia_kg_m2) * (ip * ip);
        return I_carrier_planets_trans + I_planets_spin + I_sun_rotor;
    }

    /// Reflected drivetrain inertia at corner i
    double reflected_inertia_at_wheel(int corner_idx) const noexcept {
        if (mode == Mode::PLANETARY_IN_HUB_4WD) {
            return compute_planetary_reflected_inertia();
        }
        // TeR26 RWD: only rear wheels (RL=2, RR=3) have connected motors
        if (corner_idx == RL || corner_idx == RR) {
            return rotor_inertia_kg_m2 * (gear_ratio * gear_ratio);
        }
        return 0.0;
    }
};

/// Hydraulic & EBS Brake System Parameters
struct BrakeParams {
    double max_total_brake_torque_Nm{1600.0};
    double front_brake_bias{0.60};
    double ebs_total_brake_torque_Nm{2000.0};
    double hydraulic_tau_s{0.015};
};

/// Ride-Height, Pitch, Roll, and Yaw Sensitive Aerodynamics Parameters
struct AeroParams {
    double air_density_kg_m3{1.225};
    double reference_area_m2{1.10};
    double cl_nominal{3.18};
    double cd_nominal{1.27};
    double aero_balance_front{0.46};
    double cop_height_m{0.25};
    double h_ref_front_m{0.035};
    double h_ref_rear_m{0.040};
    double dcl_dh_front_per_m{-8.5};
    double dcl_dh_rear_per_m{-10.0};
    double dcop_dtheta_m_per_rad{-1.8};
    double cl_roll_per_rad{-0.35};
    double cy_yaw_per_rad{-0.65};
};

} // namespace ter26
