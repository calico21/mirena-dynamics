#pragma once
/**
 * @file vehicle_state.hpp
 * @brief Complete 14-DOF Mechanical State, Internal Dynamical States, Control Inputs,
 *        Environment Queries, Joint Reaction Wrenches, and Step Diagnostics for TeR26.
 */

#include "ter26/math/lie_groups.hpp"
#include <array>
#include <string>

namespace ter26 {

/// Corner index enumeration matching ISO / MirenaSim convention
enum CornerIndex : int {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
    NUM_CORNERS = 4
};

/// Ground contact plane query under each wheel in Inertial World Frame (ISO 8855)
struct GroundContactQuery {
    Vec3 point_w{Vec3::Zero()};              ///< Ground point under wheel center [m]
    Vec3 normal_w{Vec3::UnitZ()};            ///< Ground unit normal (+Z up on flat ground)
    Vec3 surface_velocity_w{Vec3::Zero()};   ///< Moving ground velocity (0 on track) [m/s]
    double friction_multiplier{1.0};         ///< Local grip multiplier (1.0 dry asphalt)
};

/// Driverless / Manual Control Inputs
struct ControlInputs {
    double throttle{0.0};            ///< Normalized throttle request in [0, 1]
    double brake{0.0};               ///< Normalized brake request in [0, 1]
    double steering_cmd_rad{0.0};    ///< Commanded road-wheel steering angle [rad] (+CCW / Left)
    bool ebs_active{false};          ///< Autonomous Emergency Brake System active flag
    double torque_vectoring_gain{1.0}; ///< Multiplier for active yaw-rate torque vectoring
};

/// External Disturbance / Collision Wrench & Impulse
struct ExternalInteraction {
    Vec3 body_force_b{Vec3::Zero()};         ///< Continuous external force in chassis frame [N]
    Vec3 body_moment_b{Vec3::Zero()};        ///< Continuous external moment about CoG [N*m]
    bool has_collision_impulse{false};       ///< True if discrete collision occurred this step
    Vec3 impulse_contact_point_w{Vec3::Zero()}; ///< World-frame contact point [m]
    Vec3 impulse_normal_w{Vec3::UnitX()};    ///< Contact normal pointing into vehicle
    Vec3 other_body_vel_w{Vec3::Zero()};     ///< Velocity of collided object [m/s]
    double other_body_mass_kg{0.2};          ///< Mass of collided object (e.g. 0.2 kg cone, 1e9 wall)
    double restitution{0.25};                ///< Coefficient of restitution e in [0, 1]
    double friction_mu{0.40};                ///< Tangential collision friction coefficient
};

/// Full 6x6 Upright Link Axial Forces and Rocker Reactions (for ANSYS Structural Correlation)
struct CornerJointReactions {
    double f_uca_fore_N{0.0};        ///< Upper wishbone fore leg axial force (+tension) [N]
    double f_uca_aft_N{0.0};         ///< Upper wishbone aft leg axial force (+tension) [N]
    double f_lca_fore_N{0.0};        ///< Lower wishbone fore leg axial force (+tension) [N]
    double f_lca_aft_N{0.0};         ///< Lower wishbone aft leg axial force (+tension) [N]
    double f_tie_rod_N{0.0};         ///< Tie-rod axial force (+tension) [N]
    double f_pushrod_N{0.0};         ///< Pushrod/pullrod axial force (+tension) [N]
    double f_damper_coilover_N{0.0}; ///< Total coilover spring+damper+bumpstop force [N]
    double f_arb_droplink_N{0.0};    ///< Anti-roll bar droplink force [N]
    Vec3 rocker_pivot_force_b{Vec3::Zero()}; ///< 3D bearing reaction on chassis rocker pivot [N]
    double equilibrium_residual_norm{0.0};   ///< ||A_stat * f + W_ext||_2 verification residual
    double conditioning_rcond{1.0};          ///< Reciprocal condition number of 6x6 statics matrix
};

/// Detailed per-corner kinematic, tire, and powertrain telemetry
struct CornerDiagnostics {
    // 3D Kinematics in Chassis Frame F_B
    Vec3 wheel_center_b{Vec3::Zero()};
    Mat3 wheel_orientation_b{Mat3::Identity()};
    Vec3 spindle_axis_b{Vec3::UnitY()};
    double wheel_jounce_m{0.0};
    double wheel_jounce_vel_m_s{0.0};
    double spring_deflection_m{0.0};
    double spring_velocity_m_s{0.0};
    double motion_ratio{1.14};
    double camber_rad{0.0};
    double toe_rad{0.0};
    double caster_rad{0.0};
    double kpi_rad{0.0};
    double scrub_radius_m{0.0};
    double mechanical_trail_m{0.0};
    double roll_center_height_m{0.04};
    double kinematic_solver_residual{0.0};
    int kinematic_solver_iterations{0};

    // Compliance deflections
    double compliance_camber_rad{0.0};
    double compliance_toe_rad{0.0};
    Vec3 compliance_translation_m{Vec3::Zero()};

    // Tire kinematics & forces in Contact Patch Frame F_C (ISO 8855)
    double penetration_m{0.0};
    double penetration_vel_m_s{0.0};
    double effective_rolling_radius_m{0.2032};
    double vx_contact_m_s{0.0};
    double vy_contact_m_s{0.0};
    double kappa_steady{0.0};
    double alpha_steady_rad{0.0};
    double kappa_dynamic{0.0};
    double alpha_dynamic_rad{0.0};
    double Fz_N{0.0};
    double Fx_N{0.0};
    double Fy_N{0.0};
    double Mx_Nm{0.0};
    double My_Nm{0.0};
    double Mz_Nm{0.0};
    double friction_utilization{0.0};

    // Suspension & Powertrain torques
    double spring_force_N{0.0};
    double damper_force_N{0.0};
    double bump_stop_force_N{0.0};
    double arb_force_N{0.0};
    double suspension_vertical_wheel_force_N{0.0};
    double drive_torque_wheel_Nm{0.0};
    double brake_torque_wheel_Nm{0.0};
    double upright_reaction_torque_Nm{0.0};

    // Joint reactions for ANSYS
    CornerJointReactions joint_reactions{};
};

/// Complete 14-DOF Mechanical + Internal State Vector of TeR26
struct VehicleState {
    // --- 1. Mechanical Generalized Coordinates q (14 DOFs on SE(3) x R^4 x R^4) ---
    Vec3 p_w{0.0, 0.0, 0.305};                 ///< Chassis reference position in World ISO [m]
    Mat3 R_wb{Mat3::Identity()};               ///< Chassis orientation R_WB in SO(3)
    std::array<double, 4> q_susp{0.0, 0.0, 0.0, 0.0}; ///< Corner vertical jounce / rocker coords [m]
    std::array<double, 4> phi_wheel{0.0, 0.0, 0.0, 0.0}; ///< Wheel spin angles [rad]

    // --- 2. Mechanical Generalized Velocities v (14 DOFs in R^14) ---
    Vec3 v_b{Vec3::Zero()};                    ///< Chassis linear velocity in Body Frame F_B [m/s]
    Vec3 omega_b{Vec3::Zero()};                ///< Chassis angular velocity in Body Frame F_B [rad/s]
    std::array<double, 4> v_susp{0.0, 0.0, 0.0, 0.0}; ///< Corner jounce velocities [m/s]
    std::array<double, 4> omega_wheel{0.0, 0.0, 0.0, 0.0}; ///< Wheel spin angular velocities [rad/s]

    // --- 3. Internal Dynamical States z (Tire Relaxation, Bristle Static, Actuator, Compliance) ---
    std::array<double, 4> kappa_dyn{0.0, 0.0, 0.0, 0.0};  ///< Transient longitudinal slip [-]
    std::array<double, 4> alpha_dyn{0.0, 0.0, 0.0, 0.0};  ///< Transient slip angle [rad]
    std::array<Vec2, 4> bristle_deflection{Vec2::Zero(), Vec2::Zero(), Vec2::Zero(), Vec2::Zero()}; ///< Elastoplastic static friction deflection [m]
    std::array<double, 4> tire_temp_C{45.0, 45.0, 45.0, 45.0}; ///< Tire tread temperature [deg C]
    std::array<double, 4> compliance_camber_state{0.0, 0.0, 0.0, 0.0}; ///< Filtered camber compliance [rad]
    std::array<double, 4> compliance_toe_state{0.0, 0.0, 0.0, 0.0};    ///< Filtered toe compliance [rad]
    double steering_rack_angle_rad{0.0};       ///< Actual steering actuator angle [rad]
    double steering_rack_rate_rad_s{0.0};      ///< Actual steering actuator rate [rad/s]
    std::array<double, 4> motor_torque_actual_Nm{0.0, 0.0, 0.0, 0.0}; ///< Filtered motor electromagnetic torque [N*m]
    double brake_pressure_norm{0.0};           ///< Filtered hydraulic brake line pressure [0, 1]
    double simulation_time_s{0.0};             ///< Accumulated deterministic simulation time [s]

    /// Pack the 28 mechanical states + 10 key internal states into a flat vector for sensitivity analysis
    Eigen::Matrix<double, 38, 1> to_flat_vector() const noexcept {
        Eigen::Matrix<double, 38, 1> x;
        x.segment<3>(0) = p_w;
        x.segment<3>(3) = math::rot_to_rpy_iso(R_wb);
        for (int i = 0; i < 4; ++i) x(6 + i) = q_susp[i];
        for (int i = 0; i < 4; ++i) x(10 + i) = phi_wheel[i];
        x.segment<3>(14) = v_b;
        x.segment<3>(17) = omega_b;
        for (int i = 0; i < 4; ++i) x(20 + i) = v_susp[i];
        for (int i = 0; i < 4; ++i) x(24 + i) = omega_wheel[i];
        for (int i = 0; i < 4; ++i) x(28 + i) = kappa_dyn[i];
        for (int i = 0; i < 4; ++i) x(32 + i) = alpha_dyn[i];
        x(36) = steering_rack_angle_rad;
        x(37) = brake_pressure_norm;
        return x;
    }

    void from_flat_vector(const Eigen::Matrix<double, 38, 1>& x) noexcept {
        p_w = x.segment<3>(0);
        R_wb = math::rpy_to_rot_iso(x.segment<3>(3));
        for (int i = 0; i < 4; ++i) q_susp[i] = x(6 + i);
        for (int i = 0; i < 4; ++i) phi_wheel[i] = x(10 + i);
        v_b = x.segment<3>(14);
        omega_b = x.segment<3>(17);
        for (int i = 0; i < 4; ++i) v_susp[i] = x(20 + i);
        for (int i = 0; i < 4; ++i) omega_wheel[i] = x(24 + i);
        for (int i = 0; i < 4; ++i) kappa_dyn[i] = x(28 + i);
        for (int i = 0; i < 4; ++i) alpha_dyn[i] = x(32 + i);
        steering_rack_angle_rad = x(36);
        brake_pressure_norm = x(37);
    }
};

/// Explicit flags for non-smooth regime transitions (contact liftoff, friction saturation, bump stops, stiction)
struct NonSmoothRegimeFlags {
    std::array<bool, 4> wheel_liftoff_transition{false, false, false, false};
    std::array<bool, 4> tire_friction_saturated{false, false, false, false};
    std::array<bool, 4> bump_stop_engaged{false, false, false, false};
    std::array<bool, 4> brake_stiction_locked{false, false, false, false};
    bool external_contact_grazing{false};

    bool any_nonsmooth_event() const noexcept {
        for (int i = 0; i < 4; ++i) {
            if (wheel_liftoff_transition[i] || tire_friction_saturated[i] ||
                bump_stop_engaged[i] || brake_stiction_locked[i]) {
                return true;
            }
        }
        return external_contact_grazing;
    }
};

/// Numerical conditioning, adaptive error control, and recovery diagnostics (§5.3)
struct SolverNumericalDiagnostics {
    double mass_matrix_condition_number{1.0};
    double mass_matrix_min_eigenvalue{1.0};
    double local_truncation_error_norm{0.0};
    int adaptive_substeps_executed{1};
    int rejected_adaptive_steps{0};
    bool recovered_from_invalid_state{false};
    NonSmoothRegimeFlags nonsmooth{};
};

/// Step result returned by VehicleDynamicsEngine::step_vehicle_dynamics()
struct VehicleStepResult {
    VehicleState next_state{};
    Vec14 generalized_accel{Vec14::Zero()};    ///< dv/dt in R^14
    Mat14 mass_matrix{Mat14::Identity()};      ///< Configuration-dependent M(q) in R^{14x14}
    Vec14 coriolis_vector{Vec14::Zero()};      ///< Convective/Coriolis/gyroscopic C(q, v) in R^14
    Vec14 generalized_forces{Vec14::Zero()};   ///< Total generalized forces Q in R^14
    Vec3 specific_force_imu_b{Vec3::Zero()};   ///< Proper acceleration a_B - R_WB^T g for IMU [m/s^2]
    Vec3 aero_force_b{Vec3::Zero()};           ///< Aerodynamic force in F_B [N]
    Vec3 aero_moment_b{Vec3::Zero()};          ///< Aerodynamic moment in F_B [N*m]
    Vec3 external_impulse_applied_w{Vec3::Zero()}; ///< Equal-and-opposite impulse to apply to Godot collider [N*s]
    std::array<CornerDiagnostics, 4> corners{};
    SolverNumericalDiagnostics solver_diagnostics{};
    double total_mechanical_energy_J{0.0};
    double electrical_power_W{0.0};
    bool solver_converged{true};
};

} // namespace ter26
