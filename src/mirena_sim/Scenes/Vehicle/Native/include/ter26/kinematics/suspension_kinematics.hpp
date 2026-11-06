#pragma once
/**
 * @file suspension_kinematics.hpp
 * @brief Nonlinear 3D Double-Wishbone + Pushrod/Pullrod + Rocker + Coilover + Tie-Rod + ARB
 *        Kinematics Solver, Implicit Velocity/Acceleration/Parameter Sensitivities,
 *        and 6x6 Upright Link Reaction Force Solver for ANSYS Correlation.
 */

#include "ter26/state/vehicle_state.hpp"
#include "symforce_kernels.hpp"
#include <array>

namespace ter26 {

/// 17 3D hardpoints defining one suspension corner in the Axle Midpoint Frame (x+ fwd, y+ left, z+ up)
struct CornerHardpoints {
    Vec3 lca_in_f{ 0.150,  0.200, 0.130};
    Vec3 lca_in_r{-0.150,  0.200, 0.135};
    Vec3 lbj     { 0.008,  0.545, 0.115};
    Vec3 uca_in_f{ 0.120,  0.240, 0.270};
    Vec3 uca_in_r{-0.120,  0.240, 0.275};
    Vec3 ubj     {-0.0067, 0.520, 0.285};
    Vec3 tie_in  {-0.055,  0.215, 0.165};
    Vec3 tie_out {-0.062,  0.535, 0.152};
    Vec3 pushrod_out{ 0.015, 0.505, 0.145};
    Vec3 rocker_rod { 0.015, 0.275, 0.465};
    Vec3 rocker_piv { 0.000, 0.220, 0.450};
    Vec3 rocker_axis{ 0.100, 0.220, 0.450};
    Vec3 rocker_coi { 0.000, 0.195, 0.502};
    Vec3 spring_in  { 0.000, 0.050, 0.480};
    Vec3 antiroll_out{-0.025, 0.220, 0.415};
    Vec3 antiroll_in {-0.095, 0.220, 0.415};
    Vec3 wheel_center{ 0.000, 0.587, 0.208};

    double axle_x_offset_from_cg_m{0.876}; ///< +l_f for front, -l_r for rear
    double static_camber_rad{-2.0 * M_PI / 180.0};
    double static_toe_rad{-0.10 * M_PI / 180.0};
    double steering_rack_travel_per_rad{0.062}; ///< d(y_rack)/d(delta_steer) [m/rad]
    bool is_steered{true};
    bool is_right_side{false};

    /// Mirror a Left-side hardpoint set to the Right side (y -> -y)
    CornerHardpoints mirrored_to_right() const noexcept {
        CornerHardpoints r = *this;
        auto flip_y = [](const Vec3& v) { return Vec3(v.x(), -v.y(), v.z()); };
        r.lca_in_f = flip_y(lca_in_f);
        r.lca_in_r = flip_y(lca_in_r);
        r.lbj      = flip_y(lbj);
        r.uca_in_f = flip_y(uca_in_f);
        r.uca_in_r = flip_y(uca_in_r);
        r.ubj      = flip_y(ubj);
        r.tie_in   = flip_y(tie_in);
        r.tie_out  = flip_y(tie_out);
        r.pushrod_out = flip_y(pushrod_out);
        r.rocker_rod  = flip_y(rocker_rod);
        r.rocker_piv  = flip_y(rocker_piv);
        r.rocker_axis = flip_y(rocker_axis);
        r.rocker_coi  = flip_y(rocker_coi);
        r.spring_in   = flip_y(spring_in);
        r.antiroll_out = flip_y(antiroll_out);
        r.antiroll_in  = flip_y(antiroll_in);
        r.wheel_center = flip_y(wheel_center);
        r.is_right_side = true;
        return r;
    }
};

/// Complete Kinematic Solution of a Corner at given (wheel_jounce_z, steering_angle_rad)
struct CornerKinematicState {
    Vec6 x_upright{Vec6::Zero()};       ///< [p_lbj (3), psi_u (3)] in axle frame
    Mat3 R_upright{Mat3::Identity()};   ///< Upright orientation exp([psi_u]_x)
    double rocker_angle_rad{0.0};       ///< Solved rocker rotation angle theta_r [rad]
    Vec3 wheel_center_b{Vec3::Zero()};  ///< Wheel center in Chassis CoG Frame F_B [m]
    Mat3 wheel_rot_b{Mat3::Identity()}; ///< Wheel orientation (with static camber/toe) in F_B
    Vec3 spindle_axis_b{Vec3::UnitY()}; ///< Unit spindle rotation axis in F_B

    // Partial Jacobians w.r.t. wheel jounce z_s and steering angle delta
    Vec3 dp_wc_dz{Vec3::UnitZ()};       ///< d(p_WC)/d(z_s) in F_B [-]
    Vec3 domega_u_dz{Vec3::Zero()};     ///< Upright angular velocity Jacobian wrt z_s [rad/m]
    Vec3 d2p_wc_dz2{Vec3::Zero()};      ///< Second derivative d^2(p_WC)/dz_s^2 for convective accel [1/m]
    Vec3 dp_wc_ddelta{Vec3::Zero()};    ///< d(p_WC)/d(delta) in F_B [m/rad]
    Vec3 domega_u_ddelta{Vec3::Zero()}; ///< Upright angular velocity Jacobian wrt delta [-]

    // Coilover & ARB displacements
    double spring_deflection_m{0.0};    ///< Coilover compression (+ in bump) [m]
    double motion_ratio{1.14};          ///< d(wheel_jounce) / d(spring_compression) [-]
    double arb_droplink_deflection_m{0.0};
    double dtheta_rocker_dz{0.0};       ///< d(theta_rocker)/d(z_s) [rad/m]
    double darb_droplink_dz{0.0};       ///< d(z_arb_droplink)/d(z_s) [-]

    // Derived K&C metrics
    double camber_rad{0.0};
    double toe_rad{0.0};
    double caster_rad{0.0};
    double kpi_rad{0.0};
    double scrub_radius_m{0.0};
    double mechanical_trail_m{0.0};
    double roll_center_height_m{0.04};

    // Solver diagnostics
    double constraint_residual_norm{0.0};
    int iterations{0};
    bool converged{true};
};

class SuspensionKinematicsSolver {
public:
    explicit SuspensionKinematicsSolver(const CornerHardpoints& hp = CornerHardpoints{});

    void set_hardpoints(const CornerHardpoints& hp);
    const CornerHardpoints& hardpoints() const noexcept { return hp_; }

    /**
     * @brief Solve the 6x6 nonlinear 3D suspension closure Phi(x_u, z_s, delta) = 0
     *        plus the 3D rocker pushrod-to-coilover triangle at given vertical jounce z_s [m]
     *        and steering command delta [rad].
     */
    CornerKinematicState solve(
        double wheel_jounce_m,
        double steering_angle_rad,
        const Vec6* warm_start_xu = nullptr) const noexcept;

    /**
     * @brief Compute the 6x39 sensitivity matrix dx_u / dp_hardpoints via Implicit Function Theorem:
     *        dx_u / dp = - Phi_x^{-1} * Phi_p
     *        for the 13 primary kinematic hardpoints (39 coordinates).
     */
    Eigen::Matrix<double, 6, 39> compute_hardpoint_sensitivity(
        const CornerKinematicState& state,
        double steering_angle_rad) const noexcept;

    /**
     * @brief Solve the 6x6 upright static/dynamic equilibrium system A_stat * f_links = -W_ext
     *        plus rocker bellcrank torque/force balance to extract all internal member forces
     *        (UCA fore/aft, LCA fore/aft, tie-rod, pushrod, coilover damper, ARB, rocker pivot)
     *        for ANSYS structural FEA correlation.
     */
    CornerJointReactions solve_joint_reactions(
        const CornerKinematicState& state,
        double steering_angle_rad,
        const Vec3& force_at_wheel_center_b,
        const Vec3& moment_at_wheel_center_b,
        double arb_force_N) const noexcept;

private:
    CornerHardpoints hp_;
    // Reference link lengths computed at nominal design position
    double L_lca_f_{0.0};
    double L_lca_r_{0.0};
    double L_uca_f_{0.0};
    double L_uca_r_{0.0};
    double L_tie_{0.0};
    double L_pushrod_{0.0};
    double L_spring_0_{0.0};
    Vec3 u_rocker_axis_{Vec3::UnitX()};
    Mat3 R_wheel_static_{Mat3::Identity()};

    void evaluate_6dof_closure_for_jounce(
        const Vec6& x_u,
        double target_wc_z,
        double y_rack_shift,
        Vec6& residual,
        Mat6& jacobian) const noexcept;

    double solve_rocker_angle(const Vec3& p_pushrod_out) const noexcept;
};

} // namespace ter26
