/**
 * @file suspension_kinematics.cpp
 * @brief Implementation of the 3D Double-Wishbone + Pushrod/Pullrod + Rocker + Coilover + Tie-Rod
 *        nonlinear constraint closure solver, implicit velocity/acceleration/parameter sensitivities,
 *        and 6x6 upright link reaction force solver for ANSYS correlation.
 */

#include "ter26/kinematics/suspension_kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace ter26 {

SuspensionKinematicsSolver::SuspensionKinematicsSolver(const CornerHardpoints& hp) {
    set_hardpoints(hp);
}

void SuspensionKinematicsSolver::set_hardpoints(const CornerHardpoints& hp) {
    hp_ = hp;
    L_lca_f_ = (hp_.lbj - hp_.lca_in_f).norm();
    L_lca_r_ = (hp_.lbj - hp_.lca_in_r).norm();
    L_uca_f_ = (hp_.ubj - hp_.uca_in_f).norm();
    L_uca_r_ = (hp_.ubj - hp_.uca_in_r).norm();
    L_tie_   = (hp_.tie_out - hp_.tie_in).norm();
    L_pushrod_ = (hp_.rocker_rod - hp_.pushrod_out).norm();
    L_spring_0_ = (hp_.rocker_coi - hp_.spring_in).norm();

    Vec3 axis_vec = hp_.rocker_axis - hp_.rocker_piv;
    const double n = axis_vec.norm();
    u_rocker_axis_ = (n > 1e-9) ? Vec3(axis_vec / n) : Vec3(1.0, 0.0, 0.0);

    // Build static wheel camber/toe alignment offset in axle frame:
    // Left side: negative camber tilts top of wheel inboard (-Y -> positive roll about +X),
    // Right side: negative camber tilts top of wheel inboard (+Y -> negative roll about +X).
    // Toe-in (>0): front of wheel points inboard (-Y on left -> negative yaw about +Z; +Y on right -> positive yaw about +Z).
    const double side_sign = hp_.is_right_side ? -1.0 : 1.0;
    const double roll_camber = -side_sign * hp_.static_camber_rad;
    const double yaw_toe     = -side_sign * hp_.static_toe_rad;
    R_wheel_static_ = math::rpy_to_rot_iso(Vec3(roll_camber, 0.0, yaw_toe));
}

void SuspensionKinematicsSolver::evaluate_6dof_closure_for_jounce(
    const Vec6& x_u,
    double target_wc_z,
    double y_rack_shift,
    Vec6& residual,
    Mat6& jacobian) const noexcept
{
    const Vec3 p_lbj = x_u.head<3>();
    const Vec3 psi_u = x_u.tail<3>();
    const Mat3 R_u   = math::exp_so3(psi_u);
    const Mat3 Jr    = math::right_jacobian_so3(psi_u);

    // Upright vectors relative to LBJ rotated into current orientation
    const Vec3 d_ubj = R_u * (hp_.ubj - hp_.lbj);
    const Vec3 d_to  = R_u * (hp_.tie_out - hp_.lbj);
    const Vec3 d_wc  = R_u * (hp_.wheel_center - hp_.lbj);

    const Vec3 p_ti_shifted = hp_.tie_in + Vec3(0.0, y_rack_shift, 0.0);

    // Links 0..4: spherical distance constraints using SymForce/SymPy generated kernel
    std::array<double, 6> row{};
    auto fill_row = [&](int r_idx, const Vec3& d_vec, const Vec3& c_pt, double L_ref) {
        double res_val = 0.0;
        sym::eval_spherical_link_constraint(
            p_lbj.x(), p_lbj.y(), p_lbj.z(),
            d_vec.x(), d_vec.y(), d_vec.z(),
            c_pt.x(),  c_pt.y(),  c_pt.z(),
            L_ref, res_val, row);
        residual(r_idx) = res_val;
        jacobian(r_idx, 0) = row[0];
        jacobian(r_idx, 1) = row[1];
        jacobian(r_idx, 2) = row[2];
        // Chain rule through R_u(psi + dpsi) = R_u(psi) * exp([Jr * dpsi]_x)
        // Note: R_u * [Jr * dpsi]_x * d0 = -[d_vec]_x * (R_u * Jr) * dpsi
        const Vec3 g_rot_world(row[3], row[4], row[5]);
        const Vec3 g_psi = (R_u * Jr).transpose() * g_rot_world;
        jacobian.block<1, 3>(r_idx, 3) = g_psi.transpose();
    };

    fill_row(0, Vec3::Zero(), hp_.lca_in_f, L_lca_f_);
    fill_row(1, Vec3::Zero(), hp_.lca_in_r, L_lca_r_);
    fill_row(2, d_ubj,        hp_.uca_in_f, L_uca_f_);
    fill_row(3, d_ubj,        hp_.uca_in_r, L_uca_r_);
    fill_row(4, d_to,         p_ti_shifted, L_tie_);

    // Equation 5: Wheel center vertical height constraint p_wc.z - target_wc_z = 0
    const Vec3 p_wc = p_lbj + d_wc;
    residual(5) = p_wc.z() - target_wc_z;
    jacobian(5, 0) = 0.0;
    jacobian(5, 1) = 0.0;
    jacobian(5, 2) = 1.0;
    // d(p_wc.z)/dpsi = e_z^T * (-[d_wc]_x * R_u * Jr) = (d_wc x e_z)^T * R_u * Jr
    const Vec3 dz_drot = d_wc.cross(Vec3::UnitZ());
    jacobian.block<1, 3>(5, 3) = ((R_u * Jr).transpose() * dz_drot).transpose();
}

double SuspensionKinematicsSolver::solve_rocker_angle(const Vec3& p_pushrod_out) const noexcept {
    // Solve 1D nonlinear closure for rocker rotation angle theta_r about (rocker_piv, u_rocker_axis_)
    // such that ||p_rocker_rod(theta_r) - p_pushrod_out||^2 - L_pushrod_^2 = 0
    const Vec3 r_rod0 = hp_.rocker_rod - hp_.rocker_piv;
    double theta = 0.0;
    for (int iter = 0; iter < 12; ++iter) {
        const Mat3 R_rock = math::exp_so3(theta * u_rocker_axis_);
        const Vec3 r_rod  = R_rock * r_rod0;
        const Vec3 p_rod  = hp_.rocker_piv + r_rod;
        const Vec3 diff   = p_rod - p_pushrod_out;
        const double f    = 0.5 * (diff.squaredNorm() - L_pushrod_ * L_pushrod_);
        if (std::abs(f) < 1e-13) break;
        const Vec3 dp_dtheta = u_rocker_axis_.cross(r_rod);
        const double df = diff.dot(dp_dtheta);
        if (std::abs(df) < 1e-9) break;
        const double step = std::clamp(-f / df, -0.25, 0.25);
        theta += step;
        if (std::abs(step) < 1e-11) break;
    }
    return theta;
}

CornerKinematicState SuspensionKinematicsSolver::solve(
    double wheel_jounce_m,
    double steering_angle_rad,
    const Vec6* warm_start_xu) const noexcept
{
    CornerKinematicState out{};
    const double clamped_jounce = std::clamp(wheel_jounce_m, -0.055, 0.055);
    const double eff_steer = hp_.is_steered ? std::clamp(steering_angle_rad, -0.55, 0.55) : 0.0;
    const double y_rack_shift = eff_steer * hp_.steering_rack_travel_per_rad;
    const double target_wc_z = hp_.wheel_center.z() + clamped_jounce;

    Vec6 x_u;
    if (warm_start_xu != nullptr && warm_start_xu->allFinite()) {
        x_u = *warm_start_xu;
    } else {
        x_u.head<3>() = hp_.lbj + Vec3(0.0, 0.0, clamped_jounce);
        x_u.tail<3>().setZero();
    }

    Vec6 res;
    Mat6 J;
    bool converged = false;
    int iter = 0;
    for (; iter < 20; ++iter) {
        evaluate_6dof_closure_for_jounce(x_u, target_wc_z, y_rack_shift, res, J);
        const double norm_inf = res.lpNorm<Eigen::Infinity>();
        if (norm_inf < 1e-12) {
            converged = true;
            break;
        }
        Eigen::ColPivHouseholderQR<Mat6> qr(J);
        Vec6 dx = qr.solve(-res);

        // Damped backtracking line search
        double alpha = 1.0;
        const double merit_0 = 0.5 * res.squaredNorm();
        for (int ls = 0; ls < 6; ++ls) {
            const Vec6 x_cand = x_u + alpha * dx;
            Vec6 res_cand;
            Mat6 J_dummy;
            evaluate_6dof_closure_for_jounce(x_cand, target_wc_z, y_rack_shift, res_cand, J_dummy);
            if (0.5 * res_cand.squaredNorm() < merit_0 || alpha <= 0.0625) {
                x_u = x_cand;
                break;
            }
            alpha *= 0.5;
        }
    }

    evaluate_6dof_closure_for_jounce(x_u, target_wc_z, y_rack_shift, res, J);
    out.x_upright = x_u;
    out.R_upright = math::exp_so3(x_u.tail<3>());
    out.constraint_residual_norm = res.lpNorm<Eigen::Infinity>();
    out.iterations = iter;
    out.converged = converged || (out.constraint_residual_norm < 1e-8);

    const Vec3 p_lbj = x_u.head<3>();
    const Vec3 p_ubj = p_lbj + out.R_upright * (hp_.ubj - hp_.lbj);
    const Vec3 p_po  = p_lbj + out.R_upright * (hp_.pushrod_out - hp_.lbj);
    const Vec3 p_wc_axle = p_lbj + out.R_upright * (hp_.wheel_center - hp_.lbj);

    // Solve rocker angle and coilover spring compression
    out.rocker_angle_rad = solve_rocker_angle(p_po);
    const Mat3 R_rock = math::exp_so3(out.rocker_angle_rad * u_rocker_axis_);
    const Vec3 p_rocker_coi = hp_.rocker_piv + R_rock * (hp_.rocker_coi - hp_.rocker_piv);
    const double L_spring = (p_rocker_coi - hp_.spring_in).norm();
    out.spring_deflection_m = L_spring_0_ - L_spring;

    const double side_sign = hp_.is_right_side ? -1.0 : 1.0;
    const double r_arb_arm = (u_rocker_axis_.cross(hp_.antiroll_out - hp_.rocker_piv)).norm();
    out.arb_droplink_deflection_m = side_sign * r_arb_arm * out.rocker_angle_rad;

    // Transform wheel center and orientation to Chassis CoG Frame F_B
    out.wheel_center_b = p_wc_axle + Vec3(hp_.axle_x_offset_from_cg_m, 0.0, 0.0);
    out.wheel_rot_b    = out.R_upright * R_wheel_static_;
    out.spindle_axis_b = out.wheel_rot_b.col(1).normalized(); // +Y axis of wheel frame

    // Implicit Velocity & Motion Ratio Jacobians via Implicit Function Theorem:
    // Phi_x * (dx_u / dz_s) = - dPhi / dz_s = [0, 0, 0, 0, 0, 1]^T
    // Phi_x * (dx_u / ddelta) = - dPhi / ddelta
    Eigen::ColPivHouseholderQR<Mat6> qr(J);
    Vec6 rhs_z = Vec6::Zero();
    rhs_z(5) = 1.0;
    const Vec6 dxu_dz = qr.solve(rhs_z);

    Vec6 rhs_delta = Vec6::Zero();
    if (hp_.is_steered) {
        const Vec3 p_to = p_lbj + out.R_upright * (hp_.tie_out - hp_.lbj);
        const Vec3 p_ti = hp_.tie_in + Vec3(0.0, y_rack_shift, 0.0);
        // d(Phi_4)/d(y_rack) = -(p_to.y - p_ti.y), so -dPhi_4/ddelta = (p_to.y - p_ti.y) * dy_rack/ddelta
        rhs_delta(4) = (p_to.y() - p_ti.y()) * hp_.steering_rack_travel_per_rad;
    }
    const Vec6 dxu_ddelta = qr.solve(rhs_delta);

    const Mat3 Jr = math::right_jacobian_so3(x_u.tail<3>());
    const Vec3 domega_u_dz_local = Jr * dxu_dz.tail<3>();
    const Vec3 domega_u_dz_world = out.R_upright * domega_u_dz_local;
    const Vec3 d_wc = out.R_upright * (hp_.wheel_center - hp_.lbj);

    out.domega_u_dz = domega_u_dz_world;
    out.dp_wc_dz    = dxu_dz.head<3>() + domega_u_dz_world.cross(d_wc);
    // Second-order wheel center path curvature d^2(p_WC)/dz_s^2 from rigid-body centripetal kinematics
    out.d2p_wc_dz2  = domega_u_dz_world.cross(domega_u_dz_world.cross(d_wc));

    const Vec3 domega_u_ddelta_world = out.R_upright * (Jr * dxu_ddelta.tail<3>());
    out.domega_u_ddelta = domega_u_ddelta_world;
    out.dp_wc_ddelta    = dxu_ddelta.head<3>() + domega_u_ddelta_world.cross(d_wc);

    // Analytical Motion Ratio MR = d(wheel_jounce) / d(spring_compression) via chain rule on rocker
    const Vec3 d_po = out.R_upright * (hp_.pushrod_out - hp_.lbj);
    const Vec3 dp_po_dz = dxu_dz.head<3>() + domega_u_dz_world.cross(d_po);
    const Vec3 r_rod  = R_rock * (hp_.rocker_rod - hp_.rocker_piv);
    const Vec3 p_rod  = hp_.rocker_piv + r_rod;
    const Vec3 diff_pr = p_rod - p_po;
    const double denom_rock = diff_pr.dot(u_rocker_axis_.cross(r_rod));
    double dtheta_r_dz = 0.0;
    if (std::abs(denom_rock) > 1e-8) {
        dtheta_r_dz = diff_pr.dot(dp_po_dz) / denom_rock;
    }
    out.dtheta_rocker_dz = dtheta_r_dz;
    out.darb_droplink_dz = side_sign * r_arb_arm * dtheta_r_dz;
    const Vec3 r_coi = R_rock * (hp_.rocker_coi - hp_.rocker_piv);
    const Vec3 dp_coi_dz = (u_rocker_axis_.cross(r_coi)) * dtheta_r_dz;
    const Vec3 u_spring = (p_rocker_coi - hp_.spring_in).normalized();
    const double dspring_comp_dz = -u_spring.dot(dp_coi_dz);
    out.motion_ratio = (std::abs(dspring_comp_dz) > 0.15)
        ? std::clamp(std::abs(1.0 / dspring_comp_dz), 0.5, 2.5)
        : 1.14;

    // Extract K&C angles: camber, toe, kingpin axis (caster, KPI, scrub, trail)
    // Camber: negative when top of tire tilts inboard
    out.camber_rad = side_sign * std::asin(std::clamp(out.spindle_axis_b.z(), -1.0, 1.0));
    // Wheel forward unit vector projected onto horizontal X-Y plane
    const Vec3 wheel_fwd = out.wheel_rot_b.col(0);
    const double raw_yaw = std::atan2(wheel_fwd.y(), wheel_fwd.x());
    // Toe-in positive convention: on left (+Y), toe-in means yaw < 0; on right (-Y), toe-in means yaw > 0
    out.toe_rad = -side_sign * raw_yaw;

    // Steering axis (Kingpin) unit vector from LBJ to UBJ
    const Vec3 kp_vec = (p_ubj - p_lbj).normalized();
    out.caster_rad = std::atan2(-kp_vec.x(), kp_vec.z());
    out.kpi_rad    = side_sign * std::atan2(-kp_vec.y(), kp_vec.z());

    // Kingpin intersection with ground plane z = 0
    if (std::abs(kp_vec.z()) > 1e-6) {
        const double t_ground = -p_lbj.z() / kp_vec.z();
        const Vec3 p_kp_ground = p_lbj + t_ground * kp_vec;
        out.scrub_radius_m     = side_sign * (p_wc_axle.y() - p_kp_ground.y());
        out.mechanical_trail_m = p_kp_ground.x() - p_wc_axle.x();
    }

    // Instant center & geometric roll center height via upper/lower wishbone planes
    const Vec3 u_lca = ((hp_.lca_in_f + hp_.lca_in_r) * 0.5 - p_lbj).normalized();
    const Vec3 u_uca = ((hp_.uca_in_f + hp_.uca_in_r) * 0.5 - p_ubj).normalized();
    const double det_ic = u_lca.y() * u_uca.z() - u_lca.z() * u_uca.y();
    if (std::abs(det_ic) > 1e-4) {
        const double dy = p_ubj.y() - p_lbj.y();
        const double dz = p_ubj.z() - p_lbj.z();
        const double s_lca = (dy * u_uca.z() - dz * u_uca.y()) / det_ic;
        const Vec3 p_ic = p_lbj + s_lca * u_lca;
        // Line from tire ground contact (0, p_wc.y, 0) to instant center p_ic evaluated at centerline y = 0
        if (std::abs(p_ic.y() - p_wc_axle.y()) > 1e-4) {
            const double alpha_rc = -p_wc_axle.y() / (p_ic.y() - p_wc_axle.y());
            out.roll_center_height_m = std::clamp(alpha_rc * p_ic.z(), -0.10, 0.25);
        }
    }

    return out;
}

Eigen::Matrix<double, 6, 39> SuspensionKinematicsSolver::compute_hardpoint_sensitivity(
    const CornerKinematicState& state,
    double steering_angle_rad) const noexcept
{
    // Compute dx_u / dp = - Phi_x^{-1} * Phi_p for the 13 primary hardpoints (39 scalar coordinates)
    const double eff_steer = hp_.is_steered ? std::clamp(steering_angle_rad, -0.55, 0.55) : 0.0;
    const double y_rack_shift = eff_steer * hp_.steering_rack_travel_per_rad;
    const Vec3 p_lbj = state.x_upright.head<3>();
    const Vec3 p_wc  = p_lbj + state.R_upright * (hp_.wheel_center - hp_.lbj);

    Vec6 res;
    Mat6 Phi_x;
    evaluate_6dof_closure_for_jounce(state.x_upright, p_wc.z(), y_rack_shift, res, Phi_x);
    Eigen::ColPivHouseholderQR<Mat6> qr(Phi_x);

    Eigen::Matrix<double, 6, 39> Phi_p = Eigen::Matrix<double, 6, 39>::Zero();
    const Vec3 p_ubj = p_lbj + state.R_upright * (hp_.ubj - hp_.lbj);
    const Vec3 p_to  = p_lbj + state.R_upright * (hp_.tie_out - hp_.lbj);
    const Vec3 p_ti  = hp_.tie_in + Vec3(0.0, y_rack_shift, 0.0);

    // Partial derivatives wrt chassis-side points: d(0.5 ||p_out - p_in||^2)/dp_in = -(p_out - p_in)^T
    Phi_p.block<1, 3>(0, 0)  = -(p_lbj - hp_.lca_in_f).transpose();
    Phi_p.block<1, 3>(1, 3)  = -(p_lbj - hp_.lca_in_r).transpose();
    Phi_p.block<1, 3>(2, 6)  = -(p_ubj - hp_.uca_in_f).transpose();
    Phi_p.block<1, 3>(3, 9)  = -(p_ubj - hp_.uca_in_r).transpose();
    Phi_p.block<1, 3>(4, 12) = -(p_to - p_ti).transpose();

    return qr.solve(-Phi_p);
}

CornerJointReactions SuspensionKinematicsSolver::solve_joint_reactions(
    const CornerKinematicState& state,
    double steering_angle_rad,
    const Vec3& force_at_wheel_center_b,
    const Vec3& moment_at_wheel_center_b,
    double arb_force_N) const noexcept
{
    CornerJointReactions react{};
    const Vec3 p_lbj = state.x_upright.head<3>();
    const Mat3& R_u  = state.R_upright;

    const Vec3 p_ubj = p_lbj + R_u * (hp_.ubj - hp_.lbj);
    const Vec3 p_to  = p_lbj + R_u * (hp_.tie_out - hp_.lbj);
    const Vec3 p_po  = p_lbj + R_u * (hp_.pushrod_out - hp_.lbj);
    const Vec3 p_wc  = p_lbj + R_u * (hp_.wheel_center - hp_.lbj);

    const double eff_steer = hp_.is_steered ? std::clamp(steering_angle_rad, -0.55, 0.55) : 0.0;
    const Vec3 p_ti = hp_.tie_in + Vec3(0.0, eff_steer * hp_.steering_rack_travel_per_rad, 0.0);

    const Mat3 R_rock = math::exp_so3(state.rocker_angle_rad * u_rocker_axis_);
    const Vec3 p_pr   = hp_.rocker_piv + R_rock * (hp_.rocker_rod - hp_.rocker_piv);

    // 6 unit vectors pointing from upright attachment to inboard attachment (+force = tension)
    const Vec3 e_uf = (hp_.uca_in_f - p_ubj).normalized();
    const Vec3 e_ua = (hp_.uca_in_r - p_ubj).normalized();
    const Vec3 e_lf = (hp_.lca_in_f - p_lbj).normalized();
    const Vec3 e_la = (hp_.lca_in_r - p_lbj).normalized();
    const Vec3 e_tr = (p_ti - p_to).normalized();
    const Vec3 e_pr = (p_pr - p_po).normalized();

    // Moment arms from wheel center p_wc to upright ball joints
    const Vec3 r_ubj = p_ubj - p_wc;
    const Vec3 r_lbj = p_lbj - p_wc;
    const Vec3 r_to  = p_to  - p_wc;
    const Vec3 r_po  = p_po  - p_wc;

    // Assemble 6x6 statics matrix A_stat:
    // Top 3 rows: sum of link forces; Bottom 3 rows: sum of link moments about wheel center
    Mat6 A_stat;
    A_stat.col(0) << e_uf, r_ubj.cross(e_uf);
    A_stat.col(1) << e_ua, r_ubj.cross(e_ua);
    A_stat.col(2) << e_lf, r_lbj.cross(e_lf);
    A_stat.col(3) << e_la, r_lbj.cross(e_la);
    A_stat.col(4) << e_tr, r_to.cross(e_tr);
    A_stat.col(5) << e_pr, r_po.cross(e_pr);

    Vec6 W_ext;
    W_ext.head<3>() = force_at_wheel_center_b;
    W_ext.tail<3>() = moment_at_wheel_center_b;

    Eigen::JacobiSVD<Mat6> svd(A_stat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& sing_vals = svd.singularValues();
    react.conditioning_rcond = (sing_vals(0) > 1e-12) ? (sing_vals(5) / sing_vals(0)) : 0.0;

    const Vec6 f_links = svd.solve(-W_ext);
    react.f_uca_fore_N = f_links(0);
    react.f_uca_aft_N  = f_links(1);
    react.f_lca_fore_N = f_links(2);
    react.f_lca_aft_N  = f_links(3);
    react.f_tie_rod_N  = f_links(4);
    react.f_pushrod_N  = f_links(5);
    react.equilibrium_residual_norm = (A_stat * f_links + W_ext).norm();

    // Propagate pushrod + ARB droplink forces onto the rocker bellcrank to compute coilover damper & pivot reaction
    const Vec3 p_coi = hp_.rocker_piv + R_rock * (hp_.rocker_coi   - hp_.rocker_piv);
    const Vec3 p_arb = hp_.rocker_piv + R_rock * (hp_.antiroll_out - hp_.rocker_piv);
    const Vec3 e_coi = (hp_.spring_in - p_coi).normalized();
    const Vec3 r_rock_pr  = p_pr  - hp_.rocker_piv;
    const Vec3 r_rock_coi = p_coi - hp_.rocker_piv;
    const Vec3 r_rock_arb = p_arb - hp_.rocker_piv;

    // Force exerted by pushrod on rocker is -f_pushrod_N * e_pr; ARB droplink force is along Z
    const Vec3 F_pr_on_rocker  = -react.f_pushrod_N * e_pr;
    const Vec3 F_arb_on_rocker = Vec3(0.0, 0.0, -arb_force_N);
    const double M_ext_axis = u_rocker_axis_.dot(
        r_rock_pr.cross(F_pr_on_rocker) + r_rock_arb.cross(F_arb_on_rocker));
    const double arm_coi    = u_rocker_axis_.dot(r_rock_coi.cross(e_coi));

    react.f_arb_droplink_N = arb_force_N;
    if (std::abs(arm_coi) > 1e-5) {
        react.f_damper_coilover_N = -M_ext_axis / arm_coi;
    } else {
        react.f_damper_coilover_N = -react.f_pushrod_N * state.motion_ratio;
    }

    const Vec3 F_coi_on_rocker = react.f_damper_coilover_N * e_coi;
    // Rocker pivot bearing reaction force on the rocker: F_piv + F_pr + F_coi + F_arb = 0
    // (Equal-and-opposite force applied TO the chassis at rocker_piv is -(F_piv) = F_pr + F_coi + F_arb)
    react.rocker_pivot_force_b = -(F_pr_on_rocker + F_coi_on_rocker + F_arb_on_rocker);

    return react;
}

} // namespace ter26
