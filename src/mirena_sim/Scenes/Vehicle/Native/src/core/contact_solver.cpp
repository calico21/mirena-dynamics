/**
 * @file contact_solver.cpp
 * @brief Implementation of the Multi-Class 3D Frictional Contact & Impact Mechanics Solver:
 *        - Two-body 3D off-center frictional impact with exact momentum & energy laws
 *        - Multi-point coupled 3D Projected Gauss-Seidel (PGS) solver on the Delassus matrix
 *        - Chassis skidblock ground-bottoming guard
 */

#include "ter26/contact/contact_solver.hpp"
#include <cmath>
#include <algorithm>

namespace ter26 {

void ContactMechanicsSolver::build_contact_basis(
    const Vec3& normal_w,
    Vec3& n_unit,
    Vec3& t1_unit,
    Vec3& t2_unit) noexcept
{
    const double n_norm = normal_w.norm();
    n_unit = (n_norm > 1e-9) ? Vec3(normal_w / n_norm) : Vec3::UnitZ();
    const Vec3 ref = (std::abs(n_unit.x()) < 0.85) ? Vec3::UnitX() : Vec3::UnitY();
    t1_unit = (n_unit.cross(ref)).normalized();
    t2_unit = n_unit.cross(t1_unit);
}

TwoBodyImpactResult ContactMechanicsSolver::solve_two_body_impact(
    RigidBodyComponent& body_a,
    RigidBodyComponent& body_b,
    const Vec3& contact_point_w,
    const Vec3& normal_b_to_a_w,
    double restitution,
    double friction_mu) noexcept
{
    TwoBodyImpactResult res{};
    Vec3 n, t1, t2;
    build_contact_basis(normal_b_to_a_w, n, t1, t2);
    Mat3 R_c;
    R_c.col(0) = n;
    R_c.col(1) = t1;
    R_c.col(2) = t2;

    const Vec3 p_com_a = body_a.com_world();
    const Vec3 p_com_b = body_b.com_world();
    const Vec3 r_a = contact_point_w - p_com_a;
    const Vec3 r_b = contact_point_w - p_com_b;

    const Vec3 v_com_a_pre = body_a.com_velocity_world();
    const Vec3 v_com_b_pre = body_b.com_velocity_world();
    const Vec3 omega_a_w_pre = body_a.rot_wb * body_a.omega_b;
    const Vec3 omega_b_w_pre = body_b.rot_wb * body_b.omega_b;

    const Mat3 I_a_w = body_a.rot_wb * body_a.inertia_com_b * body_a.rot_wb.transpose();
    const Mat3 I_b_w = body_b.rot_wb * body_b.inertia_com_b * body_b.rot_wb.transpose();

    const Vec3 P_pre = (body_a.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(body_a.mass_kg * v_com_a_pre))
                     + (body_b.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(body_b.mass_kg * v_com_b_pre));
    const Vec3 L_pre = (body_a.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(p_com_a.cross(body_a.mass_kg * v_com_a_pre) + I_a_w * omega_a_w_pre))
                     + (body_b.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(p_com_b.cross(body_b.mass_kg * v_com_b_pre) + I_b_w * omega_b_w_pre));

    res.pre_kinetic_energy_J = body_a.kinetic_energy() + body_b.kinetic_energy();

    const Vec3 v_pt_a = v_com_a_pre + omega_a_w_pre.cross(r_a);
    const Vec3 v_pt_b = v_com_b_pre + omega_b_w_pre.cross(r_b);
    const Vec3 v_rel_w = v_pt_a - v_pt_b;
    const double v_n_pre = n.dot(v_rel_w);

    if (v_n_pre >= 0.0) {
        res.regime = (std::abs(v_n_pre) < 1e-4) ? ContactRegime::GRAZING_NONDIFF : ContactRegime::SEPARATED;
        res.post_kinetic_energy_J = res.pre_kinetic_energy_J;
        return res;
    }

    // Assemble 3x3 Delassus collision matrix K_w in world frame:
    // v_rel_post = v_rel_pre + K_w * J_w
    Mat3 K_w = Mat3::Zero();
    if (!body_a.is_fixed_to_world && body_a.mass_kg > 1e-9) {
        const Mat3 rx_a = math::skew(r_a);
        K_w += (1.0 / body_a.mass_kg) * Mat3::Identity() - rx_a * I_a_w.inverse() * rx_a;
    }
    if (!body_b.is_fixed_to_world && body_b.mass_kg > 1e-9) {
        const Mat3 rx_b = math::skew(r_b);
        K_w += (1.0 / body_b.mass_kg) * Mat3::Identity() - rx_b * I_b_w.inverse() * rx_b;
    }

    // Transform to local contact basis [n, t1, t2]: K_c = R_c^T * K_w * R_c
    const Mat3 K_c = R_c.transpose() * K_w * R_c;
    const Vec3 v_c_pre = R_c.transpose() * v_rel_w;
    const double e = std::clamp(restitution, 0.0, 1.0);
    const double mu = std::max(0.0, friction_mu);

    // Target post-impact velocity under full sticking: v_c_target = [-e * v_n_pre, 0, 0]^T
    const Vec3 v_c_target(-e * v_c_pre(0), 0.0, 0.0);
    const Vec3 J_stick = K_c.ldlt().solve(v_c_target - v_c_pre);

    Vec3 J_c = Vec3::Zero();
    const double J_t_stick = std::hypot(J_stick(1), J_stick(2));
    if (J_stick(0) > 0.0 && J_t_stick <= mu * J_stick(0)) {
        J_c = J_stick;
        res.regime = (std::abs(J_t_stick - mu * J_stick(0)) < 1e-4 * J_stick(0))
                     ? ContactRegime::GRAZING_NONDIFF
                     : ContactRegime::STICKING;
    } else {
        // Sliding regime: solve PGS on the 3x3 local Delassus matrix K_c
        J_c(0) = std::max(0.0, -(1.0 + e) * v_c_pre(0) / std::max(1e-9, K_c(0, 0)));
        for (int iter = 0; iter < 25; ++iter) {
            const double vn_res = v_c_pre(0) + K_c(0, 1) * J_c(1) + K_c(0, 2) * J_c(2);
            J_c(0) = std::max(0.0, (-e * v_c_pre(0) - vn_res) / std::max(1e-9, K_c(0, 0)));

            const double vt1_res = v_c_pre(1) + K_c(1, 0) * J_c(0) + K_c(1, 2) * J_c(2);
            J_c(1) = -vt1_res / std::max(1e-9, K_c(1, 1));

            const double vt2_res = v_c_pre(2) + K_c(2, 0) * J_c(0) + K_c(2, 1) * J_c(1);
            J_c(2) = -vt2_res / std::max(1e-9, K_c(2, 2));

            const double jt_norm = std::hypot(J_c(1), J_c(2));
            const double jt_max  = mu * J_c(0);
            if (jt_norm > jt_max && jt_norm > 1e-12) {
                J_c(1) *= (jt_max / jt_norm);
                J_c(2) *= (jt_max / jt_norm);
            }
        }
        res.regime = ContactRegime::SLIDING;
    }

    const Vec3 J_w = R_c * J_c;
    res.impulse_on_a_w = J_w;
    res.impulse_on_b_w = -J_w;
    res.normal_impulse_Ns = J_c(0);
    res.tangential_impulse_Ns = Vec2(J_c(1), J_c(2));

    // Apply impulses to Body A (+J_w) and Body B (-J_w) about their respective CoMs
    if (!body_a.is_fixed_to_world && body_a.mass_kg > 1e-9) {
        const Vec3 v_com_a_post = v_com_a_pre + J_w / body_a.mass_kg;
        const Vec3 omega_a_w_post = omega_a_w_pre + I_a_w.inverse() * (r_a.cross(J_w));
        body_a.omega_b = body_a.rot_wb.transpose() * omega_a_w_post;
        body_a.vel_b   = body_a.rot_wb.transpose() * v_com_a_post - body_a.omega_b.cross(body_a.com_local_m);
    }
    if (!body_b.is_fixed_to_world && body_b.mass_kg > 1e-9) {
        const Vec3 v_com_b_post = v_com_b_pre - J_w / body_b.mass_kg;
        const Vec3 omega_b_w_post = omega_b_w_pre - I_b_w.inverse() * (r_b.cross(J_w));
        body_b.omega_b = body_b.rot_wb.transpose() * omega_b_w_post;
        body_b.vel_b   = body_b.rot_wb.transpose() * v_com_b_post - body_b.omega_b.cross(body_b.com_local_m);
    }

    const Vec3 v_com_a_post = body_a.com_velocity_world();
    const Vec3 v_com_b_post = body_b.com_velocity_world();
    const Vec3 omega_a_w_post = body_a.rot_wb * body_a.omega_b;
    const Vec3 omega_b_w_post = body_b.rot_wb * body_b.omega_b;

    const Vec3 P_post = (body_a.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(body_a.mass_kg * v_com_a_post))
                      + (body_b.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(body_b.mass_kg * v_com_b_post));
    const Vec3 L_post = (body_a.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(p_com_a.cross(body_a.mass_kg * v_com_a_post) + I_a_w * omega_a_w_post))
                      + (body_b.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(p_com_b.cross(body_b.mass_kg * v_com_b_post) + I_b_w * omega_b_w_post));

    res.post_kinetic_energy_J = body_a.kinetic_energy() + body_b.kinetic_energy();
    res.linear_momentum_err = P_post - P_pre;
    res.angular_momentum_err = L_post - L_pre;
    return res;
}

MultiContactPGSResult ContactMechanicsSolver::solve_multi_contact_pgs(
    const Vec3& chassis_pos_w,
    const Quat& chassis_q_wb,
    const Vec3& chassis_v_b,
    const Vec3& chassis_omega_b,
    double chassis_mass_kg,
    const Mat3& chassis_inertia_b,
    const std::vector<ContactManifoldPoint>& contacts,
    double dt_s,
    int max_iterations,
    double tol) noexcept
{
    MultiContactPGSResult res{};
    const size_t nc = contacts.size();
    res.impulses_w.assign(nc, Vec3::Zero());
    res.normal_impulses_Ns.assign(nc, 0.0);
    res.regimes.assign(nc, ContactRegime::SEPARATED);
    if (nc == 0) return res;

    const Mat3 R_wb = chassis_q_wb.toRotationMatrix();
    const Mat3 I_inv_b = chassis_inertia_b.inverse();
    const double m_inv = 1.0 / std::max(1.0, chassis_mass_kg);

    const int dim = static_cast<int>(3 * nc);
    Eigen::MatrixXd J_c(dim, 6);
    Eigen::VectorXd v_bias(dim);
    std::vector<Mat3> R_bases(nc);

    for (size_t i = 0; i < nc; ++i) {
        Vec3 n_w, t1_w, t2_w;
        build_contact_basis(contacts[i].normal_w, n_w, t1_w, t2_w);
        R_bases[i].col(0) = n_w;
        R_bases[i].col(1) = t1_w;
        R_bases[i].col(2) = t2_w;

        const Vec3 r_b = R_wb.transpose() * (contacts[i].point_w - chassis_pos_w);
        for (int axis = 0; axis < 3; ++axis) {
            const Vec3 u_b = R_wb.transpose() * R_bases[i].col(axis);
            J_c.block<1, 3>(static_cast<int>(3 * i + axis), 0) = u_b.transpose();
            J_c.block<1, 3>(static_cast<int>(3 * i + axis), 3) = (r_b.cross(u_b)).transpose();
        }

        const Vec3 v_pt_b = chassis_v_b + chassis_omega_b.cross(r_b);
        const Vec3 n_b = R_wb.transpose() * n_w;
        const double vn_pre = n_b.dot(v_pt_b);
        const double e_rest = (vn_pre < -0.25) ? std::clamp(contacts[i].restitution, 0.0, 1.0) : 0.0;
        const double pen_vel = (dt_s > 1e-6 && contacts[i].penetration_m > 0.0)
                               ? (contacts[i].baumgarte_erp * contacts[i].penetration_m / dt_s)
                               : 0.0;
        v_bias(static_cast<int>(3 * i + 0)) = e_rest * vn_pre - pen_vel;
        v_bias(static_cast<int>(3 * i + 1)) = 0.0;
        v_bias(static_cast<int>(3 * i + 2)) = 0.0;
    }

    Mat6 M_inv = Mat6::Zero();
    M_inv.block<3, 3>(0, 0) = m_inv * Mat3::Identity();
    M_inv.block<3, 3>(3, 3) = I_inv_b;

    // Delassus matrix W = J_c * M^{-1} * J_c^T
    const Eigen::MatrixXd W = J_c * M_inv * J_c.transpose();
    Vec6 v_gen;
    v_gen.head<3>() = chassis_v_b;
    v_gen.tail<3>() = chassis_omega_b;
    const Eigen::VectorXd b_rhs = J_c * v_gen + v_bias;

    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(dim);
    int iter = 0;
    for (; iter < max_iterations; ++iter) {
        double max_delta = 0.0;
        for (size_t i = 0; i < nc; ++i) {
            const int r0 = static_cast<int>(3 * i);
            // 1. Normal impulse update with Signorini non-negativity lambda_n >= 0
            const double w_nn = std::max(1e-9, W(r0, r0));
            const double res_n = b_rhs(r0) + W.row(r0).dot(lambda) - W(r0, r0) * lambda(r0);
            const double ln_new = std::max(0.0, -res_n / w_nn);
            max_delta = std::max(max_delta, std::abs(ln_new - lambda(r0)));
            lambda(r0) = ln_new;

            // 2. Tangential impulses (t1, t2) with 3D Coulomb cone projection
            const int r1 = r0 + 1;
            const int r2 = r0 + 2;
            const double w_t1 = std::max(1e-9, W(r1, r1));
            const double res_t1 = b_rhs(r1) + W.row(r1).dot(lambda) - W(r1, r1) * lambda(r1);
            double lt1 = -res_t1 / w_t1;

            lambda(r1) = lt1;
            const double w_t2 = std::max(1e-9, W(r2, r2));
            const double res_t2 = b_rhs(r2) + W.row(r2).dot(lambda) - W(r2, r2) * lambda(r2);
            double lt2 = -res_t2 / w_t2;

            const double mu = std::max(0.0, contacts[i].friction_mu);
            const double lt_max = mu * ln_new;
            const double lt_norm = std::hypot(lt1, lt2);
            if (lt_norm > lt_max && lt_norm > 1e-12) {
                lt1 *= (lt_max / lt_norm);
                lt2 *= (lt_max / lt_norm);
            }
            max_delta = std::max(max_delta, std::hypot(lt1 - lambda(r1), lt2 - lambda(r2)));
            lambda(r1) = lt1;
            lambda(r2) = lt2;
        }
        if (max_delta < tol) {
            ++iter;
            break;
        }
    }

    res.iterations = iter;
    res.generalized_chassis_impulse_b = J_c.transpose() * lambda;

    // Compute Signorini complementarity error and contact regime diagnostics
    const Eigen::VectorXd v_post = W * lambda + b_rhs;
    double max_comp_err = 0.0;
    for (size_t i = 0; i < nc; ++i) {
        const int r0 = static_cast<int>(3 * i);
        const double ln = lambda(r0);
        const double vn = v_post(r0);
        const double lt_norm = std::hypot(lambda(r0 + 1), lambda(r0 + 2));
        const double mu = std::max(0.0, contacts[i].friction_mu);

        // Complementarity residual: min(lambda_n, v_n_post) == 0
        const double comp_err = std::abs(std::min(ln, vn));
        max_comp_err = std::max(max_comp_err, comp_err);

        res.normal_impulses_Ns[i] = ln;
        res.impulses_w[i] = R_bases[i] * Vec3(ln, lambda(r0 + 1), lambda(r0 + 2));

        if (ln <= 1e-8) {
            res.regimes[i] = (std::abs(vn) < 1e-3) ? ContactRegime::GRAZING_NONDIFF : ContactRegime::SEPARATED;
        } else if (lt_norm < 0.98 * mu * ln) {
            res.regimes[i] = ContactRegime::STICKING;
        } else if (lt_norm < 1.0001 * mu * ln && std::abs(lt_norm - mu * ln) < 1e-3 * ln) {
            res.regimes[i] = ContactRegime::SLIDING;
        } else {
            res.regimes[i] = ContactRegime::GRAZING_NONDIFF;
        }
        if (res.regimes[i] == ContactRegime::GRAZING_NONDIFF) {
            res.has_nondiff_grazing = true;
        }
    }
    res.max_complementarity_error = max_comp_err;
    res.converged = (max_comp_err < 1e-4);
    return res;
}

void ContactMechanicsSolver::append_chassis_bottoming_contacts(
    const Vec3& chassis_pos_w,
    const Quat& chassis_q_wb,
    double ground_z_w,
    const Vec3& ground_normal_w,
    std::vector<ContactManifoldPoint>& out_contacts) noexcept
{
    // 4 Skidblock / undertray corner points in Chassis CoG Frame F_B (z = -0.245 m below CoG)
    static const std::array<Vec3, 4> kSkidPointsB = {
        Vec3( 0.90,  0.30, -0.245),
        Vec3( 0.90, -0.30, -0.245),
        Vec3(-0.75,  0.30, -0.245),
        Vec3(-0.75, -0.30, -0.245)
    };
    const Mat3 R_wb = chassis_q_wb.toRotationMatrix();
    for (const auto& pt_b : kSkidPointsB) {
        const Vec3 pt_w = chassis_pos_w + R_wb * pt_b;
        const double pen = ground_z_w - pt_w.z();
        if (pen > 0.0) {
            ContactManifoldPoint cp{};
            cp.contact_class = ContactClass::CHASSIS_GROUND_BOTTOMING;
            cp.point_w = pt_w;
            cp.normal_w = ground_normal_w;
            cp.penetration_m = pen;
            cp.restitution = 0.05;
            cp.friction_mu = 0.35;
            cp.baumgarte_erp = 0.35;
            out_contacts.push_back(cp);
        }
    }
}

} // namespace ter26
