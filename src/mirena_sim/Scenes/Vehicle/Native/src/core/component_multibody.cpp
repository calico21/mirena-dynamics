/**
 * @file component_multibody.cpp
 * @brief Implementation of the Component-Based Rigid-Body Multibody Dynamics Architecture,
 *        Holonomic Constraints, DAE Saddle-Point Assembler, Manifold Projection,
 *        Physical 3D ARB, Dynamic Component Inertias, and ANSYS Structural Load Exporter.
 */

#include "ter26/multibody/component_multibody.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace ter26::multibody {

// ============================================================================
// 1. Holonomic Mechanical Constraints (Position, Velocity Jacobian, Convective)
// ============================================================================

void SphericalJointConstraint::evaluate(
    const RigidBodyComponent& body_a,
    const RigidBodyComponent& body_b,
    Eigen::VectorXd& phi_out,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
    Eigen::VectorXd& gamma_conv_out) const noexcept
{
    phi_out.resize(3);
    J_a_out.setZero(3, 6);
    J_b_out.setZero(3, 6);
    gamma_conv_out.setZero(3);

    const Vec3 r_a_w = body_a.rot_wb * r_a_;
    const Vec3 r_b_w = body_b.rot_wb * r_b_;
    const Vec3 p_a_w = body_a.pos_w + r_a_w;
    const Vec3 p_b_w = body_b.pos_w + r_b_w;

    // Position residual in World Frame F_W: Phi = p_A^W - p_B^W = 0
    phi_out = p_a_w - p_b_w;

    // Velocity of attachment point in World Frame given body-frame twist V_i^B = [v_i^B; omega_i^B]:
    // dp_A^W/dt = R_{W,A} * (v_A^B - [r_A^B]_x * omega_A^B)
    if (!body_a.is_fixed_to_world) {
        J_a_out.block<3, 3>(0, 0) = body_a.rot_wb;
        J_a_out.block<3, 3>(0, 3) = -body_a.rot_wb * math::skew(r_a_);
    }
    if (!body_b.is_fixed_to_world) {
        J_b_out.block<3, 3>(0, 0) = -body_b.rot_wb;
        J_b_out.block<3, 3>(0, 3) =  body_b.rot_wb * math::skew(r_b_);
    }

    // Acceleration of attachment point in World Frame:
    // d^2 p_A^W / dt^2 = R_{W,A} * (dv_A^B/dt - [r_A^B]_x domega_A^B/dt)
    //                  + R_{W,A} * (omega_A^B x v_A^B + omega_A^B x (omega_A^B x r_A^B))
    Vec3 a_conv_a_w = Vec3::Zero();
    Vec3 a_conv_b_w = Vec3::Zero();
    if (!body_a.is_fixed_to_world) {
        a_conv_a_w = body_a.rot_wb * (
            body_a.omega_b.cross(body_a.vel_b) + body_a.omega_b.cross(body_a.omega_b.cross(r_a_)));
    }
    if (!body_b.is_fixed_to_world) {
        a_conv_b_w = body_b.rot_wb * (
            body_b.omega_b.cross(body_b.vel_b) + body_b.omega_b.cross(body_b.omega_b.cross(r_b_)));
    }
    gamma_conv_out = a_conv_a_w - a_conv_b_w;
}

RevoluteJointConstraint::RevoluteJointConstraint(
    std::string joint_name,
    int id_a,
    int id_b,
    const Vec3& r_a_local,
    const Vec3& r_b_local,
    const Vec3& axis_a_local,
    const Vec3& axis_b_local)
    : name_(std::move(joint_name)), id_a_(id_a), id_b_(id_b),
      r_a_(r_a_local), r_b_(r_b_local),
      u_a_(axis_a_local.normalized()), u_b_(axis_b_local.normalized())
{
    // Construct two unit vectors n1_b_, n2_b_ orthogonal to u_b_ in Body B frame
    Vec3 ref = (std::abs(u_b_.x()) < 0.8) ? Vec3::UnitX() : Vec3::UnitY();
    n1_b_ = (u_b_.cross(ref)).normalized();
    n2_b_ = (u_b_.cross(n1_b_)).normalized();
}

void RevoluteJointConstraint::evaluate(
    const RigidBodyComponent& body_a,
    const RigidBodyComponent& body_b,
    Eigen::VectorXd& phi_out,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
    Eigen::VectorXd& gamma_conv_out) const noexcept
{
    phi_out.resize(5);
    J_a_out.setZero(5, 6);
    J_b_out.setZero(5, 6);
    gamma_conv_out.setZero(5);

    // 1. First 3 equations: Spherical constraint
    Eigen::VectorXd phi_sph(3), gamma_sph(3);
    Eigen::Matrix<double, Eigen::Dynamic, 6> Ja_sph(3, 6), Jb_sph(3, 6);
    SphericalJointConstraint sph(name_ + "_sph", id_a_, id_b_, r_a_, r_b_);
    sph.evaluate(body_a, body_b, phi_sph, Ja_sph, Jb_sph, gamma_sph);

    phi_out.head<3>()        = phi_sph;
    J_a_out.topRows<3>()     = Ja_sph;
    J_b_out.topRows<3>()     = Jb_sph;
    gamma_conv_out.head<3>() = gamma_sph;

    // 2. Equations 3 & 4: Hinge axis orthogonality:
    //    Phi_3 = (u_A^W)^T * n1_B^W = 0
    //    Phi_4 = (u_A^W)^T * n2_B^W = 0
    const Vec3 u_a_w  = body_a.rot_wb * u_a_;
    const Vec3 n1_b_w = body_b.rot_wb * n1_b_;
    const Vec3 n2_b_w = body_b.rot_wb * n2_b_;

    phi_out(3) = u_a_w.dot(n1_b_w);
    phi_out(4) = u_a_w.dot(n2_b_w);

    const Vec3 omega_a_w = body_a.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(body_a.rot_wb * body_a.omega_b);
    const Vec3 omega_b_w = body_b.is_fixed_to_world ? Vec3(Vec3::Zero()) : Vec3(body_b.rot_wb * body_b.omega_b);

    const Vec3 du_a_w  = omega_a_w.cross(u_a_w);
    const Vec3 dn1_b_w = omega_b_w.cross(n1_b_w);
    const Vec3 dn2_b_w = omega_b_w.cross(n2_b_w);

    // d/dt (u_a_w . n_k_w) = (omega_a_w x u_a_w) . n_k_w + u_a_w . (omega_b_w x n_k_w)
    //                      = (u_a_w x n_k_w) . omega_a_w - (u_a_w x n_k_w) . omega_b_w
    const Vec3 c1_w = u_a_w.cross(n1_b_w);
    const Vec3 c2_w = u_a_w.cross(n2_b_w);

    if (!body_a.is_fixed_to_world) {
        J_a_out.block<1, 3>(3, 3) = (body_a.rot_wb.transpose() * c1_w).transpose();
        J_a_out.block<1, 3>(4, 3) = (body_a.rot_wb.transpose() * c2_w).transpose();
    }
    if (!body_b.is_fixed_to_world) {
        J_b_out.block<1, 3>(3, 3) = -(body_b.rot_wb.transpose() * c1_w).transpose();
        J_b_out.block<1, 3>(4, 3) = -(body_b.rot_wb.transpose() * c2_w).transpose();
    }

    // Second time derivative convective terms:
    // d^2/dt^2 (u_a_w . n_k_w) = J * domega/dt + [omega_a_w x (omega_a_w x u_a_w)] . n_k_w
    //                          + 2 * (du_a_w . dn_k_w) + u_a_w . [omega_b_w x (omega_b_w x n_k_w)]
    const Vec3 d2u_a_conv  = omega_a_w.cross(du_a_w);
    const Vec3 d2n1_b_conv = omega_b_w.cross(dn1_b_w);
    const Vec3 d2n2_b_conv = omega_b_w.cross(dn2_b_w);

    gamma_conv_out(3) = d2u_a_conv.dot(n1_b_w) + 2.0 * du_a_w.dot(dn1_b_w) + u_a_w.dot(d2n1_b_conv);
    gamma_conv_out(4) = d2u_a_conv.dot(n2_b_w) + 2.0 * du_a_w.dot(dn2_b_w) + u_a_w.dot(d2n2_b_conv);
}

void FixedDistanceLinkConstraint::evaluate(
    const RigidBodyComponent& body_a,
    const RigidBodyComponent& body_b,
    Eigen::VectorXd& phi_out,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
    Eigen::VectorXd& gamma_conv_out) const noexcept
{
    phi_out.resize(1);
    J_a_out.setZero(1, 6);
    J_b_out.setZero(1, 6);
    gamma_conv_out.setZero(1);

    Eigen::VectorXd d_vec(3), gamma_sph(3);
    Eigen::Matrix<double, Eigen::Dynamic, 6> Ja_sph(3, 6), Jb_sph(3, 6);
    SphericalJointConstraint sph(name_ + "_pts", id_a_, id_b_, r_a_, r_b_);
    sph.evaluate(body_a, body_b, d_vec, Ja_sph, Jb_sph, gamma_sph);

    // Normalized distance constraint: Phi = (||d||^2 - L0^2) / (2 * L0) = 0 (units of meters)
    phi_out(0) = 0.5 * (d_vec.squaredNorm() - L0_ * L0_) / L0_;

    const Vec3 u_d = d_vec / L0_;
    J_a_out = u_d.transpose() * Ja_sph;
    J_b_out = u_d.transpose() * Jb_sph;

    // Relative velocity of endpoints in World Frame
    const Vec3 v_a_w = body_a.is_fixed_to_world ? Vec3::Zero() : body_a.point_velocity_world(r_a_);
    const Vec3 v_b_w = body_b.is_fixed_to_world ? Vec3::Zero() : body_b.point_velocity_world(r_b_);
    const Vec3 v_rel_w = v_a_w - v_b_w;

    // d^2 Phi / dt^2 = u_d^T * (d^2 d / dt^2) + ||v_rel||^2 / L0
    gamma_conv_out(0) = u_d.dot(gamma_sph) + v_rel_w.squaredNorm() / L0_;
}

// ============================================================================
// 2. ComponentMultibodySystem DAE Assembly, Manifold Projection & RK4 Stepper
// ============================================================================

int ComponentMultibodySystem::add_body(const RigidBodyComponent& body) {
    RigidBodyComponent b = body;
    b.id = static_cast<int>(bodies_.size());
    bodies_.push_back(b);
    return b.id;
}

void ComponentMultibodySystem::add_constraint(std::shared_ptr<MechanicalConstraint> constraint) {
    constraints_.push_back(std::move(constraint));
}

void ComponentMultibodySystem::add_spring_damper(const PointSpringDamperElement& elem) {
    springs_.push_back(elem);
}

double ComponentMultibodySystem::total_kinetic_energy() const noexcept {
    double T = 0.0;
    for (const auto& b : bodies_) {
        T += b.kinetic_energy();
    }
    return T;
}

double ComponentMultibodySystem::total_potential_energy() const noexcept {
    double V = 0.0;
    for (const auto& b : bodies_) {
        V += b.potential_energy(gravity_w_);
    }
    for (const auto& s : springs_) {
        if (s.body_a_id >= 0 && s.body_a_id < static_cast<int>(bodies_.size()) &&
            s.body_b_id >= 0 && s.body_b_id < static_cast<int>(bodies_.size())) {
            V += s.potential_energy(bodies_[s.body_a_id], bodies_[s.body_b_id]);
        }
    }
    return V;
}

Vec3 ComponentMultibodySystem::total_linear_momentum_world() const noexcept {
    Vec3 P = Vec3::Zero();
    for (const auto& b : bodies_) {
        P += b.linear_momentum_world();
    }
    return P;
}

Vec3 ComponentMultibodySystem::total_angular_momentum_world() const noexcept {
    Vec3 L = Vec3::Zero();
    for (const auto& b : bodies_) {
        L += b.angular_momentum_world();
    }
    return L;
}

MultibodyDAEResult ComponentMultibodySystem::evaluate_dae(
    double baumgarte_alpha,
    double baumgarte_beta) const noexcept
{
    MultibodyDAEResult res{};
    const int n_bodies = static_cast<int>(bodies_.size());
    const int n_dof    = 6 * n_bodies;

    int m_total = 0;
    for (const auto& c : constraints_) {
        m_total += c->num_equations();
    }

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_dof, n_dof);
    Eigen::VectorXd rhs_dyn = Eigen::VectorXd::Zero(n_dof);
    Eigen::VectorXd v_all   = Eigen::VectorXd::Zero(n_dof);

    // 1. Assemble block-diagonal Spatial Mass Matrix M and Coriolis + Gravity + External Wrenches
    for (int i = 0; i < n_bodies; ++i) {
        const auto& b = bodies_[i];
        SpatialVec V_i;
        V_i.head<3>() = b.vel_b;
        V_i.tail<3>() = b.omega_b;
        v_all.segment<6>(6 * i) = V_i;

        if (b.is_fixed_to_world) {
            M.block<6, 6>(6 * i, 6 * i) = SpatialMat::Identity() * 1e9;
            rhs_dyn.segment<6>(6 * i).setZero();
            continue;
        }

        const SpatialMat M_i = b.spatial_inertia_b();
        M.block<6, 6>(6 * i, 6 * i) = M_i;

        // Euler-Poincare Coriolis / centrifugal / gyroscopic wrench h_i^B
        const SpatialVec h_i = body_coriolis_wrench(b.mass_kg, b.com_local_m, b.inertia_com_b, V_i);

        // Gravity wrench in Body Frame F_i
        const Vec3 g_b = b.rot_wb.transpose() * gravity_w_;
        const Vec3 F_grav_b = b.mass_kg * g_b;
        const Vec3 M_grav_b = b.com_local_m.cross(F_grav_b);

        SpatialVec Q_i;
        Q_i.head<3>() = F_grav_b + b.applied_force_b;
        Q_i.tail<3>() = M_grav_b + b.applied_moment_b;

        rhs_dyn.segment<6>(6 * i) = Q_i - h_i;
    }

    // 2. Add Point-to-Point Spring-Damper Force Elements
    for (const auto& sd : springs_) {
        if (sd.body_a_id < 0 || sd.body_a_id >= n_bodies || sd.body_b_id < 0 || sd.body_b_id >= n_bodies) continue;
        const auto& ba = bodies_[sd.body_a_id];
        const auto& bb = bodies_[sd.body_b_id];
        const Vec3 p_a_w = ba.point_world(sd.r_a_local);
        const Vec3 p_b_w = bb.point_world(sd.r_b_local);
        const Vec3 v_a_w = ba.point_velocity_world(sd.r_a_local);
        const Vec3 v_b_w = bb.point_velocity_world(sd.r_b_local);

        const Vec3 d_w = p_b_w - p_a_w;
        const double L = d_w.norm();
        if (L < 1e-8) continue;
        const Vec3 u_w = d_w / L;
        const double dx = L - sd.free_length_m;
        const double dL_dt = (v_b_w - v_a_w).dot(u_w);

        // Tension force f_sd (> 0 pulls A toward B and B toward A)
        const double f_sd = sd.k_lin_N_m * dx + sd.k_cubic_N_m3 * (dx * dx * dx) + sd.preload_N + sd.c_damping_N_s_m * dL_dt;
        const Vec3 F_on_a_w =  f_sd * u_w;
        const Vec3 F_on_b_w = -f_sd * u_w;

        if (!ba.is_fixed_to_world) {
            const Vec3 F_a_b = ba.rot_wb.transpose() * F_on_a_w;
            rhs_dyn.segment<3>(6 * sd.body_a_id)     += F_a_b;
            rhs_dyn.segment<3>(6 * sd.body_a_id + 3) += sd.r_a_local.cross(F_a_b);
        }
        if (!bb.is_fixed_to_world) {
            const Vec3 F_b_b = bb.rot_wb.transpose() * F_on_b_w;
            rhs_dyn.segment<3>(6 * sd.body_b_id)     += F_b_b;
            rhs_dyn.segment<3>(6 * sd.body_b_id + 3) += sd.r_b_local.cross(F_b_b);
        }
    }

    // 3. Unconstrained fast path if m_total == 0
    if (m_total == 0) {
        res.generalized_accel = M.ldlt().solve(rhs_dyn);
        res.lagrange_multipliers.resize(0);
        res.constraint_pos_residual.resize(0);
        res.constraint_vel_residual.resize(0);
        res.constraint_accel_residual.resize(0);
        res.singular_values.resize(0);
        res.kinetic_energy_J   = total_kinetic_energy();
        res.potential_energy_J = total_potential_energy();
        res.total_energy_J     = res.kinetic_energy_J + res.potential_energy_J;
        res.solver_ok          = res.generalized_accel.allFinite();
        return res;
    }

    // 4. Assemble Global Constraint Jacobian J_Phi in R^{m x 6N}, Phi in R^m, gamma_conv in R^m
    Eigen::MatrixXd J_phi = Eigen::MatrixXd::Zero(m_total, n_dof);
    Eigen::VectorXd phi_all = Eigen::VectorXd::Zero(m_total);
    Eigen::VectorXd gamma_all = Eigen::VectorXd::Zero(m_total);

    int row_offset = 0;
    for (const auto& c : constraints_) {
        const int m_c = c->num_equations();
        const int ida = c->body_a_id();
        const int idb = c->body_b_id();
        Eigen::VectorXd phi_c(m_c), gamma_c(m_c);
        Eigen::Matrix<double, Eigen::Dynamic, 6> Ja(m_c, 6), Jb(m_c, 6);
        c->evaluate(bodies_[ida], bodies_[idb], phi_c, Ja, Jb, gamma_c);

        phi_all.segment(row_offset, m_c)   = phi_c;
        gamma_all.segment(row_offset, m_c) = gamma_c;
        if (!bodies_[ida].is_fixed_to_world) {
            J_phi.block(row_offset, 6 * ida, m_c, 6) += Ja;
        }
        if (!bodies_[idb].is_fixed_to_world) {
            J_phi.block(row_offset, 6 * idb, m_c, 6) += Jb;
        }
        row_offset += m_c;
    }

    const Eigen::VectorXd vel_res = J_phi * v_all;
    res.constraint_pos_residual = phi_all;
    res.constraint_vel_residual = vel_res;
    res.max_pos_drift = phi_all.lpNorm<Eigen::Infinity>();
    res.max_vel_drift = vel_res.lpNorm<Eigen::Infinity>();

    // SVD & Rank-Revealing QR diagnostics on J_Phi (§2.1)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_phi);
    const auto& sing_vals = svd.singularValues();
    res.singular_values = sing_vals;
    if (sing_vals.size() > 0 && sing_vals(0) > 1e-12) {
        const double s_max = sing_vals(0);
        const double s_min = sing_vals(sing_vals.size() - 1);
        res.jacobian_rcond = s_min / s_max;
        int rnk = 0;
        double min_active_s = s_max;
        for (int k = 0; k < sing_vals.size(); ++k) {
            if (sing_vals(k) > 1e-8 * s_max) {
                ++rnk;
                min_active_s = sing_vals(k);
            }
        }
        res.jacobian_rank = rnk;
        res.num_redundant_constraints = m_total - rnk;
        res.condition_number = s_max / std::max(1e-15, min_active_s);
        res.near_singular_configuration = (min_active_s < 1e-4 * s_max);

        // Identify redundant constraint row indices via Column-Pivoted Householder QR on J_Phi^T
        if (res.num_redundant_constraints > 0) {
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_t(J_phi.transpose());
            qr_t.setThreshold(1e-8);
            const auto& perm = qr_t.colsPermutation().indices();
            for (int k = rnk; k < m_total; ++k) {
                res.redundant_constraint_indices.push_back(perm(k));
            }
            std::sort(res.redundant_constraint_indices.begin(), res.redundant_constraint_indices.end());
        }
    }

    // 5. Solve Index-1 DAE via Schur Complement on Delassus Matrix W = J_Phi * M^{-1} * J_Phi^T
    //    M * dv/dt + J_Phi^T * lambda = rhs_dyn
    //    J_Phi * dv/dt = rhs_acc = -gamma_all - 2*alpha * vel_res - beta^2 * phi_all
    const Eigen::VectorXd rhs_acc = -gamma_all - (2.0 * baumgarte_alpha) * vel_res
                                  - (baumgarte_beta * baumgarte_beta) * phi_all;

    Eigen::LDLT<Eigen::MatrixXd> M_ldlt(M);
    const Eigen::VectorXd dv_free = M_ldlt.solve(rhs_dyn);
    const Eigen::MatrixXd Minv_Jt = M_ldlt.solve(J_phi.transpose());
    Eigen::MatrixXd W_delassus = J_phi * Minv_Jt;
    const Eigen::VectorXd w_rhs = J_phi * dv_free - rhs_acc;

    if (solver_mode_ == ConstraintSolverMode::SVD_MINIMUM_NORM) {
        // Minimum-norm truncated SVD pseudoinverse of Delassus matrix W (exact on redundant constraints)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_W(W_delassus, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto& sw = svd_W.singularValues();
        const double tol_w = (sw.size() > 0) ? (1e-10 * sw(0)) : 1e-12;
        Eigen::VectorXd inv_sw = Eigen::VectorXd::Zero(sw.size());
        for (int k = 0; k < sw.size(); ++k) {
            if (sw(k) > tol_w) inv_sw(k) = 1.0 / sw(k);
        }
        res.lagrange_multipliers = svd_W.matrixV() * (inv_sw.asDiagonal() * (svd_W.matrixU().transpose() * w_rhs));
    } else if (solver_mode_ == ConstraintSolverMode::QR_RANK_REVEALING) {
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_W(W_delassus);
        qr_W.setThreshold(1e-10);
        res.lagrange_multipliers = qr_W.solve(w_rhs);
    } else {
        // TIKHONOV_LDLT: Regularized Cholesky (W + eps * I)
        W_delassus.diagonal().array() += tikhonov_eps_;
        res.lagrange_multipliers = W_delassus.ldlt().solve(w_rhs);
    }
    res.generalized_accel = dv_free - Minv_Jt * res.lagrange_multipliers;

    // Acceleration-level constraint residual J_Phi * dv/dt + gamma_conv
    res.constraint_accel_residual = J_phi * res.generalized_accel + gamma_all;
    res.max_accel_residual = res.constraint_accel_residual.lpNorm<Eigen::Infinity>();

    // 6. Extract Joint Reaction Wrenches -J_{A,k}^T * lambda_k on Body A for each constraint
    row_offset = 0;
    res.joint_reaction_wrenches_on_a.resize(constraints_.size(), SpatialVec::Zero());
    for (size_t idx = 0; idx < constraints_.size(); ++idx) {
        const auto& c = constraints_[idx];
        const int m_c = c->num_equations();
        const int ida = c->body_a_id();
        const int idb = c->body_b_id();
        Eigen::VectorXd phi_c(m_c), gamma_c(m_c);
        Eigen::Matrix<double, Eigen::Dynamic, 6> Ja(m_c, 6), Jb(m_c, 6);
        c->evaluate(bodies_[ida], bodies_[idb], phi_c, Ja, Jb, gamma_c);
        const Eigen::VectorXd lam_c = res.lagrange_multipliers.segment(row_offset, m_c);
        res.joint_reaction_wrenches_on_a[idx] = -Ja.transpose() * lam_c;
        row_offset += m_c;
    }

    res.kinetic_energy_J   = total_kinetic_energy();
    res.potential_energy_J = total_potential_energy();
    res.total_energy_J     = res.kinetic_energy_J + res.potential_energy_J;
    res.solver_ok          = res.generalized_accel.allFinite() && res.lagrange_multipliers.allFinite();
    return res;
}

void ComponentMultibodySystem::project_onto_constraint_manifold(int max_iters, double tol, bool use_mass_metric) noexcept {
    if (constraints_.empty()) return;
    const int n_bodies = static_cast<int>(bodies_.size());
    const int n_dof    = 6 * n_bodies;
    int m_total = 0;
    for (const auto& c : constraints_) m_total += c->num_equations();
    if (m_total == 0) return;

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_dof, n_dof);
    for (int i = 0; i < n_bodies; ++i) {
        if (bodies_[i].is_fixed_to_world) {
            M.block<6, 6>(6 * i, 6 * i) = SpatialMat::Identity() * 1e9;
        } else if (use_mass_metric) {
            M.block<6, 6>(6 * i, 6 * i) = bodies_[i].spatial_inertia_b();
        } else {
            M.block<6, 6>(6 * i, 6 * i) = SpatialMat::Identity();
        }
    }
    Eigen::LDLT<Eigen::MatrixXd> M_ldlt(M);

    auto solve_projection_step = [&](const Eigen::MatrixXd& J_phi, const Eigen::VectorXd& res_vec) -> Eigen::VectorXd {
        const Eigen::MatrixXd Minv_Jt = M_ldlt.solve(J_phi.transpose());
        Eigen::MatrixXd W = J_phi * Minv_Jt;
        if (solver_mode_ == ConstraintSolverMode::SVD_MINIMUM_NORM) {
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_W(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const auto& sw = svd_W.singularValues();
            const double tol_w = (sw.size() > 0) ? (1e-10 * sw(0)) : 1e-12;
            Eigen::VectorXd inv_sw = Eigen::VectorXd::Zero(sw.size());
            for (int k = 0; k < sw.size(); ++k) {
                if (sw(k) > tol_w) inv_sw(k) = 1.0 / sw(k);
            }
            const Eigen::VectorXd lam = svd_W.matrixV() * (inv_sw.asDiagonal() * (svd_W.matrixU().transpose() * res_vec));
            return -Minv_Jt * lam;
        }
        W.diagonal().array() += 1e-11;
        return -Minv_Jt * W.ldlt().solve(res_vec);
    };

    // 1. Gauss-Newton Position Manifold Projection: Phi(q) -> 0
    for (int iter = 0; iter < max_iters; ++iter) {
        Eigen::MatrixXd J_phi = Eigen::MatrixXd::Zero(m_total, n_dof);
        Eigen::VectorXd phi_all = Eigen::VectorXd::Zero(m_total);
        int row_offset = 0;
        for (const auto& c : constraints_) {
            const int m_c = c->num_equations();
            const int ida = c->body_a_id();
            const int idb = c->body_b_id();
            Eigen::VectorXd phi_c(m_c), gamma_c(m_c);
            Eigen::Matrix<double, Eigen::Dynamic, 6> Ja(m_c, 6), Jb(m_c, 6);
            c->evaluate(bodies_[ida], bodies_[idb], phi_c, Ja, Jb, gamma_c);
            phi_all.segment(row_offset, m_c) = phi_c;
            if (!bodies_[ida].is_fixed_to_world) J_phi.block(row_offset, 6 * ida, m_c, 6) += Ja;
            if (!bodies_[idb].is_fixed_to_world) J_phi.block(row_offset, 6 * idb, m_c, 6) += Jb;
            row_offset += m_c;
        }
        if (phi_all.lpNorm<Eigen::Infinity>() < tol) break;

        const Eigen::VectorXd dq = solve_projection_step(J_phi, phi_all);
        for (int i = 0; i < n_bodies; ++i) {
            if (bodies_[i].is_fixed_to_world) continue;
            const Vec3 dp_b   = dq.segment<3>(6 * i);
            const Vec3 dpsi_b = dq.segment<3>(6 * i + 3);
            bodies_[i].pos_w += bodies_[i].rot_wb * dp_b;
            bodies_[i].rot_wb = math::project_to_so3(bodies_[i].rot_wb * math::exp_so3(dpsi_b));
        }
    }

    // 2. Velocity Manifold Projection: J_Phi(q) * v -> 0
    Eigen::MatrixXd J_phi = Eigen::MatrixXd::Zero(m_total, n_dof);
    Eigen::VectorXd v_all = Eigen::VectorXd::Zero(n_dof);
    for (int i = 0; i < n_bodies; ++i) {
        v_all.segment<3>(6 * i)     = bodies_[i].vel_b;
        v_all.segment<3>(6 * i + 3) = bodies_[i].omega_b;
    }
    int row_offset = 0;
    for (const auto& c : constraints_) {
        const int m_c = c->num_equations();
        const int ida = c->body_a_id();
        const int idb = c->body_b_id();
        Eigen::VectorXd phi_c(m_c), gamma_c(m_c);
        Eigen::Matrix<double, Eigen::Dynamic, 6> Ja(m_c, 6), Jb(m_c, 6);
        c->evaluate(bodies_[ida], bodies_[idb], phi_c, Ja, Jb, gamma_c);
        if (!bodies_[ida].is_fixed_to_world) J_phi.block(row_offset, 6 * ida, m_c, 6) += Ja;
        if (!bodies_[idb].is_fixed_to_world) J_phi.block(row_offset, 6 * idb, m_c, 6) += Jb;
        row_offset += m_c;
    }
    const Eigen::VectorXd vel_err = J_phi * v_all;
    if (vel_err.lpNorm<Eigen::Infinity>() > tol) {
        const Eigen::VectorXd dv = solve_projection_step(J_phi, vel_err);
        for (int i = 0; i < n_bodies; ++i) {
            if (bodies_[i].is_fixed_to_world) continue;
            bodies_[i].vel_b   += dv.segment<3>(6 * i);
            bodies_[i].omega_b += dv.segment<3>(6 * i + 3);
        }
    }
}

MultibodyDAEResult ComponentMultibodySystem::step_rk4_projected(double dt_s) noexcept {
    const int n_bodies = static_cast<int>(bodies_.size());
    const auto base_bodies = bodies_;

    // Only apply Baumgarte inside RK stages if BAUMGARTE_STABILIZATION is selected;
    // when manifold projection is used, unpenalized Index-1 DAE (alpha=0, beta=0) preserves 4th-order accuracy!
    const double stage_alpha = (stabilization_mode_ == ConstraintStabilizationMode::BAUMGARTE_STABILIZATION)
                               ? baumgarte_alpha_ : 0.0;
    const double stage_beta  = (stabilization_mode_ == ConstraintStabilizationMode::BAUMGARTE_STABILIZATION)
                               ? baumgarte_beta_ : 0.0;

    // Munthe-Kaas 4-stage Lie-Group RK4 on SE(3)^N:
    auto eval_stage = [&](const Eigen::VectorXd& xi, const Eigen::VectorXd& v,
                          Eigen::VectorXd& dxi_out, Eigen::VectorXd& dv_out) {
        bodies_ = base_bodies;
        for (int i = 0; i < n_bodies; ++i) {
            if (bodies_[i].is_fixed_to_world) continue;
            const Vec3 u_i   = xi.segment<3>(6 * i);
            const Vec3 psi_i = xi.segment<3>(6 * i + 3);
            bodies_[i].pos_w   = base_bodies[i].pos_w + u_i;
            bodies_[i].rot_wb  = base_bodies[i].rot_wb * math::exp_so3(psi_i);
            bodies_[i].vel_b   = v.segment<3>(6 * i);
            bodies_[i].omega_b = v.segment<3>(6 * i + 3);
        }
        const MultibodyDAEResult dae = evaluate_dae(stage_alpha, stage_beta);
        dv_out = dae.generalized_accel;
        dxi_out = Eigen::VectorXd::Zero(6 * n_bodies);
        for (int i = 0; i < n_bodies; ++i) {
            if (bodies_[i].is_fixed_to_world) continue;
            const Vec3 psi_i = xi.segment<3>(6 * i + 3);
            dxi_out.segment<3>(6 * i)     = bodies_[i].rot_wb * bodies_[i].vel_b;
            dxi_out.segment<3>(6 * i + 3) = math::right_jacobian_inv_so3(psi_i) * bodies_[i].omega_b;
        }
    };

    const Eigen::VectorXd xi0 = Eigen::VectorXd::Zero(6 * n_bodies);
    Eigen::VectorXd v0(6 * n_bodies);
    for (int i = 0; i < n_bodies; ++i) {
        v0.segment<3>(6 * i)     = base_bodies[i].vel_b;
        v0.segment<3>(6 * i + 3) = base_bodies[i].omega_b;
    }

    Eigen::VectorXd dxi1, dv1, dxi2, dv2, dxi3, dv3, dxi4, dv4;
    eval_stage(xi0, v0, dxi1, dv1);
    eval_stage(xi0 + 0.5 * dt_s * dxi1, v0 + 0.5 * dt_s * dv1, dxi2, dv2);
    eval_stage(xi0 + 0.5 * dt_s * dxi2, v0 + 0.5 * dt_s * dv2, dxi3, dv3);
    eval_stage(xi0 + dt_s * dxi3,       v0 + dt_s * dv3,       dxi4, dv4);

    const Eigen::VectorXd xi_rk = (dxi1 + 2.0 * dxi2 + 2.0 * dxi3 + dxi4) * (dt_s / 6.0);
    const Eigen::VectorXd v_rk  = v0 + (dv1 + 2.0 * dv2 + 2.0 * dv3 + dv4) * (dt_s / 6.0);

    bodies_ = base_bodies;
    for (int i = 0; i < n_bodies; ++i) {
        if (bodies_[i].is_fixed_to_world) continue;
        bodies_[i].pos_w   = base_bodies[i].pos_w + xi_rk.segment<3>(6 * i);
        bodies_[i].rot_wb  = math::project_to_so3(base_bodies[i].rot_wb * math::exp_so3(xi_rk.segment<3>(6 * i + 3)));
        bodies_[i].vel_b   = v_rk.segment<3>(6 * i);
        bodies_[i].omega_b = v_rk.segment<3>(6 * i + 3);
    }

    if (stabilization_mode_ == ConstraintStabilizationMode::MASS_METRIC_MANIFOLD_PROJECTION) {
        project_onto_constraint_manifold(6, 1e-12, true);
    } else if (stabilization_mode_ == ConstraintStabilizationMode::EUCLIDEAN_MANIFOLD_PROJECTION) {
        project_onto_constraint_manifold(6, 1e-12, false);
    }

    MultibodyDAEResult final_res = evaluate_dae(stage_alpha, stage_beta);
    for (int i = 0; i < n_bodies; ++i) {
        bodies_[i].accel_b = final_res.generalized_accel.segment<3>(6 * i);
        bodies_[i].alpha_b = final_res.generalized_accel.segment<3>(6 * i + 3);
    }
    return final_res;
}

// ============================================================================
// 3. Physical 3D U-Bar Anti-Roll Bar (ARB) Coupling Across Left & Right Rockers
// ============================================================================

void PhysicalARBParams::evaluate_coupled_arb(
    const CornerKinematicState& kin_left,
    const CornerKinematicState& kin_right,
    double& f_wheel_left_N,
    double& f_wheel_right_N,
    double& f_droplink_left_N,
    double& f_droplink_right_N,
    double& tau_bar_Nm,
    double& strain_energy_J) const noexcept
{
    const double R_arm = std::max(0.02, blade_lever_arm_m);
    const double dz_dl_L = kin_left.arb_droplink_deflection_m;
    const double dz_dl_R = kin_right.arb_droplink_deflection_m;

    // ARB blade angular twist on Left and Right ends: theta_arm = dz_droplink / R_arm
    const double theta_L = dz_dl_L / R_arm;
    const double theta_R = dz_dl_R / R_arm;
    const double delta_theta_bar = theta_L - theta_R;

    // Series compliance of torsion bar shaft (k_tau) and two ARB blades (k_blade * R_arm^2)
    const double k_tau_blades = 0.5 * blade_bending_stiffness_N_m * (R_arm * R_arm);
    const double k_tau_eff = (k_tau_blades > 1.0)
        ? ((torsion_bar_stiffness_Nm_rad * k_tau_blades) / (torsion_bar_stiffness_Nm_rad + k_tau_blades))
        : torsion_bar_stiffness_Nm_rad;

    tau_bar_Nm      = k_tau_eff * delta_theta_bar;
    strain_energy_J = 0.5 * k_tau_eff * (delta_theta_bar * delta_theta_bar);

    // Droplink axial force F_dl = tau_bar / R_arm (equal and opposite on Left and Right rockers!)
    f_droplink_left_N  =  tau_bar_Nm / R_arm;
    f_droplink_right_N = -tau_bar_Nm / R_arm;

    // Project droplink force through each rocker's kinematic Jacobian d(dz_droplink)/d(z_wheel)
    // to obtain the exact generalized wheel force Q_ARB = -dV_ARB / dz_s
    const double d_dl_dz_L = kin_left.darb_droplink_dz;
    const double d_dl_dz_R = kin_right.darb_droplink_dz;

    f_wheel_left_N  = f_droplink_left_N  * d_dl_dz_L;
    f_wheel_right_N = f_droplink_right_N * d_dl_dz_R;
}

// ============================================================================
// 4. Corner Suspension Component Inertias (UCA, LCA, Pushrod, Tie-Rod, Rocker)
// ============================================================================

void assemble_corner_component_dynamics(
    int corner_idx,
    const CornerHardpoints& hp,
    const SuspensionComponentInertias& comp_inertias,
    const CornerKinematicState& kin,
    double h_cg_m,
    const Vec3& p_chassis_w,
    const Mat3& R_wb,
    const Vec3& v_chassis_b,
    const Vec3& omega_chassis_b,
    double v_susp_jounce,
    const Vec3& g_w,
    Mat14& M_accum,
    Vec14& C_accum,
    Vec14& Q_grav_accum,
    double& T_components_out_J,
    double& V_grav_components_out_J,
    std::vector<RigidBodyComponent>* out_bodies) noexcept
{
    T_components_out_J      = 0.0;
    V_grav_components_out_J = 0.0;
    if (!comp_inertias.include_component_inertias) {
        return;
    }

    const char* c_names[4] = {"FL", "FR", "RL", "RR"};
    const std::string prefix = std::string(c_names[std::clamp(corner_idx, 0, 3)]) + "_";
    const Vec3 g_b = R_wb.transpose() * g_w;

    const Vec3 axle_offset(hp.axle_x_offset_from_cg_m, 0.0, -h_cg_m);
    const Vec3 p_lbj = kin.x_upright.head<3>();
    const Vec3 p_ubj = p_lbj + kin.R_upright * (hp.ubj - hp.lbj);
    const Vec3 p_po  = p_lbj + kin.R_upright * (hp.pushrod_out - hp.lbj);
    const Vec3 p_to  = p_lbj + kin.R_upright * (hp.tie_out - hp.lbj);

    // Velocity Jacobians of outboard upright points wrt wheel jounce z_s:
    // dp_pt / dz_s = dp_wc_dz + domega_u_dz x (p_pt - p_wc_axle)
    const Vec3 p_wc_axle = p_lbj + kin.R_upright * (hp.wheel_center - hp.lbj);
    const Vec3 dp_lbj_dz = kin.dp_wc_dz + kin.domega_u_dz.cross(p_lbj - p_wc_axle);
    const Vec3 dp_ubj_dz = kin.dp_wc_dz + kin.domega_u_dz.cross(p_ubj - p_wc_axle);
    const Vec3 dp_po_dz  = kin.dp_wc_dz + kin.domega_u_dz.cross(p_po  - p_wc_axle);
    const Vec3 dp_to_dz  = kin.dp_wc_dz + kin.domega_u_dz.cross(p_to  - p_wc_axle);

    Vec14 v_gen = Vec14::Zero();
    v_gen.segment<3>(0) = v_chassis_b;
    v_gen.segment<3>(3) = omega_chassis_b;
    v_gen(6 + corner_idx) = v_susp_jounce;

    // Helper lambda to add a link/component rigid body with CoM position r_cm_b,
    // translational Jacobian jp_cm = dr_cm_b / dz_s, and rotational Jacobian jw_cm = domega_c / dz_s
    auto add_component = [&](const std::string& comp_name, double mass_c, const Vec3& I_diag_c,
                             const Vec3& r_cm_b, const Vec3& jp_cm, const Vec3& jw_cm) {
        Mat3x14 J_v = Mat3x14::Zero();
        J_v.block<3, 3>(0, 0)  = Mat3::Identity();
        J_v.block<3, 3>(0, 3)  = -math::skew(r_cm_b);
        J_v.col(6 + corner_idx) = jp_cm;

        Mat3x14 J_w = Mat3x14::Zero();
        J_w.block<3, 3>(0, 3)  = Mat3::Identity();
        J_w.col(6 + corner_idx) = jw_cm;

        const Mat3 I_c = I_diag_c.asDiagonal();
        const Mat14 dM = mass_c * (J_v.transpose() * J_v) + J_w.transpose() * I_c * J_w;
        M_accum.noalias() += dM;
        T_components_out_J += 0.5 * v_gen.dot(dM * v_gen);

        // Gravitational generalized force Q_grav = J_v^T * (m_c * g_b) and potential energy V_grav = -m_c * g_w^T * p_cm_w
        const Vec3 F_grav_c_b = mass_c * g_b;
        Q_grav_accum.noalias() += J_v.transpose() * F_grav_c_b;
        const Vec3 p_cm_w = p_chassis_w + R_wb * r_cm_b;
        V_grav_components_out_J += -mass_c * g_w.dot(p_cm_w);

        // Convective linear & angular accelerations in F_B
        const Vec3 v_rel = jp_cm * v_susp_jounce;
        const Vec3 a_conv = omega_chassis_b.cross(v_chassis_b)
                          + omega_chassis_b.cross(omega_chassis_b.cross(r_cm_b))
                          + 2.0 * omega_chassis_b.cross(v_rel);
        const Vec3 omega_c = omega_chassis_b + jw_cm * v_susp_jounce;
        const Vec3 alpha_conv = omega_chassis_b.cross(jw_cm * v_susp_jounce);
        const Vec3 gyro_c = I_c * alpha_conv + omega_c.cross(I_c * omega_c);

        C_accum.noalias() += J_v.transpose() * (mass_c * a_conv) + J_w.transpose() * gyro_c;

        if (out_bodies != nullptr) {
            RigidBodyComponent b;
            b.id = static_cast<int>(out_bodies->size());
            b.name = prefix + comp_name;
            b.mass_kg = mass_c;
            b.com_local_m = Vec3::Zero();
            b.inertia_com_b = I_c;
            b.pos_w  = p_cm_w;
            b.rot_wb = R_wb;
            b.vel_b   = J_v * v_gen;
            b.omega_b = J_w * v_gen;
            out_bodies->push_back(b);
        }
    };

    // 1. Upper Wishbone (UCA): pivots about inboard axis (uca_in_f -> uca_in_r), CoM at midpoint to UBJ
    const Vec3 p_uca_in = 0.5 * (hp.uca_in_f + hp.uca_in_r);
    const Vec3 r_uca_cm = axle_offset + 0.5 * (p_uca_in + p_ubj);
    const Vec3 d_uca    = p_ubj - p_uca_in;
    const Vec3 jp_uca   = 0.5 * dp_ubj_dz;
    const Vec3 jw_uca   = d_uca.cross(dp_ubj_dz) / std::max(1e-4, d_uca.squaredNorm());
    add_component("UCA", comp_inertias.m_uca_kg, comp_inertias.I_uca_diag, r_uca_cm, jp_uca, jw_uca);

    // 2. Lower Wishbone (LCA): pivots about inboard axis (lca_in_f -> lca_in_r), CoM at midpoint to LBJ
    const Vec3 p_lca_in = 0.5 * (hp.lca_in_f + hp.lca_in_r);
    const Vec3 r_lca_cm = axle_offset + 0.5 * (p_lca_in + p_lbj);
    const Vec3 d_lca    = p_lbj - p_lca_in;
    const Vec3 jp_lca   = 0.5 * dp_lbj_dz;
    const Vec3 jw_lca   = d_lca.cross(dp_lbj_dz) / std::max(1e-4, d_lca.squaredNorm());
    add_component("LCA", comp_inertias.m_lca_kg, comp_inertias.I_lca_diag, r_lca_cm, jp_lca, jw_lca);

    // 3. Tie-Rod: connects tie_in to tie_out
    const Vec3 r_tr_cm = axle_offset + 0.5 * (hp.tie_in + p_to);
    const Vec3 d_tr    = p_to - hp.tie_in;
    const Vec3 jp_tr   = 0.5 * dp_to_dz;
    const Vec3 jw_tr   = d_tr.cross(dp_to_dz) / std::max(1e-4, d_tr.squaredNorm());
    add_component("TieRod", comp_inertias.m_tie_rod_kg, comp_inertias.I_tie_rod_diag, r_tr_cm, jp_tr, jw_tr);

    // 4. Rocker / Bellcrank: rotates about rocker_piv by theta_r(z_s)
    const Vec3 u_rock_axis = (hp.rocker_axis - hp.rocker_piv).normalized();
    const Mat3 R_rock = math::exp_so3(kin.rocker_angle_rad * u_rock_axis);
    const Vec3 p_rock_rod = hp.rocker_piv + R_rock * (hp.rocker_rod - hp.rocker_piv);
    const Vec3 p_rock_coi = hp.rocker_piv + R_rock * (hp.rocker_coi - hp.rocker_piv);
    const Vec3 r_rock_cm_local = 0.5 * ((p_rock_rod - hp.rocker_piv) + (p_rock_coi - hp.rocker_piv));
    const Vec3 r_rock_cm = axle_offset + hp.rocker_piv + r_rock_cm_local;
    const Vec3 jw_rock = u_rock_axis * kin.dtheta_rocker_dz;
    const Vec3 jp_rock = jw_rock.cross(r_rock_cm_local);
    add_component("Rocker", comp_inertias.m_rocker_kg, comp_inertias.I_rocker_diag, r_rock_cm, jp_rock, jw_rock);

    // 5. Pushrod / Pullrod: connects pushrod_out (p_po) to rocker_rod (p_rock_rod)
    const Vec3 dp_rock_rod_dz = jw_rock.cross(p_rock_rod - hp.rocker_piv);
    const Vec3 r_pr_cm = axle_offset + 0.5 * (p_po + p_rock_rod);
    const Vec3 d_pr    = p_rock_rod - p_po;
    const Vec3 jp_pr   = 0.5 * (dp_po_dz + dp_rock_rod_dz);
    const Vec3 jw_pr   = d_pr.cross(dp_rock_rod_dz - dp_po_dz) / std::max(1e-4, d_pr.squaredNorm());
    add_component("Pushrod", comp_inertias.m_pushrod_kg, comp_inertias.I_pushrod_diag, r_pr_cm, jp_pr, jw_pr);
}

// ============================================================================
// 5. Dynamic ANSYS Structural Load Extractor & Machine-Readable JSON Exporter
// ============================================================================

CornerStructuralLoadsANSYS compute_dynamic_structural_loads_ansys(
    int corner_idx,
    const CornerHardpoints& hp,
    const SuspensionComponentInertias& comp_inertias,
    double h_cg_m,
    double unsprung_mass_kg,
    const Mat3& upright_inertia_b,
    double wheel_spin_inertia,
    double wheel_omega_rad_s,
    const CornerKinematicState& kin,
    const SuspensionKinematicsSolver& kin_solver,
    double steering_angle_rad,
    double arb_droplink_force_N,
    const Vec3& F_tire_b,
    const Vec3& M_tire_at_wc_b,
    double brake_reaction_torque_Nm,
    double drive_upright_reaction_torque_Nm,
    const Vec3& a_chassis_b,
    const Vec3& alpha_chassis_b,
    const Vec3& omega_chassis_b,
    double dz_susp,
    double d2z_susp,
    const Vec3& g_b) noexcept
{
    CornerStructuralLoadsANSYS out{};
    const char* names[4] = {"FL", "FR", "RL", "RR"};
    out.corner_index = corner_idx;
    out.corner_name  = names[std::clamp(corner_idx, 0, 3)];

    const Vec3 p_lbj = kin.x_upright.head<3>();
    const Vec3 p_ubj = p_lbj + kin.R_upright * (hp.ubj - hp.lbj);
    const Vec3 p_po  = p_lbj + kin.R_upright * (hp.pushrod_out - hp.lbj);
    const Vec3 p_to  = p_lbj + kin.R_upright * (hp.tie_out - hp.lbj);
    const Vec3 p_wc  = p_lbj + kin.R_upright * (hp.wheel_center - hp.lbj);

    const double eff_steer = hp.is_steered ? std::clamp(steering_angle_rad, -0.55, 0.55) : 0.0;
    const Vec3 p_ti = hp.tie_in + Vec3(0.0, eff_steer * hp.steering_rack_travel_per_rad, 0.0);

    const Vec3 u_rock_axis = (hp.rocker_axis - hp.rocker_piv).normalized();
    const Mat3 R_rock = math::exp_so3(kin.rocker_angle_rad * u_rock_axis);
    const Vec3 p_rock_rod = hp.rocker_piv + R_rock * (hp.rocker_rod - hp.rocker_piv);
    const Vec3 p_rock_coi = hp.rocker_piv + R_rock * (hp.rocker_coi - hp.rocker_piv);
    const Vec3 p_rock_arb = hp.rocker_piv + R_rock * (hp.antiroll_out - hp.rocker_piv);

    // Unit link vectors pointing from Chassis/Rocker Inboard -> Upright Outboard
    const Vec3 u_lca_f = (p_lbj - hp.lca_in_f).normalized();
    const Vec3 u_lca_r = (p_lbj - hp.lca_in_r).normalized();
    const Vec3 u_uca_f = (p_ubj - hp.uca_in_f).normalized();
    const Vec3 u_uca_r = (p_ubj - hp.uca_in_r).normalized();
    const Vec3 u_tie   = (p_to  - p_ti).normalized();
    const Vec3 u_push  = (p_po  - p_rock_rod).normalized();
    const Vec3 u_spring= (p_rock_coi - hp.spring_in).normalized();

    // 1. Dynamic D'Alembert Inertial Wrench of Unsprung Corner + Outboard Link Mass in F_B
    const Vec3 r_wc_from_cg = kin.wheel_center_b - Vec3(0.0, 0.0, h_cg_m);
    const double m_eff_corner = unsprung_mass_kg + (comp_inertias.include_component_inertias
        ? 0.5 * (comp_inertias.m_uca_kg + comp_inertias.m_lca_kg + comp_inertias.m_pushrod_kg + comp_inertias.m_tie_rod_kg)
        : 0.0);

    const Vec3 a_corner_b = a_chassis_b
                          + alpha_chassis_b.cross(r_wc_from_cg)
                          + omega_chassis_b.cross(omega_chassis_b.cross(r_wc_from_cg))
                          + 2.0 * omega_chassis_b.cross(kin.dp_wc_dz * dz_susp)
                          + kin.dp_wc_dz * d2z_susp
                          + kin.d2p_wc_dz2 * (dz_susp * dz_susp);

    const Vec3 omega_u_b = omega_chassis_b + kin.domega_u_dz * dz_susp;
    const Vec3 alpha_u_b = alpha_chassis_b + kin.domega_u_dz * d2z_susp + omega_chassis_b.cross(kin.domega_u_dz * dz_susp);
    const Vec3 gyro_wheel_b = omega_u_b.cross(wheel_spin_inertia * wheel_omega_rad_s * kin.spindle_axis_b);

    // D'Alembert force & moment about wheel center (gravity - inertial rate of change of momentum)
    out.F_inertial_d_alembert_b = m_eff_corner * (g_b - a_corner_b);
    out.M_inertial_d_alembert_b = -(upright_inertia_b * alpha_u_b + omega_u_b.cross(upright_inertia_b * omega_u_b) + gyro_wheel_b);

    // Total hub bearing wrench (external tire wrench + upright-mounted brake/drivetrain reaction torque)
    out.F_hub_bearing_b = F_tire_b;
    out.M_hub_bearing_b = M_tire_at_wc_b + (brake_reaction_torque_Nm + drive_upright_reaction_torque_Nm) * kin.spindle_axis_b;

    // 2. Solve BOTH Quasi-Static (tire+brake/drive only) and Full Dynamic (+ D'Alembert inertial wrench) 6x6 Link Reactions!
    out.static_link_reactions = kin_solver.solve_joint_reactions(
        kin, steering_angle_rad, out.F_hub_bearing_b, out.M_hub_bearing_b, arb_droplink_force_N);

    const Vec3 F_total_dyn_b = out.F_hub_bearing_b + out.F_inertial_d_alembert_b;
    const Vec3 M_total_dyn_b = out.M_hub_bearing_b + out.M_inertial_d_alembert_b;
    out.dynamic_link_reactions = kin_solver.solve_joint_reactions(
        kin, steering_angle_rad, F_total_dyn_b, M_total_dyn_b, arb_droplink_force_N);

    const auto& dyn_r = out.dynamic_link_reactions;
    out.statics_conditioning_rcond = dyn_r.conditioning_rcond;
    out.statics_matrix_rank = (dyn_r.conditioning_rcond > 1e-8) ? 6 : 5;
    out.static_equilibrium_residual_N = out.static_link_reactions.equilibrium_residual_norm;

    // 3. Outboard Ball-Joint Forces on Upright in F_B (from the Full Dynamic D'Alembert Reaction Solve)
    //    Convention in link_reactions: f_link > 0 in tension (pulling upright toward inboard pickup, i.e., along -u_link)
    out.F_upright_ubj_b     = -dyn_r.f_uca_fore_N * u_uca_f - dyn_r.f_uca_aft_N * u_uca_r;
    out.F_upright_lbj_b     = -dyn_r.f_lca_fore_N * u_lca_f - dyn_r.f_lca_aft_N * u_lca_r;
    out.F_upright_pushrod_b = -dyn_r.f_pushrod_N  * u_push;
    out.F_upright_tie_rod_b = -dyn_r.f_tie_rod_N  * u_tie;

    // 4. Equal-and-Opposite Forces Applied TO the Chassis at Inboard Pickups in F_B
    out.F_chassis_uca_fore_b   =  dyn_r.f_uca_fore_N * u_uca_f;
    out.F_chassis_uca_aft_b    =  dyn_r.f_uca_aft_N  * u_uca_r;
    out.F_chassis_lca_fore_b   =  dyn_r.f_lca_fore_N * u_lca_f;
    out.F_chassis_lca_aft_b    =  dyn_r.f_lca_aft_N  * u_lca_r;
    out.F_chassis_tie_rod_in_b =  dyn_r.f_tie_rod_N  * u_tie;

    // Coilover top mount & ARB mount forces on chassis
    out.F_chassis_coilover_top_b = dyn_r.f_damper_coilover_N * u_spring;
    out.F_chassis_arb_mount_b    = Vec3(0.0, 0.0, dyn_r.f_arb_droplink_N);
    out.M_chassis_arb_mount_b    = Vec3(dyn_r.f_arb_droplink_N * 0.085, 0.0, 0.0);

    // Rocker pivot bearing force & moment on chassis (balancing pushrod + coilover + ARB on the rocker)
    out.F_chassis_rocker_pivot_b = dyn_r.rocker_pivot_force_b;
    const Vec3 r_rod_piv = p_rock_rod - hp.rocker_piv;
    const Vec3 r_coi_piv = p_rock_coi - hp.rocker_piv;
    const Vec3 r_arb_piv = p_rock_arb - hp.rocker_piv;
    const Vec3 M_rock_total = r_rod_piv.cross(dyn_r.f_pushrod_N * u_push)
                            + r_coi_piv.cross(-dyn_r.f_damper_coilover_N * u_spring)
                            + r_arb_piv.cross(Vec3(0.0, 0.0, -dyn_r.f_arb_droplink_N));
    // Component orthogonal to rocker hinge axis is reacted by the rocker pivot bearing
    out.M_chassis_rocker_pivot_b = M_rock_total - (M_rock_total.dot(u_rock_axis)) * u_rock_axis;

    // 5. Verify Exact 3D Dynamic Force AND Moment Equilibrium on the Upright about Wheel Center p_wc:
    //    Sum of link forces + F_hub_bearing + F_inertial_d_alembert == 0
    //    Sum of link moments + M_hub_bearing + M_inertial_d_alembert == 0
    const Vec3 F_upright_dyn_sum = out.F_upright_ubj_b + out.F_upright_lbj_b
                                 + out.F_upright_pushrod_b + out.F_upright_tie_rod_b
                                 + out.F_hub_bearing_b + out.F_inertial_d_alembert_b;
    const Vec3 M_upright_dyn_sum = (p_ubj - p_wc).cross(out.F_upright_ubj_b)
                                 + (p_lbj - p_wc).cross(out.F_upright_lbj_b)
                                 + (p_po  - p_wc).cross(out.F_upright_pushrod_b)
                                 + (p_to  - p_wc).cross(out.F_upright_tie_rod_b)
                                 + out.M_hub_bearing_b + out.M_inertial_d_alembert_b;
    out.dynamic_equilibrium_residual_N        = F_upright_dyn_sum.norm();
    out.dynamic_moment_equilibrium_residual_Nm = M_upright_dyn_sum.norm();

    return out;
}

bool export_ansys_structural_loads_json(
    const std::string& output_json_path,
    const std::string& load_case_name,
    double sim_time_s,
    const std::array<CornerStructuralLoadsANSYS, 4>& corner_loads,
    const std::array<CornerHardpoints, 4>& hardpoints,
    double h_cg_m)
{
    std::ofstream f(output_json_path);
    if (!f.good()) return false;

    auto vec_json = [](const Vec3& v) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
        return ss.str();
    };

    f << std::fixed << std::setprecision(4);
    f << "{\n";
    f << "  \"format\": \"TeR26_ANSYS_Structural_Loads_v2\",\n";
    f << "  \"coordinate_frame\": \"ISO_8855_Chassis_CoG_Frame_FB (X_fwd, Y_left, Z_up, SI_N_Nm_m)\",\n";
    f << "  \"load_case\": \"" << load_case_name << "\",\n";
    f << "  \"simulation_time_s\": " << sim_time_s << ",\n";
    f << "  \"corners\": {\n";

    for (int i = 0; i < 4; ++i) {
        const auto& cl = corner_loads[i];
        const auto& hp = hardpoints[i];
        const Vec3 off(hp.axle_x_offset_from_cg_m, 0.0, -h_cg_m);
        f << "    \"" << cl.corner_name << "\": {\n";
        f << "      \"equilibrium_residual_N\": " << cl.dynamic_equilibrium_residual_N << ",\n";
        f << "      \"moment_equilibrium_residual_Nm\": " << cl.dynamic_moment_equilibrium_residual_Nm << ",\n";
        f << "      \"statics_matrix_rank\": " << cl.statics_matrix_rank << ",\n";
        f << "      \"statics_conditioning_rcond\": " << cl.statics_conditioning_rcond << ",\n";
        f << "      \"chassis_attachments_FB\": {\n";
        f << "        \"UCA_fore\": {\"point_m\": " << vec_json(hp.uca_in_f + off) << ", \"force_N\": " << vec_json(cl.F_chassis_uca_fore_b) << "},\n";
        f << "        \"UCA_aft\":  {\"point_m\": " << vec_json(hp.uca_in_r + off) << ", \"force_N\": " << vec_json(cl.F_chassis_uca_aft_b) << "},\n";
        f << "        \"LCA_fore\": {\"point_m\": " << vec_json(hp.lca_in_f + off) << ", \"force_N\": " << vec_json(cl.F_chassis_lca_fore_b) << "},\n";
        f << "        \"LCA_aft\":  {\"point_m\": " << vec_json(hp.lca_in_r + off) << ", \"force_N\": " << vec_json(cl.F_chassis_lca_aft_b) << "},\n";
        f << "        \"TieRod_in\":{\"point_m\": " << vec_json(hp.tie_in   + off) << ", \"force_N\": " << vec_json(cl.F_chassis_tie_rod_in_b) << "},\n";
        f << "        \"Rocker_piv\":{\"point_m\": " << vec_json(hp.rocker_piv + off) << ", \"force_N\": " << vec_json(cl.F_chassis_rocker_pivot_b) << ", \"moment_Nm\": " << vec_json(cl.M_chassis_rocker_pivot_b) << "},\n";
        f << "        \"Coilover_top\":{\"point_m\": " << vec_json(hp.spring_in + off) << ", \"force_N\": " << vec_json(cl.F_chassis_coilover_top_b) << "},\n";
        f << "        \"ARB_mount\": {\"point_m\": " << vec_json(hp.antiroll_in + off) << ", \"force_N\": " << vec_json(cl.F_chassis_arb_mount_b) << ", \"moment_Nm\": " << vec_json(cl.M_chassis_arb_mount_b) << "}\n";
        f << "      },\n";
        f << "      \"upright_attachments_FB\": {\n";
        f << "        \"UBJ_force_N\": " << vec_json(cl.F_upright_ubj_b) << ",\n";
        f << "        \"LBJ_force_N\": " << vec_json(cl.F_upright_lbj_b) << ",\n";
        f << "        \"Pushrod_force_N\": " << vec_json(cl.F_upright_pushrod_b) << ",\n";
        f << "        \"TieRod_force_N\": " << vec_json(cl.F_upright_tie_rod_b) << ",\n";
        f << "        \"HubBearing_force_N\": " << vec_json(cl.F_hub_bearing_b) << ",\n";
        f << "        \"HubBearing_moment_Nm\": " << vec_json(cl.M_hub_bearing_b) << "\n";
        f << "      },\n";
        f << "      \"d_alembert_inertial_wrench_FB\": {\n";
        f << "        \"force_N\": " << vec_json(cl.F_inertial_d_alembert_b) << ",\n";
        f << "        \"moment_Nm\": " << vec_json(cl.M_inertial_d_alembert_b) << "\n";
        f << "      }\n";
        f << "    }" << (i < 3 ? "," : "") << "\n";
    }

    f << "  }\n";
    f << "}\n";
    f.close();
    return true;
}

} // namespace ter26::multibody
