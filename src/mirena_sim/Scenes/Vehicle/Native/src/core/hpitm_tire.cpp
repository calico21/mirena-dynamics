/**
 * @file hpitm_tire.cpp
 * @brief Implementation of the Hybrid Physics-Informed Tire Model (HPITM):
 *        - Full Pacejka Magic Formula 6.2 (Fx, Fy, Fz, Mx, My, Mz) pure & combined slip
 *        - Bounded symmetry-preserving neural parameter modulation
 *        - Unconditionally stable exponential-Euler relaxation-length slip dynamics
 *        - Elastoplastic 2D bristle static friction model for standstill slope holding
 */

#include "ter26/forces/tires/hpitm_tire.hpp"
#include <cmath>
#include <algorithm>

namespace ter26 {

void HPITMTireModel::evaluate_neural_modulation(
    double alpha_rad,
    double dfz,
    double gamma_rad,
    double& delta_lambda_muy,
    double& delta_lambda_kya) const noexcept
{
    if (!params_.enable_neural_correction) {
        delta_lambda_muy = 0.0;
        delta_lambda_kya = 0.0;
        return;
    }
    // Strictly even inputs in (alpha, gamma) guarantee exact antisymmetry Fy(-alpha, -gamma) = -Fy(alpha, gamma)
    const double a2 = std::min(alpha_rad * alpha_rad, 0.50);
    const double g2 = std::min(gamma_rad * gamma_rad, 0.05);
    const double feat1 = a2 * (1.0 + 0.5 * dfz);
    const double feat2 = g2;

    double sum_muy = 0.0;
    double sum_kya = 0.0;
    for (size_t i = 0; i < 8; ++i) {
        const double z = params_.w1_alpha_sq_dfz[i] * feat1 + params_.w1_gamma_sq[i] * feat2 + params_.b1[i];
        // SiLU (Swish) C^infinity activation: z / (1 + exp(-z))
        const double silu = z / (1.0 + std::exp(-std::clamp(z, -20.0, 20.0)));
        sum_muy += params_.w2_muy[i] * silu;
        sum_kya += params_.w2_kya[i] * silu;
    }
    delta_lambda_muy = params_.max_neural_bound * std::tanh(sum_muy);
    delta_lambda_kya = params_.max_neural_bound * std::tanh(sum_kya);
}

TireForceOutput HPITMTireModel::evaluate_steady_state(
    double kappa,
    double alpha_rad,
    double Fz_N,
    double gamma_rad,
    double vx_m_s,
    double friction_mult,
    double turn_rate_rad_s) const noexcept
{
    TireForceOutput out{};
    if (Fz_N <= 1e-4) {
        out.r_eff_m = params_.R0_m;
        return out;
    }

    const double Fz = std::clamp(Fz_N, 1.0, params_.max_Fz_N);
    out.Fz_N = Fz;

    const double Fz0_prime = params_.Fz0_N;
    const double dfz = (Fz - Fz0_prime) / Fz0_prime;
    const double Fz_norm = Fz / Fz0_prime;
    const double gamma = std::clamp(gamma_rad, -0.25, 0.25);

    // Effective rolling radius r_eff(Fz) and contact patch half-length a(Fz)
    const double defl_0 = Fz0_prime / std::max(params_.kz_N_m, 1000.0);
    const double defl_z = Fz / std::max(params_.kz_N_m, 1000.0);
    out.r_eff_m = params_.R0_m - defl_0 * (
        params_.dreff * std::atan(params_.breff * Fz_norm) + params_.freff * Fz_norm);
    out.contact_patch_half_length_m = std::clamp(
        std::sqrt(std::max(0.0, 2.0 * params_.R0_m * defl_z)), 0.015, 0.14);

    // Turn-slip curvature phi_t = -turn_rate / sqrt(vx^2 + v_reg^2)
    const double v_turn_reg = std::hypot(vx_m_s, params_.v_min_reg_m_s);
    const double phi_turn = -turn_rate_rad_s / v_turn_reg;
    const double zeta_turn = std::cos(std::atan(params_.B_y_phi * params_.R0_m * std::abs(phi_turn)));

    // Bounded neural modulation
    double d_muy = 0.0;
    double d_kya = 0.0;
    evaluate_neural_modulation(alpha_rad, dfz, gamma, d_muy, d_kya);

    // --- 1. Pure Longitudinal Force Fx0 (MF6.2) ---
    const double SHx = params_.pHx1 + params_.pHx2 * dfz;
    const double kx  = std::clamp(kappa, -0.99, 0.99) + SHx;
    const double Cx  = params_.pCx1 * params_.lambda_Cx;
    const double mux = std::max(0.1, (params_.pDx1 + params_.pDx2 * dfz) *
                        (1.0 - params_.pDx3 * gamma * gamma)) *
                        params_.lambda_mux * friction_mult;
    const double Dx  = mux * Fz;
    const double Kxk = Fz * (params_.pKx1 + params_.pKx2 * dfz) *
                       std::exp(params_.pKx3 * dfz) * params_.lambda_Kxk;
    const double Bx  = Kxk / (Cx * Dx + 1e-6);
    const double Ex  = std::min(1.0, (params_.pEx1 + params_.pEx2 * dfz + params_.pEx3 * dfz * dfz) *
                       (1.0 - params_.pEx4 * std::tanh(10.0 * kx)) * params_.lambda_Ex);
    const double SVx = Fz * (params_.pVx1 + params_.pVx2 * dfz) * params_.lambda_mux * friction_mult;
    const double Bx_kx = Bx * kx;
    const double Fx0 = Dx * std::sin(Cx * std::atan(Bx_kx - Ex * (Bx_kx - std::atan(Bx_kx)))) + SVx;

    // --- 2. Pure Lateral Force Fy0 (MF6.2 + Turn-Slip Reduction zeta_turn) ---
    const double SHy = (params_.pHy1 + params_.pHy2 * dfz) * gamma;
    const double ay  = std::tan(std::clamp(alpha_rad, -1.2, 1.2)) + SHy;
    const double Cy  = params_.pCy1 * params_.lambda_Cy;
    const double muy = std::max(0.1, (params_.pDy1 + params_.pDy2 * dfz) *
                        (1.0 - params_.pDy3 * gamma * gamma)) *
                        params_.lambda_muy * (1.0 + d_muy) * friction_mult;
    const double Dy  = zeta_turn * muy * Fz;
    const double Kya = std::abs(params_.pKy1) * Fz0_prime *
                       std::sin(params_.pKy4 * std::atan(Fz_norm / std::max(0.1, params_.pKy2))) *
                       (1.0 - params_.pKy3 * std::abs(gamma)) *
                       params_.lambda_Kya * (1.0 + d_kya);
    const double By  = Kya / (Cy * (muy * Fz) + 1e-6);
    const double Ey  = std::min(1.0, (params_.pEy1 + params_.pEy2 * dfz) *
                       (1.0 - (params_.pEy3 * gamma + params_.pEy4 * gamma) * std::tanh(10.0 * ay)) *
                       params_.lambda_Ey);
    const double SVy = Fz * (params_.pVy3 + params_.pVy4 * dfz) * gamma * params_.lambda_muy * friction_mult;
    const double By_ay = By * ay;
    // ISO 8855 sign convention: positive slip angle alpha > 0 produces negative Fy < 0
    const double Fy0 = -Dy * std::sin(Cy * std::atan(By_ay - Ey * (By_ay - std::atan(By_ay)))) + SVy;

    // --- 3. Combined Slip Weighting Functions Gxa, Gyk (MF6.2) ---
    const double alpha_s = std::tan(std::clamp(alpha_rad, -1.2, 1.2));
    const double Bxa = params_.rBx1 * std::cos(std::atan(params_.rBx2 * kappa));
    const double Cxa = params_.rCx1;
    const double Exa = params_.rEx1;
    const double Bxa_as = Bxa * alpha_s;
    const double Gxa = std::clamp(
        std::cos(Cxa * std::atan(Bxa_as - Exa * (Bxa_as - std::atan(Bxa_as)))),
        0.05, 1.0);

    const double Byk = params_.rBy1 * std::cos(std::atan(params_.rBy2 * alpha_s));
    const double Cyk = params_.rCy1;
    const double Eyk = params_.rEy1;
    const double Byk_ks = Byk * kappa;
    const double Gyk = std::clamp(
        std::cos(Cyk * std::atan(Byk_ks - Eyk * (Byk_ks - std::atan(Byk_ks)))),
        0.05, 1.0);

    out.Fx_N = Gxa * Fx0;
    out.Fy_N = Gyk * Fy0;

    // Strict Combined Friction-Ellipse Envelope: (Fx / Dx)^2 + (Fy / (muy*Fz))^2 <= 1
    const double Dx_lim = std::max(Dx + std::abs(SVx), 1e-6);
    const double Dy_lim = std::max(muy * Fz + std::abs(SVy), 1e-6);
    const double ell_u  = std::hypot(out.Fx_N / Dx_lim, out.Fy_N / Dy_lim);
    if (ell_u > 1.0) {
        out.Fx_N /= ell_u;
        out.Fy_N /= ell_u;
    }

    // --- 4. Tire Moments Mx, My, Mz & Turn-Slip Bore Torque (MF6.2) ---
    out.Mx_Nm = params_.R0_m * Fz * (
        params_.qSx1 * gamma - params_.qSx2 * gamma + params_.qSx3 * (out.Fy_N / Fz0_prime)
    ) * params_.lambda_Mx;

    // Rolling resistance moment My (opposes wheel rotation / forward velocity)
    const double v_ratio = std::abs(vx_m_s) / 15.0;
    const double f_rr = params_.qSy1 + params_.qSy2 * (std::abs(out.Fx_N) / Fz0_prime)
                      + params_.qSy3 * v_ratio + params_.qSy4 * (v_ratio * v_ratio * v_ratio * v_ratio);
    out.My_Nm = -params_.R0_m * Fz * f_rr * std::tanh(vx_m_s / 0.15) * params_.lambda_My;

    // Self-aligning moment Mz = -t(alpha_t, kappa) * Fy + s * Fx + Mz_turn_slip
    const double Bt = (params_.qBz1 + params_.qBz2 * dfz + params_.qBz3 * dfz * dfz) *
                      (1.0 + params_.qBz9 * std::abs(gamma));
    const double Ct = params_.qCz1;
    const double Dt = Fz * (params_.R0_m / Fz0_prime) *
                      (params_.qDz1 + params_.qDz2 * dfz) * (1.0 + params_.qDz3 * std::abs(gamma));
    const double Et = std::min(1.0, params_.qEz1 + params_.qEz2 * dfz);
    const double alpha_eq = std::sqrt(alpha_s * alpha_s + (Kxk / std::max(Kya, 100.0)) * (Kxk / std::max(Kya, 100.0)) * kappa * kappa);
    const double Bt_aeq = Bt * alpha_eq;
    const double trail_t = Dt * std::cos(Ct * std::atan(Bt_aeq - Et * (Bt_aeq - std::atan(Bt_aeq)))) * std::cos(alpha_rad);
    const double s_arm = params_.R0_m * (params_.sSz1 + params_.sSz2 * (out.Fy_N / Fz0_prime));

    // Turn-slip scrubbing / parking bore torque: Mz_turn = -(2/3) * muy * Fz * R_patch * tanh(...)
    const double R_patch = std::sqrt(out.contact_patch_half_length_m * (0.5 * params_.width_m));
    const double Mz_bore_max = (2.0 / 3.0) * muy * Fz * R_patch;
    out.Mz_turn_slip_Nm = -Mz_bore_max * std::tanh(params_.B_y_phi * R_patch * (-phi_turn))
                          - params_.c_bore_Nm_s_rad * (R_patch * R_patch / (0.08 * 0.08)) * turn_rate_rad_s;
    out.Mz_Nm = (-trail_t * out.Fy_N + s_arm * out.Fx_N) * params_.lambda_Mz + out.Mz_turn_slip_Nm;

    out.friction_ellipse_limit_N = std::max(Dx_lim, Dy_lim);
    out.friction_utilization = std::min(1.0, ell_u);
    out.kappa_steady = kappa;
    out.alpha_steady_rad = alpha_rad;
    out.kappa_next = kappa;
    out.alpha_next_rad = alpha_rad;

    return out;
}

TireForceOutput HPITMTireModel::evaluate_dynamic(
    double penetration_m,
    double penetration_vel_m_s,
    double vx_contact_m_s,
    double vy_contact_m_s,
    double omega_wheel_rad_s,
    double gamma_rad,
    double kappa_dyn_curr,
    double alpha_dyn_curr,
    const Vec2& bristle_curr_m,
    double dt_s,
    double friction_mult,
    bool differentiable_softplus,
    double turn_rate_rad_s,
    const TireCarcassThermalState* carcass_state_in) const noexcept
{
    TireForceOutput out{};
    if (carcass_state_in != nullptr) {
        out.carcass_thermal_next = *carcass_state_in;
    }

    const double pen = differentiable_softplus
        ? math::smooth_pos(penetration_m, 500.0)
        : std::max(0.0, penetration_m);

    if (pen <= 1e-8) {
        out.r_eff_m = params_.R0_m;
        // Exponentially decay transient slip and bristle states during wheel liftoff
        const double decay = std::exp(-dt_s / 0.02);
        out.kappa_next = kappa_dyn_curr * decay;
        out.alpha_next_rad = alpha_dyn_curr * decay;
        out.bristle_next_m = bristle_curr_m * decay;
        out.carcass_thermal_next.carcass_defl_m *= decay;
        out.carcass_thermal_next.carcass_vel_m_s *= decay;
        out.carcass_thermal_next.bore_twist_rad *= decay;
        return out;
    }

    // 1. Unilateral vertical normal force Fz with penetration-gated damping (never negative on liftoff)
    const double k_camber = 1.0 - params_.camber_stiffness_factor * (gamma_rad * gamma_rad);
    const double f_elastic = params_.kz_N_m * k_camber * pen + params_.kz_nl_N_m2 * (pen * pen);
    const double gate = pen / (pen + 0.0015);
    const double f_raw = f_elastic + params_.cz_N_s_m * gate * penetration_vel_m_s;
    const double Fz = std::clamp(
        differentiable_softplus ? math::smooth_pos(f_raw, 10.0) : std::max(0.0, f_raw),
        0.0, params_.max_Fz_N);

    if (Fz <= 1e-4) {
        out.r_eff_m = params_.R0_m;
        return out;
    }

    // Optional thermal grip factor lambda_T in [1 - max_thermal_penalty, 1.0]
    double thermal_mult = 1.0;
    if (params_.enable_thermal_modulation && carcass_state_in != nullptr) {
        const double dT = (carcass_state_in->T_surface_C - params_.T_opt_C) /
                          std::max(5.0, params_.dT_grip_halfwidth_C);
        thermal_mult = 1.0 - params_.max_thermal_penalty * (1.0 - std::exp(-dT * dT));
    }
    const double eff_friction_mult = friction_mult * thermal_mult;

    const double Fz_norm = Fz / params_.Fz0_N;
    const double defl_0  = params_.Fz0_N / std::max(params_.kz_N_m, 1000.0);
    const double r_eff   = params_.R0_m - defl_0 * (
        params_.dreff * std::atan(params_.breff * Fz_norm) + params_.freff * Fz_norm);

    // 2. Smooth low-speed regularized kinematic slip ratios
    const double v_reg = std::sqrt(vx_contact_m_s * vx_contact_m_s + params_.v_min_reg_m_s * params_.v_min_reg_m_s);
    const double vs_x  = r_eff * omega_wheel_rad_s - vx_contact_m_s;
    const double kappa_ss = std::clamp(vs_x / v_reg, -0.95, 0.95);
    const double alpha_ss = std::atan2(vy_contact_m_s, v_reg);

    double kappa_next = kappa_ss;
    double alpha_next = alpha_ss;
    TireForceOutput rolling{};

    // 3. Mutually exclusive Transient Formulation (1st-order Relaxation vs. 2nd-order Belt/Carcass Oscillator)
    if (params_.transient_mode == TransientTireMode::DYNAMIC_CARCASS_2ND_ORDER && carcass_state_in != nullptr) {
        // In 2nd-order carcass mode, the belt deflects by e_c = (e_cx, e_cy) with rate v_c = de_c/dt.
        // Effective contact patch slip velocity includes belt velocity relative to rim:
        const Vec2 e_c = carcass_state_in->carcass_defl_m;
        const Vec2 v_c = carcass_state_in->carcass_vel_m_s;
        const double vs_x_belt = vs_x - v_c.x();
        const double vy_belt   = vy_contact_m_s + v_c.y();
        kappa_next = std::clamp(vs_x_belt / v_reg, -0.95, 0.95);
        alpha_next = std::atan2(vy_belt, v_reg);

        rolling = evaluate_steady_state(
            kappa_next, alpha_next, Fz, gamma_rad, vx_contact_m_s, eff_friction_mult, turn_rate_rad_s);

        // Semi-implicit integration of m_belt * dv_c/dt + C_c * v_c + K_c * e_c = F_tread
        const double m_b = std::max(0.25, params_.m_belt_kg);
        const Vec2 k_c(params_.k_carcass_x_N_m, params_.k_carcass_y_N_m);
        const Vec2 c_c(params_.c_carcass_x_N_s_m, params_.c_carcass_y_N_s_m);
        const Vec2 F_tread(rolling.Fx_N, rolling.Fy_N);

        Vec2 v_c_next, e_c_next;
        for (int axis = 0; axis < 2; ++axis) {
            const double denom = m_b + dt_s * c_c(axis) + (dt_s * dt_s) * k_c(axis);
            v_c_next(axis) = (m_b * v_c(axis) + dt_s * (F_tread(axis) - k_c(axis) * e_c(axis))) / denom;
            e_c_next(axis) = std::clamp(e_c(axis) + dt_s * v_c_next(axis), -0.035, 0.035);
        }
        out.carcass_thermal_next.carcass_defl_m = e_c_next;
        out.carcass_thermal_next.carcass_vel_m_s = v_c_next;
        // Force transmitted to the wheel hub through the carcass springs & dampers
        rolling.Fx_N = k_c.x() * e_c_next.x() + c_c.x() * v_c_next.x();
        rolling.Fy_N = k_c.y() * e_c_next.y() + c_c.y() * v_c_next.y();
    } else {
        // 1st-Order Exponential-Euler integration of relaxation-length ODEs
        const double load_scale = std::clamp(0.75 + params_.sigma_y_m * 0.25 * Fz_norm, 0.5, 2.0);
        const double sigma_x = std::max(0.03, params_.sigma_x_m * load_scale);
        const double sigma_y = std::max(0.04, params_.sigma_y_m * load_scale);

        const double exp_x = std::exp(-std::clamp((v_reg / sigma_x) * dt_s, 0.0, 50.0));
        const double exp_y = std::exp(-std::clamp((v_reg / sigma_y) * dt_s, 0.0, 50.0));

        kappa_next = exp_x * kappa_dyn_curr + (1.0 - exp_x) * kappa_ss;
        alpha_next = exp_y * alpha_dyn_curr + (1.0 - exp_y) * alpha_ss;

        rolling = evaluate_steady_state(
            kappa_next, alpha_next, Fz, gamma_rad, vx_contact_m_s, eff_friction_mult, turn_rate_rad_s);
    }

    // 5. Elastoplastic 2D Bristle Static Friction for true zero-speed slope holding
    const double v_rim_static = (std::abs(r_eff * omega_wheel_rad_s) < 0.12) ? 0.0 : (r_eff * omega_wheel_rad_s);
    const Vec2 v_slip(v_rim_static - vx_contact_m_s, -vy_contact_m_s);
    const double mu_static = 0.85 * (params_.pDy1 * params_.lambda_muy * eff_friction_mult);
    const double F_coulomb_max = mu_static * Fz;
    const double xi_max = std::max(1e-6, F_coulomb_max / params_.k_bristle_N_m);

    Vec2 xi_cand = bristle_curr_m + dt_s * v_slip;
    const double xi_norm = xi_cand.norm();
    if (xi_norm > xi_max) {
        xi_cand *= (xi_max / xi_norm);
    }
    const Vec2 xi_dot = (dt_s > 1e-9) ? ((xi_cand - bristle_curr_m) / dt_s) : v_slip;
    Vec2 F_bristle = params_.k_bristle_N_m * xi_cand + params_.c_bristle_N_s_m * xi_dot;
    const double fb_norm = F_bristle.norm();
    if (fb_norm > F_coulomb_max) {
        F_bristle *= (F_coulomb_max / fb_norm);
    }

    // Standstill bore torque bristle for zero-speed parking steering
    const double R_patch = std::sqrt(rolling.contact_patch_half_length_m * (0.5 * params_.width_m));
    const double Mz_bore_limit = (2.0 / 3.0) * mu_static * Fz * R_patch;
    double bore_twist = (carcass_state_in != nullptr ? carcass_state_in->bore_twist_rad : 0.0)
                        - dt_s * turn_rate_rad_s;
    const double max_twist = std::max(1e-5, Mz_bore_limit / std::max(1.0, params_.k_bore_Nm_rad));
    bore_twist = std::clamp(bore_twist, -max_twist, max_twist);
    const double Mz_bore_static = std::clamp(
        params_.k_bore_Nm_rad * bore_twist - params_.c_bore_Nm_s_rad * turn_rate_rad_s,
        -Mz_bore_limit, Mz_bore_limit);

    // 6. Partition-of-unity C^1 velocity blend between bristle static regime (v -> 0) and MF6.2 rolling regime
    const double v_patch = std::hypot(vx_contact_m_s, vy_contact_m_s) + 0.5 * std::abs(r_eff * omega_wheel_rad_s);
    const double s_blend = std::clamp((v_patch - 0.15) / params_.v_blend_m_s, 0.0, 1.0);
    const double w_roll  = s_blend * s_blend * (3.0 - 2.0 * s_blend);

    out.Fz_N = Fz;
    out.r_eff_m = r_eff;
    out.contact_patch_half_length_m = rolling.contact_patch_half_length_m;
    out.Fx_N = w_roll * rolling.Fx_N + (1.0 - w_roll) * F_bristle.x();
    out.Fy_N = w_roll * rolling.Fy_N + (1.0 - w_roll) * F_bristle.y();

    // Final non-duplicating combined friction envelope check across all blended layers
    const double F_env_max = std::max(rolling.friction_ellipse_limit_N, F_coulomb_max);
    const double F_tot_xy  = std::hypot(out.Fx_N, out.Fy_N);
    if (F_tot_xy > F_env_max && F_env_max > 1e-6) {
        const double scale = F_env_max / F_tot_xy;
        out.Fx_N *= scale;
        out.Fy_N *= scale;
    }

    out.Mx_Nm = w_roll * rolling.Mx_Nm;
    out.My_Nm = w_roll * rolling.My_Nm;
    out.Mz_turn_slip_Nm = w_roll * rolling.Mz_turn_slip_Nm + (1.0 - w_roll) * Mz_bore_static;
    out.Mz_Nm = w_roll * (rolling.Mz_Nm - rolling.Mz_turn_slip_Nm) + out.Mz_turn_slip_Nm;
    out.kappa_steady = kappa_ss;
    out.alpha_steady_rad = alpha_ss;
    out.kappa_next = kappa_next;
    out.alpha_next_rad = alpha_next;
    out.bristle_next_m = (1.0 - w_roll) * xi_cand;
    out.carcass_thermal_next.bore_twist_rad = (1.0 - w_roll) * bore_twist;
    out.friction_ellipse_limit_N = F_env_max;
    out.friction_utilization = (F_env_max > 1e-6) ? std::min(1.0, std::hypot(out.Fx_N, out.Fy_N) / F_env_max) : 0.0;

    // 7. 2-Node Thermal Update (Surface + Core) driven by frictional slip power dissipation
    if (carcass_state_in != nullptr && dt_s > 0.0) {
        const double P_fric_W = std::abs(out.Fx_N * vs_x) + std::abs(out.Fy_N * vy_contact_m_s)
                              + std::abs(out.Mz_turn_slip_Nm * turn_rate_rad_s);
        const double T_s = carcass_state_in->T_surface_C;
        const double T_c = carcass_state_in->T_core_C;
        const double q_cond = params_.h_cond_surf_core_W_K * (T_s - T_c);
        const double q_conv = params_.h_conv_ambient_W_K * (1.0 + 0.08 * std::abs(vx_contact_m_s)) * (T_s - params_.T_ambient_C);
        out.carcass_thermal_next.T_surface_C = std::clamp(
            T_s + dt_s * (0.85 * P_fric_W - q_cond - q_conv) / std::max(50.0, params_.C_thermal_surf_J_K),
            params_.T_ambient_C, 180.0);
        out.carcass_thermal_next.T_core_C = std::clamp(
            T_c + dt_s * (0.15 * P_fric_W + q_cond - 0.25 * params_.h_conv_ambient_W_K * (T_c - params_.T_ambient_C)) /
            std::max(200.0, params_.C_thermal_core_J_K),
            params_.T_ambient_C, 160.0);
    }

    return out;
}

} // namespace ter26
