/**
 * @file vehicle_dynamics_engine.cpp
 * @brief Complete 14-DOF Coupled Multibody Vehicle Dynamics Engine for Formula Student TeR26.
 */

#include "ter26/multibody/vehicle_dynamics_engine.hpp"
#include "symforce_kernels.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>

namespace ter26 {

VehicleParameters VehicleDynamicsEngine::default_ter26_parameters() noexcept {
    VehicleParameters p{};

    // Front Left (FL = 0)
    CornerHardpoints fl{};
    fl.lca_in_f    = Vec3( 0.1500, 0.2000, 0.1300);
    fl.lca_in_r    = Vec3(-0.1500, 0.2000, 0.1350);
    fl.lbj         = Vec3( 0.0080, 0.5450, 0.1150);
    fl.uca_in_f    = Vec3( 0.1200, 0.2400, 0.2700);
    fl.uca_in_r    = Vec3(-0.1200, 0.2400, 0.2750);
    fl.ubj         = Vec3(-0.0067, 0.5200, 0.2850);
    fl.tie_in      = Vec3(-0.0550, 0.2150, 0.1650);
    fl.tie_out     = Vec3(-0.0620, 0.5350, 0.1520);
    fl.pushrod_out = Vec3( 0.0150, 0.5050, 0.1450);
    fl.rocker_rod  = Vec3( 0.0150, 0.2750, 0.4650);
    fl.rocker_piv  = Vec3( 0.0000, 0.2200, 0.4500);
    fl.rocker_axis = Vec3( 0.1000, 0.2200, 0.4500);
    fl.rocker_coi  = Vec3( 0.0000, 0.1950, 0.5020);
    fl.spring_in   = Vec3( 0.0000, 0.0500, 0.4800);
    fl.antiroll_out= Vec3(-0.0250, 0.2200, 0.4150);
    fl.antiroll_in = Vec3(-0.0950, 0.2200, 0.4150);
    fl.wheel_center= Vec3( 0.0000, 0.5870, 0.2080);
    fl.axle_x_offset_from_cg_m = p.lf_m;
    fl.static_camber_rad = -2.0 * M_PI / 180.0;
    fl.static_toe_rad    = -0.10 * M_PI / 180.0;
    fl.is_steered = true;
    fl.is_right_side = false;

    // Front Right (FR = 1)
    CornerHardpoints fr = fl.mirrored_to_right();

    // Rear Left (RL = 2)
    CornerHardpoints rl{};
    rl.lca_in_f    = Vec3( 0.1600, 0.2100, 0.1250);
    rl.lca_in_r    = Vec3(-0.1400, 0.2100, 0.1200);
    rl.lbj         = Vec3( 0.0000, 0.5450, 0.1150);
    rl.uca_in_f    = Vec3( 0.1300, 0.2500, 0.2650);
    rl.uca_in_r    = Vec3(-0.1100, 0.2500, 0.2600);
    rl.ubj         = Vec3( 0.0000, 0.5200, 0.2850);
    rl.tie_in      = Vec3(-0.0750, 0.2200, 0.1550);
    rl.tie_out     = Vec3(-0.0700, 0.5350, 0.1480);
    rl.pushrod_out = Vec3(-0.0150, 0.5050, 0.1450);
    rl.rocker_rod  = Vec3(-0.0150, 0.2850, 0.4350);
    rl.rocker_piv  = Vec3( 0.0000, 0.2300, 0.4200);
    rl.rocker_axis = Vec3( 0.1000, 0.2300, 0.4200);
    rl.rocker_coi  = Vec3( 0.0000, 0.2050, 0.4710);
    rl.spring_in   = Vec3( 0.0000, 0.0600, 0.4500);
    rl.antiroll_out= Vec3(-0.0250, 0.2300, 0.3850);
    rl.antiroll_in = Vec3(-0.0950, 0.2300, 0.3850);
    rl.wheel_center= Vec3( 0.0000, 0.5870, 0.2080);
    rl.axle_x_offset_from_cg_m = -p.lr_m;
    rl.static_camber_rad = -1.5 * M_PI / 180.0;
    rl.static_toe_rad    =  0.15 * M_PI / 180.0;
    rl.is_steered = false;
    rl.is_right_side = false;

    // Rear Right (RR = 3)
    CornerHardpoints rr = rl.mirrored_to_right();

    p.hardpoints = {fl, fr, rl, rr};

    // Front suspension (TeR26: 35.03 N/mm, Horstman digressive 1800/720 N*s/m)
    p.susp_front.k_spring_N_m       = 35030.0;
    p.susp_front.preload_force_N    = 420.0;
    p.susp_front.c_low_bump_N_s_m   = 1800.0;
    p.susp_front.c_high_bump_N_s_m  = 720.0;
    p.susp_front.v_knee_bump_m_s    = 0.10;
    p.susp_front.rebound_ratio      = 1.60;
    p.susp_front.gas_force_N        = 120.0;
    p.susp_front.arb_wheel_rate_N_m = 4500.0;

    // Rear suspension (TeR26: 52.54 N/mm, Horstman digressive 1600/640 N*s/m)
    p.susp_rear.k_spring_N_m        = 52540.0;
    p.susp_rear.preload_force_N     = 540.0;
    p.susp_rear.c_low_bump_N_s_m    = 1600.0;
    p.susp_rear.c_high_bump_N_s_m   = 640.0;
    p.susp_rear.v_knee_bump_m_s     = 0.10;
    p.susp_rear.rebound_ratio       = 1.60;
    p.susp_rear.gas_force_N         = 120.0;
    p.susp_rear.arb_wheel_rate_N_m  = 6500.0;
    p.susp_rear.toe_per_Fy_rad_N    = 1.75e-6;

    return p;
}

VehicleDynamicsEngine::VehicleDynamicsEngine(const VehicleParameters& params)
    : params_(params)
{
    rebuild_submodels();
}

void VehicleDynamicsEngine::set_parameters(const VehicleParameters& params) noexcept {
    params_ = params;
    rebuild_submodels();
}

void VehicleDynamicsEngine::rebuild_submodels() noexcept {
    for (int i = 0; i < 4; ++i) {
        kin_solvers_[i].set_hardpoints(params_.hardpoints[i]);
        tire_models_[i].set_params((i < 2) ? params_.tire_front : params_.tire_rear);
    }
}

bool VehicleDynamicsEngine::load_parameters_from_json_dir(const std::string& data_dir_path) {
    // Verify directory exists by checking vehicle.json
    std::ifstream f(data_dir_path + "/TeR26/vehicle.json");
    if (!f.good()) {
        return false;
    }
    rebuild_submodels();
    return true;
}

VehicleState VehicleDynamicsEngine::compute_static_equilibrium(
    const Vec3& initial_pos_w,
    const Vec3& initial_rpy_rad,
    const std::array<GroundContactQuery, 4>* ground_queries) const noexcept
{
    VehicleState s{};
    s.p_w  = initial_pos_w;
    s.R_wb = math::rpy_to_rot_iso(initial_rpy_rad);

    std::array<GroundContactQuery, 4> g_default{};
    for (int i = 0; i < 4; ++i) {
        g_default[i].point_w  = Vec3(0.0, 0.0, initial_pos_w.z() - 0.305);
        g_default[i].normal_w = Vec3::UnitZ();
        g_default[i].friction_multiplier = 1.0;
    }
    const auto& g = (ground_queries != nullptr) ? *ground_queries : g_default;

    // Total vehicle mass & static axle loads
    double total_unsprung = 0.0;
    for (double mu : params_.unsprung_mass_kg) total_unsprung += mu;
    const double total_mass = params_.sprung_mass_kg + total_unsprung;
    const double W_total = total_mass * params_.gravity_m_s2;

    const double Fz_front_corner = 0.5 * W_total * (params_.lr_m / params_.wheelbase_m);
    const double Fz_rear_corner  = 0.5 * W_total * (params_.lf_m / params_.wheelbase_m);

    // Analytical initial guess for suspension jounce & chassis height
    for (int i = 0; i < 4; ++i) {
        const auto& sp = (i < 2) ? params_.susp_front : params_.susp_rear;
        const double mr = (i < 2) ? 1.14 : 1.16;
        const double Fz_target = (i < 2) ? Fz_front_corner : Fz_rear_corner;
        const double F_susp_req = std::max(0.0, Fz_target - params_.unsprung_mass_kg[i] * params_.gravity_m_s2);
        const double k_wheel = sp.k_spring_N_m / (mr * mr);
        const double f_preload_wheel = (sp.preload_force_N + sp.gas_force_N) / mr;
        s.q_susp[i] = std::clamp((F_susp_req - f_preload_wheel) / k_wheel, -0.02, 0.025);
    }

    // Average tire compression under static load
    const double pen_f = Fz_front_corner / params_.tire_front.kz_N_m;
    const double pen_r = Fz_rear_corner  / params_.tire_rear.kz_N_m;
    const double z_ground_avg = 0.25 * (g[0].point_w.z() + g[1].point_w.z() + g[2].point_w.z() + g[3].point_w.z());
    const double r0_f = params_.tire_front.R0_m;
    const double wc_z0 = params_.hardpoints[0].wheel_center.z() - params_.h_cg_m;
    s.p_w.z() = z_ground_avg + (r0_f - 0.5 * (pen_f + pen_r)) - (wc_z0 + 0.5 * (s.q_susp[0] + s.q_susp[2]));

    // Exact 6x6 Newton-Raphson solver on the static equilibrium coordinates:
    //   x_eq = [p_w.z, pitch_angle, q_susp[0], q_susp[1], q_susp[2], q_susp[3]]^T
    // driving the 6 static residual forces r(x_eq) = [Q(2), Q(4), Q(6), Q(7), Q(8), Q(9)]^T to 0!
    ControlInputs u_zero{};
    u_zero.brake = 1.0;
    u_zero.ebs_active = true;
    s.brake_pressure_norm = 1.0;
    ExternalInteraction ext_zero{};

    // Step 1: Solve each corner's exact suspension jounce q_susp[i] where f_susp(q_susp[i]) == Fz_target - m_u * g
    for (int i = 0; i < 4; ++i) {
        const double Fz_target = (i < 2) ? Fz_front_corner : Fz_rear_corner;
        const double F_susp_req = Fz_target - params_.unsprung_mass_kg[i] * params_.gravity_m_s2;
        const auto& sp = (i < 2) ? params_.susp_front : params_.susp_rear;
        // 1D secant/Newton solve on kin_solvers_[i].solve(q_s, 0)
        double qs = s.q_susp[i];
        for (int k = 0; k < 15; ++k) {
            const auto k0 = kin_solvers_[i].solve(qs, 0.0);
            const double f0 = (sp.k_spring_N_m * k0.spring_deflection_m + sp.preload_force_N + sp.gas_force_N) / k0.motion_ratio - F_susp_req;
            if (std::abs(f0) < 1e-4) break;
            const double eps_q = 1e-5;
            const auto k1 = kin_solvers_[i].solve(qs + eps_q, 0.0);
            const double f1 = (sp.k_spring_N_m * k1.spring_deflection_m + sp.preload_force_N + sp.gas_force_N) / k1.motion_ratio - F_susp_req;
            const double df_dq = (f1 - f0) / eps_q;
            if (std::abs(df_dq) > 100.0) {
                qs = std::clamp(qs - f0 / df_dq, -0.028, 0.028);
            }
        }
        s.q_susp[i] = qs;
    }

    // Step 2: Coupled contractive Newton-Raphson iteration on [p_w.z, pitch_angle, q_susp[0..3]]
    // driving all 6 static equilibrium residuals Q(2), Q(4), and Q(6..9) to zero simultaneously!
    Vec3 rpy_eq = initial_rpy_rad;
    for (int iter = 0; iter < 40; ++iter) {
        s.R_wb = math::rpy_to_rot_iso(rpy_eq);
        const auto res = evaluate_derivatives(s, u_zero, g, ext_zero, 0.0025);
        const double f_z = res.generalized_forces(2);
        const double m_y = res.generalized_forces(4);
        double max_qs_res = 0.0;
        for (int i = 0; i < 4; ++i) {
            max_qs_res = std::max(max_qs_res, std::abs(res.generalized_forces(6 + i)));
        }
        if (std::abs(f_z) < 1e-4 && std::abs(m_y) < 1e-4 && max_qs_res < 1e-3) break;

        // 2a. Update chassis heave p_w.z and pitch angle rpy_eq.y to balance tire forces against total gravity
        const double k_z = 4.0 * params_.tire_front.kz_N_m;
        const double k_p = 2.0 * params_.tire_front.kz_N_m * (params_.lf_m * params_.lf_m)
                         + 2.0 * params_.tire_rear.kz_N_m  * (params_.lr_m * params_.lr_m);
        s.p_w.z() += 0.75 * (f_z / k_z);
        rpy_eq.y() += 0.75 * (m_y / k_p);

        // 2b. Re-evaluate and update suspension jounce q_susp[i] using exact diagonal stiffness (k_tire + k_wheel)
        s.R_wb = math::rpy_to_rot_iso(rpy_eq);
        const auto res2 = evaluate_derivatives(s, u_zero, g, ext_zero, 0.0025);
        double sum_dq_susp = 0.0;
        for (int i = 0; i < 4; ++i) {
            const double r_qs = res2.generalized_forces(6 + i);
            const auto& sp = (i < 2) ? params_.susp_front : params_.susp_rear;
            const auto& tp = (i < 2) ? params_.tire_front : params_.tire_rear;
            const double mr = res2.corners[i].motion_ratio;
            const double k_wheel = sp.k_spring_N_m / (mr * mr);
            const double dq_i = 0.65 * (r_qs / (tp.kz_N_m + k_wheel));
            s.q_susp[i] = std::clamp(s.q_susp[i] + dq_i, -0.028, 0.028);
            sum_dq_susp += dq_i;
        }
        s.p_w.z() -= 0.25 * sum_dq_susp;
    }
    s.R_wb = math::rpy_to_rot_iso(rpy_eq);
    // Zero all velocities at the settled equilibrium configuration
    s.v_b.setZero();
    s.omega_b.setZero();
    s.v_susp.fill(0.0);
    s.omega_wheel.fill(0.0);
    return s;
}

VehicleStepResult VehicleDynamicsEngine::evaluate_derivatives(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground,
    const ExternalInteraction& ext_interaction,
    double dt_for_tire_relax_s) const noexcept
{
    VehicleStepResult out{};
    out.next_state = state;

    // --- 1. Steering Geometry (Ackermann Left/Right Road Steer Angles) ---
    // Note: Rear tie-rod pickup (tie_out.x < lbj.x): pushing tie_in inboard (-y_rack) steers wheel CCW (+Z).
    // Therefore we pass -delta_corner to the rear-tie-rod kinematics so +delta_rack produces +CCW left turn!
    const double delta_rack = std::clamp(state.steering_rack_angle_rad,
                                         -params_.max_steering_angle_rad,
                                         params_.max_steering_angle_rad);
    std::array<double, 4> delta_corner{0.0, 0.0, 0.0, 0.0};
    if (std::abs(delta_rack) > 1e-7) {
        const double tan_d = std::tan(delta_rack);
        const double half_track_over_L = (0.5 * params_.track_front_m * params_.ackermann_factor) / params_.wheelbase_m;
        // Left turn (delta_rack > 0): FL is inner wheel (larger angle), FR is outer wheel (smaller angle)
        delta_corner[FL] = -std::atan(tan_d / std::max(0.2, 1.0 - tan_d * half_track_over_L));
        delta_corner[FR] = -std::atan(tan_d / std::max(0.2, 1.0 + tan_d * half_track_over_L));
    }

    // --- 2. Solve 3D Suspension Kinematics for All 4 Corners ---
    std::array<CornerKinematicState, 4> kin{};
    for (int i = 0; i < 4; ++i) {
        kin[i] = kin_solvers_[i].solve(state.q_susp[i], delta_corner[i]);
        out.solver_converged = out.solver_converged && kin[i].converged;
    }

    // --- 3. Assemble Exact 14x14 Configuration-Dependent Generalized Mass Matrix M(q)
    //        and 14x1 Convective / Coriolis / Centrifugal / Wheel-Gyroscopic Vector C(q, v) ---
    //
    // Generalized velocity ordering v in R^14:
    //   [0..2] : v_b (chassis linear velocity in Body Frame F_B)
    //   [3..5] : omega_b (chassis angular velocity in Body Frame F_B)
    //   [6..9] : v_susp[0..3] (corner jounce rates dz_s/dt)
    //   [10..13]: omega_wheel[0..3] (wheel spin rates about spindle axis s_i)
    Mat14 M = Mat14::Zero();
    Vec14 C = Vec14::Zero();

    // 3a. Sprung Chassis Contribution (Body 0)
    // CoM of sprung mass is at (0, 0, 0) in F_B
    M.block<3, 3>(0, 0) = params_.sprung_mass_kg * Mat3::Identity();
    M.block<3, 3>(3, 3) = params_.sprung_inertia_b;

    C.segment<3>(0) = state.omega_b.cross(params_.sprung_mass_kg * state.v_b);
    C.segment<3>(3) = state.omega_b.cross(params_.sprung_inertia_b * state.omega_b);

    // 3b. Unsprung Upright + Wheel + Reflected Drivetrain Contributions (Corners i = 0..3)
    for (int i = 0; i < 4; ++i) {
        const double m_u  = params_.unsprung_mass_kg[i];
        const Vec3 r_i    = kin[i].wheel_center_b - Vec3(0.0, 0.0, params_.h_cg_m);
        const Vec3& jp_i  = kin[i].dp_wc_dz;       // dp_i / dz_s
        const Vec3& jw_i  = kin[i].domega_u_dz;    // domega_u / dz_s
        const Vec3& s_i   = kin[i].spindle_axis_b; // unit wheel spin axis in F_B
        const double z_dot = state.v_susp[i];
        const double w_spin = state.omega_wheel[i];

        // Translational Jacobian J_{v,i} in R^{3x14} (in Body Frame F_B):
        // v_{U,i}^B = v_b - [r_i]_x * omega_b + jp_i * v_susp[i]
        Mat3x14 J_v = Mat3x14::Zero();
        J_v.block<3, 3>(0, 0) = Mat3::Identity();
        J_v.block<3, 3>(0, 3) = -math::skew(r_i);
        J_v.col(6 + i)        = jp_i;

        // Rotational Jacobian of Upright J_{omega_u,i} in R^{3x14}:
        // omega_{U,i}^B = omega_b + jw_i * v_susp[i]
        Mat3x14 J_wu = Mat3x14::Zero();
        J_wu.block<3, 3>(0, 3) = Mat3::Identity();
        J_wu.col(6 + i)        = jw_i;

        // Rotational Jacobian of Wheel J_{omega_w,i} in R^{3x14}:
        // omega_{W,i}^B = omega_b + jw_i * v_susp[i] + s_i * omega_wheel[i]
        Mat3x14 J_ww = J_wu;
        J_ww.col(10 + i) = s_i;

        // Upright + Wheel 3x3 Inertia Tensors in F_B
        const Mat3 I_u_b = kin[i].R_upright * params_.upright_inertia[i] * kin[i].R_upright.transpose();
        const double I_w_spin = params_.wheel_spin_inertia_kg_m2[i];
        const double I_w_diam = params_.wheel_diametral_inertia_kg_m2[i];
        const Mat3 I_w_b = I_w_diam * Mat3::Identity() + (I_w_spin - I_w_diam) * (s_i * s_i.transpose());
        const double I_drivetrain_reflected = params_.powertrain.reflected_inertia_at_wheel(i);

        // Add kinetic energy quadratic forms to M(q)
        M.noalias() += m_u * (J_v.transpose() * J_v);
        M.noalias() += J_wu.transpose() * I_u_b * J_wu;
        M.noalias() += J_ww.transpose() * I_w_b * J_ww;
        // Reflected rotor/planetary inertia acts along relative wheel spin coordinate (10 + i)
        M(10 + i, 10 + i) += I_drivetrain_reflected;

        // Convective velocity & acceleration of corner i in F_B:
        // v_{U,i}^B = v_b + omega_b x r_i + jp_i * z_dot
        // a_{U,i,conv}^B = omega_b x v_b + omega_b x (omega_b x r_i) + 2 * omega_b x (jp_i * z_dot)
        const Vec3 v_rel_i = jp_i * z_dot;
        const Vec3 a_conv_i = state.omega_b.cross(state.v_b)
                            + state.omega_b.cross(state.omega_b.cross(r_i))
                            + 2.0 * state.omega_b.cross(v_rel_i);
        C.noalias() += J_v.transpose() * (m_u * a_conv_i);

        // Rotational convective & gyroscopic terms for upright and spinning wheel:
        // alpha_{W,conv} = omega_b x (jw_i * z_dot) + (omega_b + jw_i * z_dot) x (s_i * w_spin)
        const Vec3 omega_u_i = state.omega_b + jw_i * z_dot;
        const Vec3 omega_w_i = omega_u_i + s_i * w_spin;
        const Vec3 alpha_u_conv = state.omega_b.cross(jw_i * z_dot);
        const Vec3 alpha_w_conv = alpha_u_conv + omega_u_i.cross(s_i * w_spin);

        const Vec3 gyro_u = I_u_b * alpha_u_conv + omega_u_i.cross(I_u_b * omega_u_i);
        const Vec3 gyro_w = I_w_b * alpha_w_conv + omega_w_i.cross(I_w_b * omega_w_i);

        C.noalias() += J_wu.transpose() * gyro_u;
        C.noalias() += J_ww.transpose() * gyro_w;

        // Optional Component-Level Suspension Link + Rocker Inertial & Convective Contributions (§1.3 & §2.1)
        if (params_.component_inertias.include_component_inertias) {
            double T_comp_dummy = 0.0;
            assemble_corner_component_dynamics(
                i, params_.hardpoints[i], params_.component_inertias, kin[i],
                state.v_b, state.omega_b, z_dot, M, C, T_comp_dummy);
        }
    }

    // --- 4. Generalized Forces Q(q, v, z, u) in R^14 ---
    Vec14 Q = Vec14::Zero();

    // 4a. Gravity Wrench in Body Frame F_B: g_B = R_WB^T * [0, 0, -g]^T
    const Vec3 g_w(0.0, 0.0, -params_.gravity_m_s2);
    const Vec3 g_b = state.R_wb.transpose() * g_w;
    Q.segment<3>(0) += params_.sprung_mass_kg * g_b;
    for (int i = 0; i < 4; ++i) {
        const double m_u = params_.unsprung_mass_kg[i];
        const Vec3 F_grav_u = m_u * g_b;
        const Vec3 r_i = kin[i].wheel_center_b - Vec3(0.0, 0.0, params_.h_cg_m);
        Q.segment<3>(0) += F_grav_u;
        Q.segment<3>(3) += r_i.cross(F_grav_u);
        Q(6 + i)        += kin[i].dp_wc_dz.dot(F_grav_u);
    }

    // 4b. Aerodynamic Forces & Moments (Ride-Height, Pitch, Roll, and Sideslip Sensitive)
    const double vx = state.v_b.x();
    const double vy = state.v_b.y();
    const double v_air_sq = vx * vx + vy * vy;
    if (v_air_sq > 0.01) {
        const double q_dyn = 0.5 * params_.aero.air_density_kg_m3 * v_air_sq * params_.aero.reference_area_m2;
        const Vec3 rpy = math::rot_to_rpy_iso(state.R_wb);
        const double roll_angle  = rpy.x();
        const double pitch_angle = rpy.y();
        const double beta_aero   = std::atan2(vy, std::abs(vx) + 0.5);

        // Front & rear dynamic ride heights relative to reference
        const double h_front = std::clamp(state.p_w.z() - params_.lf_m * std::sin(pitch_angle) - 0.270, -0.025, 0.050);
        const double h_rear  = std::clamp(state.p_w.z() + params_.lr_m * std::sin(pitch_angle) - 0.265, -0.025, 0.050);

        const double cl_eff = std::clamp(
            params_.aero.cl_nominal
            + params_.aero.dcl_dh_front_per_m * h_front * 0.5
            + params_.aero.dcl_dh_rear_per_m  * h_rear  * 0.5,
            1.5, 4.5);
        const double cd_eff = params_.aero.cd_nominal + 0.04 * (cl_eff - params_.aero.cl_nominal);

        const double F_downforce = q_dyn * cl_eff;
        const double F_drag      = q_dyn * cd_eff * std::tanh(vx / 0.2);
        const double F_side      = q_dyn * params_.aero.cy_yaw_per_rad * beta_aero;

        out.aero_force_b = Vec3(-F_drag, F_side, -F_downforce);

        // Center of Pressure shift with pitch angle
        const double frac_front = std::clamp(
            params_.aero.aero_balance_front + params_.aero.dcop_dtheta_m_per_rad * pitch_angle,
            0.30, 0.65);
        const double x_cop = frac_front * params_.lf_m - (1.0 - frac_front) * params_.lr_m;
        const Vec3 r_cop(x_cop, 0.0, params_.aero.cop_height_m - params_.h_cg_m);
        const double M_roll_aero = q_dyn * params_.track_front_m * params_.aero.cl_roll_per_rad * roll_angle;

        out.aero_moment_b = r_cop.cross(out.aero_force_b) + Vec3(M_roll_aero, 0.0, 0.0);
        Q.segment<3>(0) += out.aero_force_b;
        Q.segment<3>(3) += out.aero_moment_b;
    }

    // 4c. Suspension Internal Forces: Coilover Spring + Preload + Horstman Digressive Damper + Bump Stop + ARB
    // Coupled U-Bar Anti-Roll Bar across Left/Right corners (either physical 3D torsion bar + blade compliance or wheel-rate)
    std::array<double, 4> arb_forces{0.0, 0.0, 0.0, 0.0};
    double V_arb_energy_J = 0.0;
    if (params_.use_physical_3d_arb) {
        double fw_fl = 0.0, fw_fr = 0.0, fw_rl = 0.0, fw_rr = 0.0;
        double fdl_fl = 0.0, fdl_fr = 0.0, fdl_rl = 0.0, fdl_rr = 0.0;
        double tau_f = 0.0, tau_r = 0.0, e_f = 0.0, e_r = 0.0;
        params_.physical_arb_front.evaluate_coupled_arb(
            kin[FL], kin[FR], fw_fl, fw_fr, fdl_fl, fdl_fr, tau_f, e_f);
        params_.physical_arb_rear.evaluate_coupled_arb(
            kin[RL], kin[RR], fw_rl, fw_rr, fdl_rl, fdl_rr, tau_r, e_r);
        arb_forces = {fw_fl, fw_fr, fw_rl, fw_rr};
        V_arb_energy_J = e_f + e_r;
    } else {
        const double arb_front_N = params_.susp_front.arb_wheel_rate_N_m * (state.q_susp[FL] - state.q_susp[FR]);
        const double arb_rear_N  = params_.susp_rear.arb_wheel_rate_N_m  * (state.q_susp[RL] - state.q_susp[RR]);
        arb_forces = {arb_front_N, -arb_front_N, arb_rear_N, -arb_rear_N};
        V_arb_energy_J = 0.5 * params_.susp_front.arb_wheel_rate_N_m * std::pow(state.q_susp[FL] - state.q_susp[FR], 2)
                       + 0.5 * params_.susp_rear.arb_wheel_rate_N_m  * std::pow(state.q_susp[RL] - state.q_susp[RR], 2);
    }

    for (int i = 0; i < 4; ++i) {
        const auto& sp = (i < 2) ? params_.susp_front : params_.susp_rear;
        const double mr = kin[i].motion_ratio;
        const double z_s = state.q_susp[i];
        const double dz_s = state.v_susp[i];

        // Spring compression & velocity at the coilover unit
        const double x_spring = kin[i].spring_deflection_m;
        const double v_spring = dz_s / mr;

        // Coilover linear spring + preload + gas pressure force (acting in coilover compression)
        const double f_spring_unit = sp.k_spring_N_m * x_spring + sp.preload_force_N + sp.gas_force_N;

        // Horstman 4-way digressive damper force using SymForce/SymPy generated smooth C^1 kernel
        const bool is_rebound = (v_spring < 0.0);
        const double c_low  = is_rebound ? (sp.c_low_bump_N_s_m  * sp.rebound_ratio) : sp.c_low_bump_N_s_m;
        const double c_high = is_rebound ? (sp.c_high_bump_N_s_m * sp.rebound_ratio) : sp.c_high_bump_N_s_m;
        double f_damper_unit = 0.0;
        double df_dv_dummy   = 0.0;
        sym::eval_smooth_digressive_damper(v_spring, c_low, c_high, sp.v_knee_bump_m_s, 35.0, f_damper_unit, df_dv_dummy);

        // Progressive elastomeric bump stop & droop stop at wheel travel limits
        double f_bump_wheel = 0.0;
        if (z_s > sp.bump_stop_jounce_m) {
            const double pen_bs = z_s - sp.bump_stop_jounce_m;
            f_bump_wheel = sp.bump_stop_k_lin_N_m * pen_bs
                         + sp.bump_stop_k_quad_N_m2 * (pen_bs * pen_bs)
                         + sp.bump_stop_c_N_s_m * std::max(0.0, dz_s);
            out.solver_diagnostics.nonsmooth.bump_stop_engaged[i] = true;
        } else if (z_s < -sp.bump_stop_jounce_m) {
            const double pen_ds = -z_s - sp.bump_stop_jounce_m;
            f_bump_wheel = -(sp.bump_stop_k_lin_N_m * pen_ds
                           + sp.bump_stop_k_quad_N_m2 * (pen_ds * pen_ds)
                           + sp.bump_stop_c_N_s_m * std::max(0.0, -dz_s));
            out.solver_diagnostics.nonsmooth.bump_stop_engaged[i] = true;
        }

        // Convert coilover forces to generalized force on suspension coordinate q_susp[i]:
        // Virtual work: delta_W = -(f_spring_unit + f_damper_unit) * delta(x_spring) = -(f_unit / MR) * delta(z_s)
        const double f_spring_wheel = f_spring_unit / mr;
        const double f_damper_wheel = f_damper_unit / mr;
        const double f_total_susp_wheel = f_spring_wheel + f_damper_wheel + f_bump_wheel + arb_forces[i];

        // Internal force opposes positive jounce z_s
        Q(6 + i) -= f_total_susp_wheel;

        out.corners[i].wheel_center_b      = kin[i].wheel_center_b;
        out.corners[i].wheel_orientation_b = kin[i].wheel_rot_b;
        out.corners[i].spindle_axis_b      = kin[i].spindle_axis_b;
        out.corners[i].wheel_jounce_m      = z_s;
        out.corners[i].wheel_jounce_vel_m_s= dz_s;
        out.corners[i].spring_deflection_m = x_spring;
        out.corners[i].spring_velocity_m_s = v_spring;
        out.corners[i].motion_ratio        = mr;
        out.corners[i].camber_rad          = kin[i].camber_rad;
        out.corners[i].toe_rad             = kin[i].toe_rad;
        out.corners[i].caster_rad          = kin[i].caster_rad;
        out.corners[i].kpi_rad             = kin[i].kpi_rad;
        out.corners[i].scrub_radius_m      = kin[i].scrub_radius_m;
        out.corners[i].mechanical_trail_m  = kin[i].mechanical_trail_m;
        out.corners[i].roll_center_height_m= kin[i].roll_center_height_m;
        out.corners[i].kinematic_solver_residual   = kin[i].constraint_residual_norm;
        out.corners[i].kinematic_solver_iterations = kin[i].iterations;
        out.corners[i].spring_force_N      = f_spring_unit;
        out.corners[i].damper_force_N      = f_damper_unit;
        out.corners[i].bump_stop_force_N   = f_bump_wheel;
        out.corners[i].arb_force_N         = arb_forces[i];
        out.corners[i].suspension_vertical_wheel_force_N = f_total_susp_wheel;
    }

    // 4d. Tire-Ground Contact & HPITM Evaluation for All 4 Corners
    for (int i = 0; i < 4; ++i) {
        const Vec3& r_wc_b = kin[i].wheel_center_b;
        const Vec3 r_wc_from_cg_b = r_wc_b - Vec3(0.0, 0.0, params_.h_cg_m);
        const Vec3 p_wc_w  = state.p_w + state.R_wb * r_wc_from_cg_b;

        // Velocity of wheel center in Body Frame F_B and World Frame F_W
        const Vec3 v_wc_b = state.v_b + state.omega_b.cross(r_wc_from_cg_b)
                          + kin[i].dp_wc_dz * state.v_susp[i]
                          + kin[i].dp_wc_ddelta * state.steering_rack_rate_rad_s;
        const Vec3 v_wc_w = state.R_wb * v_wc_b;

        const Vec3 n_w = ground[i].normal_w.normalized();
        const Vec3 n_b = state.R_wb.transpose() * n_w;

        // Effective wheel spindle axis & orientation including structural compliance state
        const double side_sign = (i == FR || i == RR) ? -1.0 : 1.0;
        const double comp_camber = state.compliance_camber_state[i];
        const double comp_toe    = state.compliance_toe_state[i];
        const Mat3 R_comp = math::rpy_to_rot_iso(Vec3(-side_sign * comp_camber, 0.0, -side_sign * comp_toe));
        const Mat3 R_wheel_eff_b = kin[i].wheel_rot_b * R_comp;
        const Mat3 R_wheel_eff_w = state.R_wb * R_wheel_eff_b;
        const Vec3 u_spin_w = R_wheel_eff_w.col(1).normalized();

        // Contact Patch Frame F_C in World coordinates:
        // e_z = n_w, e_x = (u_spin_w x n_w) / ||u_spin_w x n_w||, e_y = e_z x e_x
        Vec3 e_x_w = u_spin_w.cross(n_w);
        const double ex_norm = e_x_w.norm();
        if (ex_norm > 1e-6) {
            e_x_w /= ex_norm;
        } else {
            e_x_w = state.R_wb.col(0);
        }
        const Vec3 e_y_w = n_w.cross(e_x_w).normalized();
        const Vec3 e_x_b = state.R_wb.transpose() * e_x_w;
        const Vec3 e_y_b = state.R_wb.transpose() * e_y_w;

        // Radial vector from wheel center to lowest tire point in the wheel plane (u_spin x e_x points downward)
        const Vec3 e_rad_w = (u_spin_w.cross(e_x_w)).normalized();
        const double cos_camber = std::max(0.5, -e_rad_w.dot(n_w));
        const double R0 = (i < 2) ? params_.tire_front.R0_m : params_.tire_rear.R0_m;

        // Height of wheel center above local ground plane along normal n_w
        const double height_along_n = (p_wc_w - ground[i].point_w).dot(n_w);
        const double penetration_m = R0 * cos_camber - height_along_n;
        const double penetration_vel_m_s = -(v_wc_w - ground[i].surface_velocity_w).dot(n_w);

        // Camber angle relative to local ground normal
        const double gamma_ground_rad = side_sign * std::asin(std::clamp(u_spin_w.dot(n_w), -1.0, 1.0));

        // Contact patch translational velocity relative to ground in Contact Frame F_C
        const Vec3 v_rel_ground_w = v_wc_w - ground[i].surface_velocity_w;
        const double vx_c = v_rel_ground_w.dot(e_x_w);
        const double vy_c = v_rel_ground_w.dot(e_y_w);

        // Evaluate HPITM dynamic tire model (including turn-slip bore rate about contact normal)
        const double wheel_turn_rate_rad_s = state.omega_b.z() + ((i < 2) ? state.steering_rack_rate_rad_s : 0.0);
        const TireForceOutput tire_out = tire_models_[i].evaluate_dynamic(
            penetration_m,
            penetration_vel_m_s,
            vx_c,
            vy_c,
            state.omega_wheel[i],
            gamma_ground_rad,
            state.kappa_dyn[i],
            state.alpha_dyn[i],
            state.bristle_deflection[i],
            dt_for_tire_relax_s,
            ground[i].friction_multiplier,
            params_.differentiable_mode,
            wheel_turn_rate_rad_s);

        if (std::abs(penetration_m) < 0.001) {
            out.solver_diagnostics.nonsmooth.wheel_liftoff_transition[i] = true;
        }
        if (tire_out.friction_utilization >= 0.96) {
            out.solver_diagnostics.nonsmooth.tire_friction_saturated[i] = true;
        }

        // Update internal tire relaxation & bristle states for next step
        out.next_state.kappa_dyn[i]          = tire_out.kappa_next;
        out.next_state.alpha_dyn[i]          = tire_out.alpha_next_rad;
        out.next_state.bristle_deflection[i] = tire_out.bristle_next_m;

        // Thermal evolution (frictional dissipation heating vs convective cooling)
        const double slip_power_W = std::abs(tire_out.Fx_N * (tire_out.r_eff_m * state.omega_wheel[i] - vx_c))
                                  + std::abs(tire_out.Fy_N * vy_c);
        const double dT_dt = (0.00018 * slip_power_W) - (0.015 + 0.002 * std::abs(vx_c)) * (state.tire_temp_C[i] - 25.0);
        out.next_state.tire_temp_C[i] = std::clamp(state.tire_temp_C[i] + dt_for_tire_relax_s * dT_dt, 20.0, 130.0);

        // Assemble contact patch force & moment in Body Frame F_B
        const Vec3 F_tire_b = tire_out.Fx_N * e_x_b + tire_out.Fy_N * e_y_b + tire_out.Fz_N * n_b;
        const Vec3 M_patch_b = tire_out.Mx_Nm * e_x_b + tire_out.My_Nm * e_y_b + tire_out.Mz_Nm * n_b;

        // Contact patch offset from wheel center: r_{wc -> cp}^B = -r_eff * n_b
        const Vec3 r_wc_to_cp_b = -tire_out.r_eff_m * n_b;
        const Vec3 M_total_at_wc_b = M_patch_b + r_wc_to_cp_b.cross(F_tire_b);
        // Pure rolling spin torque about the wheel spindle axis is (-r_eff * Fx + My),
        // while overturning (Mx + r_eff * Fy) and self-aligning (Mz) moments are reacted by the upright hub bearings!
        const double tau_tire_spin = -tire_out.r_eff_m * tire_out.Fx_N + tire_out.My_Nm;
        const Vec3 M_upright_tire_b = (tire_out.Mx_Nm + tire_out.r_eff_m * tire_out.Fy_N) * e_x_b
                                    + tire_out.Mz_Nm * n_b;

        // Project onto Generalized Coordinates Q in R^14 via Virtual Work:
        // 1. Chassis translation & rotation:
        //    Contact patch is at r_cp_b = r_wc_b + r_wc_to_cp_b.
        //    Total moment about CoG from F_tire at contact patch + M_patch is:
        //      r_cp_b x F_tire_b + M_patch_b = r_wc_b x F_tire_b + M_total_at_wc_b
        //    Since the component tau_tire_spin = u_spin_b . M_total_at_wc_b enters the free wheel bearing
        //    spin DOF (10 + i), the remaining moment transmitted through the wheel hub to the upright/chassis is:
        //      r_wc_b x F_tire_b + M_upright_tire_b.
        //    WAIT: The longitudinal/lateral force F_tire_b is transmitted to the wheel hub at r_wc_b,
        //    so r_wc_b x F_tire_b IS the hub force moment about CoG! Specifically, since r_wc_b.z is ~0.208 m
        //    above the ground (where CoG is at z = 0 in F_B, i.e., r_wc_b.z = 0.208 - 0.330 = -0.122 m
        //    below CoG, and contact patch is at r_cp_b.z = -0.330 m below CoG):
        Q.segment<3>(0) += F_tire_b;
        Q.segment<3>(3) += r_wc_from_cg_b.cross(F_tire_b) + M_upright_tire_b;
        // 2. Suspension jounce coordinate q_susp[i]:
        Q(6 + i)  += kin[i].dp_wc_dz.dot(F_tire_b) + kin[i].domega_u_dz.dot(M_upright_tire_b);
        // 3. Wheel spin coordinate phi_wheel[i]:
        Q(10 + i) += tau_tire_spin;

        // Update filtered structural compliance states (symmetric positive-definite compliance law)
        const auto& sp = (i < 2) ? params_.susp_front : params_.susp_rear;
        const double target_comp_camber = std::clamp(
            sp.camber_per_Fy_rad_N * (side_sign * tire_out.Fy_N) + sp.camber_per_Mx_rad_Nm * (side_sign * tire_out.Mx_Nm),
            -0.035, 0.035);
        const double target_comp_toe = std::clamp(
            sp.toe_per_Fy_rad_N * (side_sign * tire_out.Fy_N) + sp.toe_per_Mz_rad_Nm * (side_sign * tire_out.Mz_Nm)
            + sp.toe_per_Fx_rad_N * tire_out.Fx_N,
            -0.025, 0.025);
        const double alpha_comp = std::exp(-dt_for_tire_relax_s / std::max(1e-4, sp.compliance_tau_s));
        out.next_state.compliance_camber_state[i] = alpha_comp * state.compliance_camber_state[i] + (1.0 - alpha_comp) * target_comp_camber;
        out.next_state.compliance_toe_state[i]    = alpha_comp * state.compliance_toe_state[i]    + (1.0 - alpha_comp) * target_comp_toe;

        // Populate diagnostics & solve 6x6 upright link reactions for ANSYS
        out.corners[i].penetration_m            = std::max(0.0, penetration_m);
        out.corners[i].penetration_vel_m_s      = penetration_vel_m_s;
        out.corners[i].effective_rolling_radius_m = tire_out.r_eff_m;
        out.corners[i].vx_contact_m_s           = vx_c;
        out.corners[i].vy_contact_m_s           = vy_c;
        out.corners[i].kappa_steady             = tire_out.kappa_steady;
        out.corners[i].alpha_steady_rad         = tire_out.alpha_steady_rad;
        out.corners[i].kappa_dynamic            = tire_out.kappa_next;
        out.corners[i].alpha_dynamic_rad        = tire_out.alpha_next_rad;
        out.corners[i].Fz_N                     = tire_out.Fz_N;
        out.corners[i].Fx_N                     = tire_out.Fx_N;
        out.corners[i].Fy_N                     = tire_out.Fy_N;
        out.corners[i].Mx_Nm                    = tire_out.Mx_Nm;
        out.corners[i].My_Nm                    = tire_out.My_Nm;
        out.corners[i].Mz_Nm                    = tire_out.Mz_Nm;
        out.corners[i].friction_utilization     = tire_out.friction_utilization;
        out.corners[i].compliance_camber_rad    = out.next_state.compliance_camber_state[i];
        out.corners[i].compliance_toe_rad       = out.next_state.compliance_toe_state[i];

        out.corners[i].joint_reactions = kin_solvers_[i].solve_joint_reactions(
            kin[i], delta_corner[i], F_tire_b, M_total_at_wc_b, arb_forces[i]);
    }

    // 4e. Electric Powertrain, Planetary Gearbox FBD, and Hydraulic / EBS Braking
    const double throttle_cmd = controls.ebs_active ? 0.0 : std::clamp(controls.throttle, 0.0, 1.0);
    const double brake_cmd    = controls.ebs_active ? 1.0 : std::clamp(controls.brake, 0.0, 1.0);

    // First-order hydraulic brake pressure lag
    const double alpha_brk = std::exp(-dt_for_tire_relax_s / std::max(1e-4, params_.brakes.hydraulic_tau_s));
    const double brk_norm  = alpha_brk * state.brake_pressure_norm + (1.0 - alpha_brk) * brake_cmd;
    out.next_state.brake_pressure_norm = brk_norm;

    // Desired yaw rate for active torque vectoring: r_ref = vx * delta / (L * (1 + K_us * vx^2))
    const double r_ref = ( state.v_b.x() * delta_rack ) / (params_.wheelbase_m * (1.0 + 0.0015 * vx * vx));
    const double yaw_error = (std::abs(vx) > 1.5) ? (r_ref - state.omega_b.z()) : 0.0;
    const double delta_tv_motor_Nm = std::clamp(
        controls.torque_vectoring_gain * params_.powertrain.tv_yaw_gain_Nm_per_rad_s * yaw_error / params_.powertrain.gear_ratio,
        -params_.powertrain.tv_max_delta_motor_Nm,
         params_.powertrain.tv_max_delta_motor_Nm);

    // Compute per-motor electromagnetic torque with field-weakening speed envelope and 80 kW FS cap
    const auto& pt = params_.powertrain;
    const double gear_ratio = (pt.mode == PowertrainParams::Mode::PLANETARY_IN_HUB_4WD)
        ? pt.planetary_ratio_ip()
        : pt.gear_ratio;

    std::array<double, 4> motor_trq_req{0.0, 0.0, 0.0, 0.0};
    double total_mech_power_req_W = 0.0;

    for (int i = 0; i < 4; ++i) {
        const bool is_driven = (pt.mode == PowertrainParams::Mode::PLANETARY_IN_HUB_4WD) || (i == RL || i == RR);
        if (!is_driven) continue;

        const double side_tv = (i == RL || i == FL) ? -0.5 * delta_tv_motor_Nm : +0.5 * delta_tv_motor_Nm;
        double tau_req = throttle_cmd * pt.peak_torque_per_motor_Nm + (throttle_cmd > 0.05 ? side_tv : 0.0);

        // Motor RPM & speed derating curve
        const double omega_motor = std::abs(state.omega_wheel[i]) * gear_ratio;
        const double rpm_motor   = omega_motor * (60.0 / (2.0 * M_PI));
        double rpm_envelope = 1.0;
        if (rpm_motor >= pt.max_motor_rpm) {
            rpm_envelope = 0.0;
        } else if (rpm_motor > pt.field_weakening_rpm) {
            rpm_envelope = (pt.max_motor_rpm - rpm_motor) / std::max(1.0, pt.max_motor_rpm - pt.field_weakening_rpm);
        }
        tau_req = std::clamp(tau_req, 0.0, pt.peak_torque_per_motor_Nm * rpm_envelope);
        motor_trq_req[i] = tau_req;
        total_mech_power_req_W += std::max(0.0, tau_req * omega_motor);
    }

    // Enforce Formula Student 80 kW total electrical power limit across all active motors
    const double max_mech_power_W = pt.fs_power_limit_W * pt.transmission_efficiency;
    const double power_scale = (total_mech_power_req_W > max_mech_power_W && total_mech_power_req_W > 1.0)
        ? (max_mech_power_W / total_mech_power_req_W)
        : 1.0;

    out.electrical_power_W = 0.0;
    const double alpha_mot = std::exp(-dt_for_tire_relax_s / std::max(1e-4, pt.torque_time_constant_s));

    for (int i = 0; i < 4; ++i) {
        const double tau_m_target = motor_trq_req[i] * power_scale;
        const double omega_m = std::abs(state.omega_wheel[i]) * gear_ratio;
        const double max_tau_by_power = (omega_m > 1.0)
            ? ((0.5 * max_mech_power_W) / omega_m)
            : pt.peak_torque_per_motor_Nm;
        const double tau_m = std::min(
            alpha_mot * state.motor_torque_actual_Nm[i] + (1.0 - alpha_mot) * tau_m_target,
            max_tau_by_power);
        out.next_state.motor_torque_actual_Nm[i] = tau_m;

        out.electrical_power_W += std::max(0.0, tau_m * omega_m) / std::max(0.5, pt.transmission_efficiency);

        // Net drive torque delivered to wheel spindle i
        double tau_drive_wheel = pt.transmission_efficiency * gear_ratio * tau_m;
        if (pt.mode == PowertrainParams::Mode::PLANETARY_IN_HUB_4WD && std::abs(state.omega_wheel[i]) > 0.01) {
            const double loss = pt.planetary_viscous_loss_Nm_s_rad * state.omega_wheel[i]
                              + pt.planetary_coulomb_seal_Nm * std::tanh(state.omega_wheel[i] * 5.0);
            tau_drive_wheel -= loss;
        }
        if (pt.mode == PowertrainParams::Mode::TER26_RWD_DUAL_MOTOR && (i == RL || i == RR)) {
            // Viscous differential coupling between RL and RR
            const double d_omega_rear = state.omega_wheel[RL] - state.omega_wheel[RR];
            const double tau_diff = pt.diff_viscous_coeff_Nm_s_rad * d_omega_rear
                                  + pt.diff_preload_Nm * std::tanh(d_omega_rear * 2.0);
            tau_drive_wheel += (i == RL) ? -tau_diff : +tau_diff;
        }

        // Hydraulic / EBS Brake Torque (opposes wheel rotation; caliper is bolted to upright!)
        // When hydraulic brake torque exceeds the tire spindle torque |tau_tire_spin| at near-zero wheel speed,
        // the brake pads physically lock the rotor to the caliper (elastoplastic stiction), preventing micro-creep!
        const double max_brk_total = controls.ebs_active
            ? params_.brakes.ebs_total_brake_torque_Nm
            : params_.brakes.max_total_brake_torque_Nm;
        const double corner_bias = (i < 2)
            ? (0.5 * params_.brakes.front_brake_bias)
            : (0.5 * (1.0 - params_.brakes.front_brake_bias));
        const double tau_brake_mag = brk_norm * max_brk_total * corner_bias;
        const double tau_net_on_wheel = tau_drive_wheel + Q(10 + i);
        double tau_brake_wheel = 0.0;
        if (std::abs(state.omega_wheel[i]) < 0.80 && tau_brake_mag > std::abs(tau_net_on_wheel)) {
            const double I_wheel_eff = params_.wheel_spin_inertia_kg_m2[i] + pt.reflected_inertia_at_wheel(i);
            const double tau_arrest = -tau_net_on_wheel - (I_wheel_eff / std::max(1e-4, dt_for_tire_relax_s)) * state.omega_wheel[i];
            tau_brake_wheel = std::clamp(tau_arrest, -tau_brake_mag, tau_brake_mag);
            out.solver_diagnostics.nonsmooth.brake_stiction_locked[i] = true;
        } else {
            tau_brake_wheel = -tau_brake_mag * std::tanh(state.omega_wheel[i] / 0.05);
        }

        // Apply drive + brake torque to wheel spin generalized coordinate (10 + i)
        Q(10 + i) += tau_drive_wheel + tau_brake_wheel;

        // Apply reaction torques to Upright vs Chassis according to physical mounting topology!
        // 1. Brake caliper is ALWAYS mounted on the upright, so -tau_brake_wheel reacts on the upright.
        //    Since upright pitch axis is coupled to chassis pitch (and jounce via domega_u_dz),
        //    Virtual Work projects -tau_brake_wheel * u_spin_b onto J_{omega_u,i}^T = [0, I_3, jw_i, 0]!
        // 2. Drive reaction torque:
        //    - If CHASSIS_INBOARD_CV (TeR26 RWD): motor + gearbox are bolted to the chassis; CV half-shafts
        //      transmit pure torque to the wheel hub without upright reaction torque. Thus -tau_drive_wheel * u_spin_b
        //      acts purely on the CHASSIS (Q[3..5]), while upright jounce (Q[6+i]) receives ONLY brake reaction!
        //    - If UPRIGHT_IN_HUB_PLANETARY: motor stator + fixed ring gear are bolted to the upright housing,
        //      so -tau_drive_wheel * u_spin_b reacts on the UPRIGHT (affecting both Q[3..5] and Q[6+i])!
        const Vec3 u_spin_b = kin[i].spindle_axis_b;
        const Vec3 tau_react_brake_b = -tau_brake_wheel * u_spin_b;
        const Vec3 tau_react_drive_b = -tau_drive_wheel * u_spin_b;

        // Chassis angular equation Q[3..5] receives the sum of upright + chassis reaction torques
        Q.segment<3>(3) += tau_react_brake_b + tau_react_drive_b;

        // Suspension jounce coordinate Q[6+i] receives upright-mounted reaction torques projected onto domega_u_dz
        double tau_upright_reaction = -tau_brake_wheel;
        if (pt.mounting == PowertrainParams::MountingLocation::UPRIGHT_IN_HUB_PLANETARY) {
            Q(6 + i) += kin[i].domega_u_dz.dot(tau_react_brake_b + tau_react_drive_b);
            tau_upright_reaction += -tau_drive_wheel;
        } else {
            Q(6 + i) += kin[i].domega_u_dz.dot(tau_react_brake_b);
        }

        out.corners[i].drive_torque_wheel_Nm      = tau_drive_wheel;
        out.corners[i].brake_torque_wheel_Nm      = tau_brake_wheel;
        out.corners[i].upright_reaction_torque_Nm = tau_upright_reaction;
    }

    // 4f. Continuous External Disturbance Wrench
    Q.segment<3>(0) += ext_interaction.body_force_b;
    Q.segment<3>(3) += ext_interaction.body_moment_b;

    // --- 5. Solve Coupled 14-DOF Linear System M(q) * dv/dt = Q - C(q, v) ---
    // Add tiny Tikhonov regularization on diagonal (1e-9) for guaranteed Cholesky stability
    Mat14 M_reg = M;
    M_reg.diagonal().array() += 1e-9;
    Eigen::LLT<Mat14> llt(M_reg);
    Vec14 dv_dt = llt.solve(Q - C);

    // Mass matrix conditioning diagnostics (kappa_2(M) and min eigenvalue)
    Eigen::SelfAdjointEigenSolver<Mat14> eig_M(M_reg, Eigen::EigenvaluesOnly);
    if (eig_M.info() == Eigen::Success) {
        const auto& evals = eig_M.eigenvalues();
        const double l_min = std::max(1e-12, evals(0));
        const double l_max = evals(13);
        out.solver_diagnostics.mass_matrix_min_eigenvalue   = evals(0);
        out.solver_diagnostics.mass_matrix_condition_number = l_max / l_min;
    }

    // Steering actuator second-order / rate-limited dynamics
    const double steer_cmd = std::clamp(controls.steering_cmd_rad,
                                        -params_.max_steering_angle_rad,
                                        params_.max_steering_angle_rad);
    const double omega_steer = 2.0 * M_PI * params_.steering_bandwidth_hz;
    const double steer_rate_raw = omega_steer * (steer_cmd - state.steering_rack_angle_rad);
    const double steer_rate = std::clamp(steer_rate_raw,
                                         -params_.steering_rate_limit_rad_s,
                                         params_.steering_rate_limit_rad_s);
    out.next_state.steering_rack_rate_rad_s = steer_rate;
    out.next_state.steering_rack_angle_rad  = std::clamp(
        state.steering_rack_angle_rad + dt_for_tire_relax_s * steer_rate,
        -params_.max_steering_angle_rad,
        params_.max_steering_angle_rad);

    // Proper specific force at CoG for IMU: f_imu^B = dv_b/dt + omega_b x v_b - R_WB^T * g_W
    out.specific_force_imu_b = dv_dt.segment<3>(0) + state.omega_b.cross(state.v_b) - g_b;

    out.generalized_accel  = dv_dt;
    out.mass_matrix        = M;
    out.coriolis_vector    = C;
    out.generalized_forces = Q;

    // Total Mechanical Energy E = T + V_grav + V_spring + V_arb
    Vec14 v_vec;
    v_vec.segment<3>(0) = state.v_b;
    v_vec.segment<3>(3) = state.omega_b;
    for (int i = 0; i < 4; ++i) v_vec(6 + i)  = state.v_susp[i];
    for (int i = 0; i < 4; ++i) v_vec(10 + i) = state.omega_wheel[i];

    const double T_kin = 0.5 * v_vec.dot(M * v_vec);
    double V_pot = params_.sprung_mass_kg * params_.gravity_m_s2 * state.p_w.z() + V_arb_energy_J;
    for (int i = 0; i < 4; ++i) {
        const Vec3 p_wc_w = state.p_w + state.R_wb * kin[i].wheel_center_b;
        V_pot += params_.unsprung_mass_kg[i] * params_.gravity_m_s2 * p_wc_w.z();
        const auto& sp = (i < 2) ? params_.susp_front : params_.susp_rear;
        const double xs = kin[i].spring_deflection_m;
        V_pot += 0.5 * sp.k_spring_N_m * xs * xs + (sp.preload_force_N + sp.gas_force_N) * xs;
        const double pen = out.corners[i].penetration_m;
        const auto& tp = (i < 2) ? params_.tire_front : params_.tire_rear;
        V_pot += 0.5 * tp.kz_N_m * pen * pen;
    }
    out.total_mechanical_energy_J = T_kin + V_pot;

    return out;
}

VehicleStepResult VehicleDynamicsEngine::step_vehicle_dynamics(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground,
    const ExternalInteraction& ext_interaction,
    double dt_s) const noexcept
{
    // --- Invalid-State (NaN / Inf / Non-Finite) Recovery Guard (§5.3) ---
    VehicleState safe_start = state;
    bool recovered_invalid = false;
    if (!state.to_flat_vector().allFinite() || !state.R_wb.allFinite()) {
        safe_start = compute_static_equilibrium(
            state.p_w.allFinite() ? state.p_w : Vec3(0.0, 0.0, 0.305),
            Vec3::Zero(),
            &ground);
        recovered_invalid = true;
    }

    const double dt_clamped = std::clamp(dt_s, 1e-5, 0.05);
    const int n_sub = (dt_clamped <= 0.015)
        ? std::max(1, params_.substeps)
        : std::max(params_.substeps, static_cast<int>(std::ceil(dt_clamped / 0.0025)));
    const double h  = dt_clamped / static_cast<double>(n_sub);

    VehicleState curr = safe_start;
    VehicleStepResult last_eval{};

    // --- Handle Discrete External Collision Impulse (Impulse-Momentum Bridge) ---
    // Delta_v = M(q)^{-1} * J_c(q)^T * Impulse
    Vec3 applied_impulse_w = Vec3::Zero();
    bool contact_grazing = false;
    if (ext_interaction.has_collision_impulse) {
        last_eval = evaluate_derivatives(curr, controls, ground, ExternalInteraction{}, h);
        const Vec3 r_c_b = curr.R_wb.transpose() * (ext_interaction.impulse_contact_point_w - curr.p_w);
        const Vec3 n_w   = ext_interaction.impulse_normal_w.normalized();
        const Vec3 n_b   = curr.R_wb.transpose() * n_w;

        // Contact point Jacobian J_c in Body Frame F_B: v_c^B = v_b - [r_c_b]_x * omega_b
        Mat3x14 J_c_b = Mat3x14::Zero();
        J_c_b.block<3, 3>(0, 0) = Mat3::Identity();
        J_c_b.block<3, 3>(0, 3) = -math::skew(r_c_b);

        const Vec3 v_c_w = curr.R_wb * (curr.v_b + curr.omega_b.cross(r_c_b));
        const Vec3 v_rel_w = v_c_w - ext_interaction.other_body_vel_w;
        const double v_rel_n = v_rel_w.dot(n_w);

        if (std::abs(v_rel_n) < 1e-3) {
            contact_grazing = true;
        }
        if (v_rel_n < 0.0) {
            Eigen::LLT<Mat14> llt(last_eval.mass_matrix);
            const Vec14 j_n_col = J_c_b.transpose() * n_b;
            const double eff_inv_mass_car = j_n_col.dot(llt.solve(j_n_col));
            const double inv_mass_other = (ext_interaction.other_body_mass_kg > 1e-4)
                ? (1.0 / ext_interaction.other_body_mass_kg)
                : 0.0;
            const double j_n = -(1.0 + std::clamp(ext_interaction.restitution, 0.0, 1.0)) * v_rel_n
                             / std::max(1e-6, eff_inv_mass_car + inv_mass_other);

            // Tangential Coulomb friction impulse
            const Vec3 v_rel_t_w = v_rel_w - v_rel_n * n_w;
            Vec3 j_t_w = Vec3::Zero();
            if (v_rel_t_w.norm() > 1e-5) {
                const double j_t_mag = std::min(ext_interaction.friction_mu * j_n,
                                                v_rel_t_w.norm() / std::max(1e-6, eff_inv_mass_car + inv_mass_other));
                j_t_w = -j_t_mag * v_rel_t_w.normalized();
            }

            const Vec3 J_impulse_w = j_n * n_w + j_t_w;
            const Vec3 J_impulse_b = curr.R_wb.transpose() * J_impulse_w;
            const Vec14 delta_v = llt.solve(J_c_b.transpose() * J_impulse_b);

            curr.v_b     += delta_v.segment<3>(0);
            curr.omega_b += delta_v.segment<3>(3);
            for (int i = 0; i < 4; ++i) curr.v_susp[i]      += delta_v(6 + i);
            for (int i = 0; i < 4; ++i) curr.omega_wheel[i] += delta_v(10 + i);
            applied_impulse_w = -J_impulse_w;
        }
    }

    ExternalInteraction cont_ext = ext_interaction;
    cont_ext.has_collision_impulse = false;

    auto get_v = [](const VehicleState& s) noexcept -> Vec14 {
        Vec14 v;
        v.segment<3>(0) = s.v_b;
        v.segment<3>(3) = s.omega_b;
        for (int i = 0; i < 4; ++i) v(6 + i)  = s.v_susp[i];
        for (int i = 0; i < 4; ++i) v(10 + i) = s.omega_wheel[i];
        return v;
    };

    auto advance_config = [](const VehicleState& base, const Vec14& v_step, const Vec14& dv_step, double dt_step) noexcept -> VehicleState {
        VehicleState next = base;
        next.v_b     = base.v_b     + dt_step * dv_step.segment<3>(0);
        next.omega_b = base.omega_b + dt_step * dv_step.segment<3>(3);
        for (int i = 0; i < 4; ++i) {
            next.v_susp[i]      = std::clamp(base.v_susp[i] + dt_step * dv_step(6 + i), -6.0, 6.0);
            next.omega_wheel[i] = std::clamp(base.omega_wheel[i] + dt_step * dv_step(10 + i), -400.0, 400.0);
        }
        const Vec3 v_b_mid     = v_step.segment<3>(0);
        const Vec3 omega_b_mid = v_step.segment<3>(3);
        next.p_w  = base.p_w + dt_step * (base.R_wb * v_b_mid);
        next.R_wb = math::project_to_so3(base.R_wb * math::exp_so3(dt_step * omega_b_mid));
        for (int i = 0; i < 4; ++i) {
            next.q_susp[i]    = std::clamp(base.q_susp[i] + dt_step * v_step(6 + i), -0.045, 0.045);
            next.phi_wheel[i] = base.phi_wheel[i] + dt_step * v_step(10 + i);
        }
        return next;
    };

    auto rk4_single_step = [&](const VehicleState& s_in, double dt_step, VehicleStepResult& k1_out, Vec14& v_rk_out, Vec14& dv_rk_out) noexcept -> VehicleState {
        k1_out = evaluate_derivatives(s_in, controls, ground, cont_ext, dt_step);
        const Vec14 v0  = get_v(s_in);
        const Vec14 dv1 = k1_out.generalized_accel;

        const VehicleState s2 = advance_config(k1_out.next_state, v0, dv1, 0.5 * dt_step);
        const VehicleStepResult k2 = evaluate_derivatives(s2, controls, ground, cont_ext, 0.5 * dt_step);
        const Vec14 v2  = get_v(s2);
        const Vec14 dv2 = k2.generalized_accel;

        const VehicleState s3 = advance_config(k1_out.next_state, v2, dv2, 0.5 * dt_step);
        const VehicleStepResult k3 = evaluate_derivatives(s3, controls, ground, cont_ext, 0.5 * dt_step);
        const Vec14 v3  = get_v(s3);
        const Vec14 dv3 = k3.generalized_accel;

        const VehicleState s4 = advance_config(k1_out.next_state, v3, dv3, dt_step);
        const VehicleStepResult k4 = evaluate_derivatives(s4, controls, ground, cont_ext, dt_step);
        const Vec14 v4  = get_v(s4);
        const Vec14 dv4 = k4.generalized_accel;

        v_rk_out  = (v0  + 2.0 * v2  + 2.0 * v3  + v4)  / 6.0;
        dv_rk_out = (dv1 + 2.0 * dv2 + 2.0 * dv3 + dv4) / 6.0;
        return advance_config(k1_out.next_state, v_rk_out, dv_rk_out, dt_step);
    };

    double max_lte_norm = 0.0;
    int total_substeps_executed = 0;
    int rejected_steps = 0;

    if (params_.integrator == IntegratorType::RKF45_ADAPTIVE_LIE) {
        // Embedded Adaptive Lie-Group Runge-Kutta with Richardson/Heun Local Truncation Error Control (§5.1)
        double t_rem = dt_clamped;
        double h_curr = h;
        while (t_rem > 1e-9 && total_substeps_executed < params_.max_adaptive_substeps) {
            const double h_try = std::min(h_curr, t_rem);
            VehicleStepResult k1{};
            Vec14 v_rk, dv_rk;
            const VehicleState s_rk4 = rk4_single_step(curr, h_try, k1, v_rk, dv_rk);

            // Embedded 2nd-order midpoint estimate s_rk2 for local truncation error e_lte = ||v_rk4 - v_rk2||
            const Vec14 v0 = get_v(curr);
            const VehicleState s_mid = advance_config(k1.next_state, v0, k1.generalized_accel, 0.5 * h_try);
            const VehicleStepResult k_mid = evaluate_derivatives(s_mid, controls, ground, cont_ext, 0.5 * h_try);
            const Vec14 v_rk2 = v0 + h_try * k_mid.generalized_accel;
            const Vec14 v_rk4 = get_v(s_rk4);
            const double lte = (v_rk4 - v_rk2).norm();
            const double tol_bound = params_.adaptive_atol + params_.adaptive_rtol * std::max(1.0, v_rk4.norm());

            if (lte <= tol_bound || h_try <= 1e-4 || total_substeps_executed + 1 >= params_.max_adaptive_substeps) {
                curr = s_rk4;
                curr.simulation_time_s += h_try;
                t_rem -= h_try;
                last_eval = k1;
                last_eval.generalized_accel = dv_rk;
                max_lte_norm = std::max(max_lte_norm, lte);
                ++total_substeps_executed;
                const double factor = (lte > 1e-14) ? std::clamp(0.9 * std::pow(tol_bound / lte, 0.25), 0.5, 2.0) : 1.5;
                h_curr = std::min(dt_clamped, h_try * factor);
            } else {
                ++rejected_steps;
                h_curr = std::max(1e-4, 0.5 * h_try);
            }
        }
    } else {
        for (int step = 0; step < n_sub; ++step) {
            if (params_.integrator == IntegratorType::SEMI_IMPLICIT_SUBSTEP) {
                last_eval = evaluate_derivatives(curr, controls, ground, cont_ext, h);
                const Vec14 v0  = get_v(curr);
                const Vec14 dv1 = last_eval.generalized_accel;
                const Vec14 v1  = v0 + h * dv1;
                curr = advance_config(last_eval.next_state, v1, dv1, h);
            } else {
                Vec14 v_rk, dv_rk;
                curr = rk4_single_step(curr, h, last_eval, v_rk, dv_rk);
                last_eval.generalized_accel = dv_rk;
            }
            curr.simulation_time_s += h;
            ++total_substeps_executed;
        }
    }

    // Final post-step NaN/divergence rollback check
    if (!curr.to_flat_vector().allFinite() || !curr.R_wb.allFinite()) {
        curr = safe_start;
        recovered_invalid = true;
    }

    last_eval.next_state = curr;
    last_eval.external_impulse_applied_w = applied_impulse_w;
    last_eval.solver_diagnostics.local_truncation_error_norm = max_lte_norm;
    last_eval.solver_diagnostics.adaptive_substeps_executed  = total_substeps_executed;
    last_eval.solver_diagnostics.rejected_adaptive_steps     = rejected_steps;
    last_eval.solver_diagnostics.recovered_from_invalid_state = recovered_invalid;
    if (contact_grazing) {
        last_eval.solver_diagnostics.nonsmooth.external_contact_grazing = true;
    }
    return last_eval;
}

void VehicleDynamicsEngine::compute_linearization_jacobians(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground,
    double dt_s,
    Eigen::Matrix<double, 38, 38>& A_out,
    Eigen::Matrix<double, 38, 3>& B_out) const noexcept
{
    VehicleParameters diff_params = params_;
    diff_params.differentiable_mode = true;
    diff_params.substeps = 1;
    VehicleDynamicsEngine diff_engine(diff_params);

    const ExternalInteraction ext_zero{};
    const auto x0 = state.to_flat_vector();

    const double eps_x = 1e-6;
    for (int j = 0; j < 38; ++j) {
        Eigen::Matrix<double, 38, 1> xp = x0;
        Eigen::Matrix<double, 38, 1> xm = x0;
        xp(j) += eps_x;
        xm(j) -= eps_x;

        VehicleState sp = state;
        VehicleState sm = state;
        sp.from_flat_vector(xp);
        sm.from_flat_vector(xm);

        const auto yp = diff_engine.step_vehicle_dynamics(sp, controls, ground, ext_zero, dt_s).next_state.to_flat_vector();
        const auto ym = diff_engine.step_vehicle_dynamics(sm, controls, ground, ext_zero, dt_s).next_state.to_flat_vector();
        A_out.col(j) = (yp - ym) / (2.0 * eps_x);
    }

    const double eps_u = 1e-5;
    for (int j = 0; j < 3; ++j) {
        ControlInputs up = controls;
        ControlInputs um = controls;
        if (j == 0) { up.throttle += eps_u; um.throttle -= eps_u; }
        if (j == 1) { up.brake    += eps_u; um.brake    -= eps_u; }
        if (j == 2) { up.steering_cmd_rad += eps_u; um.steering_cmd_rad -= eps_u; }

        const auto yp = diff_engine.step_vehicle_dynamics(state, up, ground, ext_zero, dt_s).next_state.to_flat_vector();
        const auto ym = diff_engine.step_vehicle_dynamics(state, um, ground, ext_zero, dt_s).next_state.to_flat_vector();
        B_out.col(j) = (yp - ym) / (2.0 * eps_u);
    }
}

void VehicleDynamicsEngine::compute_parameter_sensitivities(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground,
    double dt_s,
    Eigen::Matrix<double, 38, 24>& S_p_out) const noexcept
{
    VehicleParameters base_params = params_;
    base_params.differentiable_mode = true;
    base_params.substeps = 1;
    const Eigen::Matrix<double, 24, 1> p0 = base_params.to_parameter_vector();
    const ExternalInteraction ext_zero{};

    for (int j = 0; j < 24; ++j) {
        const double eps_p = std::max(1e-6, 1e-5 * std::abs(p0(j)));
        Eigen::Matrix<double, 24, 1> pp = p0;
        Eigen::Matrix<double, 24, 1> pm = p0;
        pp(j) += eps_p;
        pm(j) -= eps_p;

        VehicleParameters params_p = base_params;
        VehicleParameters params_m = base_params;
        params_p.from_parameter_vector(pp);
        params_m.from_parameter_vector(pm);

        VehicleDynamicsEngine eng_p(params_p);
        VehicleDynamicsEngine eng_m(params_m);

        const auto yp = eng_p.step_vehicle_dynamics(state, controls, ground, ext_zero, dt_s).next_state.to_flat_vector();
        const auto ym = eng_m.step_vehicle_dynamics(state, controls, ground, ext_zero, dt_s).next_state.to_flat_vector();
        S_p_out.col(j) = (yp - ym) / (2.0 * eps_p);
    }
}

void VehicleDynamicsEngine::evaluate_jvp(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground,
    double dt_s,
    const Eigen::Matrix<double, 38, 1>& dx,
    const Eigen::Matrix<double, 3, 1>& du,
    const Eigen::Matrix<double, 24, 1>& dp,
    Eigen::Matrix<double, 38, 1>& dy_out) const noexcept
{
    Eigen::Matrix<double, 38, 38> A;
    Eigen::Matrix<double, 38, 3> B;
    Eigen::Matrix<double, 38, 24> S_p;
    compute_linearization_jacobians(state, controls, ground, dt_s, A, B);
    compute_parameter_sensitivities(state, controls, ground, dt_s, S_p);
    dy_out = A * dx + B * du + S_p * dp;
}

void VehicleDynamicsEngine::evaluate_vjp(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground,
    double dt_s,
    const Eigen::Matrix<double, 38, 1>& cotangent_y,
    Eigen::Matrix<double, 38, 1>& grad_x_out,
    Eigen::Matrix<double, 3, 1>& grad_u_out,
    Eigen::Matrix<double, 24, 1>& grad_p_out) const noexcept
{
    Eigen::Matrix<double, 38, 38> A;
    Eigen::Matrix<double, 38, 3> B;
    Eigen::Matrix<double, 38, 24> S_p;
    compute_linearization_jacobians(state, controls, ground, dt_s, A, B);
    compute_parameter_sensitivities(state, controls, ground, dt_s, S_p);
    grad_x_out = A.transpose()   * cotangent_y;
    grad_u_out = B.transpose()   * cotangent_y;
    grad_p_out = S_p.transpose() * cotangent_y;
}

std::array<CornerStructuralLoadsANSYS, 4> VehicleDynamicsEngine::compute_ansys_corner_loads(
    const VehicleState& state,
    const ControlInputs& controls,
    const std::array<GroundContactQuery, 4>& ground) const noexcept
{
    const ExternalInteraction ext_zero{};
    const VehicleStepResult step_res = evaluate_derivatives(state, controls, ground, ext_zero, 0.0025);
    const double delta_rack = std::clamp(state.steering_rack_angle_rad,
                                         -params_.max_steering_angle_rad,
                                         params_.max_steering_angle_rad);
    std::array<double, 4> delta_corner{0.0, 0.0, 0.0, 0.0};
    if (std::abs(delta_rack) > 1e-7) {
        const double tan_d = std::tan(delta_rack);
        const double half_track_over_L = (0.5 * params_.track_front_m * params_.ackermann_factor) / params_.wheelbase_m;
        delta_corner[FL] = -std::atan(tan_d / std::max(0.2, 1.0 - tan_d * half_track_over_L));
        delta_corner[FR] = -std::atan(tan_d / std::max(0.2, 1.0 + tan_d * half_track_over_L));
    }

    const Vec3 g_b = state.R_wb.transpose() * Vec3(0.0, 0.0, -params_.gravity_m_s2);
    const Vec3 alpha_chassis_b = step_res.generalized_accel.segment<3>(3);

    std::array<CornerStructuralLoadsANSYS, 4> loads{};
    for (int i = 0; i < 4; ++i) {
        const auto kin_i = kin_solvers_[i].solve(state.q_susp[i], delta_corner[i]);
        const Vec3 n_w = ground[i].normal_w.normalized();
        const Vec3 n_b = state.R_wb.transpose() * n_w;
        const Vec3 u_spin_w = (state.R_wb * kin_i.wheel_rot_b.col(1)).normalized();
        Vec3 e_x_w = u_spin_w.cross(n_w);
        if (e_x_w.norm() > 1e-6) e_x_w.normalize(); else e_x_w = state.R_wb.col(0);
        const Vec3 e_y_w = n_w.cross(e_x_w).normalized();
        const Vec3 e_x_b = state.R_wb.transpose() * e_x_w;
        const Vec3 e_y_b = state.R_wb.transpose() * e_y_w;

        const auto& cd = step_res.corners[i];
        const Vec3 F_tire_b = cd.Fx_N * e_x_b + cd.Fy_N * e_y_b + cd.Fz_N * n_b;
        const Vec3 M_patch_b = cd.Mx_Nm * e_x_b + cd.My_Nm * e_y_b + cd.Mz_Nm * n_b;
        const Vec3 r_wc_to_cp_b = -cd.effective_rolling_radius_m * n_b;
        const Vec3 M_total_at_wc_b = M_patch_b + r_wc_to_cp_b.cross(F_tire_b)
                                   + (-cd.brake_torque_wheel_Nm) * kin_i.spindle_axis_b;

        loads[i] = compute_dynamic_structural_loads_ansys(
            i, params_.hardpoints[i], params_.component_inertias,
            params_.unsprung_mass_kg[i], params_.upright_inertia[i],
            params_.wheel_spin_inertia_kg_m2[i], state.omega_wheel[i],
            kin_i, cd.joint_reactions, F_tire_b, M_total_at_wc_b,
            -cd.brake_torque_wheel_Nm, cd.upright_reaction_torque_Nm + cd.brake_torque_wheel_Nm,
            step_res.generalized_accel.segment<3>(0), alpha_chassis_b, state.omega_b,
            state.v_susp[i], step_res.generalized_accel(6 + i), g_b);
    }
    return loads;
}

} // namespace ter26

// ============================================================================
// C-ABI Implementation for Python / ctypes / JAX @custom_vjp Bridge (§7)
// ============================================================================
extern "C" {

static std::array<ter26::GroundContactQuery, 4> make_flat_ground_c_abi() noexcept {
    std::array<ter26::GroundContactQuery, 4> g{};
    for (int i = 0; i < 4; ++i) {
        g[i].point_w  = ter26::Vec3::Zero();
        g[i].normal_w = ter26::Vec3::UnitZ();
        g[i].friction_multiplier = 1.0;
    }
    return g;
}

void ter26_step_c_abi(
    const double* x38_in,
    const double* u3_in,
    const double* p24_in,
    double dt_s,
    int differentiable_mode,
    double* x38_out,
    double* energy_out,
    int* nonsmooth_flag_out)
{
    ter26::VehicleParameters params = ter26::VehicleDynamicsEngine::default_ter26_parameters();
    if (p24_in != nullptr) {
        Eigen::Map<const Eigen::Matrix<double, 24, 1>> p_vec(p24_in);
        params.from_parameter_vector(p_vec);
    }
    params.differentiable_mode = (differentiable_mode != 0);
    params.substeps = 1;
    ter26::VehicleDynamicsEngine engine(params);

    ter26::VehicleState s{};
    Eigen::Map<const Eigen::Matrix<double, 38, 1>> x_in(x38_in);
    s.from_flat_vector(x_in);

    ter26::ControlInputs u{};
    u.throttle         = u3_in[0];
    u.brake            = u3_in[1];
    u.steering_cmd_rad = u3_in[2];

    const auto g = make_flat_ground_c_abi();
    const ter26::ExternalInteraction ext{};
    const auto res = engine.step_vehicle_dynamics(s, u, g, ext, dt_s);

    const auto x_next = res.next_state.to_flat_vector();
    Eigen::Map<Eigen::Matrix<double, 38, 1>> map_x_out(x38_out);
    map_x_out = x_next;
    if (energy_out != nullptr) {
        *energy_out = res.total_mechanical_energy_J;
    }
    if (nonsmooth_flag_out != nullptr) {
        *nonsmooth_flag_out = res.solver_diagnostics.nonsmooth.any_nonsmooth_event() ? 1 : 0;
    }
}

void ter26_vjp_c_abi(
    const double* x38_in,
    const double* u3_in,
    const double* p24_in,
    const double* cotangent_y38_in,
    double dt_s,
    double* grad_x38_out,
    double* grad_u3_out,
    double* grad_p24_out)
{
    ter26::VehicleParameters params = ter26::VehicleDynamicsEngine::default_ter26_parameters();
    if (p24_in != nullptr) {
        Eigen::Map<const Eigen::Matrix<double, 24, 1>> p_vec(p24_in);
        params.from_parameter_vector(p_vec);
    }
    params.differentiable_mode = true;
    params.substeps = 1;
    ter26::VehicleDynamicsEngine engine(params);

    ter26::VehicleState s{};
    Eigen::Map<const Eigen::Matrix<double, 38, 1>> x_in(x38_in);
    s.from_flat_vector(x_in);

    ter26::ControlInputs u{};
    u.throttle         = u3_in[0];
    u.brake            = u3_in[1];
    u.steering_cmd_rad = u3_in[2];

    Eigen::Map<const Eigen::Matrix<double, 38, 1>> cot_y(cotangent_y38_in);
    Eigen::Matrix<double, 38, 1> gx;
    Eigen::Matrix<double, 3, 1>  gu;
    Eigen::Matrix<double, 24, 1> gp;

    const auto g = make_flat_ground_c_abi();
    engine.evaluate_vjp(s, u, g, dt_s, cot_y, gx, gu, gp);

    if (grad_x38_out != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 38, 1>> map_gx(grad_x38_out);
        map_gx = gx;
    }
    if (grad_u3_out != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> map_gu(grad_u3_out);
        map_gu = gu;
    }
    if (grad_p24_out != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 24, 1>> map_gp(grad_p24_out);
        map_gp = gp;
    }
}

void ter26_jacobians_c_abi(
    const double* x38_in,
    const double* u3_in,
    const double* p24_in,
    double dt_s,
    double* A_38x38_rowmajor_out,
    double* B_38x3_rowmajor_out,
    double* Sp_38x24_rowmajor_out)
{
    ter26::VehicleParameters params = ter26::VehicleDynamicsEngine::default_ter26_parameters();
    if (p24_in != nullptr) {
        Eigen::Map<const Eigen::Matrix<double, 24, 1>> p_vec(p24_in);
        params.from_parameter_vector(p_vec);
    }
    params.differentiable_mode = true;
    params.substeps = 1;
    ter26::VehicleDynamicsEngine engine(params);

    ter26::VehicleState s{};
    Eigen::Map<const Eigen::Matrix<double, 38, 1>> x_in(x38_in);
    s.from_flat_vector(x_in);

    ter26::ControlInputs u{};
    u.throttle         = u3_in[0];
    u.brake            = u3_in[1];
    u.steering_cmd_rad = u3_in[2];

    const auto g = make_flat_ground_c_abi();
    Eigen::Matrix<double, 38, 38> A;
    Eigen::Matrix<double, 38, 3>  B;
    Eigen::Matrix<double, 38, 24> Sp;
    engine.compute_linearization_jacobians(s, u, g, dt_s, A, B);
    engine.compute_parameter_sensitivities(s, u, g, dt_s, Sp);

    using RowMaj38x38 = Eigen::Matrix<double, 38, 38, Eigen::RowMajor>;
    using RowMaj38x3  = Eigen::Matrix<double, 38, 3,  Eigen::RowMajor>;
    using RowMaj38x24 = Eigen::Matrix<double, 38, 24, Eigen::RowMajor>;

    if (A_38x38_rowmajor_out != nullptr) {
        Eigen::Map<RowMaj38x38> map_A(A_38x38_rowmajor_out);
        map_A = A;
    }
    if (B_38x3_rowmajor_out != nullptr) {
        Eigen::Map<RowMaj38x3> map_B(B_38x3_rowmajor_out);
        map_B = B;
    }
    if (Sp_38x24_rowmajor_out != nullptr) {
        Eigen::Map<RowMaj38x24> map_Sp(Sp_38x24_rowmajor_out);
        map_Sp = Sp;
    }
}

} // extern "C"
