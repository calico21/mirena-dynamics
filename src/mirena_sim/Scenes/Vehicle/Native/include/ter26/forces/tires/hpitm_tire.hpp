#pragma once
/**
 * @file hpitm_tire.hpp
 * @brief Hybrid Physics-Informed Tire Model (HPITM):
 *        - Full Pacejka Magic Formula 6.2 (Fx, Fy, Fz, Mx, My, Mz) with pure + combined slip
 *        - Load, camber, and inflation pressure sensitivity
 *        - Bounded symmetry-preserving neural parameter modulation
 *        - First-order relaxation-length transient slip ODEs with C^infinity low-speed regularization
 *        - Elastoplastic 2D bristle static friction model for slope standstill holding
 */

#include "ter26/state/vehicle_state.hpp"

namespace ter26 {

/// Selectable transient tire formulation (strictly mutually exclusive to prevent double-counting compliance)
enum class TransientTireMode {
    RELAXATION_LENGTH_1ST_ORDER = 0, ///< Quasi-static carcass compliance (sigma_x = C_Fk/k_cx, sigma_y = C_Fa/k_cy)
    DYNAMIC_CARCASS_2ND_ORDER   = 1  ///< Inertial belt/carcass 2nd-order oscillator (m_belt, k_c, c_c)
};

/// Optional 2nd-order carcass + 2-node thermal state per tire
struct TireCarcassThermalState {
    Vec2 carcass_defl_m{Vec2::Zero()};     ///< [e_cx, e_cy] carcass deflection [m]
    Vec2 carcass_vel_m_s{Vec2::Zero()};    ///< [de_cx/dt, de_cy/dt] carcass deflection rate [m/s]
    double bore_twist_rad{0.0};            ///< Contact patch torsional bristle twist for parking turn-slip [rad]
    double T_surface_C{65.0};              ///< Tread surface temperature [deg C]
    double T_core_C{65.0};                 ///< Bulk carcass core temperature [deg C]
};

struct HPITMTireParams {
    TransientTireMode transient_mode{TransientTireMode::RELAXATION_LENGTH_1ST_ORDER};

    // Dimensions & Nominal
    double R0_m{0.2032};
    double width_m{0.1905};
    double Fz0_N{667.0};
    double p0_kPa{82.7};
    double p_actual_kPa{82.7};
    double max_Fz_N{2500.0};

    // Vertical contact stiffness & effective radius
    double kz_N_m{98377.0};
    double kz_nl_N_m2{180000.0};
    double cz_N_s_m{320.0};
    double camber_stiffness_factor{0.08};
    double dreff{0.24};
    double breff{6.5};
    double freff{0.06};

    // Scaling factors (lambda_mu = 1.0 on Calspan sandpaper, ~0.64 on asphalt track)
    double lambda_mux{0.68};
    double lambda_muy{0.64};
    double lambda_Kxk{1.0};
    double lambda_Kya{0.92};
    double lambda_Cx{1.0};
    double lambda_Cy{1.0};
    double lambda_Ex{1.0};
    double lambda_Ey{1.0};
    double lambda_Mx{1.0};
    double lambda_My{1.0};
    double lambda_Mz{1.0};

    // Longitudinal MF6.2
    double pCx1{1.62};
    double pDx1{2.48};
    double pDx2{-0.22};
    double pDx3{4.5};
    double pEx1{0.28};
    double pEx2{-0.08};
    double pEx3{0.0};
    double pEx4{0.05};
    double pKx1{38.5};
    double pKx2{-4.2};
    double pKx3{0.12};
    double pHx1{0.0};
    double pHx2{0.0};
    double pVx1{0.0};
    double pVx2{0.0};
    double rBx1{14.5};
    double rBx2{11.2};
    double rCx1{1.04};
    double rEx1{-0.15};

    // Lateral MF6.2
    double pCy1{1.46};
    double pDy1{2.62};
    double pDy2{-0.26};
    double pDy3{6.2};
    double pEy1{-0.45};
    double pEy2{-0.18};
    double pEy3{0.05};
    double pEy4{-2.5};
    double pKy1{-36.8};
    double pKy2{1.85};
    double pKy3{0.45};
    double pKy4{2.00};
    double pKy6{-1.15};
    double pKy7{-0.25};
    double pHy1{0.0012};
    double pHy2{-0.0005};
    double pVy1{0.015};
    double pVy2{-0.010};
    double pVy3{-0.35};
    double pVy4{-0.15};
    double rBy1{11.8};
    double rBy2{9.6};
    double rCy1{1.02};
    double rEy1{-0.12};

    // Moments MF6.2 (Mx, My, Mz)
    double qSx1{0.005};
    double qSx2{1.15};
    double qSx3{0.08};
    double qSy1{0.014};
    double qSy2{0.025};
    double qSy3{0.004};
    double qSy4{0.0005};
    double qBz1{10.5};
    double qBz2{-1.8};
    double qBz3{-0.4};
    double qBz9{16.0};
    double qBz10{0.8};
    double qCz1{1.18};
    double qDz1{0.115};
    double qDz2{-0.018};
    double qDz3{0.15};
    double qDz6{0.002};
    double qDz8{-0.28};
    double qDz9{-0.05};
    double qEz1{-1.2};
    double qEz2{-0.3};
    double sSz1{0.015};
    double sSz2{0.022};

    // Transient, 2nd-Order Carcass, Turn-Slip & Bristle Static Holding
    double sigma_x_m{0.12};
    double sigma_y_m{0.18};
    double m_belt_kg{1.85};               ///< Effective belt mass for 2nd-order carcass dynamics [kg]
    double k_carcass_x_N_m{195000.0};     ///< Longitudinal carcass stiffness [N/m]
    double k_carcass_y_N_m{115000.0};     ///< Lateral carcass stiffness [N/m]
    double c_carcass_x_N_s_m{820.0};      ///< Longitudinal carcass damping [N*s/m]
    double c_carcass_y_N_s_m{680.0};      ///< Lateral carcass damping [N*s/m]
    double k_bore_Nm_rad{340.0};          ///< Torsional bristle stiffness for parking turn-slip [N*m/rad]
    double c_bore_Nm_s_rad{14.0};         ///< Torsional bore damping [N*m*s/rad]
    double B_y_phi{4.2};                  ///< Turn-slip lateral reduction factor [-]
    double v_min_reg_m_s{0.25};
    double k_bristle_N_m{135000.0};
    double c_bristle_N_s_m{1800.0};
    double v_blend_m_s{0.25};
    double dv_blend_m_s{0.08};

    // 2-Node Thermal & Grip Degradation Parameters
    bool enable_thermal_modulation{false};
    double T_opt_C{75.0};                 ///< Optimal tread temperature [deg C]
    double T_ambient_C{25.0};             ///< Ambient / track temperature [deg C]
    double dT_grip_halfwidth_C{45.0};     ///< Thermal grip bell-curve half-width [deg C]
    double max_thermal_penalty{0.18};     ///< Max grip drop off optimal temperature [-]
    double C_thermal_surf_J_K{420.0};     ///< Surface layer heat capacity [J/K]
    double C_thermal_core_J_K{2800.0};    ///< Bulk core heat capacity [J/K]
    double h_cond_surf_core_W_K{18.0};    ///< Surface-to-core thermal conductance [W/K]
    double h_conv_ambient_W_K{24.0};      ///< Convective cooling to air/road [W/K]

    // Bounded Neural HPITM Correction
    bool enable_neural_correction{true};
    double max_neural_bound{0.12};
    std::array<double, 8> w1_alpha_sq_dfz{-0.35, 0.22, -0.18, 0.41, -0.12, 0.28, -0.31, 0.19};
    std::array<double, 8> w1_gamma_sq{0.15, -0.24, 0.31, -0.14, 0.22, -0.19, 0.11, -0.27};
    std::array<double, 8> b1{-0.05, 0.04, -0.02, 0.06, -0.03, 0.02, -0.04, 0.03};
    std::array<double, 8> w2_muy{0.28, -0.19, 0.24, 0.32, -0.15, 0.21, -0.25, 0.18};
    std::array<double, 8> w2_kya{-0.14, 0.26, -0.18, 0.15, 0.29, -0.22, 0.17, -0.12};
};

struct TireForceOutput {
    double Fz_N{0.0};
    double Fx_N{0.0};
    double Fy_N{0.0};
    double Mx_Nm{0.0};
    double My_Nm{0.0};
    double Mz_Nm{0.0};
    double Mz_turn_slip_Nm{0.0};          ///< Turn-slip / parking bore torque contribution [N*m]
    double r_eff_m{0.2032};
    double contact_patch_half_length_m{0.055};
    double kappa_steady{0.0};
    double alpha_steady_rad{0.0};
    double kappa_next{0.0};
    double alpha_next_rad{0.0};
    Vec2 bristle_next_m{Vec2::Zero()};
    double friction_utilization{0.0};
    double friction_ellipse_limit_N{0.0};
    TireCarcassThermalState carcass_thermal_next{};
};

class HPITMTireModel {
public:
    explicit HPITMTireModel(const HPITMTireParams& params = HPITMTireParams{})
        : params_(params) {}

    void set_params(const HPITMTireParams& p) noexcept { params_ = p; }
    const HPITMTireParams& params() const noexcept { return params_; }

    /**
     * @brief Evaluate steady-state MF6.2 + Bounded Neural Correction forces & moments
     *        for given (kappa, alpha_rad, Fz_N, gamma_rad, vx_m_s, turn_rate_rad_s).
     *        Used both by the dynamic solver and by the Calspan TTC fitting/validation tests.
     */
    TireForceOutput evaluate_steady_state(
        double kappa,
        double alpha_rad,
        double Fz_N,
        double gamma_rad,
        double vx_m_s = 12.0,
        double friction_mult = 1.0,
        double turn_rate_rad_s = 0.0) const noexcept;

    /**
     * @brief Evaluate full dynamic tire contact patch response over timestep dt,
     *        including unilateral normal force Fz, effective rolling radius r_eff,
     *        exponentially-integrated relaxation slip states (kappa_dyn, alpha_dyn),
     *        turn-slip bore torque, and elastoplastic 2D bristle static friction.
     */
    TireForceOutput evaluate_dynamic(
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
        double friction_mult = 1.0,
        bool differentiable_softplus = false,
        double turn_rate_rad_s = 0.0,
        const TireCarcassThermalState* carcass_state_in = nullptr) const noexcept;

private:
    HPITMTireParams params_;

    void evaluate_neural_modulation(
        double alpha_rad,
        double dfz,
        double gamma_rad,
        double& delta_lambda_muy,
        double& delta_lambda_kya) const noexcept;
};

} // namespace ter26
