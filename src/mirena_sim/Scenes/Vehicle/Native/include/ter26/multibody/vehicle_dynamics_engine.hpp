#pragma once
/**
 * @file vehicle_dynamics_engine.hpp
 * @brief Complete 14-DOF Coupled Multibody Vehicle Dynamics Engine for Formula Student TeR26.
 *
 * Implements:
 *   - 6-DOF chassis dynamics on SE(3) + 4 suspension jounce DOFs + 4 wheel spin DOFs (14 DOFs)
 *   - Exact configuration-dependent mass matrix M(q) = sum_k J_k(q)^T M_k J_k(q)
 *   - Convective, Coriolis, centrifugal, and wheel gyroscopic accelerations C(q, v)
 *   - 3D nonlinear suspension kinematics & 6x6 link reaction force solver for ANSYS
 *   - HPITM Tire Model (MF6.2 + bounded neural correction + relaxation length + bristle static holding)
 *   - TeR26 RWD dual-motor powertrain AND explicit 3-element planetary gearbox FBD
 *   - Ride-height/pitch/roll sensitive aerodynamics & hydraulic/EBS braking
 *   - Impulse-momentum collision bridge for cones and external rigid bodies
 *   - Forward sensitivity / Jacobian evaluation for system identification and optimization
 */

#include "ter26/state/vehicle_state.hpp"
#include "ter26/kinematics/suspension_kinematics.hpp"
#include "ter26/forces/tires/hpitm_tire.hpp"
#include "ter26/forces/subsystems.hpp"
#include "ter26/multibody/component_multibody.hpp"
#include "ter26/contact/contact_solver.hpp"
#include <string>

namespace ter26 {

enum class IntegratorType {
    SEMI_IMPLICIT_SUBSTEP = 0,
    RK4_LIE = 1,
    RKF45_ADAPTIVE_LIE = 2
};

struct VehicleParameters {
    // Master dimensions & mass properties
    double wheelbase_m{1.593};
    double lf_m{0.876};
    double lr_m{0.717};
    double track_front_m{1.174};
    double track_rear_m{1.174};
    double h_cg_m{0.330};
    double gravity_m_s2{9.81};

    double sprung_mass_kg{269.0};
    Mat3 sprung_inertia_b{
        (Mat3() << 42.5, 0.0,  2.4,
                   0.0, 78.0,  0.0,
                   2.4,  0.0, 118.0).finished()
    };

    std::array<double, 4> unsprung_mass_kg{7.74, 7.74, 7.76, 7.76};
    std::array<Mat3, 4> upright_inertia{
        Mat3(Vec3(0.028, 0.022, 0.026).asDiagonal()),
        Mat3(Vec3(0.028, 0.022, 0.026).asDiagonal()),
        Mat3(Vec3(0.030, 0.024, 0.028).asDiagonal()),
        Mat3(Vec3(0.030, 0.024, 0.028).asDiagonal())
    };
    std::array<double, 4> wheel_spin_inertia_kg_m2{0.24, 0.24, 0.24, 0.24};
    std::array<double, 4> wheel_diametral_inertia_kg_m2{0.14, 0.14, 0.14, 0.14};

    // Steering actuator
    double max_steering_angle_rad{0.45};
    double steering_bandwidth_hz{12.0};
    double steering_rate_limit_rad_s{4.5};
    double ackermann_factor{0.85};

    // Subsystems
    std::array<CornerHardpoints, 4> hardpoints{};
    AxleSuspensionParams susp_front{};
    AxleSuspensionParams susp_rear{};
    HPITMTireParams tire_front{};
    HPITMTireParams tire_rear{};
    PowertrainParams powertrain{};
    BrakeParams brakes{};
    AeroParams aero{};

    // Component-level multibody & physical 3D U-bar ARB extensions
    SuspensionComponentInertias component_inertias{false};
    bool use_physical_3d_arb{false};
    PhysicalARBParams physical_arb_front{};
    PhysicalARBParams physical_arb_rear{};

    // Integration & adaptive step control settings
    IntegratorType integrator{IntegratorType::RK4_LIE};
    int substeps{2};
    double adaptive_rtol{1e-5};
    double adaptive_atol{1e-7};
    int max_adaptive_substeps{32};
    bool differentiable_mode{false};

    /// Pack 24 primary vehicle/suspension/tire/aero/powertrain parameters into R^24 for sensitivity/identification
    Eigen::Matrix<double, 24, 1> to_parameter_vector() const noexcept {
        Eigen::Matrix<double, 24, 1> p;
        p(0)  = sprung_mass_kg;
        p(1)  = h_cg_m;
        p(2)  = lf_m;
        p(3)  = sprung_inertia_b(2, 2);
        p(4)  = susp_front.k_spring_N_m;
        p(5)  = susp_rear.k_spring_N_m;
        p(6)  = susp_front.c_low_bump_N_s_m;
        p(7)  = susp_rear.c_low_bump_N_s_m;
        p(8)  = susp_front.arb_wheel_rate_N_m;
        p(9)  = susp_rear.arb_wheel_rate_N_m;
        p(10) = tire_front.kz_N_m;
        p(11) = tire_rear.kz_N_m;
        p(12) = tire_front.lambda_mux;
        p(13) = tire_front.lambda_muy;
        p(14) = tire_rear.lambda_mux;
        p(15) = tire_rear.lambda_muy;
        p(16) = tire_front.lambda_Kya;
        p(17) = tire_rear.lambda_Kya;
        p(18) = tire_front.sigma_y_m;
        p(19) = tire_rear.sigma_y_m;
        p(20) = aero.cl_nominal;
        p(21) = aero.cd_nominal;
        p(22) = aero.aero_balance_front;
        p(23) = powertrain.peak_torque_per_motor_Nm;
        return p;
    }

    void from_parameter_vector(const Eigen::Matrix<double, 24, 1>& p) noexcept {
        sprung_mass_kg               = p(0);
        h_cg_m                       = p(1);
        lf_m                         = p(2);
        lr_m                         = wheelbase_m - lf_m;
        hardpoints[0].axle_x_offset_from_cg_m = lf_m;
        hardpoints[1].axle_x_offset_from_cg_m = lf_m;
        hardpoints[2].axle_x_offset_from_cg_m = -lr_m;
        hardpoints[3].axle_x_offset_from_cg_m = -lr_m;
        sprung_inertia_b(2, 2)       = p(3);
        susp_front.k_spring_N_m      = p(4);
        susp_rear.k_spring_N_m       = p(5);
        susp_front.c_low_bump_N_s_m  = p(6);
        susp_rear.c_low_bump_N_s_m   = p(7);
        susp_front.arb_wheel_rate_N_m= p(8);
        susp_rear.arb_wheel_rate_N_m = p(9);
        tire_front.kz_N_m            = p(10);
        tire_rear.kz_N_m             = p(11);
        tire_front.lambda_mux        = p(12);
        tire_front.lambda_muy        = p(13);
        tire_rear.lambda_mux         = p(14);
        tire_rear.lambda_muy         = p(15);
        tire_front.lambda_Kya        = p(16);
        tire_rear.lambda_Kya         = p(17);
        tire_front.sigma_y_m         = p(18);
        tire_rear.sigma_y_m          = p(19);
        aero.cl_nominal              = p(20);
        aero.cd_nominal              = p(21);
        aero.aero_balance_front      = p(22);
        powertrain.peak_torque_per_motor_Nm = p(23);
    }
};

class VehicleDynamicsEngine {
public:
    explicit VehicleDynamicsEngine(const VehicleParameters& params = default_ter26_parameters());

    static VehicleParameters default_ter26_parameters() noexcept;

    bool load_parameters_from_json_dir(const std::string& data_dir_path);

    void set_parameters(const VehicleParameters& params) noexcept;
    const VehicleParameters& parameters() const noexcept { return params_; }

    /**
     * @brief Compute static equilibrium state on flat or inclined ground
     *        so the car starts settled at its exact equilibrium ride height and jounce.
     */
    VehicleState compute_static_equilibrium(
        const Vec3& initial_pos_w = Vec3(0.0, 0.0, 0.305),
        const Vec3& initial_rpy_rad = Vec3::Zero(),
        const std::array<GroundContactQuery, 4>* ground_queries = nullptr) const noexcept;

    /**
     * @brief Deterministic single-step integration:
     *        (x_{k+1}, diag_k) = step_vehicle_dynamics(x_k, u_k, ground_k, ext_k, dt)
     */
    VehicleStepResult step_vehicle_dynamics(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground,
        const ExternalInteraction& ext_interaction,
        double dt_s) const noexcept;

    /**
     * @brief Evaluate the continuous-time multibody equations of motion
     *        M(q) * dv/dt + C(q, v) = Q(q, v, z, u)
     *        without advancing the state.
     */
    VehicleStepResult evaluate_derivatives(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground,
        const ExternalInteraction& ext_interaction,
        double dt_for_tire_relax_s = 0.0025) const noexcept;

    /**
     * @brief Compute state transition Jacobian A = dx_{k+1}/dx_k (38x38)
     *        and control Jacobian B = dx_{k+1}/du_k (38x3) via central finite differences
     *        on smooth differentiable physics.
     */
    void compute_linearization_jacobians(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground,
        double dt_s,
        Eigen::Matrix<double, 38, 38>& A_out,
        Eigen::Matrix<double, 38, 3>& B_out) const noexcept;

    /**
     * @brief Compute parameter sensitivity matrix S_p = dx_{k+1}/dp (38x24)
     *        with respect to the 24 physical vehicle/suspension/tire/aero/powertrain parameters.
     */
    void compute_parameter_sensitivities(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground,
        double dt_s,
        Eigen::Matrix<double, 38, 24>& S_p_out) const noexcept;

    /**
     * @brief Forward-mode Jacobian-Vector Product (JVP):
     *        dy = A * dx + B * du + S_p * dp
     */
    void evaluate_jvp(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground,
        double dt_s,
        const Eigen::Matrix<double, 38, 1>& dx,
        const Eigen::Matrix<double, 3, 1>& du,
        const Eigen::Matrix<double, 24, 1>& dp,
        Eigen::Matrix<double, 38, 1>& dy_out) const noexcept;

    /**
     * @brief Reverse-mode Adjoint Vector-Jacobian Product (VJP):
     *        grad_x = A^T * cot_y,  grad_u = B^T * cot_y,  grad_p = S_p^T * cot_y
     */
    void evaluate_vjp(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground,
        double dt_s,
        const Eigen::Matrix<double, 38, 1>& cotangent_y,
        Eigen::Matrix<double, 38, 1>& grad_x_out,
        Eigen::Matrix<double, 3, 1>& grad_u_out,
        Eigen::Matrix<double, 24, 1>& grad_p_out) const noexcept;

    /**
     * @brief Compute full 3D dynamic structural load vectors & moments (11 interfaces per corner)
     *        including D'Alembert inertial & gyroscopic loads for ANSYS structural FEM analysis.
     */
    std::array<CornerStructuralLoadsANSYS, 4> compute_ansys_corner_loads(
        const VehicleState& state,
        const ControlInputs& controls,
        const std::array<GroundContactQuery, 4>& ground) const noexcept;

private:
    VehicleParameters params_;
    std::array<SuspensionKinematicsSolver, 4> kin_solvers_;
    std::array<HPITMTireModel, 4> tire_models_;

    void rebuild_submodels() noexcept;
};

} // namespace ter26

// ============================================================================
// C-ABI Bridge for Direct Python / ctypes / JAX @custom_vjp Integration (§7)
// ============================================================================
extern "C" {

/// Step the C++20 TeR26 vehicle dynamics engine from flat state/control/parameter arrays
void ter26_step_c_abi(
    const double* x38_in,
    const double* u3_in,
    const double* p24_in,
    double dt_s,
    int differentiable_mode,
    double* x38_out,
    double* energy_out,
    int* nonsmooth_flag_out);

/// Evaluate reverse-mode Adjoint VJP (grad_x, grad_u, grad_p) for JAX @custom_vjp backward pass
void ter26_vjp_c_abi(
    const double* x38_in,
    const double* u3_in,
    const double* p24_in,
    const double* cotangent_y38_in,
    double dt_s,
    double* grad_x38_out,
    double* grad_u3_out,
    double* grad_p24_out);

/// Evaluate full state (38x38), control (38x3), and parameter (38x24) Jacobians in row-major order
void ter26_jacobians_c_abi(
    const double* x38_in,
    const double* u3_in,
    const double* p24_in,
    double dt_s,
    double* A_38x38_rowmajor_out,
    double* B_38x3_rowmajor_out,
    double* Sp_38x24_rowmajor_out);

} // extern "C"
