#pragma once
/**
 * @file component_multibody.hpp
 * @brief Component-Based Rigid-Body Multibody Dynamics Architecture for TeR26.
 *
 * Provides:
 *   1. Explicit RigidBodyComponent representation (mass, CoM offset, 3x3 inertia tensor,
 *      SE(3) pose, body/world spatial velocities & accelerations, local attachment points,
 *      applied wrenches, kinetic & potential energy, and exact spatial Adjoint transforms).
 *   2. Modular Mechanical Constraints (Spherical, Revolute, FixedDistanceLink, Prismatic, Fixed)
 *      with analytical position residuals Phi(q), velocity Jacobians J_Phi(q), convective
 *      acceleration terms -dJ_Phi/dt * v, SVD rank/conditioning diagnostics, and joint reaction
 *      wrenches -J_Phi^T * lambda.
 *   3. Redundant-Coordinate DAE Multibody System Assembler & Solver (Index-1 Saddle-Point
 *      with Baumgarte stabilization + Gauss-Newton manifold projection for zero constraint drift).
 *   4. Corner Component Multibody Inertial Aggregator that computes the exact 3D kinematics,
 *      spatial velocities, convective accelerations, and configuration-dependent mass matrix
 *      contributions of all 7 suspension/drivetrain bodies per corner (UCA, LCA, Pushrod,
 *      Tie-Rod, Rocker, Upright, Wheel+Planetary) so the full component representation and
 *      reduced 14-DOF vehicle model coexist seamlessly.
 */

#include "ter26/math/lie_groups.hpp"
#include "ter26/state/vehicle_state.hpp"
#include "ter26/kinematics/suspension_kinematics.hpp"
#include <vector>
#include <string>
#include <memory>
#include <array>

namespace ter26::multibody {

/**
 * @brief 6D Spatial Vector convention:
 *        Linear component first [0..2], Angular component second [3..5]:
 *        V = [v^T, omega^T]^T in R^6,  W = [F^T, M^T]^T in R^6.
 */
using SpatialVec = Vec6;
using SpatialMat = Mat6;

/**
 * @brief SE(3) Adjoint matrix Ad_{T_{AB}} mapping spatial twists from Frame B to Frame A:
 *        V_A = Ad_{T_{AB}} * V_B,  where T_{AB} = (R_{AB}, p_{AB}).
 */
inline SpatialMat spatial_adjoint(const Mat3& R_ab, const Vec3& p_ab) noexcept {
    SpatialMat Ad = SpatialMat::Zero();
    Ad.topLeftCorner<3, 3>()     = R_ab;
    Ad.topRightCorner<3, 3>()    = math::skew(p_ab) * R_ab;
    Ad.bottomRightCorner<3, 3>() = R_ab;
    return Ad;
}

/**
 * @brief Dual Adjoint Ad_{T_{AB}}^* = Ad_{T_{BA}}^T mapping spatial wrenches from Frame B to Frame A.
 */
inline SpatialMat spatial_dual_adjoint(const Mat3& R_ab, const Vec3& p_ab) noexcept {
    SpatialMat Ad_star = SpatialMat::Zero();
    Ad_star.topLeftCorner<3, 3>()     = R_ab;
    Ad_star.bottomLeftCorner<3, 3>()  = math::skew(p_ab) * R_ab;
    Ad_star.bottomRightCorner<3, 3>() = R_ab;
    return Ad_star;
}

/**
 * @brief Spatial cross-product operator ad_V for V = [v; omega]:
 *        ad_V * U = [ omega x u_v + v x u_w ; omega x u_w ] (or in body frame for free-floating twist).
 *        For body-frame spatial velocity V^B = [v^B; omega^B], the Euler-Poincare convective/Coriolis
 *        wrench is:
 *          h_coriolis^B = [ omega^B x (m v^B + m (omega^B x r_cm)) ;
 *                           omega^B x (I_0 omega^B) + v^B x (m (omega^B x r_cm)) ].
 */
inline SpatialVec body_coriolis_wrench(
    double mass,
    const Vec3& r_cm_b,
    const Mat3& I_cm_b,
    const SpatialVec& V_b) noexcept
{
    const Vec3 v_b     = V_b.head<3>();
    const Vec3 omega_b = V_b.tail<3>();
    const Vec3 v_cm_b  = v_b + omega_b.cross(r_cm_b);
    const Vec3 F_cor   = mass * omega_b.cross(v_cm_b);
    const Vec3 M_cm    = omega_b.cross(I_cm_b * omega_b);
    const Vec3 M_cor   = M_cm + r_cm_b.cross(F_cor);
    SpatialVec h;
    h.head<3>() = F_cor;
    h.tail<3>() = M_cor;
    return h;
}

/// Named local attachment point on a RigidBodyComponent
struct AttachmentPoint {
    std::string name{};
    Vec3 local_pos_b{Vec3::Zero()}; ///< Position in body reference frame F_i [m]
    Vec3 local_axis_b{Vec3::UnitX()}; ///< Optional joint axis in body frame F_i [-]
};

/// Complete state and inertia properties of a single Rigid Body Component
struct RigidBodyComponent {
    int id{0};
    std::string name{"Body"};
    bool is_fixed_to_world{false};

    // Inertial properties in Body Reference Frame F_i
    double mass_kg{1.0};
    Vec3 com_local_m{Vec3::Zero()};              ///< CoM offset r_cm in F_i [m]
    Mat3 inertia_com_b{Mat3::Identity() * 0.01}; ///< 3x3 rotational inertia about CoM in F_i [kg*m^2]

    // Pose in World Frame F_W (ISO 8855)
    Vec3 pos_w{Vec3::Zero()};                    ///< Reference origin p_i^W [m]
    Mat3 rot_wb{Mat3::Identity()};               ///< Orientation R_{W,i} in SO(3)

    // Spatial Velocity & Acceleration in Body Reference Frame F_i
    Vec3 vel_b{Vec3::Zero()};                    ///< Linear velocity v_i^B = R_{W,i}^T * dp_i^W/dt [m/s]
    Vec3 omega_b{Vec3::Zero()};                  ///< Angular velocity omega_i^B [rad/s]
    Vec3 accel_b{Vec3::Zero()};                  ///< Linear acceleration dv_i^B/dt [m/s^2]
    Vec3 alpha_b{Vec3::Zero()};                  ///< Angular acceleration domega_i^B/dt [rad/s^2]

    // Applied external wrench in Body Reference Frame F_i (in addition to gravity)
    Vec3 applied_force_b{Vec3::Zero()};          ///< External force in F_i [N]
    Vec3 applied_moment_b{Vec3::Zero()};         ///< External moment about origin of F_i [N*m]

    std::vector<AttachmentPoint> attachments{};

    /// 6x6 Spatial Inertia Matrix in Body Reference Frame F_i
    SpatialMat spatial_inertia_b() const noexcept {
        return math::spatial_inertia(mass_kg, inertia_com_b, com_local_m);
    }

    /// World-frame position of Center of Mass
    Vec3 com_world() const noexcept {
        return pos_w + rot_wb * com_local_m;
    }

    /// World-frame velocity of Center of Mass
    Vec3 com_velocity_world() const noexcept {
        return rot_wb * (vel_b + omega_b.cross(com_local_m));
    }

    /// World-frame position of local point r_local_b
    Vec3 point_world(const Vec3& r_local_b) const noexcept {
        return pos_w + rot_wb * r_local_b;
    }

    /// World-frame velocity of local point r_local_b
    Vec3 point_velocity_world(const Vec3& r_local_b) const noexcept {
        return rot_wb * (vel_b + omega_b.cross(r_local_b));
    }

    /// Exact kinetic energy T_i = 0.5 * V_b^T * M_b * V_b [J]
    double kinetic_energy() const noexcept {
        if (is_fixed_to_world) return 0.0;
        SpatialVec V;
        V.head<3>() = vel_b;
        V.tail<3>() = omega_b;
        return 0.5 * V.dot(spatial_inertia_b() * V);
    }

    /// Exact gravitational potential energy V_i = -m_i * g_w^T * p_cm_w [J]
    double potential_energy(const Vec3& gravity_w = Vec3(0.0, 0.0, -9.81)) const noexcept {
        if (is_fixed_to_world) return 0.0;
        return -mass_kg * gravity_w.dot(com_world());
    }

    /// World-frame linear momentum P_i^W = m_i * v_cm^W [kg*m/s]
    Vec3 linear_momentum_world() const noexcept {
        if (is_fixed_to_world) return Vec3::Zero();
        return mass_kg * com_velocity_world();
    }

    /// World-frame angular momentum about World Origin: L_i^W = p_cm^W x P_i^W + R_{W,i} I_cm omega_i^B
    Vec3 angular_momentum_world() const noexcept {
        if (is_fixed_to_world) return Vec3::Zero();
        return com_world().cross(linear_momentum_world()) + rot_wb * (inertia_com_b * omega_b);
    }
};

/**
 * @brief Abstract Holonomic / Kinematic Constraint between Body A and Body B:
 *        Position level:     Phi(q) = 0 in R^m
 *        Velocity level:     J_Phi(q) * v = 0 in R^m
 *        Acceleration level: J_Phi(q) * dv/dt + gamma_conv(q, v) = 0 in R^m
 *        where v_k = [v_k^B; omega_k^B] in R^6 for each active body.
 */
class MechanicalConstraint {
public:
    virtual ~MechanicalConstraint() = default;
    virtual std::string name() const = 0;
    virtual int num_equations() const noexcept = 0;
    virtual int body_a_id() const noexcept = 0;
    virtual int body_b_id() const noexcept = 0;

    virtual void evaluate(
        const RigidBodyComponent& body_a,
        const RigidBodyComponent& body_b,
        Eigen::VectorXd& phi_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
        Eigen::VectorXd& gamma_conv_out) const noexcept = 0;
};

/// 3-DOF Spherical (Ball) Joint Constraint: p_A^W(r_A) - p_B^W(r_B) = 0 (m = 3 equations)
class SphericalJointConstraint final : public MechanicalConstraint {
public:
    SphericalJointConstraint(std::string joint_name, int id_a, int id_b, const Vec3& r_a_local, const Vec3& r_b_local)
        : name_(std::move(joint_name)), id_a_(id_a), id_b_(id_b), r_a_(r_a_local), r_b_(r_b_local) {}

    std::string name() const override { return name_; }
    int num_equations() const noexcept override { return 3; }
    int body_a_id() const noexcept override { return id_a_; }
    int body_b_id() const noexcept override { return id_b_; }

    void evaluate(
        const RigidBodyComponent& body_a,
        const RigidBodyComponent& body_b,
        Eigen::VectorXd& phi_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
        Eigen::VectorXd& gamma_conv_out) const noexcept override;

private:
    std::string name_;
    int id_a_;
    int id_b_;
    Vec3 r_a_;
    Vec3 r_b_;
};

/// 1-DOF Revolute (Hinge) Joint Constraint: 3 spherical + 2 axis-orthogonality equations (m = 5 equations)
class RevoluteJointConstraint final : public MechanicalConstraint {
public:
    RevoluteJointConstraint(
        std::string joint_name,
        int id_a,
        int id_b,
        const Vec3& r_a_local,
        const Vec3& r_b_local,
        const Vec3& axis_a_local,
        const Vec3& axis_b_local);

    std::string name() const override { return name_; }
    int num_equations() const noexcept override { return 5; }
    int body_a_id() const noexcept override { return id_a_; }
    int body_b_id() const noexcept override { return id_b_; }

    void evaluate(
        const RigidBodyComponent& body_a,
        const RigidBodyComponent& body_b,
        Eigen::VectorXd& phi_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
        Eigen::VectorXd& gamma_conv_out) const noexcept override;

private:
    std::string name_;
    int id_a_;
    int id_b_;
    Vec3 r_a_;
    Vec3 r_b_;
    Vec3 u_a_;
    Vec3 u_b_;
    Vec3 n1_b_;
    Vec3 n2_b_;
};

/// Distance Link (Rod) Constraint: 0.5 * (||p_A^W(r_A) - p_B^W(r_B)||^2 - L0^2) = 0 (m = 1 equation)
class FixedDistanceLinkConstraint final : public MechanicalConstraint {
public:
    FixedDistanceLinkConstraint(
        std::string link_name,
        int id_a,
        int id_b,
        const Vec3& r_a_local,
        const Vec3& r_b_local,
        double length_m)
        : name_(std::move(link_name)), id_a_(id_a), id_b_(id_b),
          r_a_(r_a_local), r_b_(r_b_local), L0_( std::max(1e-6, length_m) ) {}

    std::string name() const override { return name_; }
    int num_equations() const noexcept override { return 1; }
    int body_a_id() const noexcept override { return id_a_; }
    int body_b_id() const noexcept override { return id_b_; }

    void evaluate(
        const RigidBodyComponent& body_a,
        const RigidBodyComponent& body_b,
        Eigen::VectorXd& phi_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_a_out,
        Eigen::Matrix<double, Eigen::Dynamic, 6>& J_b_out,
        Eigen::VectorXd& gamma_conv_out) const noexcept override;

private:
    std::string name_;
    int id_a_;
    int id_b_;
    Vec3 r_a_;
    Vec3 r_b_;
    double L0_;
};

/// Point-to-Point Spring-Damper Force Element between Body A and Body B
struct PointSpringDamperElement {
    std::string name{"SpringDamper"};
    int body_a_id{0};
    int body_b_id{1};
    Vec3 r_a_local{Vec3::Zero()};
    Vec3 r_b_local{Vec3::Zero()};
    double free_length_m{0.25};
    double k_lin_N_m{20000.0};
    double k_cubic_N_m3{0.0};
    double preload_N{0.0};
    double c_damping_N_s_m{500.0};

    /// Potential energy stored in spring
    double potential_energy(const RigidBodyComponent& a, const RigidBodyComponent& b) const noexcept {
        const double L = (b.point_world(r_b_local) - a.point_world(r_a_local)).norm();
        const double dx = L - free_length_m;
        return 0.5 * k_lin_N_m * (dx * dx) + 0.25 * k_cubic_N_m3 * (dx * dx * dx * dx) + preload_N * dx;
    }
};

/// Selectable solver for the Delassus / Saddle-Point constraint system (handling redundant constraints)
enum class ConstraintSolverMode {
    SVD_MINIMUM_NORM = 0, ///< Truncated SVD pseudoinverse (exact minimum-norm multipliers without Tikhonov drift)
    QR_RANK_REVEALING = 1, ///< Column-pivoted Householder QR rank-revealing solve
    TIKHONOV_LDLT = 2      ///< Regularized Cholesky (W + eps*I)
};

/// Selectable constraint drift stabilization / manifold projection strategy
enum class ConstraintStabilizationMode {
    MASS_METRIC_MANIFOLD_PROJECTION = 0, ///< Exact Index-1 DAE + Mass-Metric Gauss-Newton position & velocity projection
    EUCLIDEAN_MANIFOLD_PROJECTION   = 1, ///< Exact Index-1 DAE + Unweighted Euclidean Gauss-Newton projection
    BAUMGARTE_STABILIZATION         = 2, ///< Pure Baumgarte ODE penalty (alpha, beta > 0) without post-step projection
    INDEX1_UNPENALIZED              = 3  ///< Raw Index-1 DAE (alpha=0, beta=0, no projection) for drift comparison
};

/// Diagnostics and Joint Reactions from the Component Multibody DAE Solver
struct MultibodyDAEResult {
    Eigen::VectorXd generalized_accel;        ///< dv/dt in R^{6*N_bodies}
    Eigen::VectorXd lagrange_multipliers;     ///< lambda in R^{m_constraints}
    Eigen::VectorXd constraint_pos_residual;  ///< Phi(q) in R^m
    Eigen::VectorXd constraint_vel_residual;  ///< J_Phi(q) * v in R^m
    Eigen::VectorXd constraint_accel_residual;///< J_Phi(q) * dv/dt + gamma_conv(q, v) in R^m
    Eigen::VectorXd singular_values;          ///< Singular values sigma_1 >= ... >= sigma_m of J_Phi
    double max_pos_drift{0.0};                ///< ||Phi(q)||_inf
    double max_vel_drift{0.0};                ///< ||J_Phi * v||_inf
    double max_accel_residual{0.0};           ///< ||J_Phi * dv/dt + gamma_conv||_inf
    int jacobian_rank{0};                     ///< Numerical rank of J_Phi
    int num_redundant_constraints{0};         ///< m_total - jacobian_rank
    std::vector<int> redundant_constraint_indices{}; ///< Row indices of redundant constraint equations
    double jacobian_rcond{1.0};               ///< Reciprocal condition number sigma_min / sigma_max
    double condition_number{1.0};             ///< Condition number sigma_max / sigma_min of active space
    bool near_singular_configuration{false};  ///< True if smallest active singular value < 1e-4 * sigma_max
    double kinetic_energy_J{0.0};
    double potential_energy_J{0.0};
    double total_energy_J{0.0};
    std::vector<SpatialVec> joint_reaction_wrenches_on_a; ///< Reaction wrench on body A for each constraint
    bool solver_ok{true};
};

/**
 * @brief Component-Based Multibody System supporting arbitrary rigid bodies,
 *        holonomic constraints, spring-damper force elements, DAE saddle-point assembly,
 *        and Lie-group RK4 + manifold constraint projection.
 */
class ComponentMultibodySystem {
public:
    ComponentMultibodySystem() = default;

    int add_body(const RigidBodyComponent& body);
    void add_constraint(std::shared_ptr<MechanicalConstraint> constraint);
    void add_spring_damper(const PointSpringDamperElement& elem);

    std::vector<RigidBodyComponent>& bodies() noexcept { return bodies_; }
    const std::vector<RigidBodyComponent>& bodies() const noexcept { return bodies_; }

    void set_gravity(const Vec3& g_w) noexcept { gravity_w_ = g_w; }
    const Vec3& gravity() const noexcept { return gravity_w_; }

    void set_solver_mode(ConstraintSolverMode mode, double tikhonov_eps = 1e-10) noexcept {
        solver_mode_ = mode;
        tikhonov_eps_ = tikhonov_eps;
    }
    ConstraintSolverMode solver_mode() const noexcept { return solver_mode_; }

    void set_stabilization_mode(ConstraintStabilizationMode mode, double alpha = 25.0, double beta = 25.0) noexcept {
        stabilization_mode_ = mode;
        baumgarte_alpha_ = alpha;
        baumgarte_beta_ = beta;
    }
    ConstraintStabilizationMode stabilization_mode() const noexcept { return stabilization_mode_; }

    /**
     * @brief Assemble and solve the constrained equations of motion:
     *        M(q) * dv/dt + J_Phi(q)^T * lambda = Q(q, v) - h(q, v)
     *        J_Phi(q) * dv/dt = -gamma_conv(q, v) - 2*alpha_B*(J_Phi*v) - beta_B^2 * Phi(q)
     */
    MultibodyDAEResult evaluate_dae(double baumgarte_alpha = 0.0, double baumgarte_beta = 0.0) const noexcept;

    /**
     * @brief Project body poses and velocities onto the exact constraint manifold
     *        Phi(q) = 0 and J_Phi(q) * v = 0 using Mass-Metric or Euclidean Gauss-Newton projection.
     */
    void project_onto_constraint_manifold(int max_iters = 6, double tol = 1e-12, bool use_mass_metric = true) noexcept;

    /**
     * @brief Advance the multibody system by timestep dt_s using 4-stage Lie-Group RK4
     *        on SE(3)^N followed by the configured stabilization/manifold projection.
     */
    MultibodyDAEResult step_rk4_projected(double dt_s) noexcept;

    double total_kinetic_energy() const noexcept;
    double total_potential_energy() const noexcept;
    double total_mechanical_energy() const noexcept {
        return total_kinetic_energy() + total_potential_energy();
    }
    Vec3 total_linear_momentum_world() const noexcept;
    Vec3 total_angular_momentum_world() const noexcept;

private:
    std::vector<RigidBodyComponent> bodies_;
    std::vector<std::shared_ptr<MechanicalConstraint>> constraints_;
    std::vector<PointSpringDamperElement> springs_;
    Vec3 gravity_w_{0.0, 0.0, -9.81};
    ConstraintSolverMode solver_mode_{ConstraintSolverMode::SVD_MINIMUM_NORM};
    ConstraintStabilizationMode stabilization_mode_{ConstraintStabilizationMode::MASS_METRIC_MANIFOLD_PROJECTION};
    double tikhonov_eps_{1e-10};
    double baumgarte_alpha_{25.0};
    double baumgarte_beta_{25.0};
};

// ============================================================================
// Coexistence Layer: Corner Suspension Component Inertias & Dynamic Load Export
// ============================================================================

/// Inertial parameters of the individual suspension components of one corner
struct SuspensionComponentInertias {
    bool include_component_inertias{true}; ///< Toggle between explicit component inertias vs massless links

    // Masses of individual suspension links & rocker [kg]
    double m_uca_kg{0.42};          ///< Upper wishbone (A-arm) mass [kg]
    double m_lca_kg{0.54};          ///< Lower wishbone (A-arm) mass [kg]
    double m_pushrod_kg{0.26};      ///< Pushrod / pullrod tube + rod-ends mass [kg]
    double m_tie_rod_kg{0.24};      ///< Steering tie-rod mass [kg]
    double m_rocker_kg{0.38};       ///< Bellcrank rocker + bearings mass [kg]

    // Principal rotational inertias about component CoM [kg*m^2]
    Vec3 I_uca_diag{0.0018, 0.0024, 0.0038};
    Vec3 I_lca_diag{0.0026, 0.0032, 0.0052};
    Vec3 I_pushrod_diag{0.0012, 0.0012, 0.00005};
    Vec3 I_tie_rod_diag{0.0011, 0.0011, 0.00004};
    Vec3 I_rocker_diag{0.00065, 0.00042, 0.00048};

    double total_corner_link_mass_kg() const noexcept {
        return include_component_inertias
            ? (m_uca_kg + m_lca_kg + m_pushrod_kg + m_tie_rod_kg + m_rocker_kg)
            : 0.0;
    }
};

/// Physical 3D U-Bar Anti-Roll Bar (ARB) Geometry & Torsional Stiffness Parameters
struct PhysicalARBParams {
    double torsion_bar_stiffness_Nm_rad{320.0}; ///< G * J_polar / L_bar [N*m/rad]
    double blade_lever_arm_m{0.085};            ///< ARB lever arm radius R_arm [m]
    double blade_bending_stiffness_N_m{180000.0}; ///< Blade flexural stiffness in series [N/m]
    Vec3 bar_axis_mount_left_b{0.0, 0.220, 0.415};  ///< Left chassis pillow-block mount in axle frame [m]
    Vec3 bar_axis_mount_right_b{0.0, -0.220, 0.415}; ///< Right chassis pillow-block mount in axle frame [m]

    /// Evaluate coupled left/right ARB generalized wheel forces, droplink forces, and strain energy
    /// directly from Left and Right rocker angles (theta_r_L, theta_r_R) and droplink displacements
    void evaluate_coupled_arb(
        const CornerKinematicState& kin_left,
        const CornerKinematicState& kin_right,
        double& f_wheel_left_N,
        double& f_wheel_right_N,
        double& f_droplink_left_N,
        double& f_droplink_right_N,
        double& tau_bar_Nm,
        double& strain_energy_J) const noexcept;
};

/// Complete 3D Structural Load Frame for ANSYS FEM Export (11 Mounting Interfaces per Corner)
struct CornerStructuralLoadsANSYS {
    int corner_index{0};
    std::string corner_name{"FL"};

    // 3D Force Vectors on the Chassis at each Inboard Mounting Point in Chassis Frame F_B [N]
    Vec3 F_chassis_uca_fore_b{Vec3::Zero()};
    Vec3 F_chassis_uca_aft_b{Vec3::Zero()};
    Vec3 F_chassis_lca_fore_b{Vec3::Zero()};
    Vec3 F_chassis_lca_aft_b{Vec3::Zero()};
    Vec3 F_chassis_tie_rod_in_b{Vec3::Zero()};
    Vec3 F_chassis_rocker_pivot_b{Vec3::Zero()};
    Vec3 M_chassis_rocker_pivot_b{Vec3::Zero()};
    Vec3 F_chassis_coilover_top_b{Vec3::Zero()};
    Vec3 F_chassis_arb_mount_b{Vec3::Zero()};
    Vec3 M_chassis_arb_mount_b{Vec3::Zero()};

    // 3D Forces on the Upright at Outboard Ball Joints & Wheel Hub Bearing in Chassis Frame F_B [N, N*m]
    Vec3 F_upright_ubj_b{Vec3::Zero()};
    Vec3 F_upright_lbj_b{Vec3::Zero()};
    Vec3 F_upright_pushrod_b{Vec3::Zero()};
    Vec3 F_upright_tie_rod_b{Vec3::Zero()};
    Vec3 F_hub_bearing_b{Vec3::Zero()};
    Vec3 M_hub_bearing_b{Vec3::Zero()};

    // Dynamic Inertial D'Alembert Wrench of the Corner's Unsprung & Link Components [N, N*m]
    Vec3 F_inertial_d_alembert_b{Vec3::Zero()};
    Vec3 M_inertial_d_alembert_b{Vec3::Zero()};

    // Static vs Dynamic Link Axial Reactions [N]
    CornerJointReactions static_link_reactions{};
    CornerJointReactions dynamic_link_reactions{};
    int statics_matrix_rank{6};
    double statics_conditioning_rcond{1.0};

    // Global D'Alembert Force & Moment Equilibrium Verification Residuals [N, N*m]
    double dynamic_equilibrium_residual_N{0.0};
    double dynamic_moment_equilibrium_residual_Nm{0.0};
    double static_equilibrium_residual_N{0.0};
};

/// Mathematical Equivalence Audit between the 29-Body Component Multibody System and Reduced 14-DOF Model (§1.1–1.2)
struct ComponentMultibodyEquivalenceReport {
    std::vector<RigidBodyComponent> bodies{};      ///< All 29 explicit RigidBodyComponents (1 chassis + 4x7 corner bodies)
    double T_bodies_sum_J{0.0};                    ///< sum_k 0.5 * V_k^T * M_k * V_k + T_drivetrain_rotors [J]
    double T_reduced_quad_J{0.0};                  ///< 0.5 * v^T * M(q) * v [J]
    double kinetic_energy_abs_diff_J{0.0};         ///< |T_bodies_sum - T_reduced_quad| [J]
    double V_grav_bodies_sum_J{0.0};               ///< sum_k (-m_k * g_W^T * p_{cm,k}^W) [J]
    double V_grav_reduced_J{0.0};                  ///< Reduced gravitational potential energy [J]
    double potential_energy_abs_diff_J{0.0};       ///< |V_grav_bodies - V_grav_reduced| [J]
    Vec3 linear_momentum_bodies_b{Vec3::Zero()};   ///< Total linear momentum in F_B from bodies [kg*m/s]
    Vec3 linear_momentum_reduced_b{Vec3::Zero()};  ///< (M(q) * v)[0..2] [kg*m/s]
    double linear_momentum_diff_norm{0.0};         ///< ||P_bodies - P_reduced||_2 [kg*m/s]
    Vec3 angular_momentum_bodies_b{Vec3::Zero()};  ///< Total angular momentum about CoG in F_B from bodies [kg*m^2/s]
    Vec3 angular_momentum_reduced_b{Vec3::Zero()}; ///< (M(q) * v)[3..5] [kg*m^2/s]
    double angular_momentum_diff_norm{0.0};        ///< ||L_bodies - L_reduced||_2 [kg*m^2/s]
};

/**
 * @brief Compute the configuration-dependent inertial and gravitational contribution of the
 *        5 suspension components per corner (UCA, LCA, pushrod, tie-rod, rocker) to M(q), C(q, v),
 *        Q_grav(q), kinetic energy T, and gravitational potential energy V_grav, AND optionally
 *        append their explicit RigidBodyComponent representations to `out_bodies`.
 */
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
    std::vector<RigidBodyComponent>* out_bodies = nullptr) noexcept;

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
    const Vec3& g_b) noexcept;

/// Export all 4 corners' structural loads to a machine-readable JSON file for ANSYS Mechanical / Workbench
bool export_ansys_structural_loads_json(
    const std::string& output_json_path,
    const std::string& load_case_name,
    double sim_time_s,
    const std::array<CornerStructuralLoadsANSYS, 4>& corner_loads,
    const std::array<CornerHardpoints, 4>& hardpoints,
    double h_cg_m = 0.330);

} // namespace ter26::multibody

namespace ter26 {
using namespace multibody;
} // namespace ter26
