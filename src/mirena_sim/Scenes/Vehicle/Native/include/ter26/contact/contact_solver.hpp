#pragma once
/**
 * @file contact_solver.hpp
 * @brief Multi-Class 3D Frictional Contact & Impact Mechanics Solver:
 *        - 5 Contact Classes (Wheel-Road, Chassis-Ground Bottoming, Vehicle-Cone,
 *          Vehicle-Rigid Barrier, General Two-Body Rigid Pair)
 *        - Two-Body 3D Off-Center Frictional Impact with exact Linear & Angular Momentum Conservation
 *        - Multi-Point Coupled 3D Projected Gauss-Seidel (PGS) Solver on the Delassus Matrix
 *          W = J_c * M(q)^{-1} * J_c^T with Signorini Complementarity and 3D Coulomb Friction Cone
 *        - Non-Smooth Contact Regime Diagnostics (Separated, Sticking, Sliding, Grazing)
 */

#include "ter26/state/vehicle_state.hpp"
#include "ter26/multibody/component_multibody.hpp"
#include <vector>
#include <array>

namespace ter26 {

/// Distinct physical contact classes (§4.1)
enum class ContactClass {
    WHEEL_ROAD               = 0,
    CHASSIS_GROUND_BOTTOMING = 1,
    VEHICLE_CONE             = 2,
    VEHICLE_RIGID_BARRIER    = 3,
    RIGID_BODY_PAIR          = 4
};

/// Non-smooth regime classification for differentiability & solver diagnostics (§4.2 & §6.3)
enum class ContactRegime {
    SEPARATED        = 0, ///< Inactive contact (gap > 0 or separating velocity)
    STICKING         = 1, ///< Inside strict interior of 3D Coulomb friction cone (||J_t|| < mu * J_n)
    SLIDING          = 2, ///< On boundary of 3D Coulomb friction cone (||J_t|| == mu * J_n)
    GRAZING_NONDIFF  = 3  ///< Non-differentiable transition boundary (v_n ~ 0 or ||J_t|| ~ mu * J_n)
};

/// Single 3D contact manifold point for multi-contact or two-body resolution
struct ContactManifoldPoint {
    ContactClass contact_class{ContactClass::VEHICLE_RIGID_BARRIER};
    int body_a_id{0};                      ///< 0 = vehicle chassis, >= 0 = rigid body index
    int body_b_id{-1};                     ///< -1 = immovable world/barrier, >= 0 = second rigid body
    Vec3 point_w{Vec3::Zero()};            ///< Contact point in World Frame F_W [m]
    Vec3 normal_w{Vec3::UnitZ()};          ///< Unit normal pointing from Body B to Body A [m]
    double penetration_m{0.0};             ///< Positive when overlapping [m]
    double restitution{0.25};              ///< Normal restitution coefficient e in [0, 1]
    double friction_mu{0.45};              ///< Coulomb friction coefficient mu >= 0
    double baumgarte_erp{0.20};            ///< Position drift stabilization factor in [0, 1]
};

/// Result of resolving a two-body 3D off-center frictional impact
struct TwoBodyImpactResult {
    Vec3 impulse_on_a_w{Vec3::Zero()};     ///< Total 3D impulse J_w applied to Body A [N*s]
    Vec3 impulse_on_b_w{Vec3::Zero()};     ///< Total 3D impulse -J_w applied to Body B [N*s]
    double normal_impulse_Ns{0.0};         ///< J_n >= 0 [N*s]
    Vec2 tangential_impulse_Ns{Vec2::Zero()};
    ContactRegime regime{ContactRegime::SEPARATED};
    double pre_kinetic_energy_J{0.0};
    double post_kinetic_energy_J{0.0};
    Vec3 linear_momentum_err{Vec3::Zero()};
    Vec3 angular_momentum_err{Vec3::Zero()};
};

/// Result of resolving N_c simultaneous contacts via Projected Gauss-Seidel (PGS)
struct MultiContactPGSResult {
    std::vector<Vec3> impulses_w;          ///< 3D world impulse at each contact point [N*s]
    std::vector<double> normal_impulses_Ns;
    std::vector<ContactRegime> regimes;
    Vec6 generalized_chassis_impulse_b{Vec6::Zero()}; ///< [Delta P_B (3), Delta L_B (3)] in F_B
    double max_complementarity_error{0.0};
    int iterations{0};
    bool converged{true};
    bool has_nondiff_grazing{false};
};

class ContactMechanicsSolver {
public:
    /**
     * @brief Construct an orthonormal contact frame [n, t1, t2] from unit normal n_w.
     */
    static void build_contact_basis(
        const Vec3& normal_w,
        Vec3& n_unit,
        Vec3& t1_unit,
        Vec3& t2_unit) noexcept;

    /**
     * @brief Exact analytical 3D two-body off-center frictional impact solver (§4.2).
     *        Modifies body_a and body_b velocities in-place (if !body_b.is_fixed),
     *        strictly conserving total linear momentum P_A + P_B and angular momentum L_A + L_B,
     *        and guaranteeing dE = E_post - E_pre <= 0 (with dE = 0 when e = 1, mu = 0).
     */
    static TwoBodyImpactResult solve_two_body_impact(
        RigidBodyComponent& body_a,
        RigidBodyComponent& body_b,
        const Vec3& contact_point_w,
        const Vec3& normal_b_to_a_w,
        double restitution,
        double friction_mu) noexcept;

    /**
     * @brief Coupled Multi-Point 3D Projected Gauss-Seidel (PGS) frictional impulse solver (§4.3)
     *        for simultaneous contacts acting on the vehicle chassis (mass m_eff, inertia I_eff_b)
     *        and optional movable second bodies (e.g. cones).
     *        Solves the 3N_c x 3N_c Delassus LCP/NCP:
     *          0 <= lambda_n  perp  (W * lambda + v_pre + v_bias)_n >= 0
     *          ||lambda_t||_2 <= mu * lambda_n
     */
    static MultiContactPGSResult solve_multi_contact_pgs(
        const Vec3& chassis_pos_w,
        const Quat& chassis_q_wb,
        const Vec3& chassis_v_b,
        const Vec3& chassis_omega_b,
        double chassis_mass_kg,
        const Mat3& chassis_inertia_b,
        const std::vector<ContactManifoldPoint>& contacts,
        double dt_s,
        int max_iterations = 35,
        double tol = 1e-9) noexcept;

    /**
     * @brief Generate chassis skidblock / floor ground-bottoming contacts when ride height
     *        drops below the undertray clearance plane.
     */
    static void append_chassis_bottoming_contacts(
        const Vec3& chassis_pos_w,
        const Quat& chassis_q_wb,
        double ground_z_w,
        const Vec3& ground_normal_w,
        std::vector<ContactManifoldPoint>& out_contacts) noexcept;
};

} // namespace ter26
