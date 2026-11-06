#pragma once
/**
 * @file lie_groups.hpp
 * @brief Lie Group SO(3) and SE(3) operations, Spatial Algebra, and Frame Isomorphisms
 *        for the TeR26 Multibody Vehicle Dynamics Engine.
 *
 * Conventions:
 *   - Internal Physics Core & ROS 2: ISO 8855 (+X Forward, +Y Left, +Z Up, Right-handed, SI)
 *   - Godot 4.7 3D Engine: -Z Forward, -X Left (+X Right), +Y Up
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <array>

namespace ter26 {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec8 = Eigen::Matrix<double, 8, 1>;
using Vec14 = Eigen::Matrix<double, 14, 1>;

using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat14 = Eigen::Matrix<double, 14, 14>;
using Mat6x14 = Eigen::Matrix<double, 6, 14>;
using Mat3x14 = Eigen::Matrix<double, 3, 14>;
using Quat = Eigen::Quaterniond;

namespace math {

/**
 * @brief Skew-symmetric hat operator [v]_x: R^3 -> so(3)
 *        such that [v]_x * u = v.cross(u).
 */
inline Mat3 skew(const Vec3& v) noexcept {
    Mat3 S;
    S <<  0.0,  -v.z(),  v.y(),
         v.z(),   0.0,  -v.x(),
        -v.y(),  v.x(),   0.0;
    return S;
}

/**
 * @brief Vee operator (inverse of skew): so(3) -> R^3.
 */
inline Vec3 vee(const Mat3& S) noexcept {
    return Vec3(
        0.5 * (S(2, 1) - S(1, 2)),
        0.5 * (S(0, 2) - S(2, 0)),
        0.5 * (S(1, 0) - S(0, 1))
    );
}

/**
 * @brief Exponential map exp([psi]_x): so(3) -> SO(3) (Rodrigues' rotation formula)
 *        with Taylor expansion near psi -> 0 for C^infinity smoothness.
 */
inline Mat3 exp_so3(const Vec3& psi) noexcept {
    const double theta2 = psi.squaredNorm();
    const Mat3 K = skew(psi);
    if (theta2 < 1e-12) {
        const double c1 = 1.0 - theta2 / 6.0 + (theta2 * theta2) / 120.0;
        const double c2 = 0.5 - theta2 / 24.0 + (theta2 * theta2) / 720.0;
        return Mat3::Identity() + c1 * K + c2 * (K * K);
    }
    const double theta = std::sqrt(theta2);
    const double c1 = std::sin(theta) / theta;
    const double c2 = (1.0 - std::cos(theta)) / theta2;
    return Mat3::Identity() + c1 * K + c2 * (K * K);
}

/**
 * @brief Logarithmic map log(R): SO(3) -> R^3 (so(3) coordinates).
 */
inline Vec3 log_so3(const Mat3& R) noexcept {
    const double cos_theta = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    const Vec3 omega_hat = vee(R - R.transpose()); // = 2 * sin(theta) * axis
    if (theta < 1e-7) {
        const double scale = 0.5 * (1.0 + (theta * theta) / 6.0);
        return scale * omega_hat;
    }
    if (std::abs(M_PI - theta) < 1e-5) {
        Eigen::AngleAxisd aa(R);
        return aa.angle() * aa.axis();
    }
    return (0.5 * theta / std::sin(theta)) * omega_hat;
}

/**
 * @brief Right Jacobian of SO(3), J_r(psi), satisfying:
 *        exp([psi + dpsi]_x) approx exp([psi]_x) * exp([J_r(psi) * dpsi]_x).
 */
inline Mat3 right_jacobian_so3(const Vec3& psi) noexcept {
    const double theta2 = psi.squaredNorm();
    const Mat3 K = skew(psi);
    if (theta2 < 1e-12) {
        const double c1 = 0.5 - theta2 / 24.0;
        const double c2 = 1.0 / 6.0 - theta2 / 120.0;
        return Mat3::Identity() - c1 * K + c2 * (K * K);
    }
    const double theta = std::sqrt(theta2);
    const double c1 = (1.0 - std::cos(theta)) / theta2;
    const double c2 = (theta - std::sin(theta)) / (theta2 * theta);
    return Mat3::Identity() - c1 * K + c2 * (K * K);
}

/**
 * @brief Inverse of Right Jacobian of SO(3), J_r^{-1}(psi).
 */
inline Mat3 right_jacobian_inv_so3(const Vec3& psi) noexcept {
    const double theta2 = psi.squaredNorm();
    const Mat3 K = skew(psi);
    if (theta2 < 1e-12) {
        return Mat3::Identity() + 0.5 * K + (1.0 / 12.0) * (K * K);
    }
    const double theta = std::sqrt(theta2);
    const double half_theta = 0.5 * theta;
    const double cot_half = std::cos(half_theta) / std::sin(half_theta);
    const double c2 = (1.0 / theta2) * (1.0 - half_theta * cot_half);
    return Mat3::Identity() + 0.5 * K + c2 * (K * K);
}

/**
 * @brief Project any near-orthogonal 3x3 matrix back onto SO(3) via SVD / Polar Decomposition.
 */
inline Mat3 project_to_so3(const Mat3& M) noexcept {
    Quat q(M);
    q.normalize();
    return q.toRotationMatrix();
}

/**
 * @brief ZYX Euler angles [roll, pitch, yaw] from ISO 8855 rotation matrix R_WB:
 *        R = R_z(yaw) * R_y(pitch) * R_x(roll).
 */
inline Vec3 rot_to_rpy_iso(const Mat3& R) noexcept {
    const double roll  = std::atan2(R(2, 1), R(2, 2));
    const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    const double yaw   = std::atan2(R(1, 0), R(0, 0));
    return Vec3(roll, pitch, yaw);
}

/**
 * @brief Construct ISO 8855 rotation matrix R_WB from [roll, pitch, yaw] (rad).
 */
inline Mat3 rpy_to_rot_iso(const Vec3& rpy) noexcept {
    return (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Vec3::UnitX())).toRotationMatrix();
}

/**
 * @brief Parallel-axis theorem for 3x3 rotational inertia tensor:
 *        Shifts inertia I_cm at CoM by displacement r to point P:
 *        I_P = I_cm + m * ((r^T r) I - r r^T) = I_cm - m * [r]_x^2.
 */
inline Mat3 shift_inertia_tensor(const Mat3& I_cm, double mass, const Vec3& r) noexcept {
    const Mat3 S = skew(r);
    return I_cm - mass * (S * S);
}

/**
 * @brief Construct 6x6 spatial inertia matrix at a reference point given CoM offset r_cm:
 *        [ m I_3,      -m [r_cm]_x ]
 *        [ m [r_cm]_x, I_cm - m [r_cm]_x^2 ]
 */
inline Mat6 spatial_inertia(double mass, const Mat3& I_cm, const Vec3& r_cm = Vec3::Zero()) noexcept {
    Mat6 M = Mat6::Zero();
    const Mat3 S = skew(r_cm);
    M.topLeftCorner<3, 3>() = mass * Mat3::Identity();
    M.topRightCorner<3, 3>() = -mass * S;
    M.bottomLeftCorner<3, 3>() = mass * S;
    M.bottomRightCorner<3, 3>() = I_cm - mass * (S * S);
    return M;
}

/**
 * @brief Exact orthogonal permutation matrix P_{Godot <- ISO}:
 *        Godot X = -ISO Y, Godot Y = +ISO Z, Godot Z = -ISO X.
 */
inline Mat3 perm_godot_from_iso() noexcept {
    Mat3 P;
    P <<  0.0, -1.0,  0.0,
          0.0,  0.0,  1.0,
         -1.0,  0.0,  0.0;
    return P;
}

inline Mat3 perm_iso_from_godot() noexcept {
    return perm_godot_from_iso().transpose();
}

inline Vec3 vec_iso_to_godot(const Vec3& v_iso) noexcept {
    return Vec3(-v_iso.y(), v_iso.z(), -v_iso.x());
}

inline Vec3 vec_godot_to_iso(const Vec3& v_godot) noexcept {
    return Vec3(-v_godot.z(), -v_godot.x(), v_godot.y());
}

inline Mat3 rot_iso_to_godot(const Mat3& R_iso) noexcept {
    const Mat3 P = perm_godot_from_iso();
    return P * R_iso * P.transpose();
}

inline Mat3 rot_godot_to_iso(const Mat3& R_godot) noexcept {
    const Mat3 P_inv = perm_iso_from_godot();
    return P_inv * R_godot * P_inv.transpose();
}

/**
 * @brief Smooth C^infinity softplus function: log(1 + exp(beta * x)) / beta.
 */
inline double smooth_pos(double x, double beta = 500.0) noexcept {
    const double bx = beta * x;
    if (bx > 30.0) return x;
    if (bx < -30.0) return std::exp(bx) / beta;
    return std::log1p(std::exp(bx)) / beta;
}

/**
 * @brief Derivative of smooth_pos(x, beta): sigmoid(beta * x).
 */
inline double smooth_pos_deriv(double x, double beta = 500.0) noexcept {
    const double bx = beta * x;
    if (bx > 30.0) return 1.0;
    if (bx < -30.0) return 0.0;
    return 1.0 / (1.0 + std::exp(-bx));
}

} // namespace math
} // namespace ter26
