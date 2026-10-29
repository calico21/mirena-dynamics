#include "ekf.hpp"

#include <algorithm>
#include <cmath>

namespace mirena
{

namespace
{
/// Floor for the covariance diagonal. The short-form covariance updates below
/// can round a near-zero variance slightly negative; clamping keeps a negative
/// variance from reaching consumers. It does not make P positive-definite --
/// it only bounds the symptom.
constexpr double kMinVariance = 1e-12;
} // namespace

double Ekf::wrap_angle(double a)
{
    // IEEE remainder lands in [-pi, pi] and terminates for every input.
    return std::remainder(a, 2.0 * M_PI);
}

void Ekf::reset()
{
    x_.setZero();
    P_.setIdentity();
    stamp_ = 0.0;
    initialized_ = false;
}

Ekf::Step Ekf::commit()
{
    if (x_.allFinite() && P_.allFinite())
    {
        P_.diagonal() = P_.diagonal().cwiseMax(kMinVariance);
        return Step::Ok;
    }

    reset();
    divergences_++;
    return Step::Diverged;
}

Ekf::Step Ekf::init(const Vec6 &z, const Vec6 &r_diag, double stamp)
{
    if (!z.allFinite() || !r_diag.allFinite() || !std::isfinite(stamp)) return Step::Ignored;

    x_ = z;
    x_(2) = wrap_angle(x_(2));
    P_ = r_diag.cwiseMax(cfg_.init_cov_floor_diag).asDiagonal();
    stamp_ = stamp;
    initialized_ = true;
    return commit();
}

Ekf::Step Ekf::predict(double stamp, double ax, double ay)
{
    if (!initialized_) return Step::Ignored;
    if (!std::isfinite(stamp) || !std::isfinite(ax) || !std::isfinite(ay)) return Step::Ignored;

    const double elapsed = stamp - stamp_;
    if (elapsed <= 0.0) return Step::Ignored; // out-of-order or duplicate sample

    stamp_ = stamp;
    const double dt = std::min(elapsed, cfg_.max_dt);

    const double psi = x_(2), u = x_(3), v = x_(4), omega = x_(5);
    const double c = std::cos(psi), s = std::sin(psi);

    // Nonlinear state propagation (forward Euler)
    x_(0) += (u * c - v * s) * dt;
    x_(1) += (u * s + v * c) * dt;
    x_(2) = wrap_angle(psi + omega * dt);
    x_(3) += (ax + v * omega) * dt;
    x_(4) += (ay - u * omega) * dt;

    // Jacobian F = I + A*dt
    Mat6 F = Mat6::Identity();
    F(0, 2) = (-u * s - v * c) * dt;
    F(0, 3) = c * dt;
    F(0, 4) = -s * dt;
    F(1, 2) = (u * c - v * s) * dt;
    F(1, 3) = s * dt;
    F(1, 4) = c * dt;
    F(2, 5) = dt;
    F(3, 4) = omega * dt;
    F(3, 5) = v * dt;
    F(4, 3) = -omega * dt;
    F(4, 5) = -u * dt;

    P_ = F * P_ * F.transpose();
    P_ += (cfg_.process_noise_diag * dt).asDiagonal().toDenseMatrix();
    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_gyro(double omega_meas)
{
    if (!initialized_) return Step::Ignored;
    if (!std::isfinite(omega_meas)) return Step::Ignored;

    // Scalar update on omega: H = [0 0 0 0 0 1]
    const double innovation = omega_meas - x_(5);
    const double s_var = P_(5, 5) + cfg_.gyro_meas_var;
    if (!(s_var > 0.0)) return Step::Ignored;
    const Vec6 K = P_.col(5) / s_var;

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));
    P_ -= K * P_.row(5);
    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_odom(const Vec6 &z, const Vec6 &r_diag)
{
    if (!initialized_) return Step::Ignored;
    if (!z.allFinite() || !r_diag.allFinite()) return Step::Ignored;

    // Full-state measurement: H = I6
    Vec6 innovation = z - x_;
    innovation(2) = wrap_angle(innovation(2));

    const Mat6 R = r_diag.cwiseMax(cfg_.odom_meas_floor_diag).asDiagonal();
    const Mat6 S = P_ + R;
    const Mat6 K = P_ * S.inverse();

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));
    P_ = (Mat6::Identity() - K) * P_;
    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_pose(const Vec3 &z, const Vec3 &r_diag)
{
    if (!initialized_) return Step::Ignored;
    if (!z.allFinite() || !r_diag.allFinite()) return Step::Ignored;

    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;

    Vec3 innovation = z - x_.head<3>();
    innovation(2) = wrap_angle(innovation(2));

    const Eigen::Matrix3d R = r_diag.cwiseMax(cfg_.odom_meas_floor_diag.head<3>()).asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));
    P_ -= K * H * P_;
    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_velocity(const Vec3 &z, const Vec3 &r_diag)
{
    if (!initialized_) return Step::Ignored;
    if (!z.allFinite() || !r_diag.allFinite()) return Step::Ignored;

    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    H(2, 5) = 1.0;

    const Vec3 innovation = z - x_.tail<3>();
    const Eigen::Matrix3d R = r_diag.cwiseMax(cfg_.odom_meas_floor_diag.tail<3>()).asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * innovation;
    P_ -= K * H * P_;
    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_zupt(const Vec3 &r_diag)
{
    return update_velocity(Vec3::Zero(), r_diag);
}

// Wheel encoder

Ekf::Step Ekf::update_velocity_u(double u_meas, double var_u)
{
    if (!initialized_) return Step::Ignored;

    // Estado actual: x_ = [x, y, psi, u, v, omega]^T
    // Innovación (residuo) de la velocidad longitudinal 'u'
    double y = u_meas - x_(3);

    // S = H * P * H^T + R  (donde H = [0, 0, 0, 1, 0, 0])
    double S = P_(3, 3) + var_u;

    if (S <= 0.0 || !std::isfinite(S)) {
        return Step::Ignored;
    }

    // Ganancia de Kalman K (columna 3 de la matriz P)
    Vec6 K = P_.col(3) / S;

    // Actualización de estado y normalización de yaw
    x_ += K * y;
    x_(2) = wrap_angle(x_(2));

    // Actualización de Covarianza: P = (I - K*H) * P
    Mat6 I_KH = Mat6::Identity();
    I_KH.col(3) -= K;
    P_ = I_KH * P_;

    symmetrize();
    return commit();
}


void Ekf::symmetrize()
{
    P_ = (0.5 * (P_ + P_.transpose())).eval();
}

} // namespace mirena