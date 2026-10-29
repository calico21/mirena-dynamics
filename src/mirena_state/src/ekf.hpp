#ifndef MIRENA_STATE_EKF_HPP_
#define MIRENA_STATE_EKF_HPP_

#include <Eigen/Dense>

namespace mirena
{

class Ekf
{
public:
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;

    enum class Step
    {
        Ok,
        Ignored,
        Diverged
    };

    struct Config
    {
        Vec6 process_noise_diag;
        Vec6 odom_meas_floor_diag;
        Vec6 init_cov_floor_diag;
        double gyro_meas_var = 1e-4;
        double max_dt = 0.1;

        Config()
            : process_noise_diag(Vec6::Constant(0.01)),
              odom_meas_floor_diag(Vec6::Constant(1e-6)),
              init_cov_floor_diag(Vec6::Constant(1e-3)) {}
    };

    explicit Ekf(const Config &cfg = Config()) : cfg_(cfg) { reset(); }

    static double wrap_angle(double a);

    void reset();
    Step commit();
    bool is_initialized() const { return initialized_; }
    const Vec6 &state() const { return x_; }
    const Mat6 &covariance() const { return P_; }
    double stamp() const { return stamp_; }
    uint64_t divergences() const { return divergences_; }

    Step init(const Vec6 &z, const Vec6 &r_diag, double stamp);
    Step predict(double stamp, double ax, double ay);
    Step update_gyro(double omega_meas);
    Step update_odom(const Vec6 &z, const Vec6 &r_diag);
    Step update_pose(const Vec3 &z, const Vec3 &r_diag);
    Step update_velocity(const Vec3 &z, const Vec3 &r_diag);
    Step update_velocity_u(double u_meas, double var_u);
    Step update_zupt(const Vec3 &r_diag);

private:
    void symmetrize();

    Config cfg_;
    Vec6 x_ = Vec6::Zero();
    Mat6 P_ = Mat6::Identity();
    double stamp_ = 0.0;
    bool initialized_ = false;
    uint64_t divergences_ = 0;
};

} // namespace mirena

#endif // MIRENA_STATE_EKF_HPP_