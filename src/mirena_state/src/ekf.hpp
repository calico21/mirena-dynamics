#ifndef MIRENA_STATE_EKF_HPP_
#define MIRENA_STATE_EKF_HPP_

#include <Eigen/Dense>

#include <cstdint>

namespace mirena
{

/**
 * EKF de 7 estados para el estado del vehículo en el plano:
 *
 *   x = [X, Y, psi, u, v, omega, b_ax]^T
 *
 * donde:
 *   - X, Y       posición en el frame odom [m]
 *   - psi        orientación [rad]
 *   - u          velocidad longitudinal en el COG [m/s]
 *   - v          velocidad lateral en el COG [m/s]
 *   - omega      velocidad de guiñada [rad/s]
 *   - b_ax       bias de la aceleración longitudinal de la IMU [m/s^2]
 *
 * El sesgo se modela como un random walk:
 *
 *   b_ax(k+1) = b_ax(k) + w_b,   w_b ~ N(0, q_b * dt)
 *
 * y se usa en la predicción de u como (a_x - b_ax). El INS/GPS no mide el
 * sesgo, por lo que las actualizaciones de posición/velocidad usan una matriz
 * H que selecciona únicamente los primeros seis estados. El sesgo se observa
 * indirectamente a través de la velocidad longitudinal de los encoders.
 */
class Ekf
{
public:
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;
    using Vec7 = Eigen::Matrix<double, 7, 1>;
    using Mat7 = Eigen::Matrix<double, 7, 7>;

    enum class Step
    {
        Ok,
        Ignored,
        Rejected,
        Diverged
    };

struct Config
{
    /// Varianza continua del ruido de proceso. La última componente es la
    /// intensidad del random walk del bias de a_x, en (m/s^2)^2/s.
    Vec7 process_noise_diag;

    /// Suelo de la covarianza de las medidas de 6 estados del INS/GPS.
    /// El INS no mide b_ax, así que esta matriz sigue siendo 6x6.
    Vec6 odom_meas_floor_diag;

    /// Suelo de la covarianza inicial. La última componente es la varianza
    /// inicial del bias de a_x, en (m/s^2)^2.
    Vec7 init_cov_floor_diag;

    double gyro_meas_var = 1e-4;
    double gyro_gate_chi2 = 6.63;   // 99 %, 1 gl
    double max_dt = 0.1;

    /// Modelo cinemático de bicicleta. Si está desactivado, o si predict() se
    /// llama sin una dirección fresca, se usa el modelo Euler en el frame
    /// cuerpo. El modelo de bicicleta es el single-track cinemático expresado
    /// en el COG:
    ///
    ///   kappa      = tan(delta) / (L_F + L_R)
    ///   omega_kin  = u * kappa
    ///   v_kin      = L_R * omega_kin
    ///
    /// omega se relaja hacia omega_kin y v hacia L_R*omega con un retardo de
    /// primer orden, en lugar de sobrescribirse. Así el gyro y el INS conservan
    /// su influencia y el filtro no se vuelve singular. En régimen permanente
    /// v = L_R*omega = v_kin. Este modelo replica el usado por el MPC
    /// (K_RELAX en mirena_control/src/mpc/codegen/dbm_model.py).
    bool use_bicycle = false;
    double wheelbase = 1.55;           // L_F + L_R, m
    double l_rear = 0.744;             // COG al eje trasero, m
    double steering_relaxation = 20.0; // 1/s, retardo hacia el target cinemático
    double steering_noise_rad = 0.02;  // 1 sigma del ángulo de dirección, rad
    double bicycle_min_speed = 0.5;    // m/s; por debajo se usa Euler
                                       // (un giro puro en parado tiene omega != 0,
                                       // que el modelo cinemático no puede explicar)

    Config()
        : process_noise_diag(Vec7::Constant(0.01)),
          odom_meas_floor_diag(Vec6::Constant(1e-6)),
          init_cov_floor_diag(Vec7::Constant(1e-3))
    {
        // Ruido bajo del random walk del bias y una prior inicial razonable
        // para un IMU MEMS. Ambos son configurables desde ROS 2.
        process_noise_diag(6) = 1e-6;
        init_cov_floor_diag(6) = 0.04;
    }
};

    explicit Ekf(const Config &cfg = Config()) : cfg_(cfg) { reset(); }

    static double wrap_angle(double a);

    void reset();
    Step commit();
    bool is_initialized() const { return initialized_; }
    const Vec7 &state() const { return x_; }
    const Mat7 &covariance() const { return P_; }
    double stamp() const { return stamp_; }
    uint64_t divergences() const { return divergences_; }

    /// Inicialización completa de 7 estados. El sesgo de a_x se puede dejar en
    /// cero con una varianza inicial grande si no se conoce.
    Step init(const Vec7 &z, const Vec7 &r_diag, double stamp);

    /// Compatibilidad con las fuentes actuales de 6 estados (INS/GPS). El bias
    /// se inicializa a cero y su varianza sale de init_cov_floor_diag(6).
    Step init(const Vec6 &z, const Vec6 &r_diag, double stamp);

    Step predict(double stamp, double ax, double ay,
                 bool have_steering = false, double steering = 0.0);
    Step update_gyro(double omega_meas, double var_omega);
    Step update_odom(const Vec6 &z, const Vec6 &r_diag);
    Step update_pose(const Vec3 &z, const Vec3 &r_diag);
    Step update_velocity(const Vec3 &z, const Vec3 &r_diag);
    Step update_velocity_u(double u_meas, double var_u);
    Step update_nonholonomic(double l_rear, double var_v);
    Step update_zupt(const Vec3 &r_diag);

private:
    void symmetrize();

    Config cfg_;
    Vec7 x_ = Vec7::Zero();
    Mat7 P_ = Mat7::Identity();
    double stamp_ = 0.0;
    bool initialized_ = false;
    uint64_t divergences_ = 0;
};

} // namespace mirena

#endif // MIRENA_STATE_EKF_HPP_
