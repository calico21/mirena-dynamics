#include "ekf.hpp"

#include <algorithm>
#include <cmath>

namespace mirena
{

namespace
{
/// Suelo de la diagonal de covarianza. Las actualizaciones cortas pueden
/// redondear una varianza positiva a un valor negativo; este clamp evita que
/// una varianza negativa llegue a los consumidores. No garantiza que P sea
/// definida positiva, solo acota el síntoma.
constexpr double kMinVariance = 1e-12;

/// Límites de seguridad para la integración. Un dt negativo o nulo ya se
/// descarta antes de llegar aquí; estos valores protegen ante relojes que
/// saltan, mensajes retrasados o parámetros mal configurados.
constexpr double kMinDt = 1e-6;
constexpr double kMaxDt = 0.5;

/// Geometría mínima. Si la configuración trae una distancia entre ejes nula o
/// negativa, el modelo de bicicleta no debe generar una división por cero.
constexpr double kMinWheelbase = 0.1;

/// El ángulo de dirección se satura a ±80°. Aun así, se acota |cos(delta)|
/// para que la derivada del target de guiñada no explote si en el futuro se
/// amplía el rango de dirección.
constexpr double kMaxSteeringRad = 1.4;
constexpr double kMinCosSteering = 0.05;

/// Velocidad mínima de relajación para evitar alpha=0 por configuración.
constexpr double kMinRelaxation = 1e-3;
} // namespace

double Ekf::wrap_angle(double a)
{
    // remainder IEEE devuelve un valor en [-pi, pi] y termina para todo input.
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

Ekf::Step Ekf::init(const Vec7 &z, const Vec7 &r_diag, double stamp)
{
    if (!z.allFinite() || !r_diag.allFinite() || !std::isfinite(stamp)) return Step::Ignored;

    // Un suelo negativo en la configuración no debe crear una covarianza
    // negativa. cwiseMax(0) protege también las componentes que el usuario
    // haya dejado a cero.
    Vec7 floor = Vec7::Zero();
    if (cfg_.init_cov_floor_diag.allFinite())
        floor = cfg_.init_cov_floor_diag.cwiseMax(0.0);

    x_ = z;
    x_(2) = wrap_angle(x_(2));
    P_ = r_diag.cwiseMax(floor).cwiseMax(0.0).asDiagonal();
    stamp_ = stamp;
    initialized_ = true;
    return commit();
}

Ekf::Step Ekf::init(const Vec6 &z, const Vec6 &r_diag, double stamp)
{
    // Las fuentes actuales (INS/GPS) no miden el bias de a_x. Se inicializa a
    // cero con la prior configurada en init_cov_floor_diag(6).
    Vec7 z7;
    z7 << z, 0.0;

    const double bias_prior =
        std::max(0.0, std::isfinite(cfg_.init_cov_floor_diag(6))
                          ? cfg_.init_cov_floor_diag(6)
                          : 0.04);
    Vec7 r7;
    r7 << r_diag, bias_prior;

    return init(z7, r7, stamp);
}

Ekf::Step Ekf::predict(double stamp, double ax, double ay,
                       bool have_steering, double steering)
{
    if (!initialized_) return Step::Ignored;
    if (!std::isfinite(stamp) || !std::isfinite(ax) || !std::isfinite(ay)) return Step::Ignored;

    const double elapsed = stamp - stamp_;
    if (!(elapsed > 0.0)) return Step::Ignored; // muestra duplicada o fuera de orden

    // dt acotado: el máximo configurado se respeta, pero nunca se permite un
    // salto mayor que kMaxDt ni un dt menor que kMinDt. Esto evita que una
    // pausa del reloj o un mensaje muy retrasado meta una integración enorme.
    const double configured_max_dt =
        std::isfinite(cfg_.max_dt) ? cfg_.max_dt : 0.1;
    const double max_dt = std::clamp(configured_max_dt, kMinDt, kMaxDt);
    const double dt = std::clamp(elapsed, kMinDt, max_dt);

    const double psi = x_(2), u = x_(3), v = x_(4), omega = x_(5);
    const double b_ax = x_(6);
    if (!std::isfinite(psi) || !std::isfinite(u) || !std::isfinite(v) ||
        !std::isfinite(omega) || !std::isfinite(b_ax))
    {
        return commit(); // commit resetea si el estado ya no es finito
    }

    const double c = std::cos(psi), s = std::sin(psi);

    // Sin dirección fresca no hay nada que alimente el modelo de bicicleta;
    // se usa la propagación Euler en el frame cuerpo.
    const bool bicycle = cfg_.use_bicycle && have_steering && std::isfinite(steering) &&
                         std::abs(u) >= cfg_.bicycle_min_speed;
    const double delta = bicycle ? std::clamp(steering, -kMaxSteeringRad, kMaxSteeringRad) : 0.0;

    // Protecciones geométricas. Se usan locales para no modificar la
    // configuración ni introducir valores no finitos en el Jacobiano.
    const double wheelbase =
        std::isfinite(cfg_.wheelbase)
            ? std::max(std::abs(cfg_.wheelbase), kMinWheelbase)
            : kMinWheelbase;
    const double l_rear =
        std::isfinite(cfg_.l_rear)
            ? std::clamp(cfg_.l_rear, 0.0, wheelbase)
            : 0.0;
    const double relaxation =
        std::isfinite(cfg_.steering_relaxation)
            ? std::max(cfg_.steering_relaxation, kMinRelaxation)
            : 20.0;

    double alpha = 0.0;
    double kappa = 0.0;
    double omega_target = 0.0;
    double cos_delta = 1.0;

    if (bicycle)
    {
        // Modelo single-track cinemático en el COG. La geometría de no
        // deslizamiento del eje trasero da los targets de guiñada y velocidad
        // lateral; v y omega se aproximan a ellos con un retardo de primer
        // orden. alpha es la discretización exacta de ese retardo, así que un
        // dt grande no puede sobrepasar el target.
        cos_delta = std::max(std::abs(std::cos(delta)), kMinCosSteering);
        kappa = std::tan(delta) / wheelbase;
        omega_target = u * kappa;
        alpha = std::clamp(1.0 - std::exp(-relaxation * dt), 0.0, 1.0);

        if (!std::isfinite(kappa) || !std::isfinite(omega_target) || !std::isfinite(alpha))
            return Step::Ignored;
    }

    // La IMU mide a_x + bias. El estado estima b_ax y la predicción usa la
    // aceleración corregida. El bias evoluciona como random walk:
    // b_ax(k+1) = b_ax(k) + w_b.
    const double ax_eff = ax - b_ax;
    if (!std::isfinite(ax_eff)) return Step::Ignored;

    // A partir de aquí la muestra es válida y el reloj del filtro avanza.
    stamp_ = stamp;

    // F empieza como identidad y las diagonales de los estados con retardo se
    // SOBRESCRIBEN explícitamente con (1 - alpha). No se hace "-= alpha"
    // sobre un valor previo, para que no haya acumulación si se toca el orden
    // de las operaciones.
    Mat7 F = Mat7::Identity();

    if (bicycle)
    {
        x_(0) += (u * c - v * s) * dt;
        x_(1) += (u * s + v * c) * dt;
        x_(2) = wrap_angle(psi + omega * dt);
        x_(3) += (ax_eff + v * omega) * dt;
        // v sigue la guiñada real, no el target de dirección, para que el
        // modelo y la restricción de no deslizamiento coincidan durante el
        // transitorio de relajación.
        x_(4) += alpha * (l_rear * omega - v);
        x_(5) += alpha * (omega_target - omega);
        x_(6) += 0.0; // random walk del bias: sin término determinista

        F(0, 2) = (-u * s - v * c) * dt;
        F(0, 3) = c * dt;
        F(0, 4) = -s * dt;
        F(1, 2) = (u * c - v * s) * dt;
        F(1, 3) = s * dt;
        F(1, 4) = c * dt;
        F(2, 5) = dt;
        F(3, 4) = omega * dt;
        F(3, 5) = v * dt;
        F(3, 6) = -dt;                 // d(u)/d(b_ax)
        F(4, 4) = 1.0 - alpha;         // se sobrescribe la identidad
        F(4, 5) = alpha * l_rear;
        F(5, 3) = alpha * kappa;
        F(5, 5) = 1.0 - alpha;         // se sobrescribe la identidad
        F(6, 6) = 1.0;                 // random walk del bias

        P_ = (F * P_ * F.transpose()).eval();

        Vec7 q_diag = Vec7::Zero();
        if (cfg_.process_noise_diag.allFinite())
            q_diag = cfg_.process_noise_diag.cwiseMax(0.0);
        P_.diagonal() += q_diag * dt;

        // La incertidumbre del ángulo de dirección entra por el target
        // cinemático de guiñada:
        //   d(omega_target)/d(delta) = u / (L * cos(delta)^2)
        const double sigma_delta =
            std::isfinite(cfg_.steering_noise_rad)
                ? std::max(cfg_.steering_noise_rad, 0.0)
                : 0.0;
        const double domega_ddelta = u / (wheelbase * cos_delta * cos_delta);
        if (std::isfinite(domega_ddelta))
        {
            Vec7 Gd = Vec7::Zero();
            Gd(5) = alpha * domega_ddelta;
            P_.noalias() += (sigma_delta * sigma_delta) * Gd * Gd.transpose();
        }
    }
    else
    {
        // Propagación Euler original en el frame cuerpo. Aquí sí se usa a_y
        // para estimar la velocidad lateral, lo que captura el deslizamiento
        // que el modelo cinemático de bicicleta no representa.
        x_(0) += (u * c - v * s) * dt;
        x_(1) += (u * s + v * c) * dt;
        x_(2) = wrap_angle(psi + omega * dt);
        x_(3) += (ax_eff + v * omega) * dt;
        x_(4) += (ay - u * omega) * dt;
        x_(6) += 0.0; // random walk del bias

        F(0, 2) = (-u * s - v * c) * dt;
        F(0, 3) = c * dt;
        F(0, 4) = -s * dt;
        F(1, 2) = (u * c - v * s) * dt;
        F(1, 3) = s * dt;
        F(1, 4) = c * dt;
        F(2, 5) = dt;
        F(3, 4) = omega * dt;
        F(3, 5) = v * dt;
        F(3, 6) = -dt;                 // d(u)/d(b_ax)
        F(4, 3) = -omega * dt;
        F(4, 5) = -u * dt;
        F(6, 6) = 1.0;                 // random walk del bias

        P_ = (F * P_ * F.transpose()).eval();

        Vec7 q_diag = Vec7::Zero();
        if (cfg_.process_noise_diag.allFinite())
            q_diag = cfg_.process_noise_diag.cwiseMax(0.0);
        P_.diagonal() += q_diag * dt;
    }

    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_gyro(double omega_meas, double var_omega)
{
    if (!initialized_) return Step::Ignored;
    if (!std::isfinite(omega_meas) || !std::isfinite(var_omega) || !(var_omega > 0.0))
        return Step::Ignored;

    const double innovation = omega_meas - x_(5);
    const double R = var_omega;
    const double s_var = P_(5, 5) + R;
    if (!(s_var > 0.0) || !std::isfinite(s_var)) return Step::Ignored;

    const double nis = innovation * innovation / s_var;
    if (nis > cfg_.gyro_gate_chi2) return Step::Ignored;

    const Vec7 K = P_.col(5) / s_var;

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));

    Mat7 IKH = Mat7::Identity();
    IKH.col(5) -= K;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_odom(const Vec6 &z, const Vec6 &r_diag)
{
    if (!initialized_) return Step::Ignored;
    if (!z.allFinite() || !r_diag.allFinite()) return Step::Ignored;

    // El INS/GPS mide los primeros seis estados; no mide el bias de a_x.
    // H = [I6 | 0].
    Eigen::Matrix<double, 6, 7> H = Eigen::Matrix<double, 6, 7>::Zero();
    H.leftCols<6>().setIdentity();

    Vec6 innovation = z - x_.head<6>();
    innovation(2) = wrap_angle(innovation(2));

    Vec6 floor = Vec6::Zero();
    if (cfg_.odom_meas_floor_diag.allFinite())
        floor = cfg_.odom_meas_floor_diag.cwiseMax(0.0);
    const Mat6 R = r_diag.cwiseMax(floor).cwiseMax(0.0).asDiagonal();
    const Mat6 S = H * P_ * H.transpose() + R;

    const Mat6 S_inv = S.ldlt().solve(Mat6::Identity());
    if (!S_inv.allFinite()) return Step::Ignored;

    const Eigen::Matrix<double, 7, 6> K = P_ * H.transpose() * S_inv;

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));

    // Forma de Joseph: mantiene la simetría y la semidefinición positiva
    // incluso con aritmética de precisión finita.
    const Mat7 IKH = Mat7::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_pose(const Vec3 &z, const Vec3 &r_diag)
{
    if (!initialized_) return Step::Ignored;
    if (!z.allFinite() || !r_diag.allFinite()) return Step::Ignored;

    Eigen::Matrix<double, 3, 7> H = Eigen::Matrix<double, 3, 7>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;

    Vec3 innovation = z - x_.head<3>();
    innovation(2) = wrap_angle(innovation(2));

    const Eigen::Matrix3d R =
        r_diag.cwiseMax(cfg_.odom_meas_floor_diag.head<3>()).cwiseMax(0.0).asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;

    const Eigen::Matrix3d S_inv = S.ldlt().solve(Eigen::Matrix3d::Identity());
    if (!S_inv.allFinite()) return Step::Ignored;

    const Eigen::Matrix<double, 7, 3> K = P_ * H.transpose() * S_inv;

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));

    const Mat7 IKH = Mat7::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_velocity(const Vec3 &z, const Vec3 &r_diag)
{
    if (!initialized_) return Step::Ignored;
    if (!z.allFinite() || !r_diag.allFinite()) return Step::Ignored;

    Eigen::Matrix<double, 3, 7> H = Eigen::Matrix<double, 3, 7>::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    H(2, 5) = 1.0;

    const Vec3 innovation = z - x_.segment<3>(3);
    const Eigen::Matrix3d R =
        r_diag.cwiseMax(cfg_.odom_meas_floor_diag.tail<3>()).cwiseMax(0.0).asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;

    const Eigen::Matrix3d S_inv = S.ldlt().solve(Eigen::Matrix3d::Identity());
    if (!S_inv.allFinite()) return Step::Ignored;

    const Eigen::Matrix<double, 7, 3> K = P_ * H.transpose() * S_inv;

    x_ += K * innovation;

    const Mat7 IKH = Mat7::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    symmetrize();
    return commit();
}

Ekf::Step Ekf::update_zupt(const Vec3 &r_diag)
{
    return update_velocity(Vec3::Zero(), r_diag);
}

// Actualización escalar de la velocidad longitudinal medida por los encoders.
Ekf::Step Ekf::update_velocity_u(double u_meas, double var_u)
{
    if (!initialized_) return Step::Ignored;
    if (!std::isfinite(u_meas) || !std::isfinite(var_u) || !(var_u > 0.0))
        return Step::Ignored;

    // Estado actual: x_ = [X, Y, psi, u, v, omega, b_ax]^T
    // H = [0, 0, 0, 1, 0, 0, 0]; la innovación es u_meas - u.
    const double innovation = u_meas - x_(3);
    const double S = P_(3, 3) + var_u;

    if (!(S > 0.0) || !std::isfinite(S)) return Step::Ignored;

    // La ganancia también actualiza b_ax a través de P(6,3): la diferencia
    // entre la velocidad integrada y la medida es lo que hace observable el
    // bias de aceleración.
    const Vec7 K = P_.col(3) / S;

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));

    // Forma de Joseph para el escalar R.
    Mat7 IKH = Mat7::Identity();
    IKH.col(3) -= K;
    P_ = IKH * P_ * IKH.transpose() + K * var_u * K.transpose();

    symmetrize();
    return commit();
}

// Restricción de no deslizamiento en el eje trasero: v = l_rear * omega.
//
// Los encoders dan la velocidad longitudinal y la velocidad de guiñada del eje
// trasero, pero no la velocidad lateral. Sin esta restricción, la única
// observación de v es el INS, que puede faltar o ser ruidoso; entonces la
// posición integra una v no observada y la coordenada lateral deriva.
//
// El estado vive en el COG, así que la velocidad lateral del eje trasero es
// v - l_rear * omega. La restricción de deslizamiento nulo es, por tanto, la
// medida escalar (v - l_rear * omega) == 0. var_v es la tolerancia a
// deslizamiento real y el llamador la infla con la velocidad.
Ekf::Step Ekf::update_nonholonomic(double l_rear, double var_v)
{
    if (!initialized_) return Step::Ignored;
    if (!std::isfinite(l_rear) || !std::isfinite(var_v) || !(var_v > 0.0))
        return Step::Ignored;

    Eigen::Matrix<double, 1, 7> H = Eigen::Matrix<double, 1, 7>::Zero();
    H(0, 4) = 1.0;
    H(0, 5) = -l_rear;

    const double innovation = -(x_(4) - l_rear * x_(5));
    const double S = (H * P_ * H.transpose())(0, 0) + var_v;
    if (!(S > 0.0) || !std::isfinite(S)) return Step::Ignored;

    const Vec7 K = (P_ * H.transpose()) / S;

    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));

    // Forma de Joseph: la actualización corta (I - K H) puede perder
    // definición positiva.
    const Mat7 IKH = Mat7::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * var_v * K.transpose();

    symmetrize();
    return commit();
}

void Ekf::symmetrize()
{
    P_ = (0.5 * (P_ + P_.transpose())).eval();
}

} // namespace mirena
