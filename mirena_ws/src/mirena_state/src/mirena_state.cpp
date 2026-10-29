#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// Transforms
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
    
// JSON library for string packaging
#include <nlohmann/json.hpp>

// Interfaces
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mirena_common/msg/car.hpp"
#include "mirena_common/msg/heartbeat.hpp"
#include "mirena_common/msg/wheel_speeds.hpp"

#include "ekf.hpp"

// Topics and Frames
#define IMU_TOPIC "/sensors/imu"
#define INS_ODOM_TOPIC "/sensors/imu/odom"
#define WHEEL_SPEEDS_TOPIC "/sensors/wheel_speeds"
#define CAR_STATE_TOPIC "/state/car"

using namespace std::chrono_literals;

namespace
{
    constexpr const char *kOdomFrame = "odom";
    constexpr const char *kBaseFrame = "car/cog";
    constexpr auto kReportPeriod = 100ms;

    constexpr double kInitPoseVar = 0.0;
    constexpr double kZuptVelVar = 0.0025;   // (0.05 m/s)^2
    constexpr double kUnknownVelVar = 25.0;  // (5 m/s)^2, if it is already rolling
    constexpr double kStationaryAccel = 0.5; // m/s^2, gravity already removed
    constexpr double kStationaryRate = 0.05; // rad/s
} // namespace

class MirenaState : public rclcpp::Node
{
    // Callback Groups for thread safety
    rclcpp::CallbackGroup::SharedPtr _filter_cb_group;
    rclcpp::CallbackGroup::SharedPtr _report_cb_group;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _ins_odom_sub;
    rclcpp::Subscription<mirena_common::msg::WheelSpeeds>::SharedPtr _encoder_sub;

    // Publishers
    rclcpp::Publisher<mirena_common::msg::Car>::SharedPtr _car_pub;
    rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr _report_pub;

    // Transform broadcaster (odom -> base)
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    // Timers
    rclcpp::TimerBase::SharedPtr _report_timer;

    // Filter state & mutex
    std::mutex _filter_mutex;
    std::unique_ptr<mirena::Ekf> _ekf;
    rclcpp::Time _last_imu_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time _last_odom_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time _last_encoder_stamp{0, 0, RCL_ROS_TIME};

    double _last_published_stamp = 0.0;

    /// T_odom <- ins, latched once the INS first speaks and re-latched if the
    /// solution ever steps. See ins_odom_callback.
    bool _ins_anchored = false;
    double _anchor_tx = 0.0, _anchor_ty = 0.0, _anchor_dpsi = 0.0;

    /// How many times the anchor has been re-latched because the INS solution
    /// disagreed with the estimate by more than the threshold. Reported, not
    /// enforced: see ins_odom_callback for what a re-anchor costs.
    uint64_t _ins_reanchors = 0;

    // Parameters
    double _odom_timeout_sec = 0.5;
    double _imu_timeout_sec = 0.2;
    double _encoder_timeout_sec = 0.2;
    double _gyro_meas_var = 0.0001;
    bool _use_encoders = true;

    /// Metres by which the INS solution may disagree with the estimate before
    /// the update is absorbed by re-anchoring instead of applied to the state.
    /// <= 0 disables the gate. Measured: routine disagreement is 3-5 m (the
    /// INS's own reported noise), p99.9 is 9.2 m, and the teleports this exists
    /// to stop were 17-18 m.
    double _ins_reanchor_m = 12.0;

    // Wheel encoders
    double _wheel_radius = 0.20;       // m
    double _track_width = 1.2;        // m
    double _encoder_base_var = 0.01;  // (m/s)^2

public:
    MirenaState() : rclcpp::Node("mirena_state")
    {
        this->declare_parameter<double>("odom_timeout_sec", 0.5);
        this->declare_parameter<double>("imu_timeout_sec", 0.2);
        this->declare_parameter<double>("encoder_timeout_sec", 0.2);
        this->declare_parameter<double>("max_predict_dt_sec", 0.1);
        this->declare_parameter<std::vector<double>>(
            "process_noise_diag", {0.01, 0.01, 0.005, 0.5, 0.5, 0.5});
        this->declare_parameter<std::vector<double>>(
            "odom_meas_floor_diag", {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
        this->declare_parameter<std::vector<double>>(
            "init_cov_floor_diag", {0.01, 0.01, 0.001, 0.01, 0.01, 0.001});
        this->declare_parameter<double>("gyro_meas_var", 1.0e-5);
        this->declare_parameter<double>("wheel_radius", 0.2);
        this->declare_parameter<double>("track_width", 1.2);
        this->declare_parameter<double>("encoder_var", 0.01);
        this->declare_parameter<bool>("use_encoders", true);
        this->declare_parameter<double>("ins_reanchor_m", 12.0);

        RCLCPP_INFO(this->get_logger(), "Starting Mirena State");

        _odom_timeout_sec = this->get_parameter("odom_timeout_sec").as_double();
        _imu_timeout_sec = this->get_parameter("imu_timeout_sec").as_double();
        _encoder_timeout_sec = this->get_parameter("encoder_timeout_sec").as_double();
        _wheel_radius = this->get_parameter("wheel_radius").as_double();
        _track_width = this->get_parameter("track_width").as_double();
        _encoder_base_var = this->get_parameter("encoder_var").as_double();
        _use_encoders = this->get_parameter("use_encoders").as_bool();
        _ins_reanchor_m = this->get_parameter("ins_reanchor_m").as_double();

        _filter_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _report_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto filter_sub_options = rclcpp::SubscriptionOptions();
        filter_sub_options.callback_group = _filter_cb_group;

        // Publishers & TF
        _car_pub = this->create_publisher<mirena_common::msg::Car>(CAR_STATE_TOPIC, 10);
        _report_pub = this->create_publisher<mirena_common::msg::Heartbeat>("health", 10);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        mirena::Ekf::Config cfg;
        cfg.process_noise_diag = param_vec6("process_noise_diag");
        cfg.odom_meas_floor_diag = param_vec6("odom_meas_floor_diag");
        cfg.init_cov_floor_diag = param_vec6("init_cov_floor_diag");
        _gyro_meas_var = this->get_parameter("gyro_meas_var").as_double();
        cfg.gyro_meas_var = _gyro_meas_var;
        cfg.max_dt = this->get_parameter("max_predict_dt_sec").as_double();
        _ekf = std::make_unique<mirena::Ekf>(cfg);

        // Sensor subscriptions
        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, rclcpp::SensorDataQoS(),
            std::bind(&MirenaState::imu_callback, this, std::placeholders::_1), filter_sub_options);
        _ins_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            INS_ODOM_TOPIC, rclcpp::SensorDataQoS(),
            std::bind(&MirenaState::ins_odom_callback, this, std::placeholders::_1), filter_sub_options);
        if (_use_encoders)
        {
            _encoder_sub = this->create_subscription<mirena_common::msg::WheelSpeeds>(
                WHEEL_SPEEDS_TOPIC, rclcpp::SensorDataQoS(),
                std::bind(&MirenaState::encoder_callback, this, std::placeholders::_1), filter_sub_options);
            RCLCPP_INFO(this->get_logger(), "Subscribed to %s for wheel encoder speeds (radius=%.3f m, var=%.4f)",
                        WHEEL_SPEEDS_TOPIC, _wheel_radius, _encoder_base_var);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Wheel encoder fusion DISABLED (use_encoders=false). Not subscribing to %s.",
                        WHEEL_SPEEDS_TOPIC);
        }

        // Health/status report timer
        _report_timer = rclcpp::create_timer(
            this, this->get_clock(), kReportPeriod,
            std::bind(&MirenaState::report_callback, this), _report_cb_group);
    }

private:

    mirena::Ekf::Vec6 param_vec6(const std::string &name)
    {
        auto v = this->get_parameter(name).as_double_array();
        if (v.size() != 6)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter %s must have 6 elements, got %zu.",
                         name.c_str(), v.size());
            throw std::runtime_error("invalid " + name);
        }
        return mirena::Ekf::Vec6(v.data());
    }

    static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void log_step(mirena::Ekf::Step step, const char *what)
    {
        using Step = mirena::Ekf::Step;
        if (step == Step::Diverged)
        {
          RCLCPP_ERROR(this->get_logger(),
             "EKF diverged on %s (non-finite state or covariance) — filter reset, "
             "re-initializing from the next INS fix. Reset #%u.",
             what, static_cast<unsigned int>(_ekf->divergences()));
        }
        else if (step == Step::Ignored)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "EKF ignored %s: non-finite values or a stamp that did not "
                                 "advance. Check the sensor layer.", what);
        }
    }

    void init_at_origin(const sensor_msgs::msg::Imu &msg)
    {
        const double accel = std::hypot(msg.linear_acceleration.x, msg.linear_acceleration.y);
        const double rate = std::abs(msg.angular_velocity.z);
        const bool stationary = accel < kStationaryAccel && rate < kStationaryRate;
        const double vel_var = stationary ? kZuptVelVar : kUnknownVelVar;

        mirena::Ekf::Vec6 r;
        r << kInitPoseVar, kInitPoseVar, kInitPoseVar, vel_var, vel_var, _gyro_meas_var;

        const double t = rclcpp::Time(msg.header.stamp).seconds();
        if (_ekf->init(mirena::Ekf::Vec6::Zero(), r, t) != mirena::Ekf::Step::Ok)
        {
            log_step(mirena::Ekf::Step::Ignored, "origin initialization");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "odom origin defined at the current pose; dead reckoning until the INS "
                    "converges (%s).",
                    stationary ? "stationary, zero-velocity update applied"
                               : "MOVING at startup — velocity seeded as unknown");
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lock(_filter_mutex);
        _last_imu_stamp = msg->header.stamp;
        if (!_ekf->is_initialized())
        {
            if (!_ins_anchored) init_at_origin(*msg);
            return;
        }

        const double t = rclcpp::Time(msg->header.stamp).seconds();
        log_step(_ekf->predict(t, msg->linear_acceleration.x, msg->linear_acceleration.y), "IMU predict");
        if (!_ekf->is_initialized()) return;
        log_step(_ekf->update_gyro(msg->angular_velocity.z), "gyro update");

        publish_state();
    }
    
    void encoder_callback(const mirena_common::msg::WheelSpeeds::ConstSharedPtr &msg)
    {
        if (!_use_encoders) return;

        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "[Encoder] Ignoring message with null timestamp on %s.", WHEEL_SPEEDS_TOPIC);
            return;
        }

        std::lock_guard<std::mutex> lock(_filter_mutex);
        _last_encoder_stamp = msg->header.stamp;

        RCLCPP_INFO_ONCE(this->get_logger(),
                         "[Encoder] Callback activated: received first message on %s (rl=%.2f RPM, rr=%.2f RPM)",
                         WHEEL_SPEEDS_TOPIC, msg->rl, msg->rr);

        if (!_ekf->is_initialized())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "[Encoder] Callback active, but EKF is not initialized yet.");
            return;
        }

        if (!std::isfinite(msg->rl) || !std::isfinite(msg->rr))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "[Encoder] Ignoring message with non-finite wheel speeds (rl=%.2f, rr=%.2f).",
                                 msg->rl, msg->rr);
            return;
        }

        const double v_left = (msg->rl * 2.0 * M_PI / 60.0) * _wheel_radius;
        const double v_right = (msg->rr * 2.0 * M_PI / 60.0) * _wheel_radius;
        const double u = 0.5 * (v_left + v_right);

        if (std::abs(u) > 15.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "[Encoder] Rejecting outlier velocity u=%.2f m/s (|u| > 15.0 m/s).", u);
            return;
        }

        const double v_diff = std::abs(v_left - v_right);
        const double dynamic_var = std::min(_encoder_base_var + 0.10 * (v_diff * v_diff), 1.0);

        const double t = rclcpp::Time(msg->header.stamp).seconds();
        if (t < _ekf->stamp())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "[Encoder] Discarding out-of-order message: stamp %.6f s < EKF stamp %.6f s.",
                                 t, _ekf->stamp());
            return;
        }
        if (t > _ekf->stamp())
        {
            log_step(_ekf->predict(t, 0.0, 0.0), "encoder time sync predict");
        }

        log_step(_ekf->update_velocity_u(u, dynamic_var), "encoder speed update");
    }

    /// Recompute T_odom <- ins so the given INS solution lands exactly on the
    /// pose the filter currently carries.
    ///
    /// `innovation_m` is the disagreement that provoked the call when this is a
    /// re-anchor, and negative for the ordinary first fix. The state is not
    /// touched either way: the disagreement is absorbed into the offset, which
    /// is what keeps /state/car and the odom->base TF continuous across it.
    void latch_anchor(double ins_x, double ins_y, double ins_psi, double innovation_m = -1.0)
    {
        const bool reanchor = innovation_m >= 0.0;
        const auto &x = _ekf->state();

        // On the first fix the odom frame's orientation is *defined* here, so
        // the heading offset has to be derived from the pose.
        //
        // On a re-anchor it must not be. The pose's heading is gyro-integrated
        // and drifts badly across a dropout -- measured on the recorded run,
        // 27.7 deg of error over a 23 s outage against 0.7 deg from the INS --
        // so re-deriving here would freeze that error into the anchor for good
        // and throw away the only absolute heading source the car has. Left
        // alone, the INS heading flows through the original transform and
        // update_odom corrects it as an ordinary measurement, with the INS's
        // own yaw covariance behind it. Position is unaffected either way: the
        // translation below is still solved to put the solution on the pose.
        if (!reanchor)
        {
            _anchor_dpsi = mirena::Ekf::wrap_angle(x(2) - ins_psi);
        }

        const double c = std::cos(_anchor_dpsi), s = std::sin(_anchor_dpsi);
        _anchor_tx = x(0) - (c * ins_x - s * ins_y);
        _anchor_ty = x(1) - (s * ins_x + c * ins_y);
        _ins_anchored = true;

        if (reanchor)
        {
            RCLCPP_WARN(this->get_logger(),
                        "INS solution %.2f m from the estimate (re-anchor #%u): absorbed into the "
                        "odom offset (%.2f, %.2f) m, instead of applied to the pose. The pose "
                        "stands at (%.2f, %.2f), so no consumer of /state/car moves. The INS's "
                        "own absolute position is given up until it moves again; its heading is "
                        "not, and still corrects the estimate (heading offset held at %.2f deg).",
                        innovation_m, static_cast<unsigned int>(_ins_reanchors), _anchor_tx, _anchor_ty,
                        x(0), x(1), _anchor_dpsi * 180.0 / M_PI);
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "INS anchored into odom: offset (%.2f, %.2f) m, %.2f deg. Dead reckoning had "
                    "carried the pose to (%.2f, %.2f); absolute corrections start now.",
                    _anchor_tx, _anchor_ty, _anchor_dpsi * 180.0 / M_PI, x(0), x(1));
    }

    void ins_to_odom(double ix, double iy, double ipsi,
                     double &ox, double &oy, double &opsi) const
    {
        const double c = std::cos(_anchor_dpsi), s = std::sin(_anchor_dpsi);
        ox = c * ix - s * iy + _anchor_tx;
        oy = s * ix + c * iy + _anchor_ty;
        opsi = mirena::Ekf::wrap_angle(ipsi + _anchor_dpsi);
    }

    void ins_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lock(_filter_mutex);
        _last_odom_stamp = msg->header.stamp;
        const double t = rclcpp::Time(msg->header.stamp).seconds();

        const double ins_x = msg->pose.pose.position.x;
        const double ins_y = msg->pose.pose.position.y;
        const double ins_psi = yaw_from_quat(msg->pose.pose.orientation);

        if (!_ins_anchored)
        {
            if (_ekf->is_initialized())
            {
                latch_anchor(ins_x, ins_y, ins_psi);
            }
            else
            {
                _anchor_tx = _anchor_ty = _anchor_dpsi = 0.0;
                _ins_anchored = true;
                RCLCPP_INFO(this->get_logger(),
                            "INS arrived before any IMU; odom origin adopted from the INS.");
            }
        }

        double zx, zy, zpsi;
        ins_to_odom(ins_x, ins_y, ins_psi, zx, zy, zpsi);

        // An INS solution this far from where the filter believes the car is
        // is not a correction.
        //
        // The driver publishes odometry only while the onboard filter is
        // tracking, so during an outage the estimate runs on IMU and encoders
        // alone and drifts. When the solution returns it may be metres from
        // where the car actually is, and handing that to update_odom moves the
        // state by metres in a single sample -- the recorded run this was
        // measured on teleported 12.9 m at t=72.46 s and then cascaded a further
        // 13.4 m over four samples at t=85.18 s, both times on the first
        // message after a dropout. Everything downstream inherits it: SLAM's
        // keyframes, the track projection, the guidance error.
        //
        // The test is the innovation, not the step between two INS solutions.
        // A step cannot be judged on its own, because the car really did drive
        // during the outage: the two reacquisitions in that run stepped 16.2 m
        // over 23 s and 35.4 m over 10.7 s, both kinematically ordinary, so a
        // step threshold either misses them or fires on ordinary driving.
        // Elapsed time needs no term either -- the odometry has moved too, so
        // the innovation already carries it.
        //
        // Measured over that run's 27673 INS updates: p50 1.5 m, p99 8.5 m,
        // p99.9 9.2 m, against 16.9 m and 18.1 m at the two teleports. The
        // routine 3-5 m spread in between is the INS's own reported noise
        // (posU 2.3-3.3 m in the sim), not drift, which is why the threshold
        // sits well above it.
        //
        // Re-anchoring is the same recovery as the first fix, for the same
        // reason: the difference goes into T_odom <- ins rather than into the
        // pose. The INS's absolute reference is lost for the rest of the run,
        // which is the deliberate trade -- an odom frame that never moves is
        // worth more to the rest of the stack than the offset it gave up.
        if (_ekf->is_initialized())
        {
            // Advance to the measurement's stamp first. ins_odom_callback does
            // not otherwise move the filter's clock, so without this the
            // innovation is taken against a pose up to one IMU period stale.
            // Same time-sync predict the encoder callback uses.
            if (t > _ekf->stamp())
            {
                log_step(_ekf->predict(t, 0.0, 0.0), "INS time sync predict");
            }

            if (_ekf->is_initialized())
            {
                const auto &x = _ekf->state();
                const double innovation = std::hypot(zx - x(0), zy - x(1));
                if (_ins_reanchor_m > 0.0 && innovation > _ins_reanchor_m)
                {
                    ++_ins_reanchors;
                    latch_anchor(ins_x, ins_y, ins_psi, innovation);
                    // The anchor just moved the INS under the current pose, so
                    // the measurement has to be mapped again before it is used.
                    ins_to_odom(ins_x, ins_y, ins_psi, zx, zy, zpsi);
                }
            }
        }

        mirena::Ekf::Vec6 z;
        z << zx, zy, zpsi,
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.angular.z;

        mirena::Ekf::Vec6 r;
        r << msg->pose.covariance[0],   // x
            msg->pose.covariance[7],    // y
            msg->pose.covariance[35],   // yaw
            msg->twist.covariance[0],   // u
            msg->twist.covariance[7],   // v
            msg->twist.covariance[35];  // omega

        if (!_ekf->is_initialized())
        {
            if (_ekf->init(z, r, t) == mirena::Ekf::Step::Ok)
            {
                RCLCPP_INFO(this->get_logger(), "EKF re-initialized from INS at (%.2f, %.2f)", z(0), z(1));
                publish_state();
            }
            else
            {
                log_step(mirena::Ekf::Step::Ignored, "INS initialization");
            }
            return;
        }

        log_step(_ekf->update_odom(z, r), "INS odometry update");
    }

    void publish_state()
    {
        if (!_ekf->is_initialized()) return;

        if (_ekf->stamp() <= _last_published_stamp) return;
        _last_published_stamp = _ekf->stamp();

        const auto &x = _ekf->state();
        const auto &P = _ekf->covariance();
        const rclcpp::Time stamp(static_cast<int64_t>(_ekf->stamp() * 1e9), RCL_ROS_TIME);

        mirena_common::msg::Car car;
        car.header.stamp = stamp;
        car.header.frame_id = kOdomFrame;
        car.child_frame_id = kBaseFrame;
        car.x = x(0);
        car.y = x(1);
        car.psi = x(2);
        car.u = x(3);
        car.v = x(4);
        car.omega = x(5);
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                car.covariance[i * 6 + j] = P(i, j);
        _car_pub->publish(car);

        send_tf(x(0), x(1), x(2), stamp);
    }

    void send_tf(double x, double y, double psi, const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = kOdomFrame;
        t.child_frame_id = kBaseFrame;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, psi);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        _tf_broadcaster->sendTransform(t);
    }

    void report_callback()
    {
        mirena_common::msg::Heartbeat msg;
        msg.timestamp = this->now();
        msg.node_name = this->get_name();

        nlohmann::json report_json;
        {
            std::lock_guard<std::mutex> lock(_filter_mutex);
            const double now_s = this->now().seconds();

            const bool have_odom = _last_odom_stamp.nanoseconds() > 0;
            const bool have_imu = _last_imu_stamp.nanoseconds() > 0;
            const bool have_encoder = _last_encoder_stamp.nanoseconds() > 0;
            const double odom_age = now_s - _last_odom_stamp.seconds();
            const double imu_age = now_s - _last_imu_stamp.seconds();
            const double encoder_age = now_s - _last_encoder_stamp.seconds();

            const bool odom_stale = !have_odom || odom_age > _odom_timeout_sec;
            const bool imu_stale = !have_imu || imu_age > _imu_timeout_sec;
            const bool encoder_stale = _use_encoders &&
                                        (!have_encoder || encoder_age > _encoder_timeout_sec);

            std::string mode;
            if (!_ekf->is_initialized())
                mode = "INIT";
            else if (!_ins_anchored)
                mode = "NO_INS_YET";
            else if (odom_stale && imu_stale)
                mode = "STALE";
            else if (odom_stale)
                mode = "DEAD_RECKONING";
            else if (imu_stale)
                mode = "NO_IMU";
            else
                mode = "FUSED";

            report_json["mode"] = mode;
            report_json["ins_anchored"] = _ins_anchored;
            report_json["ins_reanchors"] = _ins_reanchors;
            report_json["ins_reanchor_m"] = _ins_reanchor_m;
            report_json["encoders_enabled"] = _use_encoders;
            report_json["odom_stale"] = odom_stale;
            report_json["imu_stale"] = imu_stale;
            report_json["encoder_stale"] = encoder_stale;
            if (have_odom) report_json["odom_age_s"] = odom_age;
            if (have_imu) report_json["imu_age_s"] = imu_age;
            if (have_encoder) report_json["encoder_age_s"] = encoder_age;
            report_json["ekf_resets"] = _ekf->divergences();
            if (_ekf->is_initialized())
            {
                const auto &x = _ekf->state();
                report_json["car_pos_x"] = x(0);
                report_json["car_pos_y"] = x(1);
                report_json["car_pos_yaw"] = x(2);
                report_json["car_vel_u"] = x(3);
            }
        }

        msg.report = report_json.dump();
        _report_pub->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MirenaState>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}