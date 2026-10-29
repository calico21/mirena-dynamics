#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

#include "imu_wrapper.hpp"
#include "imu_pipeline.hpp"
#include "stamp_estimator.hpp"

#include <vectornav/Interface/Sensor.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

namespace
{
    /// Frame names. Constants rather than parameters because nothing in this
    /// workspace has ever overridden them and the nodes on the other end of
    /// these topics hard-code the same strings (mirena_lidar's "car/lidar",
    /// mirena_perception's CAR_FRAME). Parameters that only ever take their
    /// default are configuration in appearance only.
    const mirena::IMU::Frames kFrames{"odom", "car/cog", "car/lidar", "car/gnss"};
    constexpr const char *kImuFrame = "car/imu";

    /// The base IMU rate lives in imu_wrapper.hpp, next to the link-budget
    /// check that also depends on it.
    using mirena::IMU::kBaseRateHz;

    constexpr auto kReconnectDelay = 1s;

    /// GPS epoch (Jan 6 1980) as UNIX nanoseconds, and the leap seconds GPS
    /// time does not count. Only used to report how far the host clock has
    /// drifted; a stale leap count would move that report by a second and
    /// change no stamp.
    constexpr int64_t kGpsToUnixNs = 315'964'800LL * 1'000'000'000LL;
    constexpr int64_t kLeapSecondsNs = 18LL * 1'000'000'000LL;
} // namespace

class ImuNode : public rclcpp::Node
{
public:
    ImuNode()
        : Node("mirena_imu"),
          imu_(this->get_logger()),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        imu_dev_path_ = this->declare_parameter<std::string>("imu_device", "/dev/ter_IMU");
        imu_output_rate_hz_ = this->declare_parameter<int>("imu_output_rate_hz", 100);

        // Antenna geometry (metres), in the VN-300's own body axes: x forward,
        // y right, z DOWN. That is not the ROS convention every frame in this
        // workspace uses, so a number lifted straight out of the URDF has to
        // have its y and z negated before it lands here.
        cfg_.antenna_a_offset = this->declare_parameter<std::vector<double>>("antenna_a_offset", {-0.11, 0.0, 0.52});
        cfg_.gnss_baseline = this->declare_parameter<std::vector<double>>("gnss_compass_baseline", {1.50, 0.0, 0.0});
        cfg_.gnss_baseline_unc = this->declare_parameter<std::vector<double>>("gnss_compass_uncertainty", {0.02, 0.02, 0.02});
        if (cfg_.antenna_a_offset.size() != 3 || cfg_.gnss_baseline.size() != 3 || cfg_.gnss_baseline_unc.size() != 3)
        {
            RCLCPP_FATAL(get_logger(), "antenna_a_offset / gnss_compass_baseline / gnss_compass_uncertainty must each have 3 elements.");
            throw std::runtime_error("invalid antenna geometry parameters");
        }

        // --- Onboard INS configuration ---------------------------------------
        //
        // Previously left at whatever the unit happened to have saved, which
        // meant the scenario and the compass enable were invisible state: a
        // sensor bench-tested in AHRS mode ran the whole event without GNSS
        // heading and said nothing about it.
        const int scenario = this->declare_parameter<int>("ins_scenario", 3);
        if (scenario < 0 || scenario > 5)
        {
            RCLCPP_FATAL(get_logger(), "ins_scenario must be 0..5 (see VN register 67); got %d.", scenario);
            throw std::runtime_error("invalid ins_scenario");
        }
        cfg_.ins_scenario = static_cast<uint8_t>(scenario);
        cfg_.ahrs_aiding = this->declare_parameter<bool>("ahrs_aiding", true);
        cfg_.use_gnss_compass = this->declare_parameter<bool>("use_gnss_compass", true);
        // The receiver offers 1 Hz and 5 Hz and nothing between. This is the
        // rate the GNSS fix genuinely updates at, independent of the 100 Hz the
        // binary message streams at.
        cfg_.gnss_rate_hz = this->declare_parameter<int>("gnss_rate_hz", 5);
        if (cfg_.gnss_rate_hz != 1 && cfg_.gnss_rate_hz != 5)
        {
            RCLCPP_FATAL(get_logger(), "gnss_rate_hz must be 1 or 5; got %d.", cfg_.gnss_rate_hz);
            throw std::runtime_error("invalid gnss_rate_hz");
        }
        cfg_.est_baseline = this->declare_parameter<bool>("estimate_baseline", false);
        cfg_.persist_config = this->declare_parameter<bool>("persist_config", true);

        // Per-axis variances the VN does not stream. It reports uncertainty for
        // position, velocity and attitude, but nothing for angular rate or
        // acceleration, so these have to be supplied -- they fill the Imu
        // message covariances and the odometry twist's angular block.
        //
        // Measured, not assumed. Stationary whitened noise on the 2026-08-08
        // log was (6.5e-6, 1.3e-6, 9.7e-7) rad^2/s^2 for the rates and
        // (3.3e-3, 2.9e-2, 1.9e-3) m^2/s^4 for the accelerations; both defaults
        // are an order of magnitude above that, because idle does not show the
        // vibration a run does. Re-measure from a bag if the mounting changes.
        // The VN-300 datasheet gyro figure alone (~1.9e-7) is optimistic by
        // about 5x on this car, which is why none of these come from it.
        ang_vel_var_ = param_vec3("angular_velocity_variance", {6.5e-5, 1.3e-5, 1.0e-5});
        lin_accel_var_ = param_vec3("linear_acceleration_variance", {3.0e-2, 3.0e-1, 2.0e-2});

        // How long the estimate_gnss_baseline service waits for the unit's own
        // baseline estimator to converge. It needs real driving, not idling.
        baseline_timeout_s_ = this->declare_parameter<double>("baseline_estimation_timeout_sec", 300.0);

        // Derived once, not per reconnect: the output rate is fixed at
        // construction, so recomputing it inside the publish loop only created
        // a write to cfg_ racing the service thread's reads of it.
        cfg_.rate_divisor = rate_divisor();

        pipeline_.emplace(ang_vel_var_, lin_accel_var_);

        RCLCPP_INFO(get_logger(), "Initializing IMU bridge on %s (output %d Hz)",
                    imu_dev_path_.c_str(), imu_output_rate_hz_);

        // Sensor-layer outputs only: raw IMU streams, the onboard INS solution
        // as odometry, and the GNSS fix. State fusion (/state/car) and health
        // reporting are the pipeline's job (mirena_state), not the driver's.
        const rclcpp::QoS qos = rclcpp::SensorDataQoS();
        imu_in_lidar_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sensors/lidar/imu", qos);
        imu_in_cog_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", qos);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sensors/imu/odom", qos);
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensors/gps/fix", qos);

        // The estimation polls the unit for minutes, so it gets its own
        // callback group and the executor in main() is multi-threaded: on the
        // default group it would hold the executor and stall every other
        // callback the node has for the duration.
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        estimate_baseline_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/estimate_gnss_baseline",
            std::bind(&ImuNode::handle_estimate_baseline, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), service_cb_group_);

        keep_running_ = true;
        publish_thread_ = std::thread(&ImuNode::publish_loop, this);

        RCLCPP_INFO(get_logger(), "VectorNav IMU streaming layer online.");
    }

    ~ImuNode()
    {
        keep_running_ = false;
        if (publish_thread_.joinable())
        {
            publish_thread_.join();
        }

        if (imu_.is_connected())
        {
            imu_.disconnect();
        }
        RCLCPP_INFO(get_logger(), "IMU bridge node destroyed cleanly.");
    }

private:
    /// A three-element double parameter, validated. Returned by value because
    /// the Pipeline wants a fixed-size array and a vector that might be the
    /// wrong length is exactly the bug this catches at startup rather than at
    /// the first measurement.
    std::array<double, 3> param_vec3(const std::string &name, const std::array<double, 3> &fallback)
    {
        const auto v = this->declare_parameter<std::vector<double>>(
            name, std::vector<double>(fallback.begin(), fallback.end()));
        if (v.size() != 3)
        {
            RCLCPP_FATAL(get_logger(), "%s must have 3 elements; got %zu.", name.c_str(), v.size());
            throw std::runtime_error("invalid " + name);
        }
        return {v[0], v[1], v[2]};
    }

    std::string imu_dev_path_;
    int imu_output_rate_hz_;
    std::array<double, 3> ang_vel_var_{};
    std::array<double, 3> lin_accel_var_{};
    double baseline_timeout_s_ = 300.0;
    mirena::IMU::Config cfg_;

    mirena::IMU::Wrapper imu_;
    // Constructed once the gyro variance parameter has been read, which is why
    // it is optional rather than a plain member.
    std::optional<mirena::IMU::Pipeline> pipeline_;
    mirena::IMU::StampEstimator stamp_estimator_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // The mounting geometry, looked up once.
    //
    // These are bolts, not motion: car/imu, car/lidar and car/cog do not move
    // relative to each other, so re-querying them for every one of a hundred
    // measurements a second bought nothing and cost a failure mode -- a lookup
    // that missed dropped the whole measurement, silently taking the INS
    // odometry that mirena_state runs on down with it.
    bool have_mounting_ = false;
    tf2::Transform cog_from_imu_;
    tf2::Transform lidar_from_imu_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_in_lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_in_cog_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estimate_baseline_srv_;
    /// One estimation at a time. Two concurrent callers would fight over the
    /// est-baseline enable and each see the other's half of the run.
    std::atomic<bool> estimation_running_{false};

    std::thread publish_thread_;
    std::atomic<bool> keep_running_{false};

    /// Throttles the convergence diagnostic. Not RCLCPP_*_THROTTLE, because
    /// the cost being throttled is the register round trip, not the log line.
    std::chrono::steady_clock::time_point last_progress_report_{};

    /// The measurement instant, in the clock domain the rest of the stack uses.
    ///
    /// The sensor knows *when* far better than this process does: timeStartup
    /// is stamped at the sample, inside the VN-300, before the serial link and
    /// the SDK's reader thread add their 10 ms of jitter. What the sensor does
    /// not know is ROS time. StampEstimator holds the two together.
    rclcpp::Time get_measurement_stamp(const VN::Sensor::CompositeDataQueueReturn &msg)
    {
        // Read first: this bounds when the sample was taken, and every
        // microsecond spent before reading it is bias handed to the estimator.
        const rclcpp::Time arrival = this->now();

        if (!msg->time.timeStartup.has_value())
        {
            // Requested explicitly in BinaryOutput1, so its absence means the
            // sensor is not streaming what it was configured to. Better to say
            // so and drop the measurement than to quietly stamp it with a
            // worse clock for the rest of the session.
            throw std::runtime_error("measurement has no timeStartup");
        }

        return rclcpp::Time(
            stamp_estimator_.stamp(static_cast<int64_t>(msg->time.timeStartup.value().nanoseconds()),
                                   arrival.nanoseconds()),
            arrival.get_clock_type());
    }

    /// Report how far the host clock has drifted from GPS UTC.
    ///
    /// This is the number that used to leak into the stamps. It does not stop
    /// existing just because they no longer carry it: it is still the offset
    /// every bag has to be retimed by (tools/retime_bag.py) and the reason
    /// absolute times in the logs cannot be trusted. Measured against
    /// CLOCK_REALTIME rather than now() so it means the same thing under
    /// use_sim_time, and reported rather than corrected -- the fix is gpsd +
    /// chrony on the Jetson, not arithmetic in a driver.
    void report_clock_skew(const VN::Sensor::CompositeDataQueueReturn &msg)
    {
        if (!msg->time.timeGps.has_value() || !msg->time.timeStatus.has_value() ||
            !msg->time.timeStatus.value().towValid)
        {
            return;
        }

        struct timespec ts_real;
        clock_gettime(CLOCK_REALTIME, &ts_real);
        const int64_t host_ns = static_cast<int64_t>(ts_real.tv_sec) * 1'000'000'000LL + ts_real.tv_nsec;
        const int64_t gps_ns = static_cast<int64_t>(msg->time.timeGps.value().nanoseconds()) +
                               kGpsToUnixNs - kLeapSecondsNs;
        const double skew_s = static_cast<double>(host_ns - gps_ns) * 1e-9;

        // 0.1 s is already an order of magnitude past what any time alignment
        // in this stack tolerates.
        if (std::abs(skew_s) > 0.1)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                                 "Host clock is %.3f s %s GPS UTC. Stamps are in ROS time so the stack "
                                 "stays consistent, but absolute times and bag arrival times carry this "
                                 "error (discipline the clock with gpsd + chrony; retime existing bags).",
                                 std::abs(skew_s), skew_s > 0 ? "ahead of" : "behind");
        }
    }

    /// Look the mounting geometry up once, at whatever time TF has it for.
    ///
    /// Deliberately TimePointZero rather than the measurement stamp: these are
    /// static transforms, and asking for them at a specific time only invites a
    /// mismatch with the buffer for no benefit.
    bool ensure_mounting()
    {
        if (have_mounting_)
        {
            return true;
        }
        try
        {
            tf2::fromMsg(tf_buffer_.lookupTransform(kFrames.cog, kImuFrame, tf2::TimePointZero).transform,
                         cog_from_imu_);
            tf2::fromMsg(tf_buffer_.lookupTransform(kFrames.lidar, kImuFrame, tf2::TimePointZero).transform,
                         lidar_from_imu_);
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for mounting transforms: %s", e.what());
            return false;
        }

        const tf2::Vector3 &arm = cog_from_imu_.getOrigin();
        RCLCPP_INFO(get_logger(), "Mounting geometry latched: IMU sits at (%.3f, %.3f, %.3f) in %s.",
                    arm.x(), arm.y(), arm.z(), kFrames.cog.c_str());
        have_mounting_ = true;
        return true;
    }

    void publish_loop()
    {
        while (keep_running_ && rclcpp::ok())
        {
            if (!imu_.is_connected())
            {
                imu_.connect_and_config(imu_dev_path_.c_str(), cfg_);
                if (!imu_.is_connected())
                {
                    std::this_thread::sleep_for(kReconnectDelay);
                    continue;
                }
            }

            VN::Sensor::CompositeDataQueueReturn msg = imu_.get_next_measurement();
            if (!msg)
            {
                RCLCPP_ERROR(get_logger(), "IMU read timeout — assuming dead, disconnecting.");
                imu_.disconnect();
                // It may come back power-cycled, with timeStartup restarted at
                // zero. The estimator detects that itself, but there is nothing
                // worth keeping across a reconnect either way.
                stamp_estimator_.reset();
                continue;
            }

            if (!ensure_mounting())
            {
                continue;
            }

            try
            {
                const rclcpp::Time stamp = get_measurement_stamp(msg);
                report_clock_skew(msg);

                // Only a *converged* INS gets to speak. Aligning means the
                // onboard filter has not agreed with GNSS yet, and if odometry
                // is the first thing mirena_state sees it anchors the run's
                // origin on it -- so accepting one of those pins the whole run
                // on a position the sensor itself does not stand behind.
                //
                // Not publishing is safe, not fatal: mirena_state initialises
                // from the first IMU message instead (init_at_origin) and runs
                // dead-reckoning in its NO_INS_YET mode. Silence here costs the
                // absolute reference, not the state estimate.
                const bool ins_active = ins_tracking(msg);
                report_ins_faults(msg);
                if (!ins_active)
                {
                    report_convergence_progress();
                }

                mirena::IMU::Output out = pipeline_->process(
                    kFrames, stamp, msg, cog_from_imu_, lidar_from_imu_, ins_active);

                imu_in_lidar_pub_->publish(out.lidar_imu);
                imu_in_cog_pub_->publish(out.cog_imu);
                if (out.odom)
                {
                    odom_pub_->publish(*out.odom);
                }
                if (out.fix)
                {
                    fix_pub_->publish(*out.fix);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "Skipping measurement, processing error: %s", e.what());
            }
        }
    }

    /// Surface the rest of the INS status word.
    ///
    /// The driver gates on `mode` and used to throw the other nine bits away,
    /// which is how a run can finish on a degraded GNSS compass or a flagged
    /// IMU fault with nothing in the log to say so. These cost nothing: they
    /// are already in every measurement.
    ///
    /// Deliberately reported while *tracking* too. A fault that appears after
    /// convergence never trips the convergence path, so it was the one class
    /// of problem guaranteed to stay invisible.
    void report_ins_faults(const VN::Sensor::CompositeDataQueueReturn &msg)
    {
        if (!msg->ins.insStatus.has_value())
        {
            return;
        }
        const auto status = msg->ins.insStatus.value();

        if (status.imuErr || status.gnssErr || status.magPresErr)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                                  "VN-300 subsystem fault flagged: IMU %s, GNSS %s, mag/pressure %s. "
                                  "The solution is suspect until this clears.",
                                  status.imuErr ? "ERROR" : "ok",
                                  status.gnssErr ? "ERROR" : "ok",
                                  status.magPresErr ? "ERROR" : "ok");
        }

        // 0 is no fix; the higher values are fix qualities the ICD defines and
        // this driver has no reason to rank, so it reports rather than judges.
        if (status.gnssCompassFix == 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "GNSS compass has no fix (heading is unaided). Check both antennas "
                                 "and the configured baseline.");
        }
    }

    /// Say *why* the INS has not converged, not just that it hasn't.
    ///
    /// "Waiting for the INS to converge" on its own cannot tell a stuck
    /// alignment from a slow one, which is the difference between a fault to
    /// chase in the paddock and a queue to wait in. Registers 86 and 98 answer
    /// it: a startup percentage that climbs is patience, one pinned at 0 with
    /// an antenna seeing no satellites is a cable or a sky-view problem.
    void report_convergence_progress()
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_progress_report_ < 5s)
        {
            return;
        }
        last_progress_report_ = now;

        const auto health = imu_.read_compass_health();
        if (!health)
        {
            RCLCPP_INFO(get_logger(), "Waiting for the INS to converge (no odometry until it does)...");
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Waiting for the INS to converge: compass startup %u%%, heading %.1f deg, "
                    "sats A/B %.0f/%.0f (%.0f common), best C/N0 A/B %.0f/%.0f dB-Hz.",
                    static_cast<unsigned>(health->startup_percent), health->startup_heading,
                    health->sats_a, health->sats_b, health->common_sats,
                    health->cn0_a, health->cn0_b);

        if (health->startup_percent == 0 && (health->sats_a < 4 || health->sats_b < 4))
        {
            RCLCPP_WARN(get_logger(),
                        "Compass startup is not progressing and an antenna is short of satellites "
                        "(A %.0f, B %.0f). Check both antenna cables and the sky view before waiting longer.",
                        health->sats_a, health->sats_b);
        }
    }

    /// Ask the unit to measure its own GNSS compass baseline, and report it.
    ///
    /// The configured baseline is a CAD number that nobody can verify with a
    /// tape measure to the precision the ambiguity search wants. The unit can
    /// measure it directly (register 97), so this hands back the numbers to
    /// put in the launch file along with the uncertainty they deserve. It
    /// needs real driving to converge -- idling in the paddock will time out.
    void handle_estimate_baseline(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (estimation_running_.exchange(true))
        {
            res->success = false;
            res->message = "A baseline estimation is already running.";
            return;
        }
        struct Guard
        {
            std::atomic<bool> &flag;
            ~Guard() { flag = false; }
        } guard{estimation_running_};

        if (!imu_.is_connected())
        {
            res->success = false;
            res->message = "IMU is not connected.";
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
            return;
        }

        if (!imu_.set_est_baseline(true))
        {
            res->success = false;
            res->message = "Could not enable baseline estimation on the unit (register 67).";
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            return;
        }
        RCLCPP_WARN(get_logger(),
                    "GNSS compass baseline estimation started. Drive the car — this needs real "
                    "motion and will time out after %.0f s of standing still. NOTE: enabling and "
                    "restoring the estimator writes register 67 twice, and each write restarts the "
                    "INS alignment. Run this as a deliberate calibration lap, never while staged "
                    "for a timed run.", baseline_timeout_s_);

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(baseline_timeout_s_);
        std::optional<mirena::IMU::EstimatedBaseline> last;
        while (rclcpp::ok() && keep_running_ && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(1s);
            if (!imu_.is_connected())
            {
                break;
            }
            const auto est = imu_.read_est_baseline();
            if (!est)
            {
                continue;
            }
            last = est;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "Estimating baseline: %u measurements, "
                                 "(%.4f, %.4f, %.4f) m +- (%.4f, %.4f, %.4f).",
                                 static_cast<unsigned>(est->num_meas),
                                 est->x, est->y, est->z, est->ux, est->uy, est->uz);
            if (est->complete)
            {
                break;
            }
        }

        // Put the unit back on the configured setting. The estimate is reported
        // rather than applied: writing register 93 restarts the alignment, and
        // doing that silently inside a service the driver runs is exactly the
        // behaviour this driver was just fixed to stop doing.
        imu_.set_est_baseline(cfg_.est_baseline);

        if (!last)
        {
            res->success = false;
            res->message = "No estimate was returned by the unit (register 97 unreadable).";
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            return;
        }

        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(4);
        out << (last->complete ? "complete" : "INCOMPLETE (timed out)")
            << " after " << last->num_meas << " measurements. "
            << "gnss_compass_baseline: [" << last->x << ", " << last->y << ", " << last->z << "], "
            << "gnss_compass_uncertainty: [" << last->ux << ", " << last->uy << ", " << last->uz << "]. "
            << "Configured: [" << cfg_.gnss_baseline[0] << ", " << cfg_.gnss_baseline[1] << ", "
            << cfg_.gnss_baseline[2] << "].";

        const double dx = last->x - cfg_.gnss_baseline[0];
        const double dy = last->y - cfg_.gnss_baseline[1];
        const double dz = last->z - cfg_.gnss_baseline[2];
        const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
        out << " Disagreement with the configured value: " << err << " m.";

        res->success = last->complete;
        res->message = out.str();

        RCLCPP_INFO(get_logger(), "GNSS compass baseline estimation %s", res->message.c_str());
        if (last->complete && err > 0.05)
        {
            RCLCPP_WARN(get_logger(),
                        "The unit's own estimate differs from the configured baseline by %.3f m. "
                        "The configured value is probably wrong — check the sign convention "
                        "(VN body axes are x forward, y right, z DOWN) before trusting heading.", err);
        }
    }

    /// The binary message streams at kBaseRateHz / divisor.
    uint16_t rate_divisor()
    {
        const int rate = std::max(1, imu_output_rate_hz_);
        const int divisor = std::max(1, kBaseRateHz / rate);
        const int actual = kBaseRateHz / divisor;
        if (actual != rate)
        {
            RCLCPP_WARN(get_logger(),
                        "imu_output_rate_hz=%d does not divide the %d Hz base rate; streaming at %d Hz instead.",
                        rate, kBaseRateHz, actual);
        }
        return static_cast<uint16_t>(divisor);
    }

    /// VectorNav INS mode 2 is "tracking"; 0, 1 and 3 are no solution,
    /// aligning and dead reckoning after GPS loss. Nothing here needs to tell
    /// those three apart -- none of them may publish odometry.
    static bool ins_tracking(const VN::Sensor::CompositeDataQueueReturn &msg)
    {
        return msg->ins.insStatus.has_value() && msg->ins.insStatus.value().mode == 2;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Multi-threaded so the baseline estimation service, which polls the unit
    // for minutes, cannot hold the executor against every other callback.
    auto node = std::make_shared<ImuNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
