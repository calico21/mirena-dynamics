#pragma once

#include <vectornav/Interface/Sensor.hpp>
#include <vectornav/Interface/Registers.hpp>

#include <rclcpp/time.hpp>

#include "gnss_conversions.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <optional>
#include <stdexcept>
#include <string>

namespace mirena::IMU
{
    /// Where the odom frame's origin sits on the earth. Latched once, from the
    /// first INS solution the driver decides to trust.
    struct GeodesicRef
    {
        double latitude;
        double longitude;
        double altitude;
    };

    /// Frames the driver publishes into.
    struct Frames
    {
        std::string odom;
        std::string cog;
        std::string lidar;
        std::string gnss;
    };

    /// One measurement, converted.
    ///
    /// The IMU messages are always produced; the INS solution only once the
    /// onboard filter has one, which is why those two are optional. Consumers
    /// (mirena_state) must tolerate their absence -- there is no odometry before
    /// GPS lock, and saying nothing is better than saying zero.
    struct Output
    {
        sensor_msgs::msg::Imu cog_imu;
        sensor_msgs::msg::Imu lidar_imu;
        std::optional<nav_msgs::msg::Odometry> odom;
        std::optional<sensor_msgs::msg::NavSatFix> fix;
    };

    namespace detail
    {
        /// A 180-degree turn about X.
        ///
        /// It appears twice for two different reasons that happen to be the same
        /// rotation: NED -> NWU between world frames, and the sensor's own
        /// x-forward/y-right/z-down body axes -> the ROS x-forward/y-left/z-up
        /// convention every frame in this workspace uses.
        inline const tf2::Quaternion &flip_x()
        {
            static const tf2::Quaternion q(1, 0, 0, 0);
            return q;
        }

        /// A vector the sensor measured in its own body axes, expressed in the
        /// frame that `frame_from_imu` targets.
        ///
        /// The order matters and is the whole reason this is a named function.
        /// The axis flip has to come first, because the mounting rotation that
        /// TF hands back is defined between two ROS frames and expects a ROS
        /// vector. Applying the mounting rotation to raw sensor axes and
        /// flipping afterwards lands the mounting pitch on the wrong side --
        /// with the 10-degree mainhoop pitch in transforms.yaml that is a
        /// 20-degree error that mixes forward with vertical.
        inline tf2::Vector3 to_frame(const tf2::Transform &frame_from_imu,
                                     const tf2::Vector3 &sensor_vector)
        {
            return tf2::quatRotate(frame_from_imu.getRotation(),
                                   tf2::quatRotate(flip_x(), sensor_vector));
        }

        /// The sensor's NED attitude, re-expressed as the orientation of the
        /// target frame in the ROS world frame:
        ///
        ///   world <- frame = (NWU<-NED)(NED<-sensor)(sensor<-imu)(imu<-frame)
        inline tf2::Quaternion orientation_of(const tf2::Transform &frame_from_imu,
                                              const tf2::Quaternion &ned_attitude)
        {
            return flip_x() * ned_attitude * flip_x() *
                   frame_from_imu.getRotation().inverse();
        }

        /// Rigid-body transfer of a velocity from the IMU to the origin of the
        /// frame, both already in that frame's axes.
        ///
        /// `frame_from_imu.getOrigin()` is where the IMU sits in the target
        /// frame, so the arm pointing from the IMU to that origin is its
        /// negation -- which is where the minus signs come from. Angular
        /// acceleration is neglected: estimating it means differentiating the
        /// gyro, and the term it feeds is small next to the centripetal one.
        inline tf2::Vector3 transfer_velocity(const tf2::Transform &frame_from_imu,
                                              const tf2::Vector3 &velocity,
                                              const tf2::Vector3 &angular_rate)
        {
            return velocity - angular_rate.cross(frame_from_imu.getOrigin());
        }

        inline tf2::Vector3 transfer_acceleration(const tf2::Transform &frame_from_imu,
                                                  const tf2::Vector3 &acceleration,
                                                  const tf2::Vector3 &angular_rate)
        {
            const tf2::Vector3 arm = frame_from_imu.getOrigin();
            return acceleration - angular_rate.cross(angular_rate.cross(arm));
        }

        inline void set(geometry_msgs::msg::Vector3 &out, const tf2::Vector3 &v)
        {
            out.x = v.x();
            out.y = v.y();
            out.z = v.z();
        }

        inline void set(geometry_msgs::msg::Quaternion &out, const tf2::Quaternion &q)
        {
            out.x = q.x();
            out.y = q.y();
            out.z = q.z();
            out.w = q.w();
        }
    } // namespace detail

    /// Turns one VectorNav measurement into the messages the rest of the stack
    /// consumes.
    ///
    /// One class rather than the two it replaced. The AHRS and INS paths shared
    /// every line of frame handling and had drifted apart in three of them, each
    /// difference silently changing what a downstream node was told; merging
    /// them makes that class of bug impossible rather than merely absent.
    ///
    /// NOTE on `linear_acceleration`: the Imu messages carry gravity-free
    /// acceleration, which sensor_msgs/Imu does *not* specify -- that field is
    /// defined as the raw accelerometer reading. This is deliberate and load
    /// bearing: mirena_state's EKF mechanises from gravity-free body
    /// acceleration and would integrate a 9.81 m/s^2 bias if this were changed.
    /// Anything else consuming these topics has to know.
    class Pipeline
    {
        std::optional<GeodesicRef> geodesic_ref_;

        /// Per-axis variances the filter does not stream, so unlike every other
        /// covariance here these have to be supplied. See the
        /// angular_velocity_variance and linear_acceleration_variance
        /// parameters for where the numbers come from -- they are measured off
        /// a recorded run, not taken from a datasheet, because on this car
        /// vibration dominates the datasheet figure by about 5x.
        std::array<double, 3> ang_vel_var_;
        std::array<double, 3> lin_accel_var_;

        /// GPS time of week of the last fix published, so a solution repeated
        /// across many measurements is only published once.
        std::optional<int64_t> last_gnss_tow_ns_;

    public:
        Pipeline(const std::array<double, 3> &ang_vel_var,
                 const std::array<double, 3> &lin_accel_var)
            : ang_vel_var_(ang_vel_var), lin_accel_var_(lin_accel_var) {}

        /// @param ins_active the onboard INS has a usable solution
        /// @param cog_from_imu   TF car/cog <- car/imu
        /// @param lidar_from_imu TF car/lidar <- car/imu
        Output process(const Frames &frames,
                       const rclcpp::Time &stamp,
                       const VN::Sensor::CompositeDataQueueReturn &msg,
                       const tf2::Transform &cog_from_imu,
                       const tf2::Transform &lidar_from_imu,
                       bool ins_active)
        {
            using namespace detail;

            if (!msg->attitude.quaternion.has_value() ||
                !msg->imu.angularRate.has_value())
            {
                throw std::runtime_error("measurement has no attitude or angular rate");
            }

            const auto q = msg->attitude.quaternion.value();
            const tf2::Quaternion ned_attitude(q.vector[0], q.vector[1], q.vector[2], q.scalar);
            const auto rate = msg->imu.angularRate.value();
            const tf2::Vector3 sensor_ang_vel(rate[0], rate[1], rate[2]);

            // Linear acceleration, gravity removed, still in sensor axes.
            //
            // With the INS running the onboard filter has already done it and
            // does it better. Without one, gravity is subtracted using the
            // attitude: the accelerometer measures specific force, so a level
            // stationary sensor reads -g and adding g back gives zero.
            tf2::Vector3 sensor_lin_accel;
            if (ins_active && msg->attitude.linBodyAcc.has_value())
            {
                const auto a = msg->attitude.linBodyAcc.value();
                sensor_lin_accel = tf2::Vector3(a[0], a[1], a[2]);
            }
            else if (msg->imu.accel.has_value())
            {
                constexpr double kGravity = 9.80665;
                const auto a = msg->imu.accel.value();
                sensor_lin_accel = tf2::Vector3(a[0], a[1], a[2]) +
                                   tf2::quatRotate(ned_attitude.inverse(),
                                                   tf2::Vector3(0, 0, kGravity));
            }
            else
            {
                throw std::runtime_error("measurement has no acceleration");
            }

            const AttitudeVar att_var = attitude_variances(msg);

            Output out;
            out.cog_imu = imu_message(frames.cog, stamp, cog_from_imu, ned_attitude,
                                      sensor_ang_vel, sensor_lin_accel, att_var);
            out.lidar_imu = imu_message(frames.lidar, stamp, lidar_from_imu, ned_attitude,
                                        sensor_ang_vel, sensor_lin_accel, att_var);

            // The fix is the receiver's, so it is published whenever the
            // receiver has one -- deliberately not gated on ins_active. Before
            // convergence it is the only evidence in the stack that the GNSS
            // side is alive at all, which is exactly when someone standing next
            // to a car that will not converge needs to see it.
            out.fix = fix_message(frames.gnss, stamp, msg);

            if (!ins_active)
            {
                return out;
            }
            if (!msg->ins.posLla.has_value() || !msg->ins.velBody.has_value())
            {
                throw std::runtime_error("INS is active but reported no position or velocity");
            }

            out.odom = odom_message(frames, stamp, msg, cog_from_imu, ned_attitude,
                                    sensor_ang_vel);
            return out;
        }

    private:
        /// Attitude 1-sigma from the onboard filter, as variances in rad^2,
        /// ordered roll, pitch, yaw.
        ///
        /// yprU arrives as (yaw, pitch, roll) in degrees -- reversed from every
        /// ROS convention around it, which is the whole reason this is one
        /// function rather than the same three lines in two places.
        struct AttitudeVar
        {
            bool known = false;
            double roll = 0, pitch = 0, yaw = 0;
        };

        static AttitudeVar attitude_variances(const VN::Sensor::CompositeDataQueueReturn &msg)
        {
            AttitudeVar out;
            if (!msg->attitude.yprU.has_value())
            {
                return out;
            }
            const auto ypr = msg->attitude.yprU.value();
            constexpr double to_rad = M_PI / 180.0;
            out.known = true;
            out.yaw = (ypr[0] * to_rad) * (ypr[0] * to_rad);
            out.pitch = (ypr[1] * to_rad) * (ypr[1] * to_rad);
            out.roll = (ypr[2] * to_rad) * (ypr[2] * to_rad);
            return out;
        }

        sensor_msgs::msg::Imu imu_message(const std::string &frame_id,
                                          const rclcpp::Time &stamp,
                                          const tf2::Transform &frame_from_imu,
                                          const tf2::Quaternion &ned_attitude,
                                          const tf2::Vector3 &sensor_ang_vel,
                                          const tf2::Vector3 &sensor_lin_accel,
                                          const AttitudeVar &att_var) const
        {
            using namespace detail;

            const tf2::Vector3 ang_vel = to_frame(frame_from_imu, sensor_ang_vel);
            const tf2::Vector3 lin_accel = transfer_acceleration(
                frame_from_imu, to_frame(frame_from_imu, sensor_lin_accel), ang_vel);

            sensor_msgs::msg::Imu imu;
            imu.header.frame_id = frame_id;
            imu.header.stamp = stamp;
            set(imu.orientation, orientation_of(frame_from_imu, ned_attitude));
            set(imu.angular_velocity, ang_vel);
            set(imu.linear_acceleration, lin_accel);

            // These three used to be left at zero on every message. Zero is the
            // *defined* "unknown" sentinel for sensor_msgs/Imu -- unlike the
            // pose covariance on Odometry, where zero claims perfect knowledge
            // -- so it was never a lie, just no information at all. Filling
            // them costs nothing: the attitude sigma is already being streamed
            // for the odometry, and the rate and acceleration variances are
            // measured (see the parameters that carry them).
            //
            // Rotated frames are not applied to the diagonals. The mounting
            // rotation would mix the axes and introduce off-diagonal terms; at
            // the 10 degree mainhoop pitch that is a few percent, well inside
            // how well any of these numbers are known.
            if (att_var.known)
            {
                imu.orientation_covariance[0] = att_var.roll;
                imu.orientation_covariance[4] = att_var.pitch;
                imu.orientation_covariance[8] = att_var.yaw;
            }
            for (int i = 0; i < 3; i++)
            {
                imu.angular_velocity_covariance[i * 4] = ang_vel_var_[i];
                imu.linear_acceleration_covariance[i * 4] = lin_accel_var_[i];
            }
            return imu;
        }

        nav_msgs::msg::Odometry odom_message(const Frames &frames,
                                             const rclcpp::Time &stamp,
                                             const VN::Sensor::CompositeDataQueueReturn &msg,
                                             const tf2::Transform &cog_from_imu,
                                             const tf2::Quaternion &ned_attitude,
                                             const tf2::Vector3 &sensor_ang_vel)
        {
            using namespace detail;

            const auto lla = msg->ins.posLla.value();
            const auto vel = msg->ins.velBody.value();

            // The odom frame is pinned to wherever the car was when the INS
            // first had a solution worth trusting. Latched here rather than by
            // the caller so there is exactly one place it can happen.
            if (!geodesic_ref_.has_value())
            {
                geodesic_ref_ = GeodesicRef{lla.lat, lla.lon, lla.alt};
            }

            double north, east, down;
            GNSSConversion::latLonAltToNED(lla.lat, lla.lon, lla.alt,
                                           geodesic_ref_->latitude,
                                           geodesic_ref_->longitude,
                                           geodesic_ref_->altitude,
                                           north, east, down);

            const tf2::Quaternion cog_orientation = orientation_of(cog_from_imu, ned_attitude);
            const tf2::Vector3 imu_position = tf2::quatRotate(flip_x(), tf2::Vector3(north, east, down));

            // The IMU is not the car. Shift the reported position along the arm
            // from the IMU to the cog, rotated into the world by the car's own
            // attitude -- the arm is a body-frame quantity and adding it to a
            // world-frame position unrotated is a heading-dependent error the
            // size of the arm, which here is about 0.4 m horizontally.
            const tf2::Vector3 cog_position =
                imu_position - tf2::quatRotate(cog_orientation, cog_from_imu.getOrigin());

            // Velocity and rate stay in the body frame, which is what
            // nav_msgs/Odometry asks for in twist and what the EKF consumes as
            // u, v and omega.
            const tf2::Vector3 cog_ang_vel = to_frame(cog_from_imu, sensor_ang_vel);
            const tf2::Vector3 cog_lin_vel = transfer_velocity(
                cog_from_imu, to_frame(cog_from_imu, tf2::Vector3(vel[0], vel[1], vel[2])),
                cog_ang_vel);

            nav_msgs::msg::Odometry odom;
            odom.header.frame_id = frames.odom;
            odom.header.stamp = stamp;
            odom.child_frame_id = frames.cog;

            odom.pose.pose.position.x = cog_position.x();
            odom.pose.pose.position.y = cog_position.y();
            odom.pose.pose.position.z = cog_position.z();
            set(odom.pose.pose.orientation, cog_orientation);
            set(odom.twist.twist.linear, cog_lin_vel);
            set(odom.twist.twist.angular, cog_ang_vel);

            // Covariance diagonals from the onboard filter's 1-sigma
            // uncertainties, with conservative fallbacks when not streamed.
            // mirena_state reads [0], [7] and [35] of each and floors them, so
            // the ones it does not read still have to be honest rather than
            // zero -- a zero variance claims perfect knowledge.
            const double pos_var = variance_of(msg->ins.posU, 1.0);
            const double vel_var = variance_of(msg->ins.velU, 0.5);

            // 0.05 rad^2 is about 13 degrees: a deliberately poor fallback for
            // a filter that declined to say, not an estimate.
            const AttitudeVar att = attitude_variances(msg);
            const double roll_var = att.known ? att.roll : 0.05;
            const double pitch_var = att.known ? att.pitch : 0.05;
            const double yaw_var = att.known ? att.yaw : 0.05;

            for (int i = 0; i < 3; i++)
            {
                odom.pose.covariance[i * 7] = pos_var;
                odom.twist.covariance[i * 7] = vel_var;
                odom.twist.covariance[(i + 3) * 7] = ang_vel_var_[i];
            }
            odom.pose.covariance[21] = roll_var;
            odom.pose.covariance[28] = pitch_var;
            odom.pose.covariance[35] = yaw_var;

            return odom;
        }

        /// The GNSS receiver's own solution, not the INS's.
        ///
        /// This used to republish ins.posLla, which is a different quantity
        /// wearing the same units: the INS position is reported at the IMU
        /// after the antenna lever arm has been taken out, it does not exist
        /// until the filter converges, and it carries one isotropic sigma. The
        /// GNSS group gives the position at antenna A -- which is what the
        /// car/gnss frame on this message claims -- from the moment the
        /// receiver has a fix, with a real per-axis uncertainty and a fix type
        /// that says how much to believe it.
        std::optional<sensor_msgs::msg::NavSatFix> fix_message(
            const std::string &frame_id,
            const rclcpp::Time &stamp,
            const VN::Sensor::CompositeDataQueueReturn &msg)
        {
            if (!msg->gnss.gnss1PosLla.has_value())
            {
                return std::nullopt;
            }

            // One message per *solution*, not per measurement. The receiver
            // solves at 1-5 Hz and this stream runs at 100, so without this the
            // same fix goes out up to 20 times and anything that averages the
            // topic, or feeds it to a filter as independent measurements, is
            // reading evidence that does not exist. GPS time of week changes
            // exactly when the receiver has genuinely solved again.
            if (msg->gnss.gps1Tow.has_value())
            {
                const int64_t tow_ns = static_cast<int64_t>(msg->gnss.gps1Tow.value().nanoseconds());
                if (last_gnss_tow_ns_.has_value() && tow_ns == last_gnss_tow_ns_.value())
                {
                    return std::nullopt;
                }
                last_gnss_tow_ns_ = tow_ns;
            }
            const auto lla = msg->gnss.gnss1PosLla.value();

            sensor_msgs::msg::NavSatFix fix;
            fix.header.frame_id = frame_id;
            fix.header.stamp = stamp;
            fix.status.status = fix_status(msg);
            // The VN-300 is a multi-constellation receiver, but which
            // constellations are enabled lives in a register this driver does
            // not read, so GPS is the only one it can claim without guessing.
            fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
            fix.latitude = lla.lat;
            fix.longitude = lla.lon;
            fix.altitude = lla.alt;

            // NavSatFix wants ENU; the receiver reports NED. Variance is
            // sign-agnostic so down and up share a value, but north and east
            // genuinely do have to swap -- they are different numbers, and on a
            // car in a paddock they differ by more than the rounding.
            if (msg->gnss.gnss1PosUncertainty.has_value())
            {
                const auto ned = msg->gnss.gnss1PosUncertainty.value();
                fix.position_covariance[0] = static_cast<double>(ned[1]) * ned[1]; // east
                fix.position_covariance[4] = static_cast<double>(ned[0]) * ned[0]; // north
                fix.position_covariance[8] = static_cast<double>(ned[2]) * ned[2]; // up
                fix.position_covariance_type =
                    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            }
            return fix;
        }

        /// VectorNav fix type (register 58's Gnss1Fix) to NavSatStatus.
        ///
        /// A time-only fix carries no usable position, so it is reported as no
        /// fix rather than flattered into one.
        static int8_t fix_status(const VN::Sensor::CompositeDataQueueReturn &msg)
        {
            using Status = sensor_msgs::msg::NavSatStatus;
            if (!msg->gnss.gnss1Fix.has_value())
            {
                return Status::STATUS_NO_FIX;
            }
            switch (msg->gnss.gnss1Fix.value())
            {
            case 2: // 2D
            case 3: // 3D
                return Status::STATUS_FIX;
            case 4: // SBAS corrections
                return Status::STATUS_SBAS_FIX;
            case 7: // RTK float
            case 8: // RTK fixed
                return Status::STATUS_GBAS_FIX;
            default: // 0 no fix, 1 time only
                return Status::STATUS_NO_FIX;
            }
        }

        static double variance_of(const std::optional<float> &sigma, double fallback)
        {
            if (!sigma.has_value() || !(sigma.value() > 0.0f))
            {
                return fallback;
            }
            return static_cast<double>(sigma.value()) * static_cast<double>(sigma.value());
        }
    };
} // namespace mirena::IMU
