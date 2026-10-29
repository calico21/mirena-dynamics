#include "lidar_packet.hpp"
#include "pointcloud_builder.hpp"
#include "udp_listener.hpp"
#include "lidar_socket_pair.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <poll.h>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#define POINTCLOUD_PUB_TOPIC "/sensors/lidar/points"

using namespace std::chrono_literals;
using namespace mirena;

/**
 * MirenaLidar — RS-M1 MSOP bridge.
 *
 * One thread: poll the socket, convert the packet, publish when a sweep closes.
 *
 * There used to be a second stage -- a deque of raw packets drained by a 1 kHz
 * wall timer -- and it existed only because de-skew could ask the builder for
 * "the current yaw rate" and nothing else. Packets therefore had to be held
 * until an IMU sample newer than the packet had arrived, so the queue was also
 * a delay line, and its release condition compared an IMU header stamp against
 * a host-clock arrival stamp. While those two clocks disagreed by 2 s, every
 * packet waited 2 s and roughly 12000 of them sat resident. Worse, the queue
 * had no bound and nothing released it if the IMU never spoke at all: with the
 * IMU node down, this driver quietly consumed ~7 MB/s and published nothing.
 *
 * ImuHistory made the wait unnecessary -- the yaw angle can now be asked for at
 * the instant a packet was measured -- so the queue, the timer, the mutex
 * around them and the release gate are all gone with it. The kernel's 25 MB
 * socket buffer is the only buffer, which is bounded by construction and drops
 * the oldest data rather than the machine.
 */
class MirenaLidar : public rclcpp::Node
{
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    mirena::PointcloudBuilder pc_builder_;

    in_addr_t _self_ip;
    in_addr_t _lidar_ip;
    uint16_t _msop_port;
    uint16_t _difop_port;
    mirena::LidarSocketPair sockets;

    std::thread capture_thread_;
    std::atomic<bool> keep_running_{false};
    /// Guards pc_builder_ between the IMU callback (executor thread) and the
    /// capture thread.
    std::mutex pc_builder_mtx_;

    std::atomic<bool> use_imu_{true};
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

public:
    MirenaLidar() : rclcpp::Node("mirena_lidar"),
                    pc_builder_("car/lidar")
    {
        this->declare_parameter("self_ip", "192.168.1.102");
        this->declare_parameter("lidar_ip", "192.168.1.200");
        this->declare_parameter("msop_port", 6699);
        this->declare_parameter("difop_port", 7788);
        this->declare_parameter("use_imu", true);

        auto self_ip_str = this->get_parameter("self_ip").as_string();
        auto lidar_ip_str = this->get_parameter("lidar_ip").as_string();
        auto msop_port_val = this->get_parameter("msop_port").as_int();
        auto difop_port_val = this->get_parameter("difop_port").as_int();
        this->_self_ip = inet_addr(self_ip_str.c_str());
        this->_lidar_ip = inet_addr(lidar_ip_str.c_str());
        this->_msop_port = htons(static_cast<uint16_t>(msop_port_val));
        this->_difop_port = htons(static_cast<uint16_t>(difop_port_val));

        use_imu_ = this->get_parameter("use_imu").as_bool();
        pc_builder_.set_deskew_enabled(use_imu_);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MirenaLidar::on_parameter_change, this, std::placeholders::_1));

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            POINTCLOUD_PUB_TOPIC, rclcpp::QoS(rclcpp::SensorDataQoS()));

        // Motion de-skew consumes the lidar-frame, lever-arm-compensated IMU
        // stream (not the cog stream on /sensors/imu)
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/lidar/imu", rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&MirenaLidar::imu_callback, this, std::placeholders::_1));

        keep_running_ = true;
        capture_thread_ = std::thread(&MirenaLidar::capture_loop, this);

        RCLCPP_INFO(get_logger(), "Mirena Lidar Bridge Core Initialized and Streaming.");
    }

    ~MirenaLidar()
    {
        keep_running_ = false;
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }

        sockets.invalidate();
        RCLCPP_INFO(get_logger(), "Lidar Thread Joined and Sockets Closed cleanly.");
    }

private:
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "use_imu")
            {
                use_imu_ = param.as_bool();
                {
                    std::lock_guard<std::mutex> lock(pc_builder_mtx_);
                    pc_builder_.set_deskew_enabled(use_imu_);
                }
                RCLCPP_INFO(get_logger(), "Parameter 'use_imu' updated to: %s", use_imu_ ? "true" : "false");
            }
            else
            {
                // The addressing parameters are read once, when the sockets are
                // opened. Accepting them at runtime reported success for a
                // change that never took effect.
                result.successful = false;
                result.reason = param.get_name() + " can only be set at startup";
            }
        }
        return result;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Recorded even when de-skew is off, so re-enabling use_imu at runtime
        // does not start from an empty history.
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();
        std::lock_guard<std::mutex> lock(pc_builder_mtx_);
        pc_builder_.add_imu(stamp, msg->angular_velocity.z);
    }

    void capture_loop()
    {
        while (keep_running_ && rclcpp::ok())
        {
            if (!sockets.is_valid())
            {
                RCLCPP_INFO(this->get_logger(), "Trying to Adquire sockets...");
                reset_capture();
                std::this_thread::sleep_for(1s);
                sockets = mirena::LidarSocketPair(this->_self_ip, this->_lidar_ip, this->_msop_port, this->_difop_port);
                if (sockets.is_valid())
                {
                    RCLCPP_INFO(this->get_logger(), "Adquired sockets successfully; Starting capture");
                }
                continue;
            }
            capture();
        }
    }

    void capture()
    {
        struct pollfd pfd;
        pfd.fd = sockets.get_msop_fd();
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 10);
        if (ret <= 0 || !(pfd.revents & POLLIN))
        {
            return;
        }

        // Left uninitialised on purpose: recvfrom overwrites all of it, and
        // value-initialising would memset 1.2 kB per packet, ~7 MB/s at rate.
        StampedMSOPPacket packet;
        packet.timestamp = this->get_clock()->now();
        ssize_t size = sockets.get_msop_socket()->_recvfrom(&packet.packet, sizeof(packet.packet), 0);
        if (size != sizeof(packet.packet))
        {
            return;
        }

        std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud;
        bool compensated = true;
        {
            std::lock_guard<std::mutex> lock(pc_builder_mtx_);
            cloud = pc_builder_.process_packet(packet);
            if (cloud) compensated = pc_builder_.last_frame_compensated();
        }

        if (!cloud)
        {
            return;
        }

        if (!compensated && use_imu_.load())
        {
            // Publishing anyway is the right call -- a smeared cloud still
            // finds cones, and going silent because the IMU is late would take
            // perception down with it. Saying so is what the old code did not.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Publishing un-de-skewed clouds: no IMU covering the sweep on "
                                 "/sensors/lidar/imu. Cones will smear in proportion to yaw rate.");
        }

        // Moved, not copied: publish(*msg) put a 1.26 MB memcpy of every frame
        // between here and the middleware, which is what the unique_ptr return
        // from process_packet was for in the first place.
        pointcloud_pub_->publish(std::move(cloud));
    }

    void reset_capture()
    {
        sockets.invalidate();
        std::lock_guard<std::mutex> lock(pc_builder_mtx_);
        pc_builder_.flush();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MirenaLidar>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
