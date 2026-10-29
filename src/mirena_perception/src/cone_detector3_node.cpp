#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nlohmann/json.hpp>

#include "mirena_common/msg/entity.hpp"
#include "mirena_common/msg/entity_list.hpp"
#include "mirena_common/msg/heartbeat.hpp"

#include "cone_detector3.hpp"

#define CLOUD_SUB_TOPIC "/sensors/lidar/points"
#define ENTITY_LIST_PUB_TOPIC "/perception/cones_3"
#define DEBUG_PC_PUB_TOPIC "/perception/debug/clustered_points"
#define DEBUG_STANDING_PUB_TOPIC "/perception/debug/standing_points"
#define HEALTH_PUB_TOPIC "health"

#define CAR_FRAME "car/cog"

#define HEARTBEAT_PERIOD 100ms
#define REPORT_STALE_AFTER_S 1.0

#define CONE_TYPE "unknown_cone"
#define BIG_CONE_TYPE "cone_big_orange"

using namespace std::chrono_literals;

namespace
{
    std::optional<uint32_t> float_field_offset(const sensor_msgs::msg::PointCloud2 &msg,
                                               const std::string &name)
    {
        for (const auto &field : msg.fields)
        {
            if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32)
                return field.offset;
        }
        return std::nullopt;
    }

    float read_float(const uint8_t *point, uint32_t offset)
    {
        float value;
        std::memcpy(&value, point + offset, sizeof(value));
        return value;
    }

    Eigen::MatrixXf cloud_to_matrix(const sensor_msgs::msg::PointCloud2 &msg)
    {
        const auto x_off = float_field_offset(msg, "x");
        const auto y_off = float_field_offset(msg, "y");
        const auto z_off = float_field_offset(msg, "z");
        if (!x_off || !y_off || !z_off) return {};

        const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
        Eigen::MatrixXf cloud(static_cast<Eigen::Index>(num_points), 3);

        Eigen::Index kept = 0;
        for (size_t i = 0; i < num_points; ++i)
        {
            const uint8_t *point = msg.data.data() + i * msg.point_step;
            const float x = read_float(point, *x_off);
            const float y = read_float(point, *y_off);
            const float z = read_float(point, *z_off);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            cloud(kept, 0) = x;
            cloud(kept, 1) = y;
            cloud(kept, 2) = z;
            ++kept;
        }

        cloud.conservativeResize(kept, Eigen::NoChange);
        return cloud;
    }

    void colour_by_label(int label, pcl::PointXYZRGB &point)
    {
        if (label <= 0)
        {
            point.r = point.g = point.b = 0x50;
            return;
        }
        if (label == 2) // big cone
        {
            point.r = 0xFF; point.g = 0x80; point.b = 0x00;
            return;
        }
        // standard cone
        point.r = 0xFF; point.g = 0xFF; point.b = 0x00;
    }

    sensor_msgs::msg::PointCloud2 build_debug_cloud(const std::vector<Eigen::Vector3f> &points,
                                                     const std::vector<int> &labels,
                                                     bool cones_only,
                                                     const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> coloured;
        coloured.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (cones_only && labels[i] <= 0) continue;
            pcl::PointXYZRGB point;
            point.getVector3fMap() = points[i];
            colour_by_label(labels[i], point);
            coloured.push_back(point);
        }

        coloured.width = coloured.size();
        coloured.height = 1;
        coloured.is_dense = true;

        sensor_msgs::msg::PointCloud2 cloud;
        pcl::toROSMsg(coloured, cloud);
        cloud.header = header;
        return cloud;
    }
} // namespace

class ConeDetector3Node : public rclcpp::Node
{
    rclcpp::CallbackGroup::SharedPtr _cloud_cb_group;
    rclcpp::CallbackGroup::SharedPtr _report_cb_group;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;

    rclcpp::Publisher<mirena_common::msg::EntityList>::SharedPtr _cone_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _debug_cluster_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _debug_standing_pub;
    rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr _report_pub;
    rclcpp::TimerBase::SharedPtr _report_timer;

    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    std::unique_ptr<mirena::ConeDetector> _detector;
    bool _publish_debug = true;
    bool debug_log = true;

    std::mutex _report_mutex;
    uint64_t _frames_processed = 0;
    uint64_t _dropped_no_transform = 0;
    int _cones_last_frame = 0;
    int _big_cones_last_frame = 0;
    int _standing_last_frame = 0;
    int _candidates = 0;
    float _max_h_above = 0.0f;
    float _max_z_car = 0.0f;
    mirena::StageTimes _stage;
    double _total_ms = 0.0;
    rclcpp::Time _last_frame_at;
    bool _has_frame = false;

public:
    ConeDetector3Node() : rclcpp::Node("cone_detector3"), _last_frame_at(this->now())
    {
        debug_log = this->declare_parameter("debug_log", true);
        _publish_debug = this->declare_parameter("enable_debug", true);

        if (debug_log)
            RCLCPP_INFO(this->get_logger(), "ConeDetector3 node starting (debug_log=%s)", debug_log ? "true" : "false");

        _detector = std::make_unique<mirena::ConeDetector>(declare_detector_params());
        _detector->_debug = debug_log;

        _cloud_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _report_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer, this);

        _cone_pub = this->create_publisher<mirena_common::msg::EntityList>(ENTITY_LIST_PUB_TOPIC, 10);

        _debug_cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(DEBUG_PC_PUB_TOPIC, 1);
        _debug_standing_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(DEBUG_STANDING_PUB_TOPIC, 1);

        auto cloud_options = rclcpp::SubscriptionOptions();
        cloud_options.callback_group = _cloud_cb_group;
        _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            CLOUD_SUB_TOPIC, sensor_qos(),
            std::bind(&ConeDetector3Node::cloud_callback, this, std::placeholders::_1),
            cloud_options);

        _report_pub = this->create_publisher<mirena_common::msg::Heartbeat>(HEALTH_PUB_TOPIC, 10);
        _report_timer = this->create_wall_timer(
            HEARTBEAT_PERIOD, std::bind(&ConeDetector3Node::report_callback, this), _report_cb_group);
    }

private:
    static rclcpp::QoS sensor_qos()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort().durability_volatile();
        return qos;
    }

    mirena::ConeDetectorParams declare_detector_params()
    {
        mirena::ConeDetectorParams p;
        p.max_range = this->declare_parameter("max_range", p.max_range);
        p.ego_half_length = this->declare_parameter("ego_half_length", p.ego_half_length);
        p.ego_half_width = this->declare_parameter("ego_half_width", p.ego_half_width);
        p.slope_factor = this->declare_parameter("slope_factor", p.slope_factor);
        p.candidate_height = this->declare_parameter("candidate_height", p.candidate_height);
        p.z_min = this->declare_parameter("z_min", p.z_min);
        p.z_max = this->declare_parameter("z_max", p.z_max);

        p.peak_cell = this->declare_parameter("peak_cell", p.peak_cell);
        p.peak_height = this->declare_parameter("peak_height", p.peak_height);
        p.peak_min_neighbors = this->declare_parameter("peak_min_neighbors", p.peak_min_neighbors);

        p.roi_radius = this->declare_parameter("roi_radius", p.roi_radius);
        p.ring_inner = this->declare_parameter("ring_inner", p.ring_inner);
        p.ring_outer = this->declare_parameter("ring_outer", p.ring_outer);
        p.core_radius = this->declare_parameter("core_radius", p.core_radius);
        p.stand_height = this->declare_parameter("stand_height", p.stand_height);
        p.min_points = this->declare_parameter("min_points", p.min_points);
        p.min_top = this->declare_parameter("min_top", p.min_top);
        p.max_top = this->declare_parameter("max_top", p.max_top);
        p.min_slenderness = this->declare_parameter("min_slenderness", p.min_slenderness);
        p.max_footprint = this->declare_parameter("max_footprint", p.max_footprint);
        p.max_pca_angle_deg = this->declare_parameter("max_pca_angle_deg", p.max_pca_angle_deg);

        p.big_cone_min_height = this->declare_parameter("big_cone_min_height", p.big_cone_min_height);

        p.range_sigma = this->declare_parameter("range_sigma", p.range_sigma);
        p.bearing_sigma_deg = this->declare_parameter("bearing_sigma_deg", p.bearing_sigma_deg);

        p.nms_radius = this->declare_parameter("nms_radius", p.nms_radius);
        return p;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        const auto t_start = std::chrono::steady_clock::now();

        const auto t_conv = std::chrono::steady_clock::now();
        const Eigen::MatrixXf cloud = cloud_to_matrix(*msg);
        const double conv_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_conv).count();

        if (cloud.rows() == 0)
        {
            if (debug_log)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No float32 x/y/z data in %s", CLOUD_SUB_TOPIC);
            return;
        }

        const auto t_tf = std::chrono::steady_clock::now();
        const auto car_from_lidar = lookup_car_from_lidar(msg->header);
        const double tf_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_tf).count();

        if (!car_from_lidar)
        {
            std::lock_guard<std::mutex> lock(_report_mutex);
            ++_dropped_no_transform;
            return;
        }

        const auto t_det = std::chrono::steady_clock::now();
        const auto &detections = _detector->detect(cloud, *car_from_lidar);
        const double detect_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_det).count();

        const auto t_pub = std::chrono::steady_clock::now();
        mirena_common::msg::EntityList cones;
        cones.header.frame_id = CAR_FRAME;
        cones.header.stamp = msg->header.stamp;
        cones.entities.reserve(detections.size());
        for (const auto &detection : detections)
        {
            mirena_common::msg::Entity entity;
            entity.position.x = detection.position.x();
            entity.position.y = detection.position.y();
            entity.position.z = detection.position.z();
            entity.type = detection.big ? BIG_CONE_TYPE : CONE_TYPE;
            entity.confidence = detection.confidence;
            entity.covariance = detection.covariance;
            cones.entities.push_back(entity);
        }
        _cone_pub->publish(cones);
        const double pub_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_pub).count();

        double debug_pub_ms = 0.0;
        if (_publish_debug)
        {
            const auto t_dbg = std::chrono::steady_clock::now();
            publish_debug(msg->header.stamp);
            debug_pub_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_dbg).count();
        }

        const double total_cb_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_start).count();

        if (debug_log)
            RCLCPP_INFO(this->get_logger(),
                        "[TIMING] Total: %.2f | Conv: %.2f | TF: %.2f | Detect: %.2f | Pub: %.2f | Debug: %.2f | Crop: %.2f | Hash: %.2f | ROI: %.2f | Cls: %.2f",
                        total_cb_ms, conv_ms, tf_ms, detect_ms, pub_ms, debug_pub_ms,
                        _detector->stage_times().crop_ms,
                        _detector->stage_times().hash_ms,
                        _detector->stage_times().roi_ms,
                        _detector->stage_times().classify_ms);

        {
            std::lock_guard<std::mutex> lock(_report_mutex);
            ++_frames_processed;
            _cones_last_frame = static_cast<int>(detections.size());
            _big_cones_last_frame = 0;
            for (const auto &detection : detections)
                _big_cones_last_frame += detection.big ? 1 : 0;
            _standing_last_frame = static_cast<int>(_detector->standing().size());
            _candidates = _detector->candidates();
            _stage = _detector->stage_times();
            _total_ms = _detector->total_ms();
            // Copied rather than read live: the detector writes these floats
            // from the cloud callback, which does not hold this mutex.
            _max_h_above = _detector->_stats.max_h_above;
            _max_z_car = _detector->_stats.max_z_car;
            _last_frame_at = this->now();
            _has_frame = true;
        }
    }

    std::optional<Eigen::Isometry3f> lookup_car_from_lidar(const std_msgs::msg::Header &header)
    {
        try
        {
            const auto tf = _tf_buffer->lookupTransform(CAR_FRAME, header.frame_id,
                                                        tf2_ros::fromMsg(header.stamp));
            return tf2::transformToEigen(tf).cast<float>();
        }
        catch (const tf2::TransformException &e)
        {
            if (debug_log)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No TF %s <- %s: %s", CAR_FRAME,
                                     header.frame_id.c_str(), e.what());
            return std::nullopt;
        }
    }

    void publish_debug(const builtin_interfaces::msg::Time &stamp)
    {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = CAR_FRAME;

        const auto &points = _detector->standing();
        const auto &labels = _detector->labels();

        if (_debug_cluster_pub->get_subscription_count() > 0)
            _debug_cluster_pub->publish(build_debug_cloud(points, labels, true, header));
        if (_debug_standing_pub->get_subscription_count() > 0)
            _debug_standing_pub->publish(build_debug_cloud(points, labels, false, header));
    }

    void report_callback()
    {
        mirena_common::msg::Heartbeat msg;
        msg.timestamp = this->now();
        msg.node_name = this->get_name();

        nlohmann::json report;
        {
            std::lock_guard<std::mutex> lock(_report_mutex);

            const double since_frame = (this->now() - _last_frame_at).seconds();
            const bool stale = _has_frame && since_frame > REPORT_STALE_AFTER_S;
            report["status_state"] = !_has_frame ? "IDLE" : (stale ? "PARTIAL" : "WORKING");

            report["frames_processed"] = _frames_processed;
            report["cones_last_frame"] = _cones_last_frame;
            report["big_cones_last_frame"] = _big_cones_last_frame;
            report["seconds_since_frame"] = _has_frame ? since_frame : -1.0;

            report["standing_last_frame"] = _standing_last_frame;
            report["candidates_last_frame"] = _candidates;
            report["total_ms"] = _total_ms;

            // --- rejection breakdown ---
            // The atomic counters are read directly, without the detector's own
            // debug flag, so a subscriber can watch the pipeline even when the
            // per-frame stderr report is off.
            const auto &s = _detector->_stats;
            report["elevated_last_frame"] = s.elevated.load();
            report["max_h_above"] = _max_h_above;
            report["max_z_car"] = _max_z_car;

            report["crop_rej_range"] = s.rejected_range.load();
            report["crop_rej_ego"] = s.rejected_ego.load();
            report["crop_rej_z"] = s.rejected_z.load();
            report["crop_rej_ground"] = s.rejected_ground.load();

            report["grid_points_outside"] = s.points_outside_grid.load();
            report["grid_cells_populated"] = s.cells_populated.load();
            report["grid_rej_below_peak_h"] = s.cells_below_peak_height.load();
            report["grid_rej_not_local_max"] = s.cells_not_local_max.load();
            report["grid_rej_too_few_pts"] = s.cells_too_few_points.load();

            report["roi_evaluated"] = s.roi_evaluated.load();
            report["roi_rej_ring"] = s.roi_rejected_ring.load();
            report["roi_no_ring"] = s.roi_no_ring.load();
            report["roi_plane_degenerate"] = s.roi_plane_degenerate.load();
            report["roi_rej_core"] = s.roi_rejected_core.load();
            report["roi_rej_height"] = s.roi_rejected_height.load();
            report["roi_rej_footprint"] = s.roi_rejected_footprint.load();
            report["roi_rej_slenderness"] = s.roi_rejected_slenderness.load();
            report["roi_rej_pca"] = s.roi_rejected_pca.load();
            report["roi_accepted"] = s.roi_accepted.load();
            report["nms_suppressed"] = s.roi_accepted.load() - static_cast<uint64_t>(_cones_last_frame);

            report["crop_ms"] = _stage.crop_ms;
            report["hash_ms"] = _stage.hash_ms;
            report["roi_ms"] = _stage.roi_ms;
            report["classify_ms"] = _stage.classify_ms;
            report["dropped_no_transform"] = _dropped_no_transform;
        }

        msg.report = report.dump();
        _report_pub->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConeDetector3Node>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
