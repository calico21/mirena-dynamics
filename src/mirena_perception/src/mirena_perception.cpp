/**
 * mirena_perception -- colourless, LiDAR-only cone detection.
 *
 * The car does not read cone colour. It needs neither the camera nor a GPU, and
 * the whole pipeline behind it -- SLAM, both planners -- is built to work from
 * geometry alone, so nothing downstream is waiting for a colour that never
 * comes.
 *
 * What does survive is *size*, because that is geometry too: a cluster tall
 * enough to be a big orange cone is reported as `cone_big_orange` and everything
 * else as `unknown_cone`. That single distinction is load bearing -- the four
 * big cones on the start line are what the unknown-track planner uses to find
 * the start of the track.
 *
 * The detector is three ideas: the ground under a candidate is a plane fitted to
 * a ring around it rather than through it, a cone is isolated, and the ground a
 * cone stands on is smooth. They live in cone_detector.hpp, with no ROS in them,
 * so the heuristics can be read and tuned without reading this file. This file is
 * the ROS shell: subscriptions, TF, the EntityList, the debug clouds, the
 * heartbeat.
 */

#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>

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

#include "cone_detector.hpp"

// Topic names are the pipeline's contract, unchanged by the move to LiDAR.
#define CLOUD_SUB_TOPIC "/sensors/lidar/points"
#define ENTITY_LIST_PUB_TOPIC "/perception/cones"
#define DEBUG_PC_PUB_TOPIC "/perception/debug/clustered_points"
#define DEBUG_STANDING_PUB_TOPIC "/perception/debug/standing_points"
#define HEALTH_PUB_TOPIC "health"

#define CAR_FRAME "car/cog"

// The supervisor holds the mission in CONFIGURING until every pipeline node is
// beating, so this rate is what it waits on, not a diagnostic nicety.
#define HEARTBEAT_PERIOD 100ms
#define REPORT_STALE_AFTER_S 1.0

// Colour is never measured. Size is: a cluster tall enough to be a big orange
// cone gets the type that says so, and everything else is a cone whose colour is
// unknown by design rather than by failure.
#define CONE_TYPE "unknown_cone"
#define BIG_CONE_TYPE "cone_big_orange"

using namespace std::chrono_literals;

namespace
{
    /// Byte offset of a float32 field, or nothing if the cloud has no such field.
    std::optional<uint32_t> float_field_offset(const sensor_msgs::msg::PointCloud2 &msg,
                                               const std::string &name)
    {
        for (const auto &field : msg.fields)
        {
            if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32)
            {
                return field.offset;
            }
        }
        return std::nullopt;
    }

    float read_float(const uint8_t *point, uint32_t offset)
    {
        float value;
        std::memcpy(&value, point + offset, sizeof(value));
        return value;
    }

    /// Copies a cloud into the row-per-point matrix Patchwork++ takes, dropping
    /// non-finite points. The fourth column is intensity and appears only if the
    /// cloud carries one, which is what decides whether reflected-noise removal
    /// can run at all.
    Eigen::MatrixXf cloud_to_matrix(const sensor_msgs::msg::PointCloud2 &msg)
    {
        const auto x_off = float_field_offset(msg, "x");
        const auto y_off = float_field_offset(msg, "y");
        const auto z_off = float_field_offset(msg, "z");
        if (!x_off || !y_off || !z_off)
        {
            return {};
        }
        const auto i_off = float_field_offset(msg, "intensity");

        const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
        Eigen::MatrixXf cloud(static_cast<Eigen::Index>(num_points), i_off ? 4 : 3);

        Eigen::Index kept = 0;
        for (size_t i = 0; i < num_points; ++i)
        {
            const uint8_t *point = msg.data.data() + i * msg.point_step;
            const float x = read_float(point, *x_off);
            const float y = read_float(point, *y_off);
            const float z = read_float(point, *z_off);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            cloud(kept, 0) = x;
            cloud(kept, 1) = y;
            cloud(kept, 2) = z;
            if (i_off)
            {
                cloud(kept, 3) = read_float(point, *i_off);
            }
            ++kept;
        }

        cloud.conservativeResize(kept, Eigen::NoChange);
        return cloud;
    }

    /// Stable, bright colour per cluster, so a cone keeps its colour from frame to
    /// frame while its neighbours stay distinguishable. Rejected points go grey.
    void colour_by_label(int label, pcl::PointXYZRGB &point)
    {
        if (label < 0)
        {
            point.r = point.g = point.b = 0x50;
            return;
        }
        const uint32_t hash = static_cast<uint32_t>(label) * 2654435761u; // Knuth
        point.r = static_cast<uint8_t>(96 + (hash & 0x7Fu));
        point.g = static_cast<uint8_t>(96 + ((hash >> 8) & 0x7Fu));
        point.b = static_cast<uint8_t>(96 + ((hash >> 16) & 0x7Fu));
    }

    /// Builds an XYZRGB cloud of the labelled points, coloured per cone. With
    /// `cones_only` it carries just the points that became cones; without it,
    /// everything standing above the ground, which is what shows whether the
    /// ground estimate or the cone tests need attention.
    sensor_msgs::msg::PointCloud2 build_debug_cloud(const mirena::ConeDetector::PointCloud &points,
                                                   const std::vector<int> &labels,
                                                   bool cones_only,
                                                   const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> coloured;
        coloured.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (cones_only && labels[i] < 0)
            {
                continue;
            }
            pcl::PointXYZRGB point;
            point.getVector3fMap() = points[i].getVector3fMap();
            colour_by_label(labels[i], point);
            coloured.push_back(point);
        }

        coloured.width = coloured.size();
        coloured.height = 1;
        coloured.is_dense = true;

        sensor_msgs::msg::PointCloud2 cloud;
        pcl::toROSMsg(coloured, cloud);
        // Set after the conversion: toROSMsg writes the (empty) header the PCL
        // cloud carries, which would otherwise clear the stamp and frame.
        cloud.header = header;
        return cloud;
    }
} // namespace

class MirenaPerception : public rclcpp::Node
{
    // The cloud callback runs for tens of milliseconds. It gets its own group so
    // the heartbeat is not stuck behind it -- the supervisor is watching that
    // heartbeat to decide whether this node is alive.
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

    // Written by the cloud callback, read by the heartbeat.
    std::mutex _report_mutex;
    uint64_t _frames_processed = 0;
    uint64_t _dropped_no_transform = 0;
    int _cones_last_frame = 0;
    int _big_cones_last_frame = 0;
    int _standing_last_frame = 0;
    int _candidates = 0;
    mirena::ConeDetector::StageTimes _stage;
    double _total_ms = 0.0;
    rclcpp::Time _last_frame_at;
    bool _has_frame = false;

public:
    MirenaPerception() : rclcpp::Node("mirena_perception"), _last_frame_at(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "Starting Mirena Perception (LiDAR)");

        _detector = std::make_unique<mirena::ConeDetector>(declare_detector_params());
        _publish_debug = this->declare_parameter("enable_debug", true);

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
            std::bind(&MirenaPerception::cloud_callback, this, std::placeholders::_1),
            cloud_options);

        // Last, deliberately: the first heartbeat is this node telling the
        // supervisor it is up, so it must not go out before it actually is.
        _report_pub = this->create_publisher<mirena_common::msg::Heartbeat>(HEALTH_PUB_TOPIC, 10);
        _report_timer = this->create_wall_timer(
            HEARTBEAT_PERIOD, std::bind(&MirenaPerception::report_callback, this), _report_cb_group);
    }

private:
    /// Sensor data is better dropped than queued: a stale cloud describes a place
    /// the car has already left.
    static rclcpp::QoS sensor_qos()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(20));
        qos.best_effort().durability_volatile();
        return qos;
    }

    /// Declares every tuning knob, using the defaults in ConeDetectorParams as
    /// the ROS defaults so the two cannot drift apart.
    mirena::ConeDetectorParams declare_detector_params()
    {
        mirena::ConeDetectorParams p;
        p.coarse_cell = this->declare_parameter("coarse_cell", p.coarse_cell);
        p.coarse_ring = this->declare_parameter("coarse_ring", p.coarse_ring);
        p.candidate_height = this->declare_parameter("candidate_height", p.candidate_height);
        p.peak_cell = this->declare_parameter("peak_cell", p.peak_cell);
        p.peak_height = this->declare_parameter("peak_height", p.peak_height);
        p.ego_half_length = this->declare_parameter("ego_half_length", p.ego_half_length);
        p.ego_half_width = this->declare_parameter("ego_half_width", p.ego_half_width);
        p.ring_inner = this->declare_parameter("ring_inner", p.ring_inner);
        p.ring_outer = this->declare_parameter("ring_outer", p.ring_outer);
        p.ring_cell = this->declare_parameter("ring_cell", p.ring_cell);
        p.min_top = this->declare_parameter("min_top", p.min_top);
        p.max_top = this->declare_parameter("max_top", p.max_top);
        p.core_radius = this->declare_parameter("core_radius", p.core_radius);
        p.stand_height = this->declare_parameter("stand_height", p.stand_height);
        p.max_footprint = this->declare_parameter("max_footprint", p.max_footprint);
        p.min_points = this->declare_parameter("min_points", p.min_points);
        p.max_roughness = this->declare_parameter("max_roughness", p.max_roughness);
        p.max_ring_ratio = this->declare_parameter("max_ring_ratio", p.max_ring_ratio);
        p.max_range = this->declare_parameter("max_range", p.max_range);
        p.big_cone_min_height = this->declare_parameter("big_cone_min_height", p.big_cone_min_height);
        p.range_sigma = this->declare_parameter("range_sigma", p.range_sigma);
        p.bearing_sigma = this->declare_parameter("bearing_sigma", p.bearing_sigma);
        p.cluster_spread = this->declare_parameter("cluster_spread", p.cluster_spread);
        return p;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        const Eigen::MatrixXf cloud = cloud_to_matrix(*msg);
        if (cloud.rows() == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Cloud on %s has no usable float32 x/y/z points", CLOUD_SUB_TOPIC);
            return;
        }

        // At the cloud's own stamp, not the latest: the detections are about the
        // moment the sweep was taken, and SLAM pairs them with the pose from then.
        const auto car_from_lidar = lookup_car_from_lidar(msg->header);
        if (!car_from_lidar)
        {
            std::lock_guard<std::mutex> lock(_report_mutex);
            ++_dropped_no_transform;
            return;
        }

        const auto &detections = _detector->detect(cloud, *car_from_lidar);

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
            // A cluster either passes the geometric filter or is dropped, so there
            // is no graded score to report.
            entity.confidence = 1.0;
            // How well the cone is located, which is mostly a question of how far
            // away it is. SLAM weights the measurement by this.
            entity.covariance = detection.covariance;
            cones.entities.push_back(entity);
        }
        _cone_pub->publish(cones);

        if (_publish_debug)
        {
            publish_debug(msg->header.stamp);
        }

        {
            std::lock_guard<std::mutex> lock(_report_mutex);
            ++_frames_processed;
            _cones_last_frame = static_cast<int>(detections.size());
            _big_cones_last_frame = 0;
            for (const auto &detection : detections)
            {
                _big_cones_last_frame += detection.big ? 1 : 0;
            }
            _standing_last_frame = static_cast<int>(_detector->standing().size());
            _candidates = _detector->candidates();
            _stage = _detector->stage_times();
            _total_ms = _detector->total_ms();
            _last_frame_at = this->now();
            _has_frame = true;
        }
    }

    std::optional<Eigen::Isometry3f> lookup_car_from_lidar(const std_msgs::msg::Header &header)
    {
        try
        {
            const auto tf = _tf_buffer->lookupTransform(CAR_FRAME, header.frame_id, tf2_ros::fromMsg(header.stamp));
            return tf2::transformToEigen(tf).cast<float>();
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No transform %s <- %s: %s", CAR_FRAME, header.frame_id.c_str(), e.what());
            return std::nullopt;
        }
    }

    void publish_debug(const builtin_interfaces::msg::Time &stamp)
    {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        // Everything downstream of ground removal lives in the car frame.
        header.frame_id = CAR_FRAME;

        const auto &points = _detector->standing();
        const auto &labels = _detector->labels();

        if (_debug_cluster_pub->get_subscription_count() > 0)
        {
            _debug_cluster_pub->publish(build_debug_cloud(points, labels, /*cones_only=*/true, header));
        }
        if (_debug_standing_pub->get_subscription_count() > 0)
        {
            _debug_standing_pub->publish(build_debug_cloud(points, labels, /*cones_only=*/false, header));
        }
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

            // Shared with mirena_perception, so a dashboard reading one reads both.
            report["frames_processed"] = _frames_processed;
            report["cones_last_frame"] = _cones_last_frame;
            // The same number under the name the supervisor forwards to the car
            // on DV_info, where "actual" is what the car can see right now. Kept
            // as its own key rather than renaming the one above, which is what a
            // dashboard already plots.
            report["cones_count_actual"] = _cones_last_frame;
            // The start line, as far as this node is concerned. Four is the
            // number to expect once it is in view.
            report["big_cones_last_frame"] = _big_cones_last_frame;
            report["seconds_since_frame"] = _has_frame ? since_frame : -1.0;

            // How many points stand above the coarse ground at all, and how many
            // of those became candidates worth a proper look. Candidates against
            // cones kept is the ratio that says whether the tests are still
            // rejecting anything.
            report["standing_last_frame"] = _standing_last_frame;
            report["candidates_last_frame"] = _candidates;
            report["total_ms"] = _total_ms;

            // Which stage dominates moves with the scene, so the split is on the
            // wire rather than something to rebuild the node to find out.
            report["level_ms"] = _stage.level_ms;
            report["coarse_ms"] = _stage.coarse_ms;
            report["peaks_ms"] = _stage.peaks_ms;
            report["accept_ms"] = _stage.accept_ms;
            report["dropped_no_transform"] = _dropped_no_transform;
        }

        msg.report = report.dump();
        _report_pub->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MirenaPerception>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
