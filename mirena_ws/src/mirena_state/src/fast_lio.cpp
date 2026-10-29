 

// #include <rclcpp/rclcpp.hpp>
// #include <chrono>
// #include <mutex>
// #include <shared_mutex>
// #include <vector>
// #include <deque>
// #include <unordered_map>
// #include <cmath>
// #include <algorithm>
// #include <memory>
// #include <stdexcept>
// #include <omp.h>

// // Transforms
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>

// // JSON library for heartbeat monitoring
// #include <nlohmann/json.hpp>
// #include <Eigen/Dense>

// // Interfaces
// #include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// #include "mirena_common/msg/car.hpp"
// #include "mirena_common/msg/heartbeat.hpp"

// #include "ekf.hpp"

// #pragma omp declare reduction(+: Eigen::Matrix3d: omp_out += omp_in) initializer(omp_priv = Eigen::Matrix3d::Zero())
// #pragma omp declare reduction(+: Eigen::Vector3d: omp_out += omp_in) initializer(omp_priv = Eigen::Vector3d::Zero())

// #define IMU_TOPIC "/sensors/imu"
// #define LIDAR_TOPIC "/sensors/lidar/points"
// #define CAR_STATE_TOPIC "/state/car"
// #define MAP_TOPIC "/state/map"

// using namespace std::chrono_literals;

// namespace
// {
//     constexpr const char *kOdomFrame = "odom";
//     constexpr const char *kBaseFrame = "car/cog";
//     constexpr auto kPublishPeriod = 10ms; // 100 Hz state & TF
//     constexpr auto kReportPeriod = 100ms;  // 10 Hz heartbeat

//     constexpr double kInitPoseVar = 0.0;
//     constexpr double kZuptVelVar = 0.0025;   // (0.05 m/s)^2
//     constexpr double kUnknownVelVar = 25.0;  // (5 m/s)^2
//     constexpr double kStationaryAccel = 0.5; // m/s^2
//     constexpr double kStationaryRate = 0.05; // rad/s
// }

// // Voxels structure for spatial point indexing
// struct VoxelKey {
//     int64_t x, y, z;
//     bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
// };

// struct VoxelKeyHash {
//     std::size_t operator()(const VoxelKey& k) const {
//         return ((std::hash<int64_t>()(k.x) ^ (std::hash<int64_t>()(k.y) << 1)) >> 1) ^ (std::hash<int64_t>()(k.z) << 1);
//     }
// };

// class IVoxMap {
// public:
//     explicit IVoxMap(double voxel_size = 0.4) : voxel_size_(voxel_size) {}

//     VoxelKey pointToKey(const Eigen::Vector3d& pt) const {
//         return {
//             static_cast<int64_t>(std::floor(pt.x() / voxel_size_)),
//             static_cast<int64_t>(std::floor(pt.y() / voxel_size_)),
//             static_cast<int64_t>(std::floor(pt.z() / voxel_size_))
//         };
//     }

//     void insertPoint(const Eigen::Vector3d& pt) {
//         VoxelKey key = pointToKey(pt);
//         voxels_[key].push_back(pt);
//         if (voxels_[key].size() > max_points_per_voxel_) {
//             voxels_[key].pop_front();
//         }
//     }

//     bool getFiveNN(const Eigen::Vector3d& pt, std::vector<Eigen::Vector3d>& neighbors) const {
//         neighbors.clear();
//         VoxelKey center_key = pointToKey(pt);
//         std::vector<std::pair<double, Eigen::Vector3d>> candidates;
//         candidates.reserve(27 * max_points_per_voxel_);

//         for (int dx = -1; dx <= 1; ++dx) {
//             for (int dy = -1; dy <= 1; ++dy) {
//                 for (int dz = -1; dz <= 1; ++dz) {
//                     VoxelKey key{center_key.x + dx, center_key.y + dy, center_key.z + dz};
//                     auto it = voxels_.find(key);
//                     if (it != voxels_.end()) {
//                         for (const auto& p : it->second) {
//                             candidates.push_back({(p - pt).squaredNorm(), p});
//                         }
//                     }
//                 }
//             }
//         }

//         if (candidates.size() < 5) return false;

//         std::partial_sort(candidates.begin(), candidates.begin() + 5, candidates.end(),
//             [](const auto& a, const auto& b) { return a.first < b.first; });

//         for (size_t i = 0; i < 5; ++i) {
//             neighbors.push_back(candidates[i].second);
//         }
//         return true;
//     }

//     std::vector<Eigen::Vector3d> getAllPoints() const {
//         std::vector<Eigen::Vector3d> points;
//         for (const auto& [key, voxel_points] : voxels_) {
//             points.insert(points.end(), voxel_points.begin(), voxel_points.end());
//         }
//         return points;
//     }

// private:
//     double voxel_size_;
//     size_t max_points_per_voxel_ = 10;
//     std::unordered_map<VoxelKey, std::deque<Eigen::Vector3d>, VoxelKeyHash> voxels_;
// };

// /**
//  * FastLIOState — High-rate state estimator fusing IMU predictions with 
//  * LiDAR point-cloud relative motion for body-frame velocity updates.
//  */
// class FastLIOState : public rclcpp::Node
// {
//     // Thread safety & callback groups
//     rclcpp::CallbackGroup::SharedPtr _filter_cb_group;
//     rclcpp::CallbackGroup::SharedPtr _report_cb_group;

//     // Subscriptions
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;

//     // Publishers
//     rclcpp::Publisher<mirena_common::msg::Car>::SharedPtr _car_pub;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _map_pub;
//     rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr _report_pub;

//     // Broadcaster
//     std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

//     // Timers
//     rclcpp::TimerBase::SharedPtr _publish_timer;
//     rclcpp::TimerBase::SharedPtr _report_timer;

//     // Core Filter & Synchronizations
//     std::mutex _filter_mutex;
//     std::shared_mutex _map_mutex;
//     std::unique_ptr<mirena::Ekf> _ekf;
//     IVoxMap _ivox_map{0.4};

//     rclcpp::Time _last_imu_stamp{0, 0, RCL_ROS_TIME};
//     rclcpp::Time _last_lidar_stamp{0, 0, RCL_ROS_TIME};
//     double _last_published_stamp = 0.0;

//     // Anchor frame transformation (LIO -> ODOM)
//     bool _ins_anchored = false;
//     double _anchor_tx = 0.0, _anchor_ty = 0.0, _anchor_dpsi = 0.0;

//     // Timeout Parameters
//     double _lidar_timeout_sec = 0.5;
//     double _imu_timeout_sec = 0.2;
//     double _gyro_meas_var = 1.0e-5;

// public:
//     FastLIOState() : rclcpp::Node("fast_lio_state")
//     {
//         this->declare_parameter<double>("lidar_timeout_sec", 0.5);
//         this->declare_parameter<double>("imu_timeout_sec", 0.2);
//         this->declare_parameter<double>("max_predict_dt_sec", 0.1);
//         this->declare_parameter<std::vector<double>>(
//             "process_noise_diag", {0.01, 0.01, 0.005, 0.5, 0.5, 0.5});
//         this->declare_parameter<std::vector<double>>(
//             "odom_meas_floor_diag", {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9});
//         this->declare_parameter<std::vector<double>>(
//             "init_cov_floor_diag", {0.01, 0.01, 0.001, 0.01, 0.01, 0.001});
//         this->declare_parameter<double>("gyro_meas_var", 1.0e-5);

//         RCLCPP_INFO(this->get_logger(), "Starting Robust FastLIO State Estimator");

//         _lidar_timeout_sec = this->get_parameter("lidar_timeout_sec").as_double();
//         _imu_timeout_sec = this->get_parameter("imu_timeout_sec").as_double();
//         _gyro_meas_var = this->get_parameter("gyro_meas_var").as_double();

//         _filter_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//         _report_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

//         auto filter_sub_opts = rclcpp::SubscriptionOptions();
//         filter_sub_opts.callback_group = _filter_cb_group;

//         // Publishers & TF
//         _car_pub = this->create_publisher<mirena_common::msg::Car>(CAR_STATE_TOPIC, 10);
//         _map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(MAP_TOPIC, 10);
//         _report_pub = this->create_publisher<mirena_common::msg::Heartbeat>("health", 10);
//         _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//         mirena::Ekf::Config cfg;
//         cfg.process_noise_diag = param_vec6("process_noise_diag");
//         cfg.odom_meas_floor_diag = param_vec6("odom_meas_floor_diag");
//         cfg.init_cov_floor_diag = param_vec6("init_cov_floor_diag");
//         cfg.gyro_meas_var = _gyro_meas_var;
//         cfg.max_dt = this->get_parameter("max_predict_dt_sec").as_double();
//         _ekf = std::make_unique<mirena::Ekf>(cfg);

//         // Hardware Subscriptions using SensorDataQoS (Best Effort)
//         _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
//             IMU_TOPIC, rclcpp::SensorDataQoS(),
//             std::bind(&FastLIOState::imu_callback, this, std::placeholders::_1), filter_sub_opts);

//         _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             LIDAR_TOPIC, rclcpp::SensorDataQoS(),
//             std::bind(&FastLIOState::pointcloud_callback, this, std::placeholders::_1), filter_sub_opts);

//         // Node-clock aligned Timers
//         _publish_timer = rclcpp::create_timer(
//             this, this->get_clock(), kPublishPeriod,
//             std::bind(&FastLIOState::publish_timer_callback, this), _filter_cb_group);
//         _report_timer = rclcpp::create_timer(
//             this, this->get_clock(), kReportPeriod,
//             std::bind(&FastLIOState::report_callback, this), _report_cb_group);
//     }

// private:
//     mirena::Ekf::Vec6 param_vec6(const std::string &name)
//     {
//         auto v = this->get_parameter(name).as_double_array();
//         if (v.size() != 6) {
//             RCLCPP_FATAL(this->get_logger(), "Parameter %s must have 6 elements, got %zu.", name.c_str(), v.size());
//             throw std::runtime_error("invalid " + name);
//         }
//         return mirena::Ekf::Vec6(v.data());
//     }

//     void log_step(mirena::Ekf::Step step, const char *what)
//     {
//         using Step = mirena::Ekf::Step;
//         if (step == Step::Diverged) {
//             RCLCPP_ERROR(this->get_logger(), "EKF diverged on %s — filter reset.", what);
//         } else if (step == Step::Ignored) {
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                                  "EKF ignored %s: non-finite values or stamp anomaly.", what);
//         }
//     }

//     void init_at_origin(const sensor_msgs::msg::Imu &msg)
//     {
//         const double accel = std::hypot(msg.linear_acceleration.x, msg.linear_acceleration.y);
//         const double rate = std::abs(msg.angular_velocity.z);
//         const bool stationary = accel < kStationaryAccel && rate < kStationaryRate;
//         const double vel_var = stationary ? kZuptVelVar : kUnknownVelVar;

//         mirena::Ekf::Vec6 r;
//         r << kInitPoseVar, kInitPoseVar, kInitPoseVar, vel_var, vel_var, _gyro_meas_var;

//         const double t = rclcpp::Time(msg.header.stamp).seconds();
//         if (_ekf->init(mirena::Ekf::Vec6::Zero(), r, t) != mirena::Ekf::Step::Ok) {
//             log_step(mirena::Ekf::Step::Ignored, "origin initialization");
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "odom origin defined at initial pose; dead reckoning started.");
//     }

//     void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
//     {
//         std::lock_guard<std::mutex> lock(_filter_mutex);
//         _last_imu_stamp = msg->header.stamp;

//         if (!_ekf->is_initialized()) {
//             if (!_ins_anchored) init_at_origin(*msg);
//             return;
//         }

//         const double t = rclcpp::Time(msg->header.stamp).seconds();
//         log_step(_ekf->predict(t, msg->linear_acceleration.x, msg->linear_acceleration.y), "IMU predict");
//         if (!_ekf->is_initialized()) return;
//         log_step(_ekf->update_gyro(msg->angular_velocity.z), "gyro update");
//     }

//     void latch_anchor(double lio_x, double lio_y, double lio_psi)
//     {
//         const auto &x = _ekf->state();
//         _anchor_dpsi = mirena::Ekf::wrap_angle(x(2) - lio_psi);
//         const double c = std::cos(_anchor_dpsi), s = std::sin(_anchor_dpsi);
//         _anchor_tx = x(0) - (c * lio_x - s * lio_y);
//         _anchor_ty = x(1) - (s * lio_x + c * lio_y);
//         _ins_anchored = true;

//         RCLCPP_INFO(this->get_logger(), "LIO frame anchored into odom. Offset: (%.2f, %.2f) m", _anchor_tx, _anchor_ty);
//     }

//     void ins_to_odom(double ix, double iy, double ipsi, double &ox, double &oy, double &opsi) const
//     {
//         const double c = std::cos(_anchor_dpsi), s = std::sin(_anchor_dpsi);
//         ox = c * ix - s * iy + _anchor_tx;
//         oy = s * ix + c * iy + _anchor_ty;
//         opsi = mirena::Ekf::wrap_angle(ipsi + _anchor_dpsi);
//     }

//     void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
//     {
//         if (!msg || msg->data.empty()) return;

//         double current_t = rclcpp::Time(msg->header.stamp).seconds();
//         Eigen::Vector3d current_pos;
//         double current_yaw = 0.0;
//         double dt = 0.0;

//         {
//             std::lock_guard<std::mutex> lock(_filter_mutex);
//             _last_lidar_stamp = msg->header.stamp;

//             if (!_ekf->is_initialized()) return;

//             const auto& state = _ekf->state();
//             current_pos = Eigen::Vector3d(state(0), state(1), 0.0);
//             current_yaw = state(2);

//             dt = current_t - _ekf->stamp();
//         }

//         // Downsample Cloud
//         sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
//         sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
//         sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

//         std::vector<Eigen::Vector3d> local_pts;
//         local_pts.reserve((msg->width * msg->height) / 5);
//         int counter = 0;
//         for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
//             if (++counter % 5 != 0) continue;
//             if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
//                 local_pts.emplace_back(*iter_x, *iter_y, *iter_z);
//             }
//         }

//         if (local_pts.empty()) return;

//         Eigen::Matrix3d R_curr = Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
//         std::vector<Eigen::Vector3d> world_pts(local_pts.size());
        
//         // Point-to-Plane Gauss-Newton Step to solve spatial displacement delta
//         Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
//         Eigen::Vector3d b = Eigen::Vector3d::Zero();
//         int valid_residuals = 0;

//         {
//             std::shared_lock<std::shared_mutex> read_lock(_map_mutex);

//             #pragma omp parallel for schedule(dynamic, 64) reduction(+:H,b,valid_residuals)
//             for (size_t i = 0; i < local_pts.size(); ++i) {
//                 Eigen::Vector3d p_w = R_curr * local_pts[i] + current_pos;
//                 world_pts[i] = p_w;

//                 std::vector<Eigen::Vector3d> neighbors;
//                 if (_ivox_map.getFiveNN(p_w, neighbors)) {
//                     Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//                     for (const auto& n : neighbors) mean += n;
//                     mean /= 5.0;

//                     Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
//                     for (const auto& n : neighbors) {
//                         Eigen::Vector3d diff = n - mean;
//                         cov += diff * diff.transpose();
//                     }

//                     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov, Eigen::ComputeEigenvectors);
//                     Eigen::Vector3d normal = es.eigenvectors().col(0);

//                     double dist = normal.dot(p_w - mean);
//                     if (std::abs(dist) < 0.1) {
//                         // Jacobian for [dx, dy, dpsi]
//                         Eigen::Vector3d J;
//                         J.head<2>() = normal.head<2>();
//                         J(2) = normal.x() * (-std::sin(current_yaw) * local_pts[i].x() - std::cos(current_yaw) * local_pts[i].y()) +
//                                normal.y() * ( std::cos(current_yaw) * local_pts[i].x() - std::sin(current_yaw) * local_pts[i].y());

//                         H += J * J.transpose();
//                         b += -J * dist;
//                         valid_residuals++;
//                     }
//                 }
//             }
//         }

//         // Solve SE(2) Correction Vector: [dx, dy, dpsi]
//         if (valid_residuals > 15 && std::abs(H.determinant()) > 1e-5) {
//             Eigen::Vector3d delta = H.ldlt().solve(b);

//             if (dt < 0.001) dt = 0.01; // Avoid divide-by-zero

//             // Extract Body-Frame Velocities
//             double u_meas = (delta.x() * std::cos(current_yaw) + delta.y() * std::sin(current_yaw)) / dt;
//             double v_meas = (-delta.x() * std::sin(current_yaw) + delta.y() * std::cos(current_yaw)) / dt;
//             double omega_meas = delta.z() / dt;

//             // Absolute LIO Position & Yaw estimate
//             double lio_x = current_pos.x() + delta.x();
//             double lio_y = current_pos.y() + delta.y();
//             double lio_psi = mirena::Ekf::wrap_angle(current_yaw + delta.z());

//             std::lock_guard<std::mutex> lock(_filter_mutex);
//             if (!_ins_anchored) {
//                 latch_anchor(lio_x, lio_y, lio_psi);
//             }

//             double ox, oy, opsi;
//             ins_to_odom(lio_x, lio_y, lio_psi, ox, oy, opsi);

//             // Dynamically condition variance based on Hessian eigenvalues
//             double position_var = std::clamp(1.0 / (H(0,0) + H(1,1) + 1e-6), 0.001, 0.5);
//             double yaw_var = std::clamp(1.0 / (H(2,2) + 1e-6), 0.0001, 0.1);
//             double vel_var = std::clamp(position_var / (dt * dt), 0.01, 2.0);

//             mirena::Ekf::Vec6 z, r;
//             z << ox, oy, opsi, u_meas, v_meas, omega_meas;
//             r << position_var, position_var, yaw_var, vel_var, vel_var, yaw_var / (dt * dt);

//             log_step(_ekf->update_odom(z, r), "LIO velocity & pose update");
//         }

//         // Write points safely into Voxel Map
//         {
//             std::unique_lock<std::shared_mutex> write_lock(_map_mutex);
//             for (const auto& pt : world_pts) {
//                 _ivox_map.insertPoint(pt);
//             }
//         }
//     }

//     void publish_timer_callback()
//     {
//         std::lock_guard<std::mutex> lock(_filter_mutex);
//         if (!_ekf->is_initialized()) return;

//         // Guard against duplicate TF timestamps
//         if (_ekf->stamp() <= _last_published_stamp) return;
//         _last_published_stamp = _ekf->stamp();

//         const auto &x = _ekf->state();
//         const auto &P = _ekf->covariance();
//         const rclcpp::Time stamp(static_cast<int64_t>(_ekf->stamp() * 1e9), RCL_ROS_TIME);

//         mirena_common::msg::Car car;
//         car.header.stamp = stamp;
//         car.header.frame_id = kOdomFrame;
//         car.child_frame_id = kBaseFrame;
//         car.x = x(0);
//         car.y = x(1);
//         car.psi = x(2);
//         car.u = x(3);
//         car.v = x(4);
//         car.omega = x(5);
//         for (int i = 0; i < 6; i++)
//             for (int j = 0; j < 6; j++)
//                 car.covariance[i * 6 + j] = P(i, j);

//         _car_pub->publish(car);
//         send_tf(x(0), x(1), x(2), stamp);
//     }

//     void send_tf(double x, double y, double psi, const rclcpp::Time &stamp)
//     {
//         geometry_msgs::msg::TransformStamped t;
//         t.header.stamp = stamp;
//         t.header.frame_id = kOdomFrame;
//         t.child_frame_id = kBaseFrame;
//         t.transform.translation.x = x;
//         t.transform.translation.y = y;
//         t.transform.translation.z = 0.0;

//         tf2::Quaternion q;
//         q.setRPY(0, 0, psi);
//         t.transform.rotation.x = q.x();
//         t.transform.rotation.y = q.y();
//         t.transform.rotation.z = q.z();
//         t.transform.rotation.w = q.w();

//         _tf_broadcaster->sendTransform(t);
//     }

//     void report_callback()
//     {
//         mirena_common::msg::Heartbeat msg;
//         msg.timestamp = this->now();
//         msg.node_name = this->get_name();

//         nlohmann::json report_json;
//         {
//             std::lock_guard<std::mutex> lock(_filter_mutex);
//             const double now_s = this->now().seconds();

//             const bool have_lidar = _last_lidar_stamp.nanoseconds() > 0;
//             const bool have_imu = _last_imu_stamp.nanoseconds() > 0;
//             const double lidar_age = now_s - _last_lidar_stamp.seconds();
//             const double imu_age = now_s - _last_imu_stamp.seconds();

//             const bool lidar_stale = !have_lidar || lidar_age > _lidar_timeout_sec;
//             const bool imu_stale = !have_imu || imu_age > _imu_timeout_sec;

//             std::string mode;
//             if (!_ekf->is_initialized()) mode = "INIT";
//             else if (!_ins_anchored) mode = "NO_LIO_YET";
//             else if (lidar_stale && imu_stale) mode = "STALE";
//             else if (lidar_stale) mode = "DEAD_RECKONING";
//             else if (imu_stale) mode = "NO_IMU";
//             else mode = "FUSED";

//             report_json["mode"] = mode;
//             report_json["lio_anchored"] = _ins_anchored;
//             report_json["lidar_stale"] = lidar_stale;
//             report_json["imu_stale"] = imu_stale;
//             if (have_lidar) report_json["lidar_age_s"] = lidar_age;
//             if (have_imu) report_json["imu_age_s"] = imu_age;
//             report_json["ekf_resets"] = _ekf->divergences();

//             if (_ekf->is_initialized()) {
//                 const auto &x = _ekf->state();
//                 report_json["car_pos_x"] = x(0);
//                 report_json["car_pos_y"] = x(1);
//                 report_json["car_pos_yaw"] = x(2);
//                 report_json["car_vel_u"] = x(3);
//                 report_json["car_vel_v"] = x(4);
//             }
//         }

//         msg.report = report_json.dump();
//         _report_pub->publish(msg);
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<FastLIOState>();

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }
