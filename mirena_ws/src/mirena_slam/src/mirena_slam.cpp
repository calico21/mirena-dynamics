#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex> // NUEVO: Para seguridad en hilos (Thread-safety)

// Lie-group interpolation, for easing the published correction
#include <gtsam/base/Lie.h>

// Message filters for time-synchronization
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Debug visualisation
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Transforms
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// JSON library for string packaging
#include <nlohmann/json.hpp>

// Custom Interfaces
#include "mirena_common/msg/car.hpp"
#include "mirena_common/msg/entity_list.hpp"
#include "mirena_common/msg/heartbeat.hpp"
#include "slam.hpp"

// Topics and Frames
#define CAR_STATE_TOPIC "/state/car"
#define LANDMARK_TOPIC "/perception/cones"
#define MAP_TOPIC "slam/map"
#define DEBUG_TOPIC "slam/debug/associations"

#define FIXED_FRAME "map"
#define MOVING_FRAME "odom"

// Everything that used to be a #define here is a ROS parameter now, declared in
// declare_parameters() with these as the defaults. The values and the reasoning
// behind each one live on the declarations; what follows are only the defaults,
// kept here so the node still reads as one file.
namespace defaults
{
    constexpr double kMapPublishPeriod = 0.1;   // s
    // map->odom has to keep up with the 100 Hz odom->car/cog it is composed
    // with, or every lookup at a fresh pose stamp falls off the end of the
    // buffer.
    constexpr double kTfPublishPeriod = 0.01;   // s
    constexpr double kReportPeriod = 0.1;       // s
    constexpr double kMaxSyncDeltaMs = 10.0;    // pose/detection pairs further apart are dropped
    constexpr double kReportStaleAfterS = 1.0;
    constexpr double kKeyframeMaxIntervalS = 0.5;
    constexpr double kCorrectionTimeConstantS = 0.5;

    // Odometry motion model: sigma per metre travelled and per radian turned,
    // and the floor under each. What governs normal driving -- see
    // odometry_sigmas().
    constexpr double kOdomModelXyPerM = 0.05;
    constexpr double kOdomModelXyPerRad = 0.10;
    constexpr double kOdomModelThPerM = 0.02;
    constexpr double kOdomModelThPerRad = 0.05;
    constexpr double kOdomSigmaFloorXy = 0.02;  // m
    constexpr double kOdomSigmaFloorTh = 0.005; // rad
} // namespace defaults

using namespace std::chrono_literals;
class MirenaMapping : public rclcpp::Node
{
    // Synchronize the Car state with the detected Landmarks
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<mirena_common::msg::Car, mirena_common::msg::EntityList>;

    // Callback Groups for thread safety
    rclcpp::CallbackGroup::SharedPtr _slam_cb_group;
    rclcpp::CallbackGroup::SharedPtr _report_cb_group;
    // The map->odom broadcast lives on its own group on purpose: an iSAM2
    // update takes hundreds of milliseconds once the graph grows, and sharing
    // a mutually exclusive group with it stalls the transform for that whole
    // time. Downstream that shows up as control failing its lookups with
    // "extrapolation into the future".
    rclcpp::CallbackGroup::SharedPtr _tf_cb_group;

    // Subscribers & Synchronizer
    std::shared_ptr<message_filters::Subscriber<mirena_common::msg::Car, rclcpp::Node>> _sync_car_sub;
    std::shared_ptr<message_filters::Subscriber<mirena_common::msg::EntityList, rclcpp::Node>> _sync_landmark_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;

    // Publishers
    rclcpp::Publisher<mirena_common::msg::EntityList>::SharedPtr _map_pub;
    rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr _report_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debug_pub;
    bool _publish_debug = true;
    /// Latches the one-time fatal log when the solver gives up. Touched only from
    /// the sync callback, so it needs no lock.
    bool _reported_unhealthy = false;

    // Transform broadcaster (map -> odom)
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    // Timers
    rclcpp::TimerBase::SharedPtr _map_publish_timer;
    rclcpp::TimerBase::SharedPtr _tf_publish_timer;
    rclcpp::TimerBase::SharedPtr _report_timer;

    // Report State & Mutex (NUEVO: Protege las variables que comparten ambos hilos)
    std::mutex _report_mutex;
    rclcpp::Time _last_update_timestamp;
    uint16_t _last_update_total_landmark_count = 0;
    double _last_update_ms_elapsed = 0.0; // read by the report timer before the first keyframe
    double _current_car_x = 0.0;
    double _current_car_y = 0.0;
    double _current_car_psi = 0.0;
    bool _has_keyframe = false;
    int _last_matched = 0;
    int _last_promoted = 0;
    int _last_hidden = 0;
    int _out_of_range = 0;
    int _candidates = 0;
    int _poses = 0;
    int _variables_reeliminated = 0;
    int _cliques = 0;
    int _kf_structure = 0;
    int _kf_drift = 0;
    int _kf_timer = 0;
    double _pinned_fraction = 0.0;
    bool _healthy = true;
    uint64_t _dropped_sync_pairs = 0;

    // Result of the last solve, handed to the publisher thread. Everything the
    // timers need is copied out here so they never touch _slam while the solver
    // is running -- which under an incremental solver is a wait measured in
    // hundreds of milliseconds the first time the track closes on itself.
    std::mutex _snapshot_mutex;
    gtsam::Pose2 _map_to_odom_snapshot;
    std::shared_ptr<const mirena_common::msg::EntityList> _map_snapshot;
    bool _has_snapshot = false;

    // What was last put on the wire, and when. Only the publisher touches these,
    // so they need no lock. The transform is filtered towards the snapshot
    // rather than snapped to it, which is why they have to persist between
    // ticks.
    gtsam::Pose2 _published_correction;
    rclcpp::Time _last_broadcast;
    bool _has_broadcast = false;

    // SLAM Engine.
    //
    // Guarded because the sync callback runs in _slam_cb_group under a
    // MultiThreadedExecutor while the map and report timers run in their own.
    std::mutex _slam_mutex;
    std::unique_ptr<mirena::Slam> _slam;
    /// Resolved in declare_parameters() and handed to the engine when it is
    /// built on the first synchronised frame.
    mirena::SlamParams _slam_params;
    gtsam::Pose2 _last_keyframe_raw_odom;
    /// Stamp of the last keyframe, from the /state/car header rather than the
    /// wall clock: make_odometry_noise integrates the estimator's velocity
    /// uncertainty over this interval.
    rclcpp::Time _last_keyframe_time;
    bool _is_initialized = false;

    // Node-level tuning, resolved once in declare_parameters().
    double _map_publish_period = defaults::kMapPublishPeriod;
    double _tf_publish_period = defaults::kTfPublishPeriod;
    double _report_period = defaults::kReportPeriod;
    double _max_sync_delta_ms = defaults::kMaxSyncDeltaMs;
    double _report_stale_after_s = defaults::kReportStaleAfterS;
    double _keyframe_max_interval_s = defaults::kKeyframeMaxIntervalS;
    double _correction_time_constant_s = defaults::kCorrectionTimeConstantS;
    double _odom_model_xy_per_m = defaults::kOdomModelXyPerM;
    double _odom_model_xy_per_rad = defaults::kOdomModelXyPerRad;
    double _odom_model_th_per_m = defaults::kOdomModelThPerM;
    double _odom_model_th_per_rad = defaults::kOdomModelThPerRad;
    double _odom_sigma_floor_xy = defaults::kOdomSigmaFloorXy;
    double _odom_sigma_floor_th = defaults::kOdomSigmaFloorTh;

public:
    MirenaMapping() : rclcpp::Node("mirena_mapping") {
        // Left at zero rather than now(): under use_sim_time the node is
        // constructed before /clock exists, so now() is 0 anyway, and an "age"
        // measured from it came out as the seconds since 1970 -- the report
        // carried seconds_since_update = 1.78e9 until the first keyframe. Both
        // are overwritten from message stamps before anything reads them.
        this->_last_update_timestamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        this->_last_keyframe_time = this->_last_update_timestamp;
        this->_last_update_total_landmark_count = 0;

        RCLCPP_INFO(this->get_logger(), "Starting Mirena Mapping");

        declare_parameters();

        _slam_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _report_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _tf_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto slam_sub_options = rclcpp::SubscriptionOptions();
        slam_sub_options.callback_group = _slam_cb_group;

        // Publishers & TF
        _map_pub = this->create_publisher<mirena_common::msg::EntityList>(MAP_TOPIC, 10);
        _report_pub = this->create_publisher<mirena_common::msg::Heartbeat>("health", 10);

        if (_publish_debug)
            _debug_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(DEBUG_TOPIC, 1);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Filtered Subscribers.
        //
        // Deep histories, and not as insurance. Both subscriptions share
        // _slam_cb_group with synchronized_callback, which is mutually
        // exclusive, so *nothing is taken off either topic* for as long as an
        // iSAM2 update runs -- measured at a p99 of 406 ms. At the 100 Hz
        // /state/car arrives at, the rmw default depth of 10 is 100 ms of
        // buffer, so every slow keyframe threw away the poses that the cone
        // frames arriving during it had to be paired with. The pairs were then
        // dropped for want of a pose inside max_sync_delta_ms, and the drop
        // showed up as PARTIAL rather than as anything naming the cause.
        //
        // 200 poses is two seconds, comfortably past the worst update; the
        // synchroniser's own queue of 100 is then what bounds the pose history
        // it will search, which is one second and far more than the 10 ms it
        // ever needs to reach back.
        rmw_qos_profile_t car_qos = rmw_qos_profile_default;
        car_qos.depth = 200;
        rmw_qos_profile_t cone_qos = rmw_qos_profile_default;
        cone_qos.depth = 40;

        _sync_car_sub = std::make_shared<message_filters::Subscriber<mirena_common::msg::Car, rclcpp::Node>>(
            this, CAR_STATE_TOPIC, car_qos, slam_sub_options);

        _sync_landmark_sub = std::make_shared<message_filters::Subscriber<mirena_common::msg::EntityList, rclcpp::Node>>(
            this, LANDMARK_TOPIC, cone_qos, slam_sub_options);

        _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100), *_sync_car_sub, *_sync_landmark_sub);
        // Refuse pairs the policy would otherwise match across an arbitrary gap.
        // At 15 m/s a 100 ms mismatch is 1.5 m of pose error baked into every
        // landmark of that keyframe.
        _sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(_max_sync_delta_ms / 1000.0));
        _sync->registerCallback(std::bind(&MirenaMapping::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Low-frequency Map and TF update timer
        // Node clock, not wall clock: under use_sim_time these are different
        // clocks at different rates, and the correction filter below steps on
        // elapsed node time, so a wall timer would ease at the wrong rate
        // whenever the simulator or a bag is not running at exactly 1x.
        _map_publish_timer = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration::from_seconds(_map_publish_period),
            std::bind(&MirenaMapping::map_publish_timer_callback, this), _tf_cb_group);

        _tf_publish_timer = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration::from_seconds(_tf_publish_period),
            std::bind(&MirenaMapping::tf_publish_timer_callback, this), _tf_cb_group);

        _report_timer = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration::from_seconds(_report_period),
            std::bind(&MirenaMapping::report_callback, this), _report_cb_group);

        _is_initialized = false;
    }

private:
    /// Every knob the estimator has, exposed.
    ///
    /// The SlamParams defaults are used as the ROS defaults rather than being
    /// written out again, so the two cannot drift: changing a default in
    /// slam.hpp changes it here, and a launch file that says nothing about a
    /// parameter still gets the value the header documents. Same arrangement as
    /// mirena_perception's declare_detector_params.
    ///
    /// The reasoning behind each value lives on the struct member in slam.hpp,
    /// which is where it belongs -- it is an argument about the estimator, not
    /// about ROS. Nothing here is re-read after startup and there is no
    /// parameter callback: a node's lifetime is the mission's, so a value that
    /// changed mid-run would have nowhere sensible to take effect.
    void declare_parameters()
    {
        const mirena::SlamParams p;   // the documented defaults

        _publish_debug = this->declare_parameter<bool>("publish_debug", true);

        // -- Association
        _slam_params.mahalanobis_threshold =
            this->declare_parameter<double>("mahalanobis_threshold", p.mahalanobis_threshold);
        _slam_params.max_association_distance =
            this->declare_parameter<double>("max_association_distance", p.max_association_distance);
        _slam_params.local_map_radius =
            this->declare_parameter<double>("local_map_radius", p.local_map_radius);
        _slam_params.view_half_angle =
            this->declare_parameter<double>("view_half_angle", p.view_half_angle);

        // -- Measurements
        _slam_params.default_measurement_sigma =
            this->declare_parameter<double>("default_measurement_sigma", p.default_measurement_sigma);
        _slam_params.min_measurement_sigma =
            this->declare_parameter<double>("min_measurement_sigma", p.min_measurement_sigma);

        // -- Existence
        _slam_params.p_detect = this->declare_parameter<double>("p_detect", p.p_detect);
        _slam_params.p_false = this->declare_parameter<double>("p_false", p.p_false);
        _slam_params.promote_log_odds =
            this->declare_parameter<double>("promote_log_odds", p.promote_log_odds);
        _slam_params.publish_log_odds =
            this->declare_parameter<double>("publish_log_odds", p.publish_log_odds);

        // -- Promotion
        _slam_params.separability_k =
            this->declare_parameter<double>("separability_k", p.separability_k);
        _slam_params.max_candidate_age = static_cast<int>(
            this->declare_parameter<int>("max_candidate_age", p.max_candidate_age));
        _slam_params.max_promotion_factors = static_cast<int>(
            this->declare_parameter<int>("max_promotion_factors", p.max_promotion_factors));

        // -- Keyframing
        _slam_params.keyframe_odom_sigma =
            this->declare_parameter<double>("keyframe_odom_sigma", p.keyframe_odom_sigma);
        _slam_params.keyframe_max_frames = static_cast<int>(
            this->declare_parameter<int>("keyframe_max_frames", p.keyframe_max_frames));

        // -- Node timing
        _map_publish_period =
            this->declare_parameter<double>("map_publish_period", defaults::kMapPublishPeriod);
        _tf_publish_period =
            this->declare_parameter<double>("tf_publish_period", defaults::kTfPublishPeriod);
        _report_period =
            this->declare_parameter<double>("heartbeat_period", defaults::kReportPeriod);
        _max_sync_delta_ms =
            this->declare_parameter<double>("max_sync_delta_ms", defaults::kMaxSyncDeltaMs);
        _report_stale_after_s =
            this->declare_parameter<double>("report_stale_after_s", defaults::kReportStaleAfterS);
        _keyframe_max_interval_s = this->declare_parameter<double>(
            "keyframe_max_interval_s", defaults::kKeyframeMaxIntervalS);
        _correction_time_constant_s = this->declare_parameter<double>(
            "correction_time_constant_s", defaults::kCorrectionTimeConstantS);

        // -- Odometry motion model
        _odom_model_xy_per_m =
            this->declare_parameter<double>("odom_model_xy_per_m", defaults::kOdomModelXyPerM);
        _odom_model_xy_per_rad =
            this->declare_parameter<double>("odom_model_xy_per_rad", defaults::kOdomModelXyPerRad);
        _odom_model_th_per_m =
            this->declare_parameter<double>("odom_model_th_per_m", defaults::kOdomModelThPerM);
        _odom_model_th_per_rad =
            this->declare_parameter<double>("odom_model_th_per_rad", defaults::kOdomModelThPerRad);
        _odom_sigma_floor_xy =
            this->declare_parameter<double>("odom_sigma_floor_xy", defaults::kOdomSigmaFloorXy);
        _odom_sigma_floor_th =
            this->declare_parameter<double>("odom_sigma_floor_th", defaults::kOdomSigmaFloorTh);

        // A correction time constant at or below zero divides by zero in the
        // easing filter; the rest are merely unwise rather than fatal.
        if (!(_correction_time_constant_s > 0.0))
        {
            RCLCPP_WARN(this->get_logger(),
                        "correction_time_constant_s must be positive, got %.3f; using %.3f.",
                        _correction_time_constant_s, defaults::kCorrectionTimeConstantS);
            _correction_time_constant_s = defaults::kCorrectionTimeConstantS;
        }
    }

    void synchronized_callback(const mirena_common::msg::Car::ConstSharedPtr &car_msg,
                               const mirena_common::msg::EntityList::ConstSharedPtr &landmark_msg)
    {

        rclcpp::Time car_stamp(car_msg->header.stamp);
        rclcpp::Time landmark_stamp(landmark_msg->header.stamp);
        
        double delta_ms = std::abs((car_stamp - landmark_stamp).seconds() * 1000.0);

        // Belt and braces on top of setMaxIntervalDuration: a pose and a cone
        // frame this far apart describe different moments, and pretending
        // otherwise writes the difference straight into the map.
        if (delta_ms > _max_sync_delta_ms)
        {
            {
                std::lock_guard<std::mutex> lock(_report_mutex);
                ++_dropped_sync_pairs;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Dropping sync pair, delta %.1f ms exceeds %.1f ms",
                                 delta_ms, _max_sync_delta_ms);
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Sync delta: %.2f ms", delta_ms);

        {
            std::lock_guard<std::mutex> lock(_report_mutex);
            _current_car_x = car_msg->x;
            _current_car_y = car_msg->y;
            _current_car_psi = car_msg->psi;
        }

        gtsam::Pose2 current_raw(car_msg->x, car_msg->y, car_msg->psi);

        std::unique_lock<std::mutex> slam_lock(_slam_mutex);

        if (!_is_initialized) {
            _slam = std::make_unique<mirena::Slam>(current_raw, _slam_params);
            _last_keyframe_raw_odom = current_raw;
            _last_keyframe_time = car_msg->header.stamp;
            _is_initialized = true;
            RCLCPP_INFO(this->get_logger(), "SLAM Engine Initialized at (%.2f, %.2f)", car_msg->x, car_msg->y);
            // Capture snapshot data under the slam lock, publish outside.
            {
                std::lock_guard<std::mutex> lock(_snapshot_mutex);
                _map_to_odom_snapshot = _slam->get_correction();
                auto map_msg = std::make_shared<mirena_common::msg::EntityList>();
                const std::vector<mirena::Landmark> landmarks = _slam->map_snapshot();
                map_msg->entities.reserve(landmarks.size());
                for (const auto &l : landmarks) {
                    mirena_common::msg::Entity e;
                    e.position.x = l.x;
                    e.position.y = l.y;
                    e.type = l.type;
                    e.confidence = l.confidence;
                    map_msg->entities.push_back(e);
                }
                map_msg->header.frame_id = FIXED_FRAME;
                map_msg->header.stamp = car_msg->header.stamp;
                _map_snapshot = std::move(map_msg);
                _has_snapshot = true;
            }
            return;
        }

        // 2. Every frame, match the cones against the map. This is where cones
        //    are confirmed and where ones that were never there are dropped, and
        //    it runs at the rate perception delivers rather than the rate the car
        //    moves far enough to be worth a new pose -- which on a slow lap is
        //    ten times as much evidence.
        std::vector<mirena::Observation> obs;
        obs.reserve(landmark_msg->entities.size());
        for (const auto &e : landmark_msg->entities) {
            obs.push_back({e.position.x, e.position.y, e.type,
                           {e.covariance[0], e.covariance[1],
                            e.covariance[2], e.covariance[3]}});
        }

        // Nobody watching, nothing gathered. The estimator fills three vectors a
        // frame for this and the node then serialises a marker per nearby
        // landmark, all of it on the node with the tightest budget in the stack;
        // mirena_perception and mirena_planning have both asked this question
        // before doing that work for as long as they have had debug topics, and
        // this one was the outlier.
        const bool want_debug =
            _publish_debug && _debug_pub && _debug_pub->get_subscription_count() > 0;

        mirena::SlamDebug debug;
        mirena::SlamDebug *sink = want_debug ? &debug : nullptr;

        // How much the odometry could have drifted since the last keyframe.
        // Computed once and used twice: the association gate widens by it now,
        // and if this frame becomes a keyframe its factor carries the same
        // numbers. Two different answers to that question is how a gate ends up
        // rejecting cones the graph is perfectly happy to believe in.
        gtsam::Pose2 delta = _last_keyframe_raw_odom.between(current_raw);
        const double interval_s = (car_stamp - _last_keyframe_time).seconds();
        const gtsam::Vector3 odom_sigmas = odometry_sigmas(*car_msg, delta, interval_s);

        _slam->observe(current_raw, obs, odom_sigmas, sink);

        // 3. Whether this frame is worth a permanent pose is the estimator's
        //    question, not the node's. It turns on what the frame can see and how
        //    well the map already knows it -- a keyframe is cheap to add and
        //    impossible to remove, so "the car moved half a metre" is the wrong
        //    test. All the node still owns is the very first one.
        //
        //    Stamps, not now(): the interval is a property of the data, and it is
        //    what the odometry noise integrates over. Mixing a wall reading into
        //    that would make factor weights depend on how busy this node was.
        bool keyframed = false;
        if (!_has_keyframe || _slam->wants_keyframe())
        {
            _slam->add_keyframe(odom_sigmas, sink);
            keyframed = true;
            _last_keyframe_raw_odom = current_raw;
            _last_keyframe_time = car_stamp;

            {
                std::lock_guard<std::mutex> lock(_report_mutex);
                _last_update_timestamp = this->now();
                _last_update_ms_elapsed = _slam->get_last_update_elapsed_ms();
                _has_keyframe = true;
            }
        }

        if (!_slam->healthy() && !_reported_unhealthy) {
            _reported_unhealthy = true;
            RCLCPP_FATAL(
                this->get_logger(),
                "SLAM solver failed and the estimate can no longer be trusted. "
                "The map and map->odom are frozen at their last good values.");
        }

        // The map changes on every frame, not only on keyframes, because a cone
        // can be promoted or lose its evidence between them.
        // Capture snapshot data under the slam lock, publish outside.
        {
            std::lock_guard<std::mutex> lock(_snapshot_mutex);
            _map_to_odom_snapshot = _slam->get_correction();
            auto map_msg = std::make_shared<mirena_common::msg::EntityList>();
            const std::vector<mirena::Landmark> landmarks = _slam->map_snapshot();
            map_msg->entities.reserve(landmarks.size());
            for (const auto &l : landmarks) {
                mirena_common::msg::Entity e;
                e.position.x = l.x;
                e.position.y = l.y;
                e.type = l.type;
                e.confidence = l.confidence;
                map_msg->entities.push_back(e);
            }
            map_msg->header.frame_id = FIXED_FRAME;
            map_msg->header.stamp = car_msg->header.stamp;
            _map_snapshot = std::move(map_msg);
            _has_snapshot = true;
        }

        {
            std::lock_guard<std::mutex> lock(_report_mutex);
            _last_update_total_landmark_count =
                static_cast<uint16_t>(_slam->get_landmark_count());
            _last_matched = _slam->get_last_matched_count();
            _last_promoted = _slam->get_last_promoted_count();
            _last_hidden = _slam->get_last_hidden_count();
            _out_of_range = _slam->get_last_out_of_range_count();
            _candidates = _slam->get_candidate_count();
            _poses = _slam->get_pose_count();
            _variables_reeliminated = _slam->get_variables_reeliminated();
            _cliques = _slam->get_cliques();
            _kf_structure = _slam->get_keyframes_from_structure();
            _kf_drift = _slam->get_keyframes_from_drift();
            _kf_timer = _slam->get_keyframes_from_timer();
            _pinned_fraction = _slam->get_pinned_fraction();
            _healthy = _slam->healthy();
        }

        // Deliberately outside the lock. Serialising a MarkerArray is not work the
        // estimator should be held hostage to, and under an incremental solver a
        // loop closure can make that critical section hundreds of milliseconds
        // long.
        slam_lock.unlock();
        if (want_debug && keyframed)
            publish_debug(debug, car_msg->header.stamp);
            }

    /// Uncertainty accumulated by the odometry *delta*, not the absolute pose,
    /// as sigmas over [x, y, theta] in the body frame.
    ///
    /// One answer, used in two places: the association gate widens by it on
    /// every frame, and the keyframe factor is built from it. They are the same
    /// question and must not be allowed to have different answers.
    ///
    /// This used to difference the absolute position covariance across the
    /// keyframe interval. That cannot work: a converged filter's absolute
    /// covariance is flat -- measured at 0.173 m sigma, varying by under a
    /// millimetre between keyframes -- so the difference was always ~0 and the
    /// message covariance never once influenced a factor. Every odometry
    /// constraint in the graph came from the motion model below, silently.
    ///
    /// What a relative factor actually deserves is the uncertainty accumulated
    /// by integrating the estimator's *velocity* over the interval. Velocity
    /// error out of an INS is bias-like over these timescales, so it integrates
    /// as sigma_v * dt rather than sqrt(dt), and unlike the absolute position
    /// covariance it does grow when the estimator loses its corrections.
    ///
    /// The motion model stays as a floor and still governs normal driving: it
    /// covers scale and lever-arm error the filter does not model, and it stops
    /// an over-confident estimator from locking the graph. Replayed over
    /// rosbag2_2026_08_05-20_26_23 (2780 keyframes) the median is unchanged at
    /// 0.030 m and the velocity term overrides the model on 2.0% of keyframes --
    /// the INS dropouts, where sigma_u rises from 0.12 to 0.88 m/s and this
    /// factor loosens to 0.071 m, 2.4x the model. That is the whole intent: no
    /// change when the odometry is healthy, a real loosening when it is not.
    ///
    /// Note this moves the position term only. sigma_omega out of the filter is
    /// ~0.01 rad/s, so over a 0.07 s keyframe the heading drift term is 0.0007
    /// rad against the model's 0.012, and heading stays motion-model governed on
    /// every keyframe in that bag. If the heading weight ever needs to respond
    /// to filter confidence, gyro_meas_var in mirena_state is what sets it.
    ///
    /// Caller must hold _slam_mutex.
    gtsam::Vector3 odometry_sigmas(const mirena_common::msg::Car &car,
                                   const gtsam::Pose2 &delta,
                                   double interval_s)
    {
        // Car covariance is row-major 6x6 over [x, y, psi, u, v, omega], so the
        // velocity block sits on the diagonal at 21, 28 and 35.
        const double dt = std::clamp(interval_s, 0.0, _keyframe_max_interval_s);
        const double drift_x = std::sqrt(std::max(0.0, car.covariance[21])) * dt;
        const double drift_y = std::sqrt(std::max(0.0, car.covariance[28])) * dt;
        const double drift_th = std::sqrt(std::max(0.0, car.covariance[35])) * dt;

        const double dist = delta.translation().norm();
        const double turn = std::abs(delta.theta());
        const double model_xy = _odom_model_xy_per_m * dist + _odom_model_xy_per_rad * turn;
        const double model_th = _odom_model_th_per_m * dist + _odom_model_th_per_rad * turn;

        const double sx = std::max({drift_x, model_xy, _odom_sigma_floor_xy});
        const double sy = std::max({drift_y, model_xy, _odom_sigma_floor_xy});
        const double st = std::max({drift_th, model_th, _odom_sigma_floor_th});

        return gtsam::Vector3(sx, sy, st);
    }

    /// One marker per thing worth seeing, all under the map frame.
    ///
    /// Published straight from the keyframe rather than off the snapshot timer:
    /// this describes one optimisation step, and re-sending it at 10 Hz would
    /// make a single bad association look like a persistent one.
    ///
    /// This is what the comment always said and what the caller now does. It did
    /// not: the call sat at the end of every synchronised frame, so on the three
    /// frames in four that are not keyframes the whole array was redrawn with a
    /// `promoted` list that only add_keyframe ever fills. Nothing was wrong with
    /// the associations drawn on those frames -- they were freshly matched -- but
    /// the promotion markers blinked out between keyframes, which reads as
    /// promotions being undone.
    void publish_debug(const mirena::SlamDebug &debug,
                       const builtin_interfaces::msg::Time &stamp)
    {
        using visualization_msgs::msg::Marker;
        visualization_msgs::msg::MarkerArray array;

        std_msgs::msg::Header header;
        header.frame_id = FIXED_FRAME;
        header.stamp = stamp;

        // Wipe first. Every keyframe redraws the lot, and without this the
        // labels of landmarks that dropped out of range stay on screen for ever.
        Marker clear;
        clear.header = header;
        clear.action = Marker::DELETEALL;
        array.markers.push_back(clear);

        const auto marker = [&](const char *ns, int32_t type, double scale,
                                float r, float g, float b, float a) {
            Marker m;
            m.header = header;
            m.ns = ns;
            m.id = 0;
            m.type = type;
            m.action = Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = scale;
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
            return m;
        };
        const auto point = [](const gtsam::Point2 &p) {
            geometry_msgs::msg::Point out;
            out.x = p.x();
            out.y = p.y();
            return out;
        };

        // Each line runs from where the cone was seen to the landmark it was
        // matched to, so its length is the innovation the gate let through. Short
        // and tidy is healthy; lines growing in one direction is the pose
        // drifting away from the map.
        Marker links = marker("associations", Marker::LINE_LIST, 0.03, 0.2f, 0.9f, 0.3f, 0.9f);
        for (const auto &[observed, landmark] : debug.associations) {
            links.points.push_back(point(observed));
            links.points.push_back(point(landmark));
        }
        array.markers.push_back(links);

        // Candidates promoted into the graph this keyframe. After the first lap
        // these should stop appearing; if they do not, association is failing to
        // recognise cones already mapped.
        Marker promoted = marker("promoted", Marker::SPHERE_LIST, 0.45, 0.2f, 0.8f, 1.0f, 0.9f);
        for (const auto &p : debug.promoted)
            promoted.points.push_back(point(p));
        array.markers.push_back(promoted);

        // Everything the map is still making its mind up about. These are outside
        // the graph and cost nothing to be wrong about, which is the point of
        // them. A cloud that grows and never promotes means the promotion test is
        // stuck; a cloud that never forms means perception has gone quiet.
        Marker candidates = marker("candidates", Marker::SPHERE_LIST, 0.25, 1.0f, 0.6f, 0.1f, 0.7f);
        for (const auto &[position, log_odds] : debug.candidates) {
            (void)log_odds;
            candidates.points.push_back(point(position));
        }
        array.markers.push_back(candidates);

        // Landmarks whose evidence has collapsed. They are still in the graph --
        // their measurements remain perfectly good constraints on the trajectory
        // -- but they are no longer published as cones.
        Marker hidden = marker("hidden", Marker::CUBE_LIST, 0.4, 1.0f, 0.9f, 0.1f, 0.9f);
        for (const auto &p : debug.hidden)
            hidden.points.push_back(point(p));
        array.markers.push_back(hidden);

        // How many times each nearby cone has been seen.
        int id = 0;
        for (const auto &[position, sightings] : debug.sightings) {
            Marker label = marker("sightings", Marker::TEXT_VIEW_FACING, 0.3,
                                  0.9f, 0.9f, 0.9f, 0.8f);
            label.id = id++;
            label.pose.position = point(position);
            label.pose.position.z = 0.4;
            label.text = std::to_string(sightings);
            array.markers.push_back(label);
        }

        _debug_pub->publish(array);
    }

    /// Republish the map. Slow on purpose: it only changes when SLAM solves.
    void map_publish_timer_callback()
    {
        std::shared_ptr<const mirena_common::msg::EntityList> map_msg;
        {
            std::lock_guard<std::mutex> lock(_snapshot_mutex);
            if (!_has_snapshot)
                return;
            map_msg = _map_snapshot;
        }
        // Keeps the stamp of the update that produced it, which is the time the
        // landmarks were actually valid.
        if (map_msg)
            _map_pub->publish(*map_msg);
    }

    /// Broadcast map->odom, at the rate consumers ask for it rather than the
    /// rate SLAM solves at.
    ///
    /// This used to share the map's 100 ms timer. tf2 interpolates between
    /// samples but will not extrapolate past the newest one, and mirena_control
    /// looks map->car/cog up at the freshest /state/car stamp -- which arrives
    /// at 100 Hz, from a node that publishes ten times as often as this one did.
    /// So for most of every 100 ms window the newest map->odom was older than
    /// the pose being asked about, and the lookup failed with "extrapolation
    /// into the future": 130 times in a 271 s replay of
    /// rosbag2_2026_08_05-20_26_23, each one a control cycle with no transform.
    ///
    /// The correction is piecewise-constant between keyframes, so re-emitting
    /// it under a fresh stamp adds no information -- it just keeps the buffer's
    /// newest sample abreast of the poses it has to be composed with.
    void tf_publish_timer_callback()
    {
        // Read the last optimisation result and get straight out of the way.
        gtsam::Pose2 map_to_odom;
        {
            std::lock_guard<std::mutex> lock(_snapshot_mutex);
            if (!_has_snapshot)
                return;
            map_to_odom = _map_to_odom_snapshot;
        }

        // 1. Ease the published correction towards the one SLAM last solved.
        //
        // A first-order filter, stepped with the time actually elapsed rather
        // than the nominal period, so a late tick catches up by the right
        // amount instead of falling behind. Interpolation is along the SE(2)
        // geodesic: doing it on x, y and yaw separately would drag the car
        // sideways through the turn between two corrections that differ in
        // heading.
        //
        // The first correction is taken as-is. Easing in from identity would
        // spend the first seconds of every run publishing a transform that is
        // knowingly wrong.
        const rclcpp::Time now = this->now();

        // 0. Never put a stamp on the wire that is not newer than the last one.
        //
        // map->odom is the only transform in the tree whose child frame (`odom`)
        // this node owns, so a stamp that goes backwards is inserted straight
        // into every listener's tf buffer as old data: TimeCache drops anything
        // more than its 10 s window behind the newest sample it already holds,
        // and once a larger stamp is in there, every later broadcast of ours is
        // rejected -- consumers can no longer compose map->car/cog and control
        // falls back to its failsafe for the rest of the run.
        //
        // The clock can only fail to advance if something outside this node is
        // driving it wrong (a second /clock publisher, a sim that restarted its
        // clock under a running mission). Dropping the tick keeps the buffer
        // consistent instead, and costs one 10 ms sample. The root cause is not
        // fixable from here; this only stops it from being permanent.
        if (_has_broadcast && now <= _last_broadcast)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Skipping map->odom broadcast: the clock did not advance (now %.6f s, last "
                "broadcast %.6f s). Publishing it would be rejected as TF_OLD_DATA by every "
                "listener and would freeze the tf tree for them.",
                now.seconds(), _last_broadcast.seconds());
            return;
        }

        if (!_has_broadcast)
        {
            _published_correction = map_to_odom;
            _has_broadcast = true;
        }
        else
        {
            const double dt = std::clamp((now - _last_broadcast).seconds(), 0.0, 1.0);
            const double alpha = 1.0 - std::exp(-dt / _correction_time_constant_s);
            _published_correction =
                gtsam::interpolate<gtsam::Pose2>(_published_correction, map_to_odom, alpha);
        }
        _last_broadcast = now;
        map_to_odom = _published_correction;

        // 2. Broadcast map -> odom.
        //
        // Stamped at broadcast time, NOT at the time of the data it was derived
        // from. map->odom is a piecewise-constant correction that only changes on
        // a keyframe, so re-publishing it under an old stamp adds no new sample to
        // the tf buffer -- the newest map->odom would then only ever advance as
        // fast as perception delivers a synchronised pair, and any consumer
        // looking up at a fresher time (control uses the latest /state/car stamp)
        // fails with "extrapolation into the future".
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = FIXED_FRAME;
        t.child_frame_id = MOVING_FRAME;

        t.transform.translation.x = map_to_odom.x();
        t.transform.translation.y = map_to_odom.y();
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, map_to_odom.theta());
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        _tf_broadcaster->sendTransform(t);
    }

    void report_callback() {
        if (!_report_pub) { return; }

        mirena_common::msg::Heartbeat msg;
        msg.timestamp = this->now();
        msg.node_name = this->get_name();

        nlohmann::json report_json;

        {
            std::lock_guard<std::mutex> lock(_report_mutex);

            // The supervisor could not previously tell healthy SLAM from SLAM
            // that had not produced a keyframe in five seconds: this field was
            // the constant "WORKING".
            const double since_update = (this->now() - _last_update_timestamp).seconds();
            const bool stale = _has_keyframe && since_update > _report_stale_after_s;
            report_json["status_state"] = !_healthy ? "NOT_WORKING"
                                          : !_has_keyframe ? "IDLE"
                                          : (stale ? "PARTIAL" : "WORKING");

            if (_has_keyframe)
                report_json["seconds_since_update"] = since_update;
            report_json["last_update_ms_elapsed"] = this->_last_update_ms_elapsed;
            report_json["last_update_total_landmark_count"] = this->_last_update_total_landmark_count;
            report_json["last_run_cones_seen"] = this->_last_update_total_landmark_count;
            // The map's size under the name the supervisor forwards to the car on
            // DV_info: every cone the run has established, as against the handful
            // perception can see from where the car is standing.
            report_json["cones_count_total"] = this->_last_update_total_landmark_count;

            // Once the track has been driven, new cones should stop appearing. If
            // they keep coming, association is failing to recognise cones already
            // mapped.
            report_json["matched_last_keyframe"] = _last_matched;
            report_json["promoted_last_keyframe"] = _last_promoted;
            report_json["hidden_landmarks"] = _last_hidden;
            // Detections thrown away for sitting beyond local_map_radius, where
            // this estimator can match them against nothing. Expected to be
            // non-zero -- perception claims cones further out than SLAM works --
            // but a figure that is most of the frame means the two radii have
            // drifted apart far enough to be discarding usable cones.
            report_json["out_of_range_last_frame"] = _out_of_range;
            // Candidates that have not earned a place in the graph. A number that
            // climbs and never falls means the promotion test is stuck.
            report_json["candidates"] = _candidates;
            // Poses in the graph. Under information-based keyframing this should
            // grow through the first laps and then flatten; if it keeps climbing
            // linearly, the pose chain is paying for laps it learns nothing from.
            report_json["poses"] = _poses;
            // Straight out of the solver, so a loop-closure spike shows up as work
            // done rather than as an unexplained slow keyframe.
            report_json["variables_reeliminated"] = _variables_reeliminated;
            report_json["cliques"] = _cliques;
            // What has been forcing poses. A chain growing on `structure` is
            // paying for a map it is still learning; one growing on `timer`
            // after the map has converged is paying for nothing.
            report_json["keyframes_from_structure"] = _kf_structure;
            report_json["keyframes_from_drift"] = _kf_drift;
            report_json["keyframes_from_timer"] = _kf_timer;
            report_json["pinned_fraction"] = _pinned_fraction;
            report_json["dropped_sync_pairs"] = _dropped_sync_pairs;

            report_json["car_pos_x"] = this->_current_car_x;
            report_json["car_pos_y"] = this->_current_car_y;
            report_json["car_pos_yaw"] = this->_current_car_psi;
        }

        msg.report = report_json.dump();
        _report_pub->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<MirenaMapping> node;
    try {
        node = std::make_shared<MirenaMapping>();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("mirena_slam"), "Startup failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}