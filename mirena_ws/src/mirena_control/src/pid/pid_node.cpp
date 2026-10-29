// PID path-following controller for the Mirena driverless car.
//
// A drop-in alternative to the MPC node: same node name, same topics, same
// mission parameters, same heartbeat contract.  Exactly one of the two runs per
// mission, chosen by which executable the mission launch file names.  The car
// cannot tell them apart -- both publish gas and a steering angle on /control,
// and both report mission_finished when the run is over.
//
// What it is for: the MPC needs acados, a generated solver and a solve that
// converges inside 50 ms.  This node needs none of that.  It is the fallback
// when the solver will not build, will not fit the compute budget, or is under
// suspicion -- and it is the reference the MPC's behaviour can be read against.
//
// Three loops run concurrently on their own callback groups:
//   * 20 Hz  guidance   -- projects the car onto the planned centreline and
//                          reduces the situation to four numbers: how far off
//                          the line the car is, how far off the heading, how
//                          hard the path bends, and how fast it should be
//                          going here;
//   * 100 Hz commander  -- carries those four numbers forward to now using the
//                          car's own motion, then closes both PID loops on them;
//   * 10 Hz  heartbeat  -- reports liveness and progress to the supervisor.
//
// Why the split, rather than one loop: the guidance pass needs TF and a spline
// projection and cannot be run at 100 Hz, but a lateral loop fed a 50 ms old
// cross-track error oscillates -- at cruise the car covers most of a metre in
// that time.  So the commander integrates the error dynamics forward itself
// (PidController::propagate) and closes the loop against a current error rather
// than a recent one.  It also never takes a lock the guidance loop can hold:
// the guidance result reaches it through a seqlock.
//
// The commander is the last line of defence, exactly as in the MPC node.  Every
// reason the guidance loop has to go quiet -- no state, a stale state, no track,
// a TF dropout -- ends with it holding numbers that are no longer about the
// situation the car is in, so once they are older than plan_timeout it falls
// back to full braking with the wheels straight and drops both integrators.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "mirena_common/msg/car.hpp"
#include "mirena_common/msg/car_control.hpp"
#include "mirena_common/msg/heartbeat.hpp"
#include "mirena_common/msg/track.hpp"

#include "common/cubic_spline.hpp"
#include "common/seqlock.hpp"

#include "pid_controller.hpp"

namespace mirena
{

namespace
{

/// Samples taken along the reference window each guidance pass.  Chosen to
/// match the MPC's path_points so the two nodes see the centreline at the same
/// resolution and their reference markers overlay.
constexpr int kRefPoints = 20;

/// Steps of the kinematic rollout drawn as the predicted path, and how long it
/// looks ahead.  Debug only; nothing reads it back.
constexpr int kPredictionSteps = 20;
constexpr double kPredictionHorizon = 1.0;   // s

/// What the commander publishes when it has nothing fresh to track: full
/// braking, wheels straight.  Not a tuning knob -- it is the one command that
/// is safe whatever the reason the guidance loop stopped talking.
constexpr double kFailsafeGas = -1.0;
constexpr double kFailsafeSteer = 0.0;

/// Sentinel for "this timestamp has not been set yet".  Zero will not do: under
/// sim time the clock legitimately starts at zero, so a zero sentinel makes the
/// first moment of a simulated run indistinguishable from never.
constexpr int64_t kUnsetTime = std::numeric_limits<int64_t>::min();

/// The same absolute/relative split the MPC node uses, and for the same reason:
/// a namespaced control node still talks to the one CAN bridge and the one
/// state estimator, while `health` stays relative because the supervisor
/// matches heartbeats by node name.
constexpr char kTrackTopic[] = "/planning/track";
constexpr char kCarStateTopic[] = "/state/car";
constexpr char kControlTopic[] = "/control";
constexpr char kHealthTopic[] = "health";

/// What the heartbeat says about the control loop.  Same four values the MPC
/// node reports, so a monitor does not have to know which controller is running.
enum class ControlState : int
{
  Idle = 0,      // waiting on the track or on a fresh state estimate
  Working = 1,   // the last guidance pass produced a usable reference
  Partial = 2,   // the pass ran but its result was rejected
  Starved = 3,   // inputs are present but the reference could not be built
};

const char * toString(ControlState state)
{
  switch (state) {
    case ControlState::Idle: return "IDLE";
    case ControlState::Partial: return "PARTIAL";
    case ControlState::Starved: return "NOT_WORKING";
    case ControlState::Working: default: return "WORKING";
  }
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/// Everything the loops need from the state estimate.  Kept as a small POD so
/// the subscription callback can publish it without allocating.
struct CarState
{
  int64_t stamp_ns;
  double u;
  double v;
  double omega;
  bool valid;
};

/// The guidance loop's whole output, and the only thing the commander reads.
///
/// Anchored at the state it was computed from, not at the instant the guidance
/// pass happened to finish, so the commander can carry it forward by the right
/// amount.  Propagating from anything later would build the estimator's latency
/// into the errors as a lag.
struct GuidancePlan
{
  int64_t state_stamp_ns;
  bool valid;
  double cross_track;      // m, positive when the centreline is to the car's left
  double heading_error;    // rad, path heading minus car yaw
  double curvature;        // 1/m at the projection, positive left
  double target_speed;     // m/s
};

/// What the commander last published, for the heartbeat and the debug rollout.
struct CommandEcho
{
  double gas;
  double steer;
  bool valid;
};

/// Immutable centreline snapshot, swapped in wholesale on each Track message.
struct TrackSnapshot
{
  CubicSpline2D spline;
  bool closed = false;
  uint64_t generation = 0;   // bumped per accepted fit
};

}  // namespace

class PidNode : public rclcpp::Node
{
public:
  PidNode()
  : rclcpp::Node("mirena_control")
  {
    // Mission parameters, identical in name, meaning and default to the MPC
    // node's, so a mission launch file drives either controller unchanged.
    declare_parameter<int>("laps", 0);
    declare_parameter<double>("stop_distance", 20.0);
    declare_parameter<double>("min_path_length", 5.0);
    declare_parameter<double>("stop_speed_eps", 0.3);
    declare_parameter<double>("stop_hold_time", 0.5);
    declare_parameter<double>("lookahead_base", 15.0);
    declare_parameter<double>("lookahead_gain", 1.2);
    declare_parameter<std::string>("robot_frame", "car/cog");
    declare_parameter<std::string>("work_frame", "odom");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("solver_period", 0.05);
    declare_parameter<double>("command_period", 0.01);
    declare_parameter<double>("heartbeat_period", 0.1);
    declare_parameter<double>("tf_timeout", 0.005);
    declare_parameter<double>("projection_window_back", 5.0);
    declare_parameter<double>("projection_window_forward", 20.0);
    declare_parameter<double>("projection_coarse_step", 0.25);
    declare_parameter<double>("state_timeout", 0.15);
    declare_parameter<double>("plan_timeout", 0.25);
    declare_parameter<double>("mission_timeout", 600.0);
    declare_parameter<int>("finish_confirm_count", 3);
    declare_parameter<double>("progress_rate_margin", 1.5);
    declare_parameter<double>("progress_rate_slack", 0.5);
    declare_parameter<bool>("publish_markers", true);

    // Controller parameters, which have no counterpart in the MPC: its cost
    // function is where these decisions live, and it is compiled in.
    declare_parameter<double>("speed_kp", 0.35);
    declare_parameter<double>("speed_ki", 0.15);
    declare_parameter<double>("speed_kd", 0.02);
    declare_parameter<double>("speed_tau_d", 0.05);
    declare_parameter<double>("cross_track_kp", 0.20);
    declare_parameter<double>("cross_track_ki", 0.02);
    declare_parameter<double>("cross_track_kd", 0.05);
    declare_parameter<double>("cross_track_tau_d", 0.08);
    declare_parameter<double>("heading_gain", 0.8);
    declare_parameter<double>("curvature_feedforward", 1.0);
    declare_parameter<double>("wheelbase", 1.55);
    declare_parameter<double>("cross_track_limit", 3.0);
    declare_parameter<double>("steer_limit", 30.0 * M_PI / 180.0);
    declare_parameter<double>("steer_rate", 5.0 * 30.0 * M_PI / 180.0);
    declare_parameter<double>("gas_rate", 5.0);
    declare_parameter<double>("max_speed", 20.0);
    declare_parameter<double>("min_speed", 1.5);
    declare_parameter<double>("max_lateral_accel", 6.0);
    declare_parameter<double>("max_decel", 4.0);
    declare_parameter<double>("track_end_margin", 3.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    RCLCPP_INFO(get_logger(), "Starting PID control node...");

    laps_ = static_cast<int>(get_parameter("laps").as_int());
    stop_distance_ = get_parameter("stop_distance").as_double();
    min_path_length_ = std::max(get_parameter("min_path_length").as_double(), 0.1);
    stop_speed_eps_ = get_parameter("stop_speed_eps").as_double();
    stop_hold_time_ = get_parameter("stop_hold_time").as_double();
    lookahead_base_ = get_parameter("lookahead_base").as_double();
    lookahead_gain_ = get_parameter("lookahead_gain").as_double();
    robot_frame_ = get_parameter("robot_frame").as_string();
    work_frame_ = get_parameter("work_frame").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    guidance_period_ = get_parameter("solver_period").as_double();
    command_period_ = get_parameter("command_period").as_double();
    heartbeat_period_ = get_parameter("heartbeat_period").as_double();
    tf_timeout_ = get_parameter("tf_timeout").as_double();
    projection_back_ = get_parameter("projection_window_back").as_double();
    projection_forward_ = get_parameter("projection_window_forward").as_double();
    projection_step_ = get_parameter("projection_coarse_step").as_double();
    state_timeout_ = std::max(get_parameter("state_timeout").as_double(), 0.0);
    plan_timeout_ = std::max(get_parameter("plan_timeout").as_double(), 0.0);
    mission_timeout_ = get_parameter("mission_timeout").as_double();
    finish_confirm_count_ =
      std::max(static_cast<int>(get_parameter("finish_confirm_count").as_int()), 1);
    progress_rate_margin_ = std::max(get_parameter("progress_rate_margin").as_double(), 1.0);
    progress_rate_slack_ = std::max(get_parameter("progress_rate_slack").as_double(), 0.0);
    publish_markers_ = get_parameter("publish_markers").as_bool();
    track_end_margin_ = std::max(get_parameter("track_end_margin").as_double(), 0.0);

    PidController::Config cfg;
    cfg.speed.kp = get_parameter("speed_kp").as_double();
    cfg.speed.ki = get_parameter("speed_ki").as_double();
    cfg.speed.kd = get_parameter("speed_kd").as_double();
    cfg.speed.tau_d = get_parameter("speed_tau_d").as_double();
    cfg.cross_track.kp = get_parameter("cross_track_kp").as_double();
    cfg.cross_track.ki = get_parameter("cross_track_ki").as_double();
    cfg.cross_track.kd = get_parameter("cross_track_kd").as_double();
    cfg.cross_track.tau_d = get_parameter("cross_track_tau_d").as_double();
    cfg.heading_gain = get_parameter("heading_gain").as_double();
    cfg.curvature_feedforward = get_parameter("curvature_feedforward").as_double();
    cfg.wheelbase = std::max(get_parameter("wheelbase").as_double(), 0.1);
    cfg.cross_track_limit = std::max(get_parameter("cross_track_limit").as_double(), 0.1);
    cfg.steer_limit = std::max(get_parameter("steer_limit").as_double(), 0.01);
    cfg.steer_rate = std::max(get_parameter("steer_rate").as_double(), 0.01);
    cfg.gas_rate = std::max(get_parameter("gas_rate").as_double(), 0.01);
    cfg.gas_min = kFailsafeGas;
    cfg.gas_max = 1.0;
    cfg.max_speed = std::max(get_parameter("max_speed").as_double(), 0.1);
    cfg.min_speed = std::max(get_parameter("min_speed").as_double(), 0.0);
    cfg.max_lateral_accel = std::max(get_parameter("max_lateral_accel").as_double(), 0.1);
    cfg.max_decel = std::max(get_parameter("max_decel").as_double(), 0.1);
    controller_ = std::make_unique<PidController>(cfg);


    if (laps_ > 0) {
      RCLCPP_INFO(
        get_logger(), "Lap mission: %d laps, stopping %.1f m past the final crossing.",
        laps_, stop_distance_);
    } else {
      RCLCPP_INFO(get_logger(), "Open-track mission: stopping at the last gate.");
    }
    RCLCPP_INFO(
      get_logger(),
      "PID ready: speed (%.2f, %.2f, %.2f), cross-track (%.2f, %.2f, %.2f), "
      "heading %.2f, curvature ff %.2f.",
      cfg.speed.kp, cfg.speed.ki, cfg.speed.kd,
      cfg.cross_track.kp, cfg.cross_track.ki, cfg.cross_track.kd,
      cfg.heading_gain, cfg.curvature_feedforward);
    RCLCPP_INFO(
      get_logger(),
      "Speed profile: %.1f m/s ceiling, %.1f m/s^2 lateral, %.1f m/s^2 braking.",
      cfg.max_speed, cfg.max_lateral_accel, cfg.max_decel);
    RCLCPP_INFO(
      get_logger(),
      "Failsafe: gas %.1f, steer %.1f rad, after %.2f s without guidance "
      "(state must be fresher than %.2f s).",
      kFailsafeGas, kFailsafeSteer, plan_timeout_, state_timeout_);

    guidance_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    track_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // A group of its own rather than the track's: refitting the centreline is
    // the longest callback in the node, and the supervisor declares a node dead
    // on a few seconds of silence.  Liveness must not queue behind a spline fit.
    heartbeat_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions state_opts;
    state_opts.callback_group = state_group_;
    rclcpp::SubscriptionOptions track_opts;
    track_opts.callback_group = track_group_;

    track_sub_ = create_subscription<mirena_common::msg::Track>(
      kTrackTopic, 10,
      std::bind(&PidNode::trackCallback, this, std::placeholders::_1), track_opts);
    car_sub_ = create_subscription<mirena_common::msg::Car>(
      kCarStateTopic, 10,
      std::bind(&PidNode::carCallback, this, std::placeholders::_1), state_opts);

    control_pub_ = create_publisher<mirena_common::msg::CarControl>(kControlTopic, 10);
    viz_pub_ = create_publisher<visualization_msgs::msg::Marker>("control/horizon_markers", 10);
    heartbeat_pub_ = create_publisher<mirena_common::msg::Heartbeat>(kHealthTopic, 10);

    initMarkers();

    // Nothing has been computed yet, so the commander starts in its failsafe.
    // Publishing full braking before the first guidance pass is the right thing
    // to be holding if the car reaches DRIVING while the pipeline is still
    // coming up -- it is what the car would get anyway if guidance never came.
    plan_.store(GuidancePlan{});
    echo_.store(CommandEcho{kFailsafeGas, kFailsafeSteer, false});

    heartbeat_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(heartbeat_period_),
      std::bind(&PidNode::publishHeartbeat, this), heartbeat_group_);
    guidance_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(guidance_period_),
      std::bind(&PidNode::guidanceIteration, this), guidance_group_);
    command_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(command_period_),
      std::bind(&PidNode::commandIteration, this), command_group_);
  }

private:
  // ---------------------------------------------------------------- callbacks

  void trackCallback(const mirena_common::msg::Track::SharedPtr msg)
  {
    if (msg->gates.size() < 2) {
      return;
    }

    // Closed tracks repeat the first gate so the periodic fit has a seam to
    // match; scratch is reused between messages to keep this allocation-free
    // once the track size settles.
    const std::size_t n = msg->gates.size() + (msg->is_closed ? 1 : 0);
    fit_scratch_.resize(2 * n);
    for (std::size_t i = 0; i < msg->gates.size(); ++i) {
      fit_scratch_[2 * i] = msg->gates[i].x;
      fit_scratch_[2 * i + 1] = msg->gates[i].y;
    }
    if (msg->is_closed) {
      fit_scratch_[2 * (n - 1)] = msg->gates.front().x;
      fit_scratch_[2 * (n - 1) + 1] = msg->gates.front().y;
    }

    auto snapshot = std::make_shared<TrackSnapshot>();
    snapshot->closed = msg->is_closed;
    const auto bc = msg->is_closed ?
      CubicSpline2D::Boundary::Periodic : CubicSpline2D::Boundary::NotAKnot;

    if (!snapshot->spline.build(fit_scratch_.data(), n, bc)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not fit a spline through %zu gates; keeping the previous track.", n);
      return;
    }

    // Tagged so the guidance loop can tell one centreline from the next: a new
    // fit can have a different arc length and a different origin, and the
    // progress accumulated against the old one is not directly comparable.
    snapshot->generation = track_generation_.fetch_add(1, std::memory_order_relaxed) + 1;

    {
      std::lock_guard<std::mutex> lock(track_mutex_);
      track_ = std::move(snapshot);
    }
    have_track_.store(true, std::memory_order_relaxed);
  }

  void carCallback(const mirena_common::msg::Car::SharedPtr msg)
  {
    // A non-finite velocity has to be dropped here or it is never caught again:
    // NaN compares false against every threshold downstream, so it would slip
    // through the stationary test in updateStopLatch as "not moving" and end the
    // mission, as well as reaching the speed loop as an error.  The last good
    // state is kept instead and simply ages out if nothing better arrives.
    if (!std::isfinite(msg->u) || !std::isfinite(msg->v) || !std::isfinite(msg->omega)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarding a state estimate with a non-finite velocity.");
      return;
    }

    CarState s;
    s.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    s.u = msg->u;
    s.v = msg->v;
    s.omega = msg->omega;
    s.valid = true;
    car_.store(s);
  }

  // -------------------------------------------------------- reference tracking

  /// One guidance pass' worth of geometry, in the work frame.
  struct Guidance
  {
    double points[2 * kRefPoints];       // the sampled centreline, for the marker
    double curvature[kRefPoints];        // 1/m at each sample, positive left
    double spacing;                      // m between samples
    double path_yaw;                     // heading of the centreline at the projection
    double path_x;
    double path_y;
    double pose[3];                      // car x, y, yaw
    double distance_to_stop;             // m of centreline left before the run ends
  };

  /// canTransform() with the configured budget, so every lookup that follows is
  /// one the buffer has actually been given a chance to fill.
  bool waitForTransform(
    const std::string & target, const std::string & source, const rclcpp::Time & when)
  {
    std::string tf_error;
    if (tf_buffer_->canTransform(
        target, source, when, rclcpp::Duration::from_seconds(tf_timeout_), &tf_error))
    {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF %s -> %s unavailable: %s", target.c_str(), source.c_str(), tf_error.c_str());
    return false;
  }

  bool generateGuidance(const CarState & car, Guidance & out)
  {
    std::shared_ptr<const TrackSnapshot> track;
    {
      std::lock_guard<std::mutex> lock(track_mutex_);
      track = track_;
    }
    if (!track || !track->spline.valid()) {
      return false;
    }

    const rclcpp::Time target_time(car.stamp_ns, get_clock()->get_clock_type());

    try {
      // The same three chains the MPC node waits on, and deliberately so: this
      // node is meant to fail in the same places for the same reasons.  map->car
      // is what the projection needs, the pose the errors are measured against
      // comes from odom->car, and the centreline is rotated into the work frame
      // with odom->map.
      if (!waitForTransform(map_frame_, robot_frame_, target_time) ||
        !waitForTransform(work_frame_, robot_frame_, target_time) ||
        !waitForTransform(work_frame_, map_frame_, target_time))
      {
        return false;
      }

      const auto t_map_car = tf_buffer_->lookupTransform(map_frame_, robot_frame_, target_time);
      const double car_map_x = t_map_car.transform.translation.x;
      const double car_map_y = t_map_car.transform.translation.y;

      const auto t_odom_car = tf_buffer_->lookupTransform(work_frame_, robot_frame_, target_time);
      out.pose[0] = t_odom_car.transform.translation.x;
      out.pose[1] = t_odom_car.transform.translation.y;
      out.pose[2] = yawFromQuaternion(t_odom_car.transform.rotation);

      const CubicSpline2D & spline = track->spline;
      const double s_max = spline.length();
      if (s_max <= 0.0) {
        return false;
      }

      // Progress must stay unwrapped so it can pass laps * s_max; only the
      // sampling below folds back into the spline's domain.
      const bool wrap = track->closed;
      const int64_t now_ns = get_clock()->now().nanoseconds();

      const double progress =
        updateProgress(*track, s_max, wrap, car, car_map_x, car_map_y, now_ns);

      // Where the run ends, in the same unwrapped arc length as progress.  A lap
      // mission needs the planner to have closed the loop before s_max means
      // "one lap"; until then nothing is truncated and the car keeps driving.
      // An open path simply ends at its last gate.  The timeout is the backstop
      // for a lap mission whose loop never closes -- see updateMissionTimeout.
      const bool stop_requested = stop_requested_.load(std::memory_order_relaxed);
      const bool finish_active = stop_requested || (laps_ == 0) || track->closed;
      double s_finish;
      double s_stop;

      if (stop_requested) {
        // Stop where the car is.
        s_finish = progress;
        s_stop = progress;
      } else if (laps_ > 0) {
        s_finish = static_cast<double>(laps_) * s_max;
        // Capped so a stop_distance >= one lap cannot fold the stop point onto
        // the following lap when the sampling below wraps.
        s_stop = s_finish + std::min(stop_distance_, 0.5 * s_max);
      } else {
        s_finish = std::max(0.0, s_max - 5);
        s_stop = s_max;
      }

      if (finish_active) {
        updateLapCount(progress, s_max, track->closed);
        updateFinishLatch(progress, s_finish);
      }
      // This is where the two controllers genuinely differ.  The MPC stops by
      // truncating its reference and letting the solver discover that it has to
      // brake; there is no stop command anywhere in that node.  A PID has no
      // such mechanism, so the distance left to the stop point is handed to the
      // speed profile explicitly and the longitudinal loop brakes against it.
      // Infinity means "nothing ahead ends the run", which is the honest answer
      // for a lap mission whose loop the planner has not closed yet.
      const double mission_stop = finish_active ?
        std::max(s_stop - progress, 0.0) : std::numeric_limits<double>::infinity();

      // ...but the mission's end is not the only thing the car has to be able to
      // stop before.  On a track the planner is still discovering, the
      // centreline ends where the map does, and beyond that point there is
      // nothing to steer by: the reference window collapses onto the last knot,
      // the car keeps driving past a path point that no longer moves, and the
      // cross-track error grows for as long as it takes the planner to catch up.
      //
      // So the mapped track is a stop point too, and the same braking law
      // applies to it: never carry more speed than can be shed inside the
      // centreline that actually exists.  That bound relaxes on its own as the
      // map grows, which is what makes it a speed limit rather than a stop --
      // and it is deliberately kept out of s_finish and the finish latch, since
      // running out of map is not the mission ending.
      const double mapped_ahead = track->closed ?
        std::numeric_limits<double>::infinity() :
        std::max(s_max - progress - track_end_margin_, 0.0);

      out.distance_to_stop = std::min(mission_stop, mapped_ahead);
      
      if (mapped_ahead < mission_stop) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Only %.1f m of centreline mapped ahead; capping the speed target at "
          "%.1f m/s so the car can stop inside it.",
          mapped_ahead, std::sqrt(2.0 * controller_->config().max_decel * mapped_ahead));
      }

      // Sample the centreline over the lookahead window.  Unlike the MPC's,
      // this window is never truncated at the finish: the samples are read for
      // their curvature, and the car still has to be steered through the last
      // metres it is braking over.
      double lookahead =
        std::max(lookahead_base_ + std::fabs(car.u) * lookahead_gain_, min_path_length_);

      // On an open track the window has to stop where the spline does.  Sampling
      // past the end clamps every remaining point onto the last knot, and a
      // window that is mostly one repeated point reports that point's curvature
      // as though it were the whole road ahead -- which is how a 20-point window
      // ends up setting the speed target from a single knot at the edge of the
      // map.  Shortening the window instead keeps every sample a real place.
      if (!track->closed) {
        const double mapped = std::max(s_max - progress, 0.0);
        lookahead = std::max(std::min(lookahead, mapped), min_path_length_);
      }
      out.spacing = lookahead / static_cast<double>(kRefPoints - 1);

      double d1[2];
      double d2[2];
      double pos[2];
      for (int i = 0; i < kRefPoints; ++i) {
        double s = progress + out.spacing * static_cast<double>(i);
        if (track->closed) {
          s = std::fmod(s, s_max);
          if (s < 0.0) {
            s += s_max;
          }
        } else {
          s = std::min(std::max(s, 0.0), s_max);
        }
        spline.evalDerivatives(s, pos, d1, d2);
        out.points[2 * i] = pos[0];
        out.points[2 * i + 1] = pos[1];
        out.curvature[i] = curvatureFrom(d1, d2);
        if (i == 0) {
          // The projection itself: where the car is measured from, and the
          // heading it is measured against.
          out.path_x = pos[0];
          out.path_y = pos[1];
          out.path_yaw = std::atan2(d1[1], d1[0]);
        }
      }

      // ...then rotate the whole window into the work frame in one pass.  The
      // curvatures are invariant under a rigid transform and need no fixing up;
      // the tangent heading does.
      const auto t_odom_map = tf_buffer_->lookupTransform(work_frame_, map_frame_, target_time);
      const double yaw = yawFromQuaternion(t_odom_map.transform.rotation);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      const double tx = t_odom_map.transform.translation.x;
      const double ty = t_odom_map.transform.translation.y;
      for (int i = 0; i < kRefPoints; ++i) {
        const double px = out.points[2 * i];
        const double py = out.points[2 * i + 1];
        out.points[2 * i] = c * px - s * py + tx;
        out.points[2 * i + 1] = s * px + c * py + ty;
      }
      out.path_x = out.points[0];
      out.path_y = out.points[1];
      out.path_yaw = wrapAngle(out.path_yaw + yaw);

      return true;

    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF lookup failed in generateGuidance: %s", e.what());
      return false;
    }
  }

  /// Signed curvature from the spline's first and second derivative.  Positive
  /// where the path bends left, which is the same sign convention the steering
  /// angle uses, so the feed-forward term needs no correction.
  static double curvatureFrom(const double (& d1)[2], const double (& d2)[2])
  {
    const double speed_sq = d1[0] * d1[0] + d1[1] * d1[1];
    if (speed_sq < 1e-12) {
      return 0.0;
    }
    const double cross = d1[0] * d2[1] - d1[1] * d2[0];
    return cross / (speed_sq * std::sqrt(speed_sq));
  }

  /// Project the car onto the centreline and fold the result into the run's
  /// cumulative progress.  Guidance thread only.
  double updateProgress(
    const TrackSnapshot & track, double s_max, bool wrap, const CarState & car,
    double car_map_x, double car_map_y, int64_t now_ns)
  {
    // Always seeded from the progress we already believe in, including the very
    // first call, where it is the 0 the members start at -- the car starts on
    // the line.  A global nearest-point sweep must not be used as a substitute:
    // the unrolled skidpad path overlaps itself exactly, so the nearest point is
    // ambiguous between passes and can jump a whole circle.
    double seed = global_s_.load(std::memory_order_relaxed);

    // A replanned centreline is a new curve with its own arc length, and the
    // planner emits one every time it refines the track.  Only the completed
    // laps are re-expressed in the new curve's metres: the position within the
    // current lap is measured from the start gate, which does not move, so a
    // track that simply grew at its far end must not drag it along.
    //
    // Only on a closed track.  On an open one s_max is not a lap, it is just
    // how much has been mapped so far, and dividing by it is meaningless: a car
    // sitting at the end of the mapped track has seed == last_s_max_ exactly, so
    // floor(seed / last_s_max_) is 1 and the re-basing credits it with a lap it
    // never drove, teleporting progress a whole track-length forward.  The rate
    // limiter below cannot catch that, because it brackets the re-based seed and
    // so measures the jump as zero.  An open track that grew at its far end
    // needs no re-basing at all -- every arc length it already had still means
    // the same place.
    if (track.generation != last_track_generation_) {
      if (wrap && last_s_max_ > 0.0) {
        const double laps_part = std::floor(seed / last_s_max_);
        const double within = std::min(seed - laps_part * last_s_max_, s_max);
        seed = laps_part * s_max + within;
      }
      last_track_generation_ = track.generation;
      last_s_max_ = s_max;
    }

    double progress = track.spline.project(
      car_map_x, car_map_y, seed, wrap,
      projection_back_, projection_forward_, projection_step_);

    // SLAM's map-frame pose steps sideways on loop closure and the projection
    // follows it.  Left unbounded, one such frame is enough to push progress
    // past the finish and arm a latch that ends the run mid-track, so progress
    // may only move as fast as the car could actually have travelled.  A genuine
    // correction still gets there, just over several frames.
    if (last_projection_ns_ != kUnsetTime) {
      const double dt = static_cast<double>(now_ns - last_projection_ns_) * 1e-9;
      if (dt > 0.0 && dt < 1.0) {
        const double max_step =
          std::fabs(car.u) * dt * progress_rate_margin_ + progress_rate_slack_;
        const double limited = std::min(std::max(progress, seed - max_step), seed + max_step);
        if (limited != progress) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Progress jumped %.1f m in %.3f s; rate-limited to %.1f m.",
            progress - seed, dt, limited - seed);
          progress = limited;
        }
      }
    }
    last_projection_ns_ = now_ns;

    global_s_.store(progress, std::memory_order_relaxed);
    return progress;
  }

  // ----------------------------------------------------------- guidance loop

  void guidanceIteration()
  {
    const CarState car = car_.load();
    const int64_t now_ns = get_clock()->now().nanoseconds();

    // A state estimate that stopped arriving stays in the seqlock forever and
    // reads as perfectly valid, so freshness is what gates the loop rather than
    // the valid flag alone.  Everything downstream -- the TF lookups at the
    // state's stamp included -- assumes the car really was there recently.
    if (!car.valid) {
      control_state_.store(static_cast<int>(ControlState::Idle), std::memory_order_relaxed);
      return;
    }
    const double state_age = static_cast<double>(now_ns - car.stamp_ns) * 1e-9;
    if (state_age > state_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "State estimate is %.2f s old (limit %.2f s); holding guidance off.",
        state_age, state_timeout_);
      control_state_.store(static_cast<int>(ControlState::Idle), std::memory_order_relaxed);
      return;
    }
    if (!have_track_.load(std::memory_order_relaxed)) {
      control_state_.store(static_cast<int>(ControlState::Idle), std::memory_order_relaxed);
      return;
    }

    // Ahead of generateGuidance, so a TF dropout on the final metres cannot stop
    // the run from ever being declared finished.
    updateStopLatch(car, now_ns);
    updateMissionTimeout(car, now_ns);

    Guidance guidance;
    if (!generateGuidance(car, guidance)) {
      // Inputs are there but the reference is not, which is a different fault
      // from waiting on an input and the supervisor is told so.
      control_state_.store(static_cast<int>(ControlState::Starved), std::memory_order_relaxed);
      return;
    }

    // Reduce the geometry to the four numbers the commander closes on.  The
    // sign convention throughout is the steering angle's: positive is left.
    const double dx = guidance.path_x - guidance.pose[0];
    const double dy = guidance.path_y - guidance.pose[1];
    const double sin_p = std::sin(guidance.path_yaw);
    const double cos_p = std::cos(guidance.path_yaw);

    GuidancePlan plan;
    plan.state_stamp_ns = car.stamp_ns;
    plan.valid = true;
    plan.cross_track = -dx * sin_p + dy * cos_p;
    plan.heading_error = wrapAngle(guidance.path_yaw - guidance.pose[2]);
    plan.curvature = guidance.curvature[0];
    plan.target_speed = controller_->speedTarget(
      guidance.curvature, kRefPoints, guidance.spacing, guidance.distance_to_stop);

    // A reference the car is nowhere near is not one to steer by.  The
    // projection is local and seeded, so an error this large means the seed and
    // the car have parted company -- a SLAM jump the rate limiter is still
    // working through, a refit that moved the line.  Reporting PARTIAL and
    // letting the plan go stale is the same outcome the MPC reaches when its
    // solve stops converging: the commander brakes.
    const double reject_distance = 4.0 * controller_->config().cross_track_limit;
    if (!std::isfinite(plan.cross_track) || std::fabs(plan.cross_track) > reject_distance) {
      control_state_.store(static_cast<int>(ControlState::Partial), std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Guidance rejected: the car is %.1f m from the centreline (limit %.1f m).",
        plan.cross_track, reject_distance);
      return;
    }

    plan_.store(plan);

    const int64_t t_end = get_clock()->now().nanoseconds();
    guidance_time_ms_.store(
      static_cast<double>(t_end - now_ns) * 1e-6, std::memory_order_relaxed);
    guidance_latency_ms_.store(
      static_cast<double>(t_end - car.stamp_ns) * 1e-6, std::memory_order_relaxed);
    last_guidance_ok_ns_.store(t_end, std::memory_order_relaxed);
    cross_track_.store(plan.cross_track, std::memory_order_relaxed);
    heading_error_.store(plan.heading_error, std::memory_order_relaxed);
    target_speed_.store(plan.target_speed, std::memory_order_relaxed);
    control_state_.store(static_cast<int>(ControlState::Working), std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "PID OK | e_y: %+5.2f m | e_psi: %+5.2f rad | k: %+5.3f 1/m | v*: %4.1f m/s | "
      "u: %4.1f m/s | to stop: %.1f m",
      plan.cross_track, plan.heading_error, plan.curvature, plan.target_speed, car.u,
      guidance.distance_to_stop);

    broadcastDebug(guidance);
  }

  /// Laps come straight from how far the car has travelled along the centreline;
  /// the spline is anchored at the start gate, so s == 0 (mod s_max) is the
  /// finish line by construction.
  void updateLapCount(double progress, double s_max, bool closed)
  {
    if (laps_ <= 0 || !closed) {
      return;
    }
    const int done = static_cast<int>(std::floor(progress / s_max));
    if (done > laps_done_.load(std::memory_order_relaxed)) {
      laps_done_.store(done, std::memory_order_relaxed);
      RCLCPP_INFO(get_logger(), "Lap %d/%d completed.", done, laps_);
    }
  }

  /// Arming the finish commits the car to braking for the rest of the run, so it
  /// is not done on the strength of a single frame: the projection has to agree
  /// finish_confirm_count times running.  One optimistic frame -- a residual map
  /// correction the rate limiter let through, a gate that moved -- would
  /// otherwise stop the car mid-track with no way back.  Guidance thread only.
  void updateFinishLatch(double progress, double s_finish)
  {
    if (finish_armed_.load(std::memory_order_relaxed)) {
      return;
    }
    if (progress < s_finish) {
      finish_confirm_ = 0;
      return;
    }
    if (++finish_confirm_ < finish_confirm_count_) {
      return;
    }
    finish_armed_.store(true, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(), "Finish reached at s = %.1f m; braking to a stop.", progress);
  }

  /// The run is only reported finished once the car is actually stationary.  The
  /// supervisor tears the stack down the moment it sees the flag, so latching
  /// any earlier would cut the controller off mid-braking.
  void updateStopLatch(const CarState & car, int64_t now_ns)
  {
    if (!finish_armed_.load(std::memory_order_relaxed) ||
      finished_.load(std::memory_order_relaxed))
    {
      return;
    }
    if (std::fabs(car.u) >= stop_speed_eps_) {
      stop_since_ns_ = kUnsetTime;     // still rolling, re-arm
      return;
    }
    if (stop_since_ns_ == kUnsetTime) {
      stop_since_ns_ = now_ns;
    } else if (static_cast<double>(now_ns - stop_since_ns_) * 1e-9 >= stop_hold_time_) {
      finished_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO(
        get_logger(), "Stopped at s = %.1f m after %d lap(s). Mission complete.",
        global_s_.load(std::memory_order_relaxed), laps_done_.load(std::memory_order_relaxed));
    }
  }

  /// A lap mission only brakes for its finish once the planner has closed the
  /// loop, because until then s_max is not a lap.  If the loop never closes,
  /// nothing in the geometry can ever end the run and the car keeps driving.
  /// This is the backstop: once the car has been moving for mission_timeout
  /// seconds the finish is forced wherever it happens to be.  Guidance thread
  /// only.
  void updateMissionTimeout(const CarState & car, int64_t now_ns)
  {
    if (mission_timeout_ <= 0.0 || stop_requested_.load(std::memory_order_relaxed)) {
      return;
    }
    // The clock starts when the car does, not at launch: the node comes up on AS
    // READY and may sit there for minutes before the car is allowed to move.
    if (motion_start_ns_ == kUnsetTime) {
      if (std::fabs(car.u) >= stop_speed_eps_) {
        motion_start_ns_ = now_ns;
      }
      return;
    }
    const double driving = static_cast<double>(now_ns - motion_start_ns_) * 1e-9;
    if (driving >= mission_timeout_) {
      stop_requested_.store(true, std::memory_order_relaxed);
      RCLCPP_WARN(
        get_logger(),
        "Mission timeout: %.0f s of driving without a finish the controller could "
        "recognise (laps %d/%d). Stopping where the car is.",
        driving, laps_done_.load(std::memory_order_relaxed), laps_);
    }
  }

  // ------------------------------------------------------------ command loop

  void commandIteration()
  {
    // No gate on the mission state here: commands go out from the moment there
    // is guidance.  The car is what decides whether to act on them -- it ignores
    // this node until its own state machine reaches DRIVING.
    const auto now = get_clock()->now();
    const int64_t now_ns = now.nanoseconds();
    command_msg_.header.stamp = now;

    // The loop's own step, measured rather than assumed, so timer jitter does
    // not show up in the integral and the derivative.
    double dt = command_period_;
    if (last_command_ns_ != kUnsetTime) {
      const double measured = static_cast<double>(now_ns - last_command_ns_) * 1e-9;
      if (measured > 0.0) {
        dt = measured;
      }
    }
    last_command_ns_ = now_ns;

    const CarState car = car_.load();
    const GuidancePlan plan = plan_.load();

    // No finish override lives here on purpose: bringing the car to a stop is
    // the job of the speed profile, which the guidance loop already collapsed
    // towards zero as the stop point came within braking distance.
    //
    // ...but guidance that has stopped being refreshed is a different matter.
    // An unchecked stale plan would leave the lateral loop steering towards a
    // line the car has since left, for as long as the node runs.  Whatever
    // silenced it -- no state, no track, no TF, a rejected projection -- the
    // answer is the same: brake, wheels straight.
    bool usable = plan.valid && car.valid;
    double age = 0.0;
    if (usable) {
      age = static_cast<double>(now_ns - plan.state_stamp_ns) * 1e-9;
      const double state_age = static_cast<double>(now_ns - car.stamp_ns) * 1e-9;
      usable = age <= plan_timeout_ && state_age <= state_timeout_;
    }

    if (!usable) {
      // failsafe() drops both integrators as well as returning the command: an
      // integral wound up against a situation the car has since left is worse
      // than no integral at all when the loop resumes.
      const PidController::Command cmd = controller_->failsafe();
      command_msg_.gas = cmd.gas;
      command_msg_.steer_angle = cmd.steer;
      control_pub_->publish(command_msg_);
      echo_.store(CommandEcho{cmd.gas, cmd.steer, false});
      if (!failsafe_active_.exchange(true, std::memory_order_relaxed)) {
        RCLCPP_ERROR(
          get_logger(),
          "No usable guidance (age %.2f s, limit %.2f s): commanding gas %.1f, steer %.1f rad.",
          plan.valid ? age : -1.0, plan_timeout_, kFailsafeGas, kFailsafeSteer);
      }
      return;
    }

    if (failsafe_active_.exchange(false, std::memory_order_relaxed)) {
      RCLCPP_INFO(get_logger(), "Guidance is fresh again; leaving the failsafe.");
    }

    // Carry the geometry forward from the state it was measured at to now.  This
    // is the counterpart of the MPC commander's index into the horizon: both
    // answer "the plan describes a moment that has passed, what does it say
    // about this one", and both measure that gap from the state's own stamp so
    // the estimator's latency is corrected for rather than dropped.
    PidController::Reference ref;
    ref.cross_track = plan.cross_track;
    ref.heading_error = plan.heading_error;
    ref.curvature = plan.curvature;
    ref.target_speed = plan.target_speed;
    ref.valid = true;
    PidController::propagate(ref, car.u, car.v, car.omega, age);

    const PidController::Command cmd = controller_->update(ref, car.u, dt);

    command_msg_.gas = cmd.gas;
    command_msg_.steer_angle = cmd.steer;
    control_pub_->publish(command_msg_);
    echo_.store(CommandEcho{cmd.gas, cmd.steer, true});
  }

  // ----------------------------------------------------------- visualisation

  void initMarkers()
  {
    auto setup = [this](visualization_msgs::msg::Marker & m, const char * ns, int id,
        float r, float g, float b, std::size_t points) {
        m.header.frame_id = work_frame_;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.1;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;
        m.pose.orientation.w = 1.0;
        m.points.resize(points);
        for (auto & p : m.points) {
          p.z = 0.1;
        }
      };

    // The same namespaces, ids and colours the MPC node uses, so an RViz config
    // built for one shows the other without being touched.
    setup(ref_marker_, "local_ref", 0, 1.0f, 0.0f, 0.0f, kRefPoints);
    setup(pred_marker_, "local_pred", 1, 0.0f, 1.0f, 1.0f, kPredictionSteps + 1);
  }

  void broadcastDebug(const Guidance & guidance)
  {
    if (!publish_markers_ || !viz_pub_) {
      return;
    }
    const auto stamp = get_clock()->now();

    ref_marker_.header.stamp = stamp;
    for (int i = 0; i < kRefPoints; ++i) {
      ref_marker_.points[i].x = guidance.points[2 * i];
      ref_marker_.points[i].y = guidance.points[2 * i + 1];
    }
    viz_pub_->publish(ref_marker_);

    // The MPC draws its solved horizon here.  A PID has no horizon, so what is
    // drawn instead is where the kinematic bicycle would go if the current
    // command were held -- which is the closest honest analogue, and reads the
    // same way in RViz: if the cyan line does not curve onto the red one, the
    // gains are wrong.  Only drawn when the commander is actually tracking;
    // rolling out a failsafe brake looks exactly like a live prediction.
    const CommandEcho echo = echo_.load();
    if (!echo.valid) {
      return;
    }
    const CarState car = car_.load();
    const double speed = std::max(std::fabs(car.u), 0.5);
    const double dt = kPredictionHorizon / static_cast<double>(kPredictionSteps);
    const double yaw_rate = speed * std::tan(echo.steer) / controller_->config().wheelbase;

    double x = guidance.pose[0];
    double y = guidance.pose[1];
    double yaw = guidance.pose[2];
    pred_marker_.header.stamp = stamp;
    for (int i = 0; i <= kPredictionSteps; ++i) {
      pred_marker_.points[i].x = x;
      pred_marker_.points[i].y = y;
      x += speed * std::cos(yaw) * dt;
      y += speed * std::sin(yaw) * dt;
      yaw += yaw_rate * dt;
    }
    viz_pub_->publish(pred_marker_);
  }

  void publishHeartbeat()
  {
    if (!heartbeat_pub_) {
      return;
    }
    const auto now = get_clock()->now();

    mirena_common::msg::Heartbeat msg;
    msg.timestamp = now;
    msg.node_name = get_name();

    // Everything below is read from another thread's writes, so it comes from
    // the seqlock or from atomics; the heartbeat never blocks the control loops.
    const int64_t now_ns = now.nanoseconds();
    const CarState car = car_.load();
    const GuidancePlan plan = plan_.load();
    const CommandEcho echo = echo_.load();
    const bool have_track = have_track_.load(std::memory_order_relaxed);
    const bool finished = finished_.load(std::memory_order_relaxed);
    const bool failsafe = failsafe_active_.load(std::memory_order_relaxed);
    const int64_t last_ok_ns = last_guidance_ok_ns_.load(std::memory_order_relaxed);
    const double since_guidance = last_ok_ns == 0 ?
      -1.0 : static_cast<double>(now_ns - last_ok_ns) * 1e-9;
    const double state_age = car.valid ?
      static_cast<double>(now_ns - car.stamp_ns) * 1e-9 : -1.0;
    const bool have_fresh_state = car.valid && state_age <= state_timeout_;
    const double plan_age = plan.valid ?
      static_cast<double>(now_ns - plan.state_stamp_ns) * 1e-9 : -1.0;
    const auto state = static_cast<ControlState>(control_state_.load(std::memory_order_relaxed));

    // Waiting on an input is a different failure from a reference that cannot be
    // built, and both differ again from a loop that cannot run at all -- the
    // supervisor can only act on the difference if the report carries it.  A
    // commander that has fallen back to braking outranks all three, because at
    // that point the car is no longer being steered.
    const char * status_state = toString(state);
    if (finished) {
      status_state = "IDLE";
    } else if (failsafe) {
      status_state = "NOT_WORKING";
    }

    nlohmann::json report;

    // "mission_finished" is the exact key the supervisor ends the mission off:
    // the lap goal is met and the car has come to a stop.  Deciding that is the
    // controller's job alone -- nothing else in the stack may declare a run over
    // -- and every occupant of this slot reports the same key.
    report["mission_finished"] = finished;
    report["mode"] = "pid";
    report["status_state"] = status_state;
    report["progress"] = global_s_.load(std::memory_order_relaxed);
    report["laps"] = laps_done_.load(std::memory_order_relaxed);

    report["laps_target"] = laps_;
    report["finish_armed"] = finish_armed_.load(std::memory_order_relaxed);
    report["stop_requested"] = stop_requested_.load(std::memory_order_relaxed);
    report["have_track"] = have_track;
    report["have_state"] = have_fresh_state;
    report["state_age_s"] = state_age;
    report["speed_mps"] = car.u;

    // The commander's own view, which is the one the car experiences.
    report["failsafe"] = failsafe;
    report["plan_age_s"] = plan_age;

    // The MPC's timing keys, kept under the same names so a dashboard plotting
    // them keeps working across a controller swap; "solve" here is one guidance
    // pass.  Its two acados-only keys, sqp_iterations and max_residual, have no
    // analogue and are deliberately absent rather than reported as zero.
    report["solver_status"] = static_cast<int>(state);
    report["solve_time_ms"] = guidance_time_ms_.load(std::memory_order_relaxed);
    report["solve_latency_ms"] = guidance_latency_ms_.load(std::memory_order_relaxed);
    report["seconds_since_solve"] = since_guidance;

    // What the loops themselves are doing.
    report["cross_track_m"] = cross_track_.load(std::memory_order_relaxed);
    report["heading_error_rad"] = heading_error_.load(std::memory_order_relaxed);
    report["target_speed_mps"] = target_speed_.load(std::memory_order_relaxed);
    report["gas"] = echo.gas;
    report["steer_angle_rad"] = echo.steer;
    report["steer_saturated"] = controller_->steerSaturated();

    msg.report = report.dump();
    heartbeat_pub_->publish(msg);
  }

  // -------------------------------------------------------------------- state

  // Parameters, read once in the constructor.  The mission launch file has
  // already supplied them by then; nothing re-reads them afterwards.
  int laps_ = 0;
  double stop_distance_;
  double min_path_length_;
  double stop_speed_eps_;
  double stop_hold_time_;
  double lookahead_base_;
  double lookahead_gain_;
  std::string robot_frame_;
  std::string work_frame_;
  std::string map_frame_;
  double guidance_period_;
  double command_period_;
  double heartbeat_period_;
  double tf_timeout_;
  double projection_back_;
  double projection_forward_;
  double projection_step_;
  double state_timeout_;
  double plan_timeout_;
  double mission_timeout_;
  int finish_confirm_count_;
  double progress_rate_margin_;
  double progress_rate_slack_;
  bool publish_markers_;
  double track_end_margin_;

  std::unique_ptr<PidController> controller_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr guidance_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr state_group_;
  rclcpp::CallbackGroup::SharedPtr track_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_group_;

  rclcpp::Subscription<mirena_common::msg::Track>::SharedPtr track_sub_;
  rclcpp::Subscription<mirena_common::msg::Car>::SharedPtr car_sub_;

  rclcpp::Publisher<mirena_common::msg::CarControl>::SharedPtr control_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr guidance_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Cross-thread hand-offs.
  Seqlock<CarState> car_{CarState{0, 0.0, 0.0, 0.0, false}};
  Seqlock<GuidancePlan> plan_{GuidancePlan{}};
  Seqlock<CommandEcho> echo_{CommandEcho{}};

  std::mutex track_mutex_;
  std::shared_ptr<const TrackSnapshot> track_;
  std::atomic<uint64_t> track_generation_{0};

  std::atomic<double> global_s_{0.0};
  std::atomic<bool> finish_armed_{false};   // the car has passed s_finish
  std::atomic<bool> finished_{false};       // ...and has since come to a stop
  std::atomic<bool> stop_requested_{false}; // the mission timeout forced a finish
  std::atomic<bool> have_track_{false};

  // Written by the command loop, read by the heartbeat.
  std::atomic<bool> failsafe_active_{false};

  // Written by the guidance loop, read by the heartbeat.
  std::atomic<int> laps_done_{0};
  std::atomic<int> control_state_{static_cast<int>(ControlState::Idle)};
  std::atomic<double> guidance_time_ms_{0.0};
  std::atomic<double> guidance_latency_ms_{0.0};
  std::atomic<double> cross_track_{0.0};
  std::atomic<double> heading_error_{0.0};
  std::atomic<double> target_speed_{0.0};
  std::atomic<int64_t> last_guidance_ok_ns_{0};

  // Track-thread only.
  std::vector<double> fit_scratch_;

  // Guidance-thread only.
  int64_t stop_since_ns_ = kUnsetTime;
  int64_t motion_start_ns_ = kUnsetTime;
  int64_t last_projection_ns_ = kUnsetTime;
  uint64_t last_track_generation_ = 0;
  double last_s_max_ = 0.0;
  int finish_confirm_ = 0;

  // Command-thread only.
  int64_t last_command_ns_ = kUnsetTime;

  // Reused outgoing messages, so the hot loops do not allocate.
  mirena_common::msg::CarControl command_msg_;
  visualization_msgs::msg::Marker ref_marker_;
  visualization_msgs::msg::Marker pred_marker_;
};

}  // namespace mirena

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<mirena::PidNode> node;
  try {
    node = std::make_shared<mirena::PidNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mirena_control"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // One thread per mutually-exclusive callback group, so the commander's
  // deadline never waits on a guidance pass, a spline fit or a heartbeat.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  executor.add_node(node);

  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Executor terminated: %s", e.what());
  }

  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
