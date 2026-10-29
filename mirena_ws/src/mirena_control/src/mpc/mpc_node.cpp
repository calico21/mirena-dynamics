// Model-predictive contouring controller for the Mirena driverless car.
//
// Four loops run concurrently on their own callback groups:
//   * 20 Hz  solver     -- projects the car onto the planned centreline, builds
//                          the local reference and runs the acados NLP;
//   * 100 Hz commander  -- replays the latest optimal input horizon at the
//                          wall-clock offset reached since the state it was
//                          solved from was measured;
//   * 10 Hz  heartbeat  -- reports liveness and progress to the supervisor;
//   * on demand         -- the Track subscription, which refits the centreline.
//
// The commander is the loop with the hard deadline, so it never takes a lock
// that the solver can hold: the solved horizon reaches it through a seqlock.
//
// The commander is also the last line of defence.  Every reason the solver has
// to go quiet -- no state, a stale state, no track, a TF dropout, a solve that
// will not converge -- ends with it holding a horizon that is no longer about
// the situation the car is in.  Rather than replay the tail of that horizon
// forever, it falls back to full braking with the wheels straight once the plan
// is older than plan_timeout.

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

#include "mpc_controller.hpp"

namespace mirena
{

using Mpc = MpcController;

namespace
{

constexpr int kPathPoints = Mpc::kPathPoints;
constexpr int kHorizon = Mpc::kN;

/// What the commander publishes when it has nothing fresh to replay: full
/// braking, wheels straight.  Not a tuning knob -- it is the one command that
/// is safe whatever the reason the solver stopped talking.
constexpr double kFailsafeGas = -1.0;
constexpr double kFailsafeSteer = 0.0;

/// Sentinel for "this timestamp has not been set yet".  Zero will not do: under
/// sim time the clock legitimately starts at zero, so a zero sentinel makes the
/// first moment of a simulated run indistinguishable from never.
constexpr int64_t kUnsetTime = std::numeric_limits<int64_t>::min();

/// Topic naming follows the stack's convention: the interfaces shared with the
/// vehicle and with other packages are absolute, so a namespaced control node
/// still talks to the one CAN bridge and the one state estimator, while `health`
/// stays relative because the supervisor matches heartbeats by node name.
constexpr char kTrackTopic[] = "/planning/track";
constexpr char kCarStateTopic[] = "/state/car";
constexpr char kControlTopic[] = "/control";
constexpr char kHealthTopic[] = "health";

/// What the heartbeat says about the control loop.  The supervisor and the
/// monitor act on the difference, so waiting for an input, a solver that runs
/// but will not converge, and a solver that cannot run at all are kept apart.
enum class ControlState : int
{
  Idle = 0,      // waiting on the track or on a fresh state estimate
  Working = 1,   // the last solve produced a horizon that was accepted
  Partial = 2,   // the solver ran and its result was rejected
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

/// Everything the solver needs from the state estimate.  Kept as a small POD so
/// the subscription callback can publish it without allocating.
struct CarState
{
  int64_t stamp_ns;
  double u;
  double v;
  double omega;
  bool valid;
};

/// The slice of the solution the 100 Hz commander replays.
///
/// The horizon starts at the state it was solved from, not at the instant the
/// solver happened to begin work, so `state_stamp_ns` is the measurement's own
/// stamp.  Replaying from anything later would build the estimator's latency
/// into the command as a lag.
struct CommandPlan
{
  int64_t state_stamp_ns;
  bool valid;
  double gas[kHorizon];
  double steer[kHorizon];
};

/// Immutable centreline snapshot, swapped in wholesale on each Track message.
struct TrackSnapshot
{
  CubicSpline2D spline;
  bool closed = false;
  uint64_t generation = 0;   // bumped per accepted fit; see generateReference
};

}  // namespace

class MpcNode : public rclcpp::Node
{
public:
  MpcNode()
  : rclcpp::Node("mirena_control")
  {
    // The mission's values arrive as launch arguments from the mission launch
    // file the supervisor started, so they are already in place by the time the
    // constructor reads them below.  Nothing here is re-read afterwards and
    // there is no parameter callback: a node's lifetime is the mission's, so a
    // parameter that changes mid-run has nowhere sensible to take effect.
    declare_parameter<int>("laps", 0);
    declare_parameter<double>("stop_distance", 20.0);
    declare_parameter<double>("min_path_length", 5.0);
    declare_parameter<double>("stop_speed_eps", 0.3);
    declare_parameter<double>("stop_hold_time", 0.5);
    declare_parameter<double>("lookahead_base", 15.0);
    // Raising lookahead_gain raises cruise speed (horizon equilibrium is
    // 2u ~= lookahead_base + lookahead_gain*u) but braking distance grows as u^2
    // while the truncation onset only grows as lookahead_gain*u. Past ~2.5 the
    // car can no longer stop inside the reference that is left to it.
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
    // Safety envelope: how stale an input or a plan may be before the loop that
    // consumes it stops trusting it.
    declare_parameter<double>("state_timeout", 0.15);
    declare_parameter<double>("plan_timeout", 0.25);
    declare_parameter<double>("mission_timeout", 600.0);
    declare_parameter<int>("finish_confirm_count", 3);
    declare_parameter<double>("progress_rate_margin", 1.5);
    declare_parameter<double>("progress_rate_slack", 0.5);
    declare_parameter<double>("max_accepted_residual", 0.1);
    declare_parameter<bool>("publish_markers", true);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // A dedicated listener thread keeps /tf ingestion off the solver's
    // callback group and is what makes a bounded canTransform() legal here.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    RCLCPP_INFO(get_logger(), "Starting local-frame control node...");

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
    solver_period_ = get_parameter("solver_period").as_double();
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
    max_accepted_residual_ = get_parameter("max_accepted_residual").as_double();
    publish_markers_ = get_parameter("publish_markers").as_bool();

    if (laps_ > 0) {
      RCLCPP_INFO(
        get_logger(), "Lap mission: %d laps, stopping %.1f m past the final crossing.",
        laps_, stop_distance_);
    } else {
      RCLCPP_INFO(get_logger(), "Open-track mission: stopping at the last gate.");
    }
    RCLCPP_INFO(
      get_logger(),
      "Failsafe: gas %.1f, steer %.1f rad, after %.2f s without a plan "
      "(state must be fresher than %.2f s).",
      kFailsafeGas, kFailsafeSteer, plan_timeout_, state_timeout_);

    mpc_ = std::make_unique<Mpc>();   // throws; main() reports and exits non-zero
    step_duration_ = mpc_->stepDuration();
    RCLCPP_INFO(
      get_logger(), "acados solver ready: %d states, %d inputs, %d steps of %.3f s",
      Mpc::kNx, Mpc::kNu, kHorizon, step_duration_);

    solver_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    track_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // The heartbeat gets a group of its own rather than sharing the track's:
    // refitting the centreline is the longest callback in the node, and the
    // supervisor declares a node dead on a few seconds of silence.  Liveness
    // must not queue behind a spline fit.
    heartbeat_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions state_opts;
    state_opts.callback_group = state_group_;
    rclcpp::SubscriptionOptions track_opts;
    track_opts.callback_group = track_group_;

    track_sub_ = create_subscription<mirena_common::msg::Track>(
      kTrackTopic, 10,
      std::bind(&MpcNode::trackCallback, this, std::placeholders::_1), track_opts);
    car_sub_ = create_subscription<mirena_common::msg::Car>(
      kCarStateTopic, 10,
      std::bind(&MpcNode::carCallback, this, std::placeholders::_1), state_opts);

    control_pub_ = create_publisher<mirena_common::msg::CarControl>(kControlTopic, 10);
    viz_pub_ = create_publisher<visualization_msgs::msg::Marker>("control/horizon_markers", 10);
    heartbeat_pub_ = create_publisher<mirena_common::msg::Heartbeat>(kHealthTopic, 10);

    initMarkers();

    // Nothing has been solved yet, so the commander starts in its failsafe.
    // Publishing full braking before the first solution is the right thing to
    // be holding if the car reaches DRIVING while the pipeline is still coming
    // up -- it is what the car would get anyway if the solver never arrived.
    plan_.store(CommandPlan{});

    heartbeat_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(heartbeat_period_),
      std::bind(&MpcNode::publishHeartbeat, this), heartbeat_group_);
    solver_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(solver_period_),
      std::bind(&MpcNode::solverIteration, this), solver_group_);
    command_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(command_period_),
      std::bind(&MpcNode::commandIteration, this), command_group_);
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

    // Tagged so the solver can tell one centreline from the next: a new fit can
    // have a different arc length and a different origin, and the progress
    // accumulated against the old one is not directly comparable.
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
    // through the stationary test in updateStopLatch as "not moving" and end
    // the mission, as well as reaching acados inside x0.  The last good state
    // is kept instead and simply ages out if nothing better arrives.
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

  /// Local reference in the work frame, plus the pose the solver starts from.
  struct Reference
  {
    double points[2 * kPathPoints];
    double length;         // arc length the points span, the solver's path_length
    double offset;         // where the car sits inside that window, in metres
    double start;          // window origin, in the same unwrapped arc length as progress
    double pose[3];        // x, y, yaw in the work frame
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

  bool generateReference(const CarState & car, Reference & ref)
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
      // All three chains are waited on, not just the first: map->car is what the
      // projection needs, but the pose the solver starts from comes from
      // odom->car and the reference is rotated into the work frame with
      // odom->map.  Checking one and looking up the other two would turn a
      // missing transform into a caught exception instead of a bounded wait.
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
      ref.pose[0] = t_odom_car.transform.translation.x;
      ref.pose[1] = t_odom_car.transform.translation.y;
      ref.pose[2] = yawFromQuaternion(t_odom_car.transform.rotation);

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

      // Where the run ends, in the same unwrapped arc length as progress.  A
      // lap mission needs the planner to have closed the loop before s_max
      // means "one lap"; until then nothing is truncated and the car keeps
      // driving.  An open path simply ends at its last gate.  The timeout is
      // the backstop for a lap mission whose loop never closes -- see
      // updateMissionTimeout.
      const bool stop_requested = stop_requested_.load(std::memory_order_relaxed);
      const bool finish_active = stop_requested || (laps_ == 0) || track->closed;
      double s_finish;
      double s_stop;
      if (stop_requested) {
        // Stop where the car is: collapse the window to its floor around the
        // current progress and let the solver brake into it, exactly as it
        // would at a finish it recognised.
        s_finish = progress;
        s_stop = progress;
      } else if (laps_ > 0) {
        s_finish = static_cast<double>(laps_) * s_max;
        // Capped so a stop_distance >= one lap cannot fold the stop point onto
        // the following lap when the sampling below wraps.
        s_stop = s_finish + std::min(stop_distance_, 0.5 * s_max);
      } else {
        s_finish = s_max;
        s_stop = s_max;
      }
      s_stop = std::max(s_stop, min_path_length_);

      if (finish_active) {
        updateLapCount(progress, s_max, track->closed);
        updateFinishLatch(progress, s_finish);
      }

      // Cut the reference at s_stop by sliding the whole window back, never by
      // clamping individual samples: the solver evaluates the reference at
      // p_norm = p / path_length over a uniform 20-point grid, so bunching the
      // tail would stop p meaning arc length exactly in the braking region, and
      // a fully collapsed window leaves the progress reward -0.08*vp with no
      // curvature to oppose it.  Sliding keeps the samples equispaced and
      // path_length honest, and simply stops the reference advancing -- which
      // is what makes the MPC decelerate of its own accord.
      const double lookahead = lookahead_base_ + car.u * lookahead_gain_;
      double ref_start = progress;
      double ref_len = lookahead;
      if (finish_active) {
        ref_start = std::max(std::min(progress, s_stop - min_path_length_), 0.0);
        ref_len = std::max(std::min(lookahead, s_stop - ref_start), min_path_length_);
      }
      ref.length = ref_len;
      ref.start = ref_start;
      ref.offset = std::min(std::max(progress - ref_start, 0.0), ref_len);

      // Sample the centreline over that window, in map coordinates.
      const double span = ref_len / static_cast<double>(kPathPoints - 1);
      for (int i = 0; i < kPathPoints; ++i) {
        double s = ref_start + span * static_cast<double>(i);
        if (track->closed) {
          s = std::fmod(s, s_max);
          if (s < 0.0) {
            s += s_max;
          }
        } else {
          s = std::min(std::max(s, 0.0), s_max);
        }
        spline.eval(s, ref.points[2 * i], ref.points[2 * i + 1]);
      }

      // ...then rotate them into the work frame in one pass.
      const auto t_odom_map = tf_buffer_->lookupTransform(work_frame_, map_frame_, target_time);
      const double yaw = yawFromQuaternion(t_odom_map.transform.rotation);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      const double tx = t_odom_map.transform.translation.x;
      const double ty = t_odom_map.transform.translation.y;
      for (int i = 0; i < kPathPoints; ++i) {
        const double px = ref.points[2 * i];
        const double py = ref.points[2 * i + 1];
        ref.points[2 * i] = c * px - s * py + tx;
        ref.points[2 * i + 1] = s * px + c * py + ty;
      }

      return true;

    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF lookup failed in generateReference: %s", e.what());
      return false;
    }
  }

  /// Project the car onto the centreline and fold the result into the run's
  /// cumulative progress.  Solver thread only.
  double updateProgress(
    const TrackSnapshot & track, double s_max, bool wrap, const CarState & car,
    double car_map_x, double car_map_y, int64_t now_ns)
  {
    // Always seeded from the progress we already believe in, including the
    // very first call, where it is the 0 the members start at -- the car starts
    // on the line.  A global nearest-point sweep must not be used as a
    // substitute: the unrolled skidpad path overlaps itself exactly, so the
    // nearest point is ambiguous between passes and can jump a whole circle.
    double seed = global_s_.load(std::memory_order_relaxed);

    // A replanned centreline is a new curve with its own arc length, and the
    // planner emits one every time it refines the track -- several times a
    // second while the layout is still being discovered.  Only the completed
    // laps are re-expressed in the new curve's metres: the position within the
    // current lap is measured from the start gate, which does not move, so a
    // track that simply grew at its far end must not drag it along.  Whatever
    // the seed is still off by, the projection resolves inside its window.
    // Only on a closed track.  On an open one s_max is not a lap, it is just how
    // much has been mapped so far, and dividing by it is meaningless: a car
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
    // may only move as fast as the car could actually have travelled.  A
    // genuine correction still gets there, just over several frames.  This
    // holds across a replanned track too -- a refit is not licence to jump,
    // which is exactly the mistake of letting a new generation bypass it while
    // the planner republishes at several hertz.
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

  // ------------------------------------------------------------- solver loop

  void solverIteration()
  {
    if (!mpc_) {
      return;
    }

    const CarState car = car_.load();
    const int64_t now_ns = get_clock()->now().nanoseconds();

    // A state estimate that stopped arriving stays in the seqlock forever and
    // reads as perfectly valid, so freshness is what gates the solver rather
    // than the valid flag alone.  Everything downstream -- the TF lookups at
    // the state's stamp included -- assumes the car really was there recently.
    if (!car.valid) {
      control_state_.store(static_cast<int>(ControlState::Idle), std::memory_order_relaxed);
      return;
    }
    const double state_age = static_cast<double>(now_ns - car.stamp_ns) * 1e-9;
    if (state_age > state_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "State estimate is %.2f s old (limit %.2f s); holding the solver off.",
        state_age, state_timeout_);
      control_state_.store(static_cast<int>(ControlState::Idle), std::memory_order_relaxed);
      return;
    }
    if (!have_track_.load(std::memory_order_relaxed)) {
      control_state_.store(static_cast<int>(ControlState::Idle), std::memory_order_relaxed);
      return;
    }

    // Ahead of generateReference, so a TF dropout on the final metres cannot
    // stop the run from ever being declared finished.
    updateStopLatch(car, now_ns);
    updateMissionTimeout(car, now_ns);

    Reference ref;
    if (!generateReference(car, ref)) {
      // Inputs are there but the reference is not, which is a different fault
      // from waiting on an input and the supervisor is told so.
      control_state_.store(static_cast<int>(ControlState::Starved), std::memory_order_relaxed);
      return;
    }

    // The reference window slides forward with the car, so the p states carried
    // over from the previous solve are measured from an origin that has since
    // moved.  Re-base them before acados uses them as its warm start.  The
    // shift lands whatever the solve then does with it -- both the iterate read
    // back and the horizon left untouched by a failure are expressed against
    // this window -- so the origin is recorded unconditionally.
    if (have_horizon_ref_) {
      mpc_->shiftProgress(ref.start - horizon_ref_start_);
    }
    horizon_ref_start_ = ref.start;
    have_horizon_ref_ = true;

    const auto & previous = mpc_->states();
    double x0[Mpc::kNx];
    x0[0] = ref.pose[0];
    x0[1] = ref.pose[1];
    x0[2] = ref.pose[2];
    x0[3] = car.u;
    x0[4] = car.v;
    x0[5] = car.omega;

    // The solver reruns every solver_period but the horizon steps at
    // step_duration, and the two are not the same number.  The actuator state
    // to start from is therefore not the previous horizon's second node, it is
    // the point between the first two that the elapsed time actually reached.
    // Taking node 1 outright would over-advance the actuator by the difference
    // every single solve, which at the steering rate limit is several degrees.
    const double frac = actuatorCarryFraction(car.stamp_ns);
    x0[6] = previous[0][6] + frac * (previous[1][6] - previous[0][6]);
    x0[7] = previous[0][7] + frac * (previous[1][7] - previous[0][7]);
    x0[8] = ref.offset;          // where the car sits inside the reference window
    x0[9] = car.u;

    double params[Mpc::kNpGlobal];
    for (int i = 0; i < kPathPoints; ++i) {
      params[i] = ref.points[2 * i];
      params[kPathPoints + i] = ref.points[2 * i + 1];
    }
    params[2 * kPathPoints] = ref.length;

    const int status = mpc_->solve(x0, params);
    const int64_t t_end = get_clock()->now().nanoseconds();
    const Mpc::Stats stats = mpc_->stats();

    // Converging is not the only useful outcome.  A solve that ran out of
    // iterations or out of its wall-clock budget still leaves a coherent
    // horizon behind, and one of those is a far better command than replaying
    // a horizon several solves old -- provided its residual says it is close.
    // Anything else, or a residual that says otherwise, is thrown away and the
    // solver's internal state is discarded before the next attempt.
    const bool accepted = status == 0 ||
      (Mpc::hasIterate(status) && max_accepted_residual_ > 0.0 &&
      std::isfinite(stats.max_residual) && stats.max_residual <= max_accepted_residual_);

    if (accepted) {
      const auto & states = mpc_->states();
      CommandPlan plan;
      plan.state_stamp_ns = car.stamp_ns;
      plan.valid = true;
      for (int i = 0; i < kHorizon; ++i) {
        plan.gas[i] = states[i][6];
        plan.steer[i] = states[i][7];
      }
      plan_.store(plan);
    } else {
      mpc_->reset();
    }

    logDiagnostics(status, accepted, stats, t_end, car.stamp_ns);
    broadcastDebug(ref, accepted);
  }

  /// How far into the previous horizon's first interval the next solve starts,
  /// as a fraction of one shooting interval.  Measured from the state stamps
  /// rather than assumed from solver_period, so jitter in the loop corrects
  /// itself.  Solver thread only.
  double actuatorCarryFraction(int64_t state_stamp_ns)
  {
    double advance = solver_period_;
    if (last_state_stamp_ns_ != kUnsetTime) {
      const double measured =
        static_cast<double>(state_stamp_ns - last_state_stamp_ns_) * 1e-9;
      if (measured > 0.0 && measured < step_duration_ * kHorizon) {
        advance = measured;
      }
    }
    last_state_stamp_ns_ = state_stamp_ns;
    if (!(step_duration_ > 0.0)) {
      return 0.0;
    }
    return std::min(std::max(advance / step_duration_, 0.0), 1.0);
  }

  void logDiagnostics(
    int status, bool accepted, const Mpc::Stats & s, int64_t end_ns, int64_t car_stamp_ns)
  {
    solver_status_.store(status, std::memory_order_relaxed);
    solve_time_ms_.store(s.solve_time_ms, std::memory_order_relaxed);
    solve_iterations_.store(s.sqp_iterations, std::memory_order_relaxed);
    solve_residual_.store(s.max_residual, std::memory_order_relaxed);

    if (!accepted) {
      control_state_.store(static_cast<int>(ControlState::Partial), std::memory_order_relaxed);
      // Throttled: once the car is told to stop the reference stops tracking it
      // and every solve fails, which at 20 Hz would otherwise flood the log.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 500,
        "MPC solve rejected: status %d, residual %.1e", status, s.max_residual);
      return;
    }

    const double latency_ms = static_cast<double>(end_ns - car_stamp_ns) * 1e-6;
    solve_latency_ms_.store(latency_ms, std::memory_order_relaxed);
    last_solve_ok_ns_.store(end_ns, std::memory_order_relaxed);
    control_state_.store(static_cast<int>(ControlState::Working), std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "LOCAL OK | Solve: %4.1fms | Lat: %4.1fms | Iters: %d | Res: %.1e | Status: %d",
      s.solve_time_ms, latency_ms, s.sqp_iterations, s.max_residual, status);
  }

  /// Laps come straight from how far the car has travelled along the
  /// centreline; the spline is anchored at the start gate, so s == 0 (mod
  /// s_max) is the finish line by construction.
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

  /// Arming the finish truncates the reference for the rest of the run, so it
  /// is not done on the strength of a single frame: the projection has to agree
  /// finish_confirm_count times running.  One optimistic frame -- a residual
  /// map correction the rate limiter let through, a gate that moved -- would
  /// otherwise stop the car mid-track with no way back.  Solver thread only.
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
      get_logger(), "Finish reached at s = %.1f m; truncating the reference to stop.", progress);
  }

  /// The run is only reported finished once the car is actually stationary.
  /// The supervisor tears the stack down the moment it sees the flag, so
  /// latching any earlier would cut the controller off mid-braking.
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

  /// A lap mission only truncates its reference once the planner has closed the
  /// loop, because until then s_max is not a lap.  If the loop never closes --
  /// SLAM never gets the match, the track is mapped but never joined -- nothing
  /// in the geometry can ever end the run and the car keeps driving.  This is
  /// the backstop: once the car has been moving for mission_timeout seconds the
  /// finish is forced wherever it happens to be, and the same truncated
  /// reference brings it to a stop.  Solver thread only.
  void updateMissionTimeout(const CarState & car, int64_t now_ns)
  {
    if (mission_timeout_ <= 0.0 || stop_requested_.load(std::memory_order_relaxed)) {
      return;
    }
    // The clock starts when the car does, not at launch: the node comes up on
    // AS READY and may sit there for minutes before the car is allowed to move.
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
    // is a solution. The car is what decides whether to act on them -- it
    // ignores this node until its own state machine reaches DRIVING.
    const auto now = get_clock()->now();
    command_msg_.header.stamp = now;

    // No finish override lives here on purpose: bringing the car to a stop is
    // the job of the truncated reference, which the solver brakes against on
    // its own.
    const CommandPlan plan = plan_.load();

    // ...but a plan that has stopped being refreshed is a different matter. The
    // index below clamps, so an unchecked stale plan would leave the car
    // holding the last gas and steering angle of a horizon that has no bearing
    // on where it now is, for as long as the node runs. Whatever silenced the
    // solver -- no state, no track, no TF, no convergence -- the answer is the
    // same: brake, wheels straight.
    bool usable = plan.valid;
    double age = 0.0;
    if (usable) {
      age = static_cast<double>(now.nanoseconds() - plan.state_stamp_ns) * 1e-9;
      usable = age <= plan_timeout_;
    }

    if (!usable) {
      command_msg_.gas = kFailsafeGas;
      command_msg_.steer_angle = kFailsafeSteer;
      control_pub_->publish(command_msg_);
      if (!failsafe_active_.exchange(true, std::memory_order_relaxed)) {
        RCLCPP_ERROR(
          get_logger(),
          "No usable plan (age %.2f s, limit %.2f s): commanding gas %.1f, steer %.1f rad.",
          plan.valid ? age : -1.0, plan_timeout_, kFailsafeGas, kFailsafeSteer);
      }
      return;
    }

    if (failsafe_active_.exchange(false, std::memory_order_relaxed)) {
      RCLCPP_INFO(get_logger(), "Plan is fresh again; leaving the failsafe.");
    }

    // The horizon is anchored at the state it was solved from, so this is the
    // true offset into it -- estimator latency included, rather than dropped.
    int index = static_cast<int>(age / step_duration_);
    index = std::min(std::max(index, 0), kHorizon - 1);

    command_msg_.gas = plan.gas[index];
    command_msg_.steer_angle = plan.steer[index];
    control_pub_->publish(command_msg_);
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

    setup(ref_marker_, "local_ref", 0, 1.0f, 0.0f, 0.0f, kPathPoints);
    setup(pred_marker_, "local_pred", 1, 0.0f, 1.0f, 1.0f, kHorizon + 1);
  }

  void broadcastDebug(const Reference & ref, bool solution_valid)
  {
    if (!publish_markers_ || !viz_pub_) {
      return;
    }
    const auto stamp = get_clock()->now();

    ref_marker_.header.stamp = stamp;
    for (int i = 0; i < kPathPoints; ++i) {
      ref_marker_.points[i].x = ref.points[2 * i];
      ref_marker_.points[i].y = ref.points[2 * i + 1];
    }
    viz_pub_->publish(ref_marker_);

    // Only when the horizon is one the car is actually being driven from --
    // drawing a rejected or stale prediction reads exactly like a live one.
    if (!solution_valid) {
      return;
    }
    const auto & states = mpc_->states();
    pred_marker_.header.stamp = stamp;
    for (int i = 0; i <= kHorizon; ++i) {
      pred_marker_.points[i].x = states[i][0];
      pred_marker_.points[i].y = states[i][1];
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
    // the seqlock or from atomics; the heartbeat never blocks the solver.
    const int64_t now_ns = now.nanoseconds();
    const CarState car = car_.load();
    const CommandPlan plan = plan_.load();
    const bool have_track = have_track_.load(std::memory_order_relaxed);
    const bool finished = finished_.load(std::memory_order_relaxed);
    const bool failsafe = failsafe_active_.load(std::memory_order_relaxed);
    const int solver_status = solver_status_.load(std::memory_order_relaxed);
    const int64_t last_ok_ns = last_solve_ok_ns_.load(std::memory_order_relaxed);
    const double since_solve = last_ok_ns == 0 ?
      -1.0 : static_cast<double>(now_ns - last_ok_ns) * 1e-9;
    const double state_age = car.valid ?
      static_cast<double>(now_ns - car.stamp_ns) * 1e-9 : -1.0;
    const bool have_fresh_state = car.valid && state_age <= state_timeout_;
    const double plan_age = plan.valid ?
      static_cast<double>(now_ns - plan.state_stamp_ns) * 1e-9 : -1.0;
    const auto state = static_cast<ControlState>(control_state_.load(std::memory_order_relaxed));

    // Waiting on an input is a different failure from a solver that runs but
    // will not converge, and both differ again from a solver that cannot run at
    // all -- the supervisor can only act on the difference if the report
    // carries it.  A commander that has fallen back to braking outranks all
    // three, because at that point the car is no longer being steered.
    const char * status_state = toString(state);
    if (finished) {
      status_state = "IDLE";
    } else if (failsafe) {
      status_state = "NOT_WORKING";
    }

    nlohmann::json report;

    // "mission_finished" is the exact key the supervisor ends the mission off:
    // the lap goal is met and the car has come to a stop. Deciding that is this
    // node's job alone -- nothing else in the stack may declare a run over --
    // and the inspection node reports the same key when its sweep completes.
    report["mission_finished"] = finished;
    report["mode"] = "mpc";
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

    report["solver_status"] = solver_status;
    report["solve_time_ms"] = solve_time_ms_.load(std::memory_order_relaxed);
    report["solve_latency_ms"] = solve_latency_ms_.load(std::memory_order_relaxed);
    report["sqp_iterations"] = solve_iterations_.load(std::memory_order_relaxed);
    report["max_residual"] = solve_residual_.load(std::memory_order_relaxed);
    report["seconds_since_solve"] = since_solve;

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
  double solver_period_;
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
  double max_accepted_residual_;
  bool publish_markers_;

  std::unique_ptr<Mpc> mpc_;
  double step_duration_ = 0.1;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr solver_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr state_group_;
  rclcpp::CallbackGroup::SharedPtr track_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_group_;

  rclcpp::Subscription<mirena_common::msg::Track>::SharedPtr track_sub_;
  rclcpp::Subscription<mirena_common::msg::Car>::SharedPtr car_sub_;

  rclcpp::Publisher<mirena_common::msg::CarControl>::SharedPtr control_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr solver_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Cross-thread hand-offs.
  Seqlock<CarState> car_{CarState{0, 0.0, 0.0, 0.0, false}};
  Seqlock<CommandPlan> plan_{CommandPlan{}};

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

  // Written by the solver, read by the heartbeat: the last solve's outcome.
  std::atomic<int> laps_done_{0};
  std::atomic<int> control_state_{static_cast<int>(ControlState::Idle)};
  std::atomic<int> solver_status_{-1};      // acados status, -1 until the first solve
  std::atomic<int> solve_iterations_{0};
  std::atomic<double> solve_time_ms_{0.0};
  std::atomic<double> solve_latency_ms_{0.0};
  std::atomic<double> solve_residual_{0.0};
  std::atomic<int64_t> last_solve_ok_ns_{0};

  // Track-thread only.
  std::vector<double> fit_scratch_;

  // Solver-thread only.
  int64_t stop_since_ns_ = kUnsetTime;
  int64_t motion_start_ns_ = kUnsetTime;
  int64_t last_projection_ns_ = kUnsetTime;
  int64_t last_state_stamp_ns_ = kUnsetTime;
  uint64_t last_track_generation_ = 0;
  double last_s_max_ = 0.0;
  double horizon_ref_start_ = 0.0;   // window the stored horizon's p is measured from
  bool have_horizon_ref_ = false;
  int finish_confirm_ = 0;

  // Reused outgoing messages, so the hot loops do not allocate.
  mirena_common::msg::CarControl command_msg_;
  visualization_msgs::msg::Marker ref_marker_;
  visualization_msgs::msg::Marker pred_marker_;
};

}  // namespace mirena

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<mirena::MpcNode> node;
  try {
    node = std::make_shared<mirena::MpcNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mirena_control"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // One thread per mutually-exclusive callback group, so the commander's
  // deadline never waits on a solve, a spline fit or a heartbeat for a runner.
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
