// Inspection controller for the Mirena driverless car.
//
// The static inspection mission: the car is on stands, the steering is swept
// with a sine and the drivetrain is held at a target wheel speed, for a fixed
// window, to demonstrate that both actuators respond. Nothing is planned and
// nothing is solved, so this node replaces the MPC one for the length of that
// mission -- it is the alternative occupant of the same slot in the stack,
// which is why it carries the same default node name and publishes the same
// /control and health topics.
//
// The run is a braked hold of inspection_start_delay from the moment the car
// reaches DRIVING, and then the sweep. The hold is not charged to the sweep, so
// the demonstration is still a full inspection_duration of movement.
//
// The drivetrain runs closed-loop. The rear wheel speeds arrive from the CAN
// bridge on /sensors/wheel_speeds in RPM, and their mean is the feedback of a
// PID whose output is gas. What the scrutineers are shown is therefore a wheel
// speed that is asked for and reached, rather than a throttle opening that
// happens to produce one: inspection_target_rpm is the number on the sheet, and
// it holds whatever the drivetrain's losses on the day turn out to be.
//
// Three loops run on their own callback groups:
//   * 100 Hz command    -- publishes the swept steering and the PID's gas;
//   * on demand         -- the wheel-speed subscription;
//   * 10 Hz  heartbeat  -- reports liveness and progress to the supervisor.
//
// The wheel speed is the only thing standing between the loop and a drivetrain
// commanded open-loop, so it is treated the way the other controllers treat a
// state estimate: an integrating loop fed a measurement that stopped arriving
// winds up and pins the throttle, on a car whose wheels are off the ground. A
// measurement older than wheel_speed_timeout therefore stops the loop, brakes
// and drops the integrator, and the sweep runs on so the mission still ends.
//
// Like the other controllers, this one is the sole authority on when its run is
// over: it latches mission_finished in the heartbeat and the supervisor ends
// the mission off that flag. It does not latch the moment the window closes,
// though -- it brakes for inspection_brake_duration first, so the wheels are
// stopped rather than still spinning when the supervisor tears the stack down.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include "mirena_common/msg/as_info.hpp"
#include "mirena_common/msg/car_control.hpp"
#include "mirena_common/msg/heartbeat.hpp"
#include "mirena_common/msg/wheel_speeds.hpp"

#include "common/pid.hpp"
#include "common/seqlock.hpp"

namespace mirena
{

namespace
{

/// Sentinel for "this has not happened yet".  Zero will not do: under sim time
/// the clock legitimately starts at zero, so a zero sentinel would make a sweep
/// that began at t = 0 look like one that never began.
constexpr int64_t kUnsetTime = std::numeric_limits<int64_t>::min();

/// Held once the window closes, and whenever the loop has nothing it can
/// safely close on: wheels straight, full braking.
constexpr double kFailsafeGas = -1.0;
constexpr double kFailsafeSteer = 0.0;

/// Topic naming follows the stack's convention -- see the MPC node.
constexpr char kAsStatusTopic[] = "/as_status";
constexpr char kWheelSpeedTopic[] = "/sensors/wheel_speeds";
constexpr char kControlTopic[] = "/control";
constexpr char kHealthTopic[] = "health";

/// The rear wheel speeds as last heard, kept as a small POD so the
/// subscription callback can publish it without allocating.
struct WheelState
{
  int64_t stamp_ns;
  double rl;        // RPM
  double rr;        // RPM
  double mean;      // RPM, the loop's feedback
  bool valid;
};

}  // namespace


class InspectionNode : public rclcpp::Node
{
public:
  InspectionNode()
  : rclcpp::Node("mirena_control")
  {
    // The mission's values arrive as launch arguments from the mission launch
    // file the supervisor started, so they are already in place by the time the
    // constructor reads them below.  Nothing re-reads them afterwards: a node's
    // lifetime is the mission's, so a parameter that changes mid-sweep has
    // nowhere sensible to take effect.
    declare_parameter<double>("inspection_frequency", 0.5);   // Hz
    declare_parameter<double>("inspection_amplitude", 0.3);   // rad, ~17 deg
    declare_parameter<double>("inspection_duration", 25.0);   // s
    // Braked hold between DRIVING and the first sweep command.  DRIVING is
    // asserted the moment the state machine reaches it, which is not the moment
    // the car is ready to be driven: the interface is still coming up and the
    // scrutineers are still watching for the sweep to begin from rest.  Sweeping
    // into that window is what this holds off.
    declare_parameter<double>("inspection_start_delay", 3.0); // s

    // The wheel-speed loop.  The target is the number the scrutineers read off
    // the sheet; everything else shapes how the drivetrain gets there.
    declare_parameter<double>("inspection_target_rpm", 200.0);
    declare_parameter<double>("inspection_rpm_kp", 0.0015);
    declare_parameter<double>("inspection_rpm_ki", 0.0020);
    declare_parameter<double>("inspection_rpm_kd", 0.0001);
    declare_parameter<double>("inspection_rpm_tau_d", 0.08);
    // The loop's authority.  Deliberately short of full: the car is on stands
    // with no road load, so the speed the drivetrain reaches for a given gas is
    // far higher than on the ground and there is no reason to hand the loop the
    // whole pedal to find that out with.
    declare_parameter<double>("inspection_gas_max", 0.5);
    declare_parameter<double>("inspection_gas_min", -0.5);
    declare_parameter<double>("inspection_gas_rate", 2.0);    // per second
    // Gas held once the window closes, and whenever the loop cannot run.
    declare_parameter<double>("inspection_brake_gas", -1.0);
    // Braking between the end of the sweep and mission_finished, so the wheels
    // are down before the supervisor acts on the flag.
    declare_parameter<double>("inspection_brake_duration", 5.0);   // s
    // How stale the wheel speed may be before the loop stops trusting it.
    declare_parameter<double>("wheel_speed_timeout", 0.2);

    declare_parameter<double>("command_period", 0.01);
    declare_parameter<double>("heartbeat_period", 0.1);

    RCLCPP_INFO(get_logger(), "Starting inspection control node...");

    frequency_ = get_parameter("inspection_frequency").as_double();
    amplitude_ = get_parameter("inspection_amplitude").as_double();
    duration_ = get_parameter("inspection_duration").as_double();
    start_delay_ = std::max(get_parameter("inspection_start_delay").as_double(), 0.0);
    target_rpm_ = get_parameter("inspection_target_rpm").as_double();
    gas_max_ = get_parameter("inspection_gas_max").as_double();
    gas_min_ = get_parameter("inspection_gas_min").as_double();
    gas_rate_ = std::max(get_parameter("inspection_gas_rate").as_double(), 0.01);
    brake_gas_ = get_parameter("inspection_brake_gas").as_double();
    brake_duration_ =
      std::max(get_parameter("inspection_brake_duration").as_double(), 0.0);
    wheel_speed_timeout_ =
      std::max(get_parameter("wheel_speed_timeout").as_double(), 0.0);
    command_period_ = get_parameter("command_period").as_double();
    const double heartbeat_period = get_parameter("heartbeat_period").as_double();

    Pid::Gains gains;
    gains.kp = get_parameter("inspection_rpm_kp").as_double();
    gains.ki = get_parameter("inspection_rpm_ki").as_double();
    gains.kd = get_parameter("inspection_rpm_kd").as_double();
    gains.tau_d = get_parameter("inspection_rpm_tau_d").as_double();
    rpm_pid_.configure(gains, gas_min_, gas_max_);

    RCLCPP_INFO(
      get_logger(),
      "Inspection mission: %.1f s braked hold, then a %.1f s steering sweep at %.2f Hz, "
      "+-%.2f rad, then %.1f s braking before the mission is reported finished.",
      start_delay_, duration_, frequency_, amplitude_, brake_duration_);
    RCLCPP_INFO(
      get_logger(),
      "Wheel-speed loop: %.0f RPM target, gains (%.4f, %.4f, %.4f), gas in [%.2f, %.2f].",
      target_rpm_, gains.kp, gains.ki, gains.kd, gas_min_, gas_max_);
    RCLCPP_INFO(
      get_logger(),
      "Feedback is the mean of %s; older than %.2f s and the loop brakes.",
      kWheelSpeedTopic, wheel_speed_timeout_);

    // The car's own state machine is what starts the clock. The sweep must be
    // a window of the wheel actually moving, and the wheel does not move until
    // the car reaches DRIVING, so counting from anything earlier -- launch,
    // mission select -- would short the run by however long that took.
    //
    // inspection_start_delay then sits between DRIVING and the first sweep
    // command, and is not counted against the sweep: the run is still a full
    // inspection_duration of movement, it just starts a few seconds later.
    as_sub_ = create_subscription<mirena_common::msg::AsInfo>(
      kAsStatusTopic, 10,
      std::bind(&InspectionNode::asCallback, this, std::placeholders::_1));

    // SensorDataQoS, matching what the CAN bridge publishes with.  A default
    // (reliable) subscription would silently never match a best-effort
    // publisher, and the first sign of it would be a loop that never gets a
    // measurement -- which now reads as a fault rather than as silence.
    rclcpp::SubscriptionOptions wheel_opts;
    wheel_opts.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    wheel_sub_ = create_subscription<mirena_common::msg::WheelSpeeds>(
      kWheelSpeedTopic, rclcpp::SensorDataQoS(),
      std::bind(&InspectionNode::wheelCallback, this, std::placeholders::_1), wheel_opts);

    control_pub_ = create_publisher<mirena_common::msg::CarControl>(kControlTopic, 10);
    heartbeat_pub_ = create_publisher<mirena_common::msg::Heartbeat>(kHealthTopic, 10);

    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    heartbeat_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(heartbeat_period),
      std::bind(&InspectionNode::publishHeartbeat, this));
    command_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(command_period_),
      std::bind(&InspectionNode::commandIteration, this), command_group_);
  }

private:
  // ---------------------------------------------------------------- callbacks

  /// Latches the moment the car starts driving; the hold, and then the sweep,
  /// are timed from there.
  void asCallback(const mirena_common::msg::AsInfo::SharedPtr msg)
  {
    if (msg->status != mirena_common::msg::AsInfo::AS_STATUS_DRIVING) {
      return;
    }
    int64_t unset = kUnsetTime;
    if (start_ns_.compare_exchange_strong(
        unset, get_clock()->now().nanoseconds(), std::memory_order_relaxed))
    {
      RCLCPP_INFO(
        get_logger(), "AS is DRIVING: holding braked for %.1f s, then a %.1f s sweep.",
        start_delay_, duration_);
    }
  }

  void wheelCallback(const mirena_common::msg::WheelSpeeds::SharedPtr msg)
  {
    // A non-finite wheel speed has to be dropped here or it is never caught
    // again: NaN compares false against every threshold downstream, so it would
    // reach the PID as an error and take the integrator with it.  The last good
    // reading is kept instead and simply ages out if nothing better arrives.
    if (!std::isfinite(msg->rl) || !std::isfinite(msg->rr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarding a wheel-speed message with a non-finite reading.");
      return;
    }

    WheelState w;
    // The CAN bridge stamps these; an unstamped message would read as
    // infinitely old and hold the loop off forever, so fall back to arrival.
    const int64_t stamped = rclcpp::Time(msg->header.stamp).nanoseconds();
    w.stamp_ns = stamped == 0 ? get_clock()->now().nanoseconds() : stamped;
    w.rl = msg->rl;
    w.rr = msg->rr;
    // The mean of the two driven wheels.  Averaging is what makes this a
    // drivetrain measurement rather than a wheel one: on stands the two are
    // free to differ through the diff, and either one alone would have the loop
    // chase whichever happens to be spinning.
    w.mean = 0.5 * (static_cast<double>(msg->rl) + static_cast<double>(msg->rr));
    w.valid = true;
    wheels_.store(w);
  }

  // ------------------------------------------------------------- command loop

  void commandIteration()
  {
    const int64_t driving_ns = start_ns_.load(std::memory_order_relaxed);
    if (driving_ns == kUnsetTime) {
      return;   // not driving yet; the hold has not begun
    }

    const auto stamp = get_clock()->now();
    const int64_t now_ns = stamp.nanoseconds();
    // Elapsed is the sweep's own clock: negative through the hold, zero at the
    // first sweep command. The hold is therefore not charged to the sweep, and
    // the window below still closes a full duration_ after movement starts.
    const double elapsed = static_cast<double>(now_ns - driving_ns) * 1e-9 - start_delay_;
    elapsed_.store(std::max(elapsed, 0.0), std::memory_order_relaxed);

    command_msg_.header.stamp = stamp;

    // The hold. Braking rather than publishing nothing: the vehicle interface
    // reads silence as a fault, and this is the one branch where the car is
    // already live but deliberately not being driven yet.
    if (elapsed < 0.0) {
      rpm_pid_.reset();
      last_gas_ = brake_gas_;
      command_msg_.steer_angle = kFailsafeSteer;
      command_msg_.gas = brake_gas_;
      control_pub_->publish(command_msg_);
      commanded_gas_.store(brake_gas_, std::memory_order_relaxed);
      measured_rpm_.store(0.0, std::memory_order_relaxed);
      holding_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Holding braked before the sweep: %.1f/%.1f s", start_delay_ + elapsed, start_delay_);
      return;
    }

    if (holding_.exchange(false, std::memory_order_relaxed)) {
      RCLCPP_INFO(get_logger(), "Hold over: starting the %.1f s sweep.", duration_);
    }

    // Past the window: brake, and only latch once the wheels have had
    // inspection_brake_duration to come down -- the supervisor tears the stack
    // down on the flag, and a flag raised the instant the sweep ends would
    // raise it on a car still spinning.  Keep publishing afterwards, since
    // until the teardown comes the vehicle interface must not be left without a
    // fresh command.
    if (elapsed > duration_) {
      rpm_pid_.reset();
      last_gas_ = brake_gas_;
      command_msg_.steer_angle = kFailsafeSteer;
      command_msg_.gas = brake_gas_;
      control_pub_->publish(command_msg_);
      commanded_gas_.store(brake_gas_, std::memory_order_relaxed);
      if (elapsed <= duration_ + brake_duration_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Sweep over: braking for %.1f/%.1f s before finishing.",
          elapsed - duration_, brake_duration_);
      } else if (!finished_.exchange(true, std::memory_order_relaxed)) {
        RCLCPP_INFO(
          get_logger(), "Sweep finished after %.1f s, wheels braked. Mission complete.",
          duration_);
      }
      return;
    }

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

    // The steering sweep is open-loop and runs regardless: it is the half of the
    // demonstration that does not depend on the drivetrain, and a wheel-speed
    // dropout is no reason to stop showing the rack works.
    command_msg_.steer_angle = amplitude_ * std::sin(2.0 * M_PI * frequency_ * elapsed);

    const WheelState wheels = wheels_.load();
    const double age = wheels.valid ?
      static_cast<double>(now_ns - wheels.stamp_ns) * 1e-9 : -1.0;
    const bool fresh = wheels.valid && age <= wheel_speed_timeout_;

    if (!fresh) {
      // No feedback, no closed loop.  Braking rather than holding the last gas
      // is the point: the alternative is a drivetrain driven open-loop by an
      // integrator that has stopped being corrected.
      rpm_pid_.reset();
      last_gas_ = brake_gas_;
      command_msg_.gas = brake_gas_;
      control_pub_->publish(command_msg_);
      commanded_gas_.store(brake_gas_, std::memory_order_relaxed);
      measured_rpm_.store(0.0, std::memory_order_relaxed);
      feedback_ok_.store(false, std::memory_order_relaxed);
      if (!wheel_fault_.exchange(true, std::memory_order_relaxed)) {
        RCLCPP_ERROR(
          get_logger(),
          "No usable wheel speed (age %.2f s, limit %.2f s): braking, the sweep runs on.",
          age, wheel_speed_timeout_);
      }
      return;
    }

    if (wheel_fault_.exchange(false, std::memory_order_relaxed)) {
      RCLCPP_INFO(get_logger(), "Wheel speed is fresh again; closing the loop.");
    }

    // The loop itself: mean rear wheel speed to gas.  Derivative on the
    // measurement rather than the error, since the target is a constant here
    // and the measurement is the noisy one.
    const double error = target_rpm_ - wheels.mean;
    double gas = rpm_pid_.update(error, wheels.mean, dt);

    // Slew limit, so a target the drivetrain is far from does not become a step
    // on the pedal.  On stands this is the difference between demonstrating a
    // controlled spin-up and shock-loading the driveline.
    const double step = gas_rate_ * dt;
    gas = std::min(std::max(gas, last_gas_ - step), last_gas_ + step);
    last_gas_ = gas;

    command_msg_.gas = gas;
    control_pub_->publish(command_msg_);

    measured_rpm_.store(wheels.mean, std::memory_order_relaxed);
    commanded_gas_.store(gas, std::memory_order_relaxed);
    feedback_ok_.store(true, std::memory_order_relaxed);

    // A loop pinned at its authority limit and still short of target is not
    // mistuned, it is asking for more pedal than it has been given -- and the
    // two look identical from outside.  Saying so is what stops the day being
    // spent on the gains when the answer is inspection_gas_max.
    const bool saturated = gas >= gas_max_ - 1e-6 || gas <= gas_min_ + 1e-6;
    const bool off_target = std::fabs(error) > 0.05 * std::fabs(target_rpm_);
    saturated_.store(saturated && off_target, std::memory_order_relaxed);
    if (saturated && off_target) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel-speed loop is at its gas limit (%.2f) and still %.0f RPM short of "
        "%.0f: raise inspection_gas_max or lower the target.",
        gas, error, target_rpm_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Sweep active: %.2f rad | %.0f/%.0f RPM (err %+.0f) | gas %.2f | %.1f/%.1f s",
      command_msg_.steer_angle, wheels.mean, target_rpm_, error, gas, elapsed, duration_);
  }

  // --------------------------------------------------------------- heartbeat

  void publishHeartbeat()
  {
    if (!heartbeat_pub_) {
      return;
    }
    mirena_common::msg::Heartbeat msg;
    msg.timestamp = get_clock()->now();
    msg.node_name = get_name();

    const bool finished = finished_.load(std::memory_order_relaxed);
    const bool fault = wheel_fault_.load(std::memory_order_relaxed);
    const WheelState wheels = wheels_.load();

    nlohmann::json report;

    // "mission_finished" is the exact key the supervisor ends the mission off,
    // spelled exactly as the other controllers spell it -- the supervisor
    // watches one control node and does not care which of them it got.
    report["mission_finished"] = finished;
    report["mode"] = "inspection";
    // A sweep whose drivetrain loop cannot run is not a healthy one, and the
    // difference is worth reporting: the steering half is still being
    // demonstrated, but the wheel-speed half is not.
    report["status_state"] = finished ? "IDLE" : (fault ? "NOT_WORKING" : "WORKING");
    report["sweep_elapsed_s"] = elapsed_.load(std::memory_order_relaxed);
    report["sweep_duration_s"] = duration_;
    // The braked hold is a healthy state, so it does not touch status_state --
    // it is reported on its own key so a sweep that has not visibly started yet
    // reads as waiting rather than as stuck.
    report["holding"] = holding_.load(std::memory_order_relaxed);
    report["start_delay_s"] = start_delay_;

    report["target_rpm"] = target_rpm_;
    report["measured_rpm"] = measured_rpm_.load(std::memory_order_relaxed);
    report["rpm_error"] = target_rpm_ - measured_rpm_.load(std::memory_order_relaxed);
    report["wheel_rl_rpm"] = wheels.valid ? wheels.rl : 0.0;
    report["wheel_rr_rpm"] = wheels.valid ? wheels.rr : 0.0;
    report["have_wheel_speed"] = feedback_ok_.load(std::memory_order_relaxed);
    report["gas"] = commanded_gas_.load(std::memory_order_relaxed);
    report["gas_saturated"] = saturated_.load(std::memory_order_relaxed);

    msg.report = report.dump();
    heartbeat_pub_->publish(msg);
  }

  // ------------------------------------------------------------------- state

  // Parameters, fixed at construction.
  double frequency_;
  double amplitude_;
  double duration_;
  double start_delay_;
  double target_rpm_;
  double gas_max_;
  double gas_min_;
  double gas_rate_;
  double brake_gas_;
  double brake_duration_;
  double wheel_speed_timeout_;
  double command_period_;

  rclcpp::CallbackGroup::SharedPtr command_group_;

  rclcpp::Subscription<mirena_common::msg::AsInfo>::SharedPtr as_sub_;
  rclcpp::Subscription<mirena_common::msg::WheelSpeeds>::SharedPtr wheel_sub_;

  rclcpp::Publisher<mirena_common::msg::CarControl>::SharedPtr control_pub_;
  rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Cross-thread hand-off: written by the wheel subscription, read by the
  // command loop and the heartbeat.
  Seqlock<WheelState> wheels_{WheelState{0, 0.0, 0.0, 0.0, false}};

  // Written by the AS callback, read by the command loop and the heartbeat.
  std::atomic<int64_t> start_ns_{kUnsetTime};
  std::atomic<double> elapsed_{0.0};
  std::atomic<bool> finished_{false};

  // Written by the command loop, read by the heartbeat: true while the braked
  // hold between DRIVING and the sweep is running.
  std::atomic<bool> holding_{false};

  // Written by the command loop, read by the heartbeat.
  std::atomic<double> measured_rpm_{0.0};
  std::atomic<double> commanded_gas_{0.0};
  std::atomic<bool> feedback_ok_{false};
  std::atomic<bool> wheel_fault_{false};
  std::atomic<bool> saturated_{false};

  // Command-thread only.
  Pid rpm_pid_;
  double last_gas_ = 0.0;
  int64_t last_command_ns_ = kUnsetTime;

  // Reused outgoing message, so the command loop does not allocate.
  mirena_common::msg::CarControl command_msg_;
};

}  // namespace mirena

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<mirena::InspectionNode> node;
  try {
    node = std::make_shared<mirena::InspectionNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mirena_control"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // One thread per mutually-exclusive callback group, so the command loop's
  // deadline never waits on a wheel-speed callback or a heartbeat.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
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
