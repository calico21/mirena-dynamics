#include "pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mirena
{

double wrapAngle(double angle) noexcept
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}


// ----------------------------------------------------------- PidController

PidController::PidController(const Config & config)
{
  configure(config);
}

void PidController::configure(const Config & config)
{
  config_ = config;
  speed_pid_.configure(config_.speed, config_.gas_min, config_.gas_max);
  // The lateral PID owns only the feedback part of the steering angle; the
  // heading term and the curvature feed-forward are added on top and the sum is
  // clamped once.  Giving the PID the full steer limit as its own range would
  // let its integral wind past what is left after those two.
  lateral_pid_.configure(config_.cross_track, -config_.steer_limit, config_.steer_limit);
  reset();
}

void PidController::reset() noexcept
{
  speed_pid_.reset();
  lateral_pid_.reset();
  last_gas_ = 0.0;
  last_steer_ = 0.0;
  last_steer_raw_ = 0.0;
  steer_saturated_ = false;
  primed_ = false;
}

double PidController::cornerSpeed(double kappa) const noexcept
{
  const double k = std::fabs(kappa);
  if (!std::isfinite(k) || k < 1e-6) {
    return config_.max_speed;
  }
  return std::min(std::sqrt(config_.max_lateral_accel / k), config_.max_speed);
}

double PidController::approachSpeed(double distance, double speed_there) const noexcept
{
  if (!(distance > 0.0) || !std::isfinite(distance)) {
    return speed_there;
  }
  return std::sqrt(
    speed_there * speed_there + 2.0 * config_.max_decel * distance);
}

double PidController::speedTarget(
  const double * curvature, std::size_t count, double spacing,
  double distance_to_stop) const noexcept
{
  double target = config_.max_speed;

  if (curvature != nullptr && spacing > 0.0) {
    for (std::size_t i = 0; i < count; ++i) {
      const double corner = cornerSpeed(curvature[i]);
      const double distance = spacing * static_cast<double>(i);
      target = std::min(target, approachSpeed(distance, corner));
    }
  }

  // The floor keeps the car rolling through a corner tight enough that the
  // lateral budget alone would ask for a crawl -- but it is lifted near the
  // stop point, which is the one place the controller genuinely wants zero.
  target = std::max(target, config_.min_speed);

  // Same braking law applied to the mission's end, which is what brings the car
  // to a halt: the MPC gets this from a truncated reference, the PID has to
  // state it.  Applied after the floor so it always wins.
  if (std::isfinite(distance_to_stop)) {
    target = std::min(target, approachSpeed(std::max(distance_to_stop, 0.0), 0.0));
  }

  return target;
}

void PidController::propagate(
  Reference & ref, double u, double v, double omega, double dt) noexcept
{
  if (!ref.valid || !(dt > 0.0) || dt > Pid::kMaxStep) {
    return;
  }
  if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(omega)) {
    return;
  }

  const double e_psi = ref.heading_error;
  const double de_y = u * std::sin(e_psi) - v * std::cos(e_psi);
  const double de_psi = u * ref.curvature - omega;

  ref.cross_track += de_y * dt;
  ref.heading_error = wrapAngle(e_psi + de_psi * dt);
}

double PidController::rateLimit(
  double desired, double previous, double rate, double dt) noexcept
{
  if (!(rate > 0.0) || !(dt > 0.0)) {
    return desired;
  }
  const double step = rate * dt;
  return std::min(std::max(desired, previous - step), previous + step);
}

PidController::Command PidController::update(
  const Reference & ref, double u, double dt) noexcept
{
  if (!ref.valid) {
    return failsafe();
  }

  // ---- longitudinal: speed error -> gas -----------------------------------
  const double target = std::min(
    std::max(ref.target_speed, 0.0), config_.max_speed);
  last_target_speed_ = target;
  double gas = speed_pid_.update(target - u, u, dt);

  // ---- lateral: cross-track + heading + curvature feed-forward -> steer ----
  //
  // The cross-track error is saturated before it reaches the PID.  A car that
  // is ten metres off the line is not in a situation more steering fixes, and
  // an unsaturated proportional term there simply pins the wheel at the limit
  // and holds the integrator against it.
  const double e_y = std::min(
    std::max(ref.cross_track, -config_.cross_track_limit), config_.cross_track_limit);

  // The setpoint is a constant zero offset, so the saturated error is the only
  // signal there is to differentiate.  Using the saturated value rather than
  // the raw one also keeps the derivative quiet while the car is far off the
  // line and the proportional term is already at the limit.
  const double feedback = lateral_pid_.updateOnError(e_y, dt);
  const double heading = config_.heading_gain * ref.heading_error;

  // What the kinematic bicycle needs just to hold the path's own curvature.
  // Without it the feedback terms have to generate the whole steering angle
  // from an error that therefore never reaches zero in a steady corner -- the
  // familiar constant offset of a pure proportional path follower.
  const double feedforward =
    config_.curvature_feedforward * std::atan(config_.wheelbase * ref.curvature);

  double steer = feedback + heading + feedforward;
  last_steer_raw_ = steer;
  steer_saturated_ = std::fabs(steer) > config_.steer_limit;
  steer = std::min(std::max(steer, -config_.steer_limit), config_.steer_limit);

  // ---- actuator rate limits ----------------------------------------------
  //
  // The same bounds the OCP puts on g_dot and delta_dot.  Commanding a step the
  // steering rack cannot follow does not make it arrive sooner; it just makes
  // the commanded and actual angle disagree, which is exactly what the
  // derivative term then reacts to.
  if (primed_) {
    gas = rateLimit(gas, last_gas_, config_.gas_rate, dt);
    steer = rateLimit(steer, last_steer_, config_.steer_rate, dt);
  }
  primed_ = true;

  last_gas_ = gas;
  last_steer_ = steer;
  return Command{gas, steer};
}

PidController::Command PidController::failsafe() noexcept
{
  reset();
  last_gas_ = config_.gas_min;
  last_steer_ = 0.0;
  primed_ = true;
  return Command{config_.gas_min, 0.0};
}

}  // namespace mirena
