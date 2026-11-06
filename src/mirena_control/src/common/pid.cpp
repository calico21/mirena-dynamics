#include "common/pid.hpp"

#include <algorithm>
#include <cmath>

namespace mirena
{

void Pid::configure(const Gains & gains, double out_min, double out_max) noexcept
{
  gains_ = gains;
  out_min_ = out_min;
  out_max_ = out_max;
  reset();
}

void Pid::reset() noexcept
{
  integral_ = 0.0;
  derivative_ = 0.0;
  last_measurement_ = 0.0;
  output_ = 0.0;
  primed_ = false;
}

double Pid::update(double error, double measurement, double dt) noexcept
{
  if (!std::isfinite(error) || !std::isfinite(measurement)) {
    return output_;
  }
  if (!(dt > 0.0) || dt > kMaxStep) {
    return output_;
  }

  // Derivative on the measurement, negated so it still opposes the error.  The
  // first call has no previous measurement to difference against, so it
  // contributes nothing rather than a spike proportional to the initial value.
  double raw_derivative = 0.0;
  if (primed_) {
    raw_derivative = -(measurement - last_measurement_) / dt;
  }
  last_measurement_ = measurement;
  primed_ = true;

  if (gains_.tau_d > 0.0) {
    const double alpha = dt / (gains_.tau_d + dt);
    derivative_ += alpha * (raw_derivative - derivative_);
  } else {
    derivative_ = raw_derivative;
  }

  const double proportional = gains_.kp * error;
  const double differential = gains_.kd * derivative_;

  // Conditional integration: the candidate integral is only kept if the result
  // stays inside the output range, or if the error is pushing the output back
  // towards it.  Clamping the output alone would let the integral grow without
  // bound while the actuator is already saturated -- the classic windup, which
  // on a wheel-speed loop shows up as the drivetrain staying pinned for a
  // second after the target is reached because the integral has to unwind.
  const double candidate = integral_ + gains_.ki * error * dt;
  const double unsaturated = proportional + candidate + differential;
  if (unsaturated > out_max_) {
    if (error < 0.0) {
      integral_ = candidate;
    }
  } else if (unsaturated < out_min_) {
    if (error > 0.0) {
      integral_ = candidate;
    }
  } else {
    integral_ = candidate;
  }

  output_ = std::min(
    std::max(proportional + integral_ + differential, out_min_), out_max_);
  return output_;
}

}  // namespace mirena
