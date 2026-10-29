#ifndef MIRENA_CONTROL__PID_HPP_
#define MIRENA_CONTROL__PID_HPP_

namespace mirena
{

/// Scalar PID with output clamping, conditional-integration anti-windup and a
/// low-pass filtered derivative.
///
/// The derivative is taken on the measurement rather than on the error, which
/// is what keeps a step in the setpoint -- a new speed target when the car
/// enters a corner, a wheel-speed target raised mid-sweep -- from kicking the
/// output.  `update()` is given the error and the measurement separately for
/// exactly that reason; `updateOnError()` is the shorthand for the loops whose
/// setpoint never moves.
class Pid
{
public:
  /// Longest step the loop is willing to integrate over.  A gap larger than
  /// this means the loop was not running -- a preempted thread, a paused sim
  /// clock, a node that was starved -- and integrating across it would produce
  /// an integral term and a derivative that describe nothing that happened.
  static constexpr double kMaxStep = 0.25;

  struct Gains
  {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    /// Time constant of the first-order filter on the derivative, in seconds.
    /// Differentiating a measurement raw amplifies its noise straight into the
    /// actuator; 0 disables the filter.
    double tau_d = 0.05;
  };

  void configure(const Gains & gains, double out_min, double out_max) noexcept;

  /// One step, differentiating the measurement.  `error` must be
  /// `setpoint - measurement`.  `dt` in seconds; a non-positive or absurd dt is
  /// ignored and the previous output is returned, so a stalled or rewound clock
  /// cannot inject a derivative spike.
  double update(double error, double measurement, double dt) noexcept;

  /// One step for a loop whose setpoint is a constant, where differentiating
  /// the error and differentiating the measurement come to the same thing and
  /// only the error is on hand.
  double updateOnError(double error, double dt) noexcept
  {
    return update(error, -error, dt);
  }

  /// Drop the integrator and the derivative history.  Called whenever the loop
  /// has not been running -- a stale measurement, a failsafe -- because an
  /// integral accumulated against a situation that has since passed is worse
  /// than no integral at all.
  void reset() noexcept;

  double integral() const noexcept {return integral_;}
  double output() const noexcept {return output_;}

private:
  Gains gains_{};
  double out_min_ = -1.0;
  double out_max_ = 1.0;

  double integral_ = 0.0;
  double derivative_ = 0.0;
  double last_measurement_ = 0.0;
  double output_ = 0.0;
  bool primed_ = false;      // a measurement has been seen, so d/dt is defined
};

}  // namespace mirena

#endif  // MIRENA_CONTROL__PID_HPP_
