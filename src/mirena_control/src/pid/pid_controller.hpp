#ifndef MIRENA_CONTROL__PID_CONTROLLER_HPP_
#define MIRENA_CONTROL__PID_CONTROLLER_HPP_

#include <cstddef>

#include "common/pid.hpp"

namespace mirena
{

/// Longitudinal and lateral PID path follower.
///
/// The split mirrors how the two loops are actually driven: `Reference` is the
/// geometry, produced a few times per solver period against the centreline and
/// the TF tree, while `update()` runs at the command rate against the freshest
/// state estimate.  Between geometry updates the errors are carried forward by
/// `propagate()` rather than held constant -- at cruise the car covers most of a
/// metre in one geometry period, and a lateral loop fed a half-period-old
/// cross-track error oscillates for no reason other than that lag.
class PidController
{
public:
  struct Config
  {
    // -- longitudinal ------------------------------------------------------
    Pid::Gains speed;
    double gas_min = -1.0;
    double gas_max = 1.0;
    double gas_rate = 5.0;             // units per second, matches the OCP's g_dot bound

    // -- lateral -----------------------------------------------------------
    Pid::Gains cross_track;            // metres of offset -> radians of steer
    double heading_gain = 0.8;         // rad of steer per rad of heading error
    double curvature_feedforward = 1.0;// scaling of the kinematic steer for the path's own bend
    double wheelbase = 1.55;           // m, L_F + L_R of the model in codegen/dbm_model.py
    double cross_track_limit = 3.0;    // m; error saturates here so a lost car does not slam the wheel
    double steer_limit = 0.5235987756; // rad, the OCP's RAD30
    double steer_rate = 2.617993878;   // rad/s, the OCP's 5 * RAD30

    // -- speed profile -----------------------------------------------------
    double max_speed = 20.0;           // m/s ceiling regardless of geometry
    double min_speed = 1.5;            // m/s floor away from a stop, so the car keeps rolling
    double max_lateral_accel = 6.0;    // m/s^2 the tyres are trusted with in a corner
    double max_decel = 4.0;            // m/s^2 used to plan the approach to a corner or the finish
  };

  /// The geometry the lateral loop closes on, in the path's local frame.
  ///
  /// Every sign here is the steering angle's: positive is left.  Stated that
  /// way, all three terms of the steering law take positive gains -- the car is
  /// steered towards a path on its left, towards a heading to its left, and
  /// into a curve that bends left.
  struct Reference
  {
    /// Signed lateral offset of the centreline from the car, positive when the
    /// path lies to the car's left.
    double cross_track = 0.0;
    /// Path heading minus car yaw, wrapped to [-pi, pi].
    double heading_error = 0.0;
    /// Signed curvature of the centreline at the projection, 1/m, positive left.
    double curvature = 0.0;
    /// Speed the profile asks for here, m/s.  Already accounts for the corners
    /// ahead and for the distance left to the mission's stop point.
    double target_speed = 0.0;
    bool valid = false;
  };

  struct Command
  {
    double gas = 0.0;
    double steer = 0.0;
  };

  explicit PidController(const Config & config);

  void configure(const Config & config);
  const Config & config() const noexcept {return config_;}

  /// Carry the geometry forward by `dt` using the car's own motion, so the
  /// command rate is not limited by the rate the geometry is rebuilt at.
  ///
  /// This is the error dynamics of a car following a curve, in the sign
  /// convention above:
  ///   d(e_y)/dt   = u sin(e_psi) - v cos(e_psi)
  ///   d(e_psi)/dt = u kappa - omega
  /// integrated with one explicit Euler step, which over the few tens of
  /// milliseconds between geometry updates is far closer than holding still.
  /// The second line takes the along-path speed as u, which is exact only for
  /// small errors -- the regime the loop is meant to be in anyway.
  static void propagate(
    Reference & ref, double u, double v, double omega, double dt) noexcept;

  /// One control step against the freshest state estimate.  Only the
  /// longitudinal speed is read here; the lateral state reaches the loop
  /// through `ref`, which `propagate()` has already carried forward.
  Command update(const Reference & ref, double u, double dt) noexcept;

  /// The command the car should be given when there is nothing to track: full
  /// braking, wheels straight.  Also drops both integrators and the rate
  /// limiter's history, so recovery starts from a clean loop rather than from
  /// whatever had wound up while the plan was going stale.
  Command failsafe() noexcept;

  void reset() noexcept;

  /// Highest speed that respects the lateral-acceleration budget at a corner of
  /// curvature `kappa`.  Unbounded curvature-free straights are capped by
  /// `max_speed`.
  double cornerSpeed(double kappa) const noexcept;

  /// Speed to hold now so that `distance` metres from here the car is down to
  /// `speed_there`, decelerating at no more than max_decel.
  double approachSpeed(double distance, double speed_there) const noexcept;

  /// Build the speed target from the curvature of the window ahead and the
  /// distance left to the mission's stop point.
  ///
  /// `curvature[i]` is the centreline curvature `spacing * i` metres ahead of
  /// the car.  Every sample is treated as a corner the car must already be slow
  /// enough to enter after braking the intervening distance, and the tightest
  /// such constraint wins -- which is a forward pass of the usual backward
  /// speed-profile sweep, done over just the window the car can see.
  double speedTarget(
    const double * curvature, std::size_t count, double spacing,
    double distance_to_stop) const noexcept;

  /// Diagnostics for the heartbeat; no control decision reads these.
  double lastTargetSpeed() const noexcept {return last_target_speed_;}
  double lastSteerUnclamped() const noexcept {return last_steer_raw_;}
  bool steerSaturated() const noexcept {return steer_saturated_;}

private:
  /// Slew `desired` towards `previous` no faster than `rate` per second.
  static double rateLimit(double desired, double previous, double rate, double dt) noexcept;

  Config config_;
  Pid speed_pid_;
  Pid lateral_pid_;

  double last_gas_ = 0.0;
  double last_steer_ = 0.0;
  double last_target_speed_ = 0.0;
  double last_steer_raw_ = 0.0;
  bool steer_saturated_ = false;
  bool primed_ = false;      // last_gas_/last_steer_ hold a real previous command
};

/// Wrap an angle to [-pi, pi].
double wrapAngle(double angle) noexcept;

}  // namespace mirena

#endif  // MIRENA_CONTROL__PID_CONTROLLER_HPP_
