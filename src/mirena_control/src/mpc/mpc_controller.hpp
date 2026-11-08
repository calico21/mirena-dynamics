#ifndef MIRENA_CONTROL__MPC_CONTROLLER_HPP_
#define MIRENA_CONTROL__MPC_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <string>

extern "C" {
#include "acados_solver_mpc_problem.h"
}

namespace mirena
{

/// Thin owner of the generated acados solver.
///
/// The horizon buffers live inside the controller and are handed to acados
/// row by row, so warm starting and reading the solution back involve no
/// allocation and no intermediate copies.  A failed solve leaves the previous
/// horizon in place, which is what the next warm start wants.
class MpcController
{
public:
  static constexpr int kNx = MPC_PROBLEM_NX;                 // 10
  static constexpr int kNu = MPC_PROBLEM_NU;                 // 3
  static constexpr int kN = MPC_PROBLEM_N;                   // shooting intervals
  static constexpr int kNpGlobal = MPC_PROBLEM_NP_GLOBAL;    // 2 * path points + 1
  static constexpr int kPathPoints = (kNpGlobal - 1) / 2;

  /// Index of the progress state p in the augmented state vector
  /// [x, y, phi, u, v, omega, g, delta, p, vp].  Only shiftProgress() needs to
  /// know the layout; everything else treats the horizon as opaque.
  static constexpr int kProgressIndex = 8;

  /// Row-major horizons; each row is contiguous and passed straight to acados.
  using StateHorizon = std::array<std::array<double, kNx>, kN + 1>;
  using InputHorizon = std::array<std::array<double, kNu>, kN>;

  struct Stats
  {
    double solve_time_ms;
    double max_residual;
    int sqp_iterations;
  };

  MpcController();
  ~MpcController();

  MpcController(const MpcController &) = delete;
  MpcController & operator=(const MpcController &) = delete;

  /// Length of one shooting interval, read back from the generated solver.
  double stepDuration() const noexcept {return step_duration_;}

  /// Warm start from the stored horizon, solve, and overwrite it whenever the
  /// solve left a usable iterate behind.  Returns the acados status
  /// (0 == converged); see hasIterate() for which other statuses still carry a
  /// solution the caller may inspect.
  int solve(const double (& x0)[kNx], const double (& p_global)[kNpGlobal]);

  /// True if `status` leaves a well-defined iterate in the horizon: a converged
  /// solve, one that ran out of iterations, and one that hit the wall-clock
  /// timeout all do.  A NaN or a QP failure does not.
  static bool hasIterate(int status) noexcept;

  /// Slide the stored horizon's progress state back by `delta` metres.
  ///
  /// p is measured from the start of the reference window, and that window
  /// slides forward with the car between solves, so a warm start seeded from
  /// the previous horizon has to be re-based or it starts the solver off by
  /// however far the car travelled.  Clamped at zero to stay inside p >= 0.
  void shiftProgress(double delta) noexcept;

  /// Discard the internal solver state after a failure.
  void reset();

  Stats stats() const;

  const StateHorizon & states() const noexcept {return states_;}
  const InputHorizon & inputs() const noexcept {return inputs_;}

private:
  mpc_problem_solver_capsule * capsule_ = nullptr;
  ocp_nlp_config * config_ = nullptr;
  ocp_nlp_dims * dims_ = nullptr;
  ocp_nlp_in * in_ = nullptr;
  ocp_nlp_out * out_ = nullptr;
  ocp_nlp_solver * solver_ = nullptr;

  StateHorizon states_{};
  InputHorizon inputs_{};
  double step_duration_ = 0.0;
};

}  // namespace mirena

#endif  // MIRENA_CONTROL__MPC_CONTROLLER_HPP_
