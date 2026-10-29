#ifndef MIRENA_CONTROL__CUBIC_SPLINE_HPP_
#define MIRENA_CONTROL__CUBIC_SPLINE_HPP_

#include <cstddef>
#include <vector>

namespace mirena
{

/// Arc-length parameterised planar cubic spline.
///
/// Interpolation matches `scipy.interpolate.CubicSpline` for the two boundary
/// conditions the planner produces: `not-a-knot` for an open track and
/// `periodic` for a closed one.  Coefficients are precomputed once per track
/// update into power-basis form so evaluation is a segment lookup plus two
/// Horner passes.
class CubicSpline2D
{
public:
  enum class Boundary
  {
    NotAKnot,
    Periodic,
  };

  /// Fit through `n` points given as interleaved x,y.  Knots are the cumulative
  /// chord lengths.  Points closer than `kMinSegment` to their predecessor are
  /// dropped, since a repeated knot makes the interpolation singular.
  /// Returns false and leaves the spline untouched if the fit is not possible.
  bool build(const double * xy, std::size_t n, Boundary bc);

  bool valid() const noexcept {return !segments_.empty();}

  /// Total arc length; the spline is defined on [0, length()].
  double length() const noexcept {return knots_.empty() ? 0.0 : knots_.back();}

  std::size_t knotCount() const noexcept {return knots_.size();}

  /// Position at `s`, clamped to the parameter range.
  void eval(double s, double & x, double & y) const noexcept;

  /// Position, first and second derivative w.r.t. s, clamped to the range.
  /// `d1` and `d2` may be null.
  void evalDerivatives(double s, double * pos, double * d1, double * d2) const noexcept;

  /// Arc length of the closest point on the spline, searched near `s_guess`.
  ///
  /// Replaces the Nelder-Mead projection of the Python node: a coarse sweep
  /// brackets the minimum, then damped Newton iterations on
  /// d/ds |P(s) - target|^2 converge quadratically.
  ///
  /// The search is deliberately local and must always be seeded with the
  /// progress already believed in.  There is no global variant on purpose: a
  /// nearest-point sweep of the whole spline is ambiguous wherever the path
  /// overlaps itself, which the unrolled skidpad path does exactly.
  ///
  /// The returned value is *unwrapped* -- on a closed track it keeps growing
  /// past `length()` so it doubles as a cumulative progress measure.  When
  /// `wrap` is false the search is clamped to [0, length()] instead.
  double project(
    double target_x, double target_y, double s_guess, bool wrap,
    double window_back, double window_forward, double coarse_step) const noexcept;

private:
  /// p(t) = c[0] + c[1] t + c[2] t^2 + c[3] t^3, t = s - knot, one cache line.
  struct Segment
  {
    double x[4];
    double y[4];
  };

  static constexpr double kMinSegment = 1e-9;

  std::size_t locate(double s) const noexcept;

  /// Wrap `s` into [0, length()) or clamp it to [0, length()].
  double fold(double s, bool wrap) const noexcept;

  /// Solve the tridiagonal slope system for one coordinate (not-a-knot).
  static bool slopesNotAKnot(
    const std::vector<double> & h, const std::vector<double> & knots,
    const std::vector<double> & delta, const std::vector<double> & y,
    std::vector<double> & m);

  /// Solve the cyclic tridiagonal slope system for one coordinate (periodic).
  static bool slopesPeriodic(
    const std::vector<double> & h, const std::vector<double> & delta,
    std::vector<double> & m);

  std::vector<double> knots_;
  std::vector<Segment> segments_;

  /// Last segment hit, used to start the lookup.  Queries walk forward along
  /// the path, so this turns the binary search into a compare in the common
  /// case.  Only ever touched by the single thread that owns the spline.
  mutable std::size_t hint_ = 0;
};

}  // namespace mirena

#endif  // MIRENA_CONTROL__CUBIC_SPLINE_HPP_
