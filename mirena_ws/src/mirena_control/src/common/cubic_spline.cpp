#include "common/cubic_spline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mirena
{

namespace
{

/// Thomas algorithm.  `lower[0]` and `upper[n-1]` are ignored.
/// Overwrites `rhs` with the solution.  False if a pivot collapses.
bool solveTridiagonal(
  std::vector<double> & lower, std::vector<double> & diag,
  std::vector<double> & upper, std::vector<double> & rhs)
{
  const std::size_t n = diag.size();
  if (n == 0) {
    return false;
  }

  for (std::size_t i = 1; i < n; ++i) {
    if (std::fabs(diag[i - 1]) < 1e-300) {
      return false;
    }
    const double w = lower[i] / diag[i - 1];
    diag[i] -= w * upper[i - 1];
    rhs[i] -= w * rhs[i - 1];
  }

  if (std::fabs(diag[n - 1]) < 1e-300) {
    return false;
  }
  rhs[n - 1] /= diag[n - 1];
  for (std::size_t i = n - 1; i-- > 0; ) {
    rhs[i] = (rhs[i] - upper[i] * rhs[i + 1]) / diag[i];
  }
  return true;
}

}  // namespace

bool CubicSpline2D::build(const double * xy, std::size_t n, Boundary bc)
{
  if (xy == nullptr || n < 2) {
    return false;
  }

  // Cumulative chord length, dropping duplicate points.
  std::vector<double> knots;
  std::vector<double> px;
  std::vector<double> py;
  knots.reserve(n);
  px.reserve(n);
  py.reserve(n);

  knots.push_back(0.0);
  px.push_back(xy[0]);
  py.push_back(xy[1]);

  for (std::size_t i = 1; i < n; ++i) {
    const double dx = xy[2 * i] - px.back();
    const double dy = xy[2 * i + 1] - py.back();
    const double d = std::sqrt(dx * dx + dy * dy);
    if (d < kMinSegment) {
      continue;
    }
    knots.push_back(knots.back() + d);
    px.push_back(xy[2 * i]);
    py.push_back(xy[2 * i + 1]);
  }

  const std::size_t m = knots.size();
  if (m < 2) {
    return false;
  }

  // A periodic fit needs the caller to have closed the loop; if the last point
  // was dropped as a duplicate of the first there is nothing left to close.
  const bool periodic = (bc == Boundary::Periodic) &&
    std::fabs(px.front() - px.back()) < 1e-9 &&
    std::fabs(py.front() - py.back()) < 1e-9 &&
    m >= 3;

  const std::size_t nseg = m - 1;
  std::vector<double> h(nseg);
  std::vector<double> dx(nseg);
  std::vector<double> dy(nseg);
  for (std::size_t i = 0; i < nseg; ++i) {
    h[i] = knots[i + 1] - knots[i];
    dx[i] = (px[i + 1] - px[i]) / h[i];
    dy[i] = (py[i + 1] - py[i]) / h[i];
  }

  std::vector<double> mx;
  std::vector<double> my;
  bool ok;
  if (periodic) {
    ok = slopesPeriodic(h, dx, mx) && slopesPeriodic(h, dy, my);
  } else {
    ok = slopesNotAKnot(h, knots, dx, px, mx) && slopesNotAKnot(h, knots, dy, py, my);
  }
  if (!ok) {
    return false;
  }

  // Hermite -> power basis.
  std::vector<Segment> segments(nseg);
  for (std::size_t i = 0; i < nseg; ++i) {
    const double hi = h[i];
    Segment & s = segments[i];

    s.x[0] = px[i];
    s.x[1] = mx[i];
    s.x[2] = (3.0 * dx[i] - 2.0 * mx[i] - mx[i + 1]) / hi;
    s.x[3] = (mx[i] + mx[i + 1] - 2.0 * dx[i]) / (hi * hi);

    s.y[0] = py[i];
    s.y[1] = my[i];
    s.y[2] = (3.0 * dy[i] - 2.0 * my[i] - my[i + 1]) / hi;
    s.y[3] = (my[i] + my[i + 1] - 2.0 * dy[i]) / (hi * hi);
  }

  knots_ = std::move(knots);
  segments_ = std::move(segments);
  hint_ = 0;
  return true;
}

bool CubicSpline2D::slopesNotAKnot(
  const std::vector<double> & h, const std::vector<double> & knots,
  const std::vector<double> & delta, const std::vector<double> & y,
  std::vector<double> & m)
{
  const std::size_t n = y.size();
  m.assign(n, 0.0);

  if (n == 2) {                     // single segment: straight line
    m[0] = delta[0];
    m[1] = delta[0];
    return true;
  }

  if (n == 3) {                     // not-a-knot degenerates to one parabola
    // Fit y = a + b (s - s0) + c (s - s0)^2 through the three points.
    const double h0 = h[0];
    const double h1 = h[1];
    const double c = (delta[1] - delta[0]) / (h0 + h1);
    const double b = delta[0] - c * h0;
    m[0] = b;
    m[1] = b + 2.0 * c * h0;
    m[2] = b + 2.0 * c * (h0 + h1);
    return true;
  }

  std::vector<double> lower(n, 0.0);
  std::vector<double> diag(n, 0.0);
  std::vector<double> upper(n, 0.0);
  std::vector<double> rhs(n, 0.0);

  // First row: third derivative continuous across the second knot.
  {
    const double d = knots[2] - knots[0];
    diag[0] = h[1];
    upper[0] = d;
    rhs[0] = ((h[0] + 2.0 * d) * h[1] * delta[0] + h[0] * h[0] * delta[1]) / d;
  }

  for (std::size_t i = 1; i + 1 < n; ++i) {
    lower[i] = h[i];
    diag[i] = 2.0 * (h[i - 1] + h[i]);
    upper[i] = h[i - 1];
    rhs[i] = 3.0 * (h[i] * delta[i - 1] + h[i - 1] * delta[i]);
  }

  // Last row: third derivative continuous across the second-to-last knot.
  {
    const std::size_t last = n - 1;
    const double d = knots[last] - knots[last - 2];
    lower[last] = d;
    diag[last] = h[last - 2];
    rhs[last] = (h[last - 1] * h[last - 1] * delta[last - 2] +
      (2.0 * d + h[last - 1]) * h[last - 2] * delta[last - 1]) / d;
  }

  if (!solveTridiagonal(lower, diag, upper, rhs)) {
    return false;
  }
  m = std::move(rhs);
  return true;
}

bool CubicSpline2D::slopesPeriodic(
  const std::vector<double> & h, const std::vector<double> & delta,
  std::vector<double> & m)
{
  // Unknowns are the slopes at knots 0..n-2; the slope at the closing knot
  // equals the one at knot 0.
  const std::size_t k = h.size();          // == number of intervals == n - 1
  m.assign(k + 1, 0.0);

  if (k == 1) {
    return true;                            // degenerate, slope 0 everywhere
  }

  std::vector<double> lower(k, 0.0);
  std::vector<double> diag(k, 0.0);
  std::vector<double> upper(k, 0.0);
  std::vector<double> rhs(k, 0.0);
  double corner_ur = 0.0;                   // element (0, k-1)
  double corner_ll = 0.0;                   // element (k-1, 0)

  for (std::size_t i = 0; i < k; ++i) {
    const std::size_t prev = (i + k - 1) % k;
    const double hp = h[prev];
    const double hc = h[i];
    diag[i] = 2.0 * (hp + hc);
    rhs[i] = 3.0 * (hc * delta[prev] + hp * delta[i]);
    if (i == 0) {
      corner_ur = hc;                       // coefficient of m[k-1]
      upper[0] = hp;
    } else if (i + 1 == k) {
      lower[i] = hc;
      corner_ll = hp;                       // coefficient of m[0]
    } else {
      lower[i] = hc;
      upper[i] = hp;
    }
  }

  if (k == 2) {
    // Sherman-Morrison needs a genuine tridiagonal core; solve the 2x2 directly.
    const double a = diag[0];
    const double b = upper[0] + corner_ur;
    const double c = lower[1] + corner_ll;
    const double d = diag[1];
    const double det = a * d - b * c;
    if (std::fabs(det) < 1e-300) {
      return false;
    }
    m[0] = (rhs[0] * d - b * rhs[1]) / det;
    m[1] = (a * rhs[1] - c * rhs[0]) / det;
    m[2] = m[0];
    return true;
  }

  // Sherman-Morrison: fold the two corners into a rank-1 update of a
  // tridiagonal system.  u = (gamma, 0, ..., corner_ll), v = (1, 0, ..., corner_ur/gamma).
  const double gamma = -diag[0];
  if (std::fabs(gamma) < 1e-300) {
    return false;
  }

  std::vector<double> diag_mod = diag;
  diag_mod[0] -= gamma;
  diag_mod[k - 1] -= corner_ur * corner_ll / gamma;

  std::vector<double> u(k, 0.0);
  u[0] = gamma;
  u[k - 1] = corner_ll;

  std::vector<double> l1 = lower, d1 = diag_mod, u1 = upper, b1 = rhs;
  if (!solveTridiagonal(l1, d1, u1, b1)) {
    return false;
  }
  std::vector<double> l2 = lower, d2 = diag_mod, u2 = upper, b2 = u;
  if (!solveTridiagonal(l2, d2, u2, b2)) {
    return false;
  }

  const double vy = b1[0] + corner_ur * b1[k - 1] / gamma;
  const double vz = b2[0] + corner_ur * b2[k - 1] / gamma;
  const double denom = 1.0 + vz;
  if (std::fabs(denom) < 1e-300) {
    return false;
  }
  const double factor = vy / denom;

  for (std::size_t i = 0; i < k; ++i) {
    m[i] = b1[i] - factor * b2[i];
  }
  m[k] = m[0];
  return true;
}

std::size_t CubicSpline2D::locate(double s) const noexcept
{
  const std::size_t nseg = segments_.size();
  std::size_t i = hint_ < nseg ? hint_ : 0;

  // Fast path: same segment, or the next one or two (the query point marches
  // forward along the path between calls).
  if (s >= knots_[i] && s < knots_[i + 1]) {
    return i;
  }
  if (i + 1 < nseg && s >= knots_[i + 1] && s < knots_[i + 2]) {
    hint_ = i + 1;
    return i + 1;
  }

  const auto it = std::upper_bound(knots_.begin(), knots_.end(), s);
  const std::size_t idx = static_cast<std::size_t>(it - knots_.begin());
  i = idx == 0 ? 0 : std::min(idx - 1, nseg - 1);
  hint_ = i;
  return i;
}

void CubicSpline2D::eval(double s, double & x, double & y) const noexcept
{
  if (segments_.empty()) {
    x = 0.0;
    y = 0.0;
    return;
  }
  s = std::min(std::max(s, 0.0), knots_.back());
  const std::size_t i = locate(s);
  const Segment & c = segments_[i];
  const double t = s - knots_[i];
  x = c.x[0] + t * (c.x[1] + t * (c.x[2] + t * c.x[3]));
  y = c.y[0] + t * (c.y[1] + t * (c.y[2] + t * c.y[3]));
}

void CubicSpline2D::evalDerivatives(
  double s, double * pos, double * d1, double * d2) const noexcept
{
  if (segments_.empty()) {
    if (pos) {pos[0] = pos[1] = 0.0;}
    if (d1) {d1[0] = d1[1] = 0.0;}
    if (d2) {d2[0] = d2[1] = 0.0;}
    return;
  }
  s = std::min(std::max(s, 0.0), knots_.back());
  const std::size_t i = locate(s);
  const Segment & c = segments_[i];
  const double t = s - knots_[i];

  if (pos) {
    pos[0] = c.x[0] + t * (c.x[1] + t * (c.x[2] + t * c.x[3]));
    pos[1] = c.y[0] + t * (c.y[1] + t * (c.y[2] + t * c.y[3]));
  }
  if (d1) {
    d1[0] = c.x[1] + t * (2.0 * c.x[2] + t * 3.0 * c.x[3]);
    d1[1] = c.y[1] + t * (2.0 * c.y[2] + t * 3.0 * c.y[3]);
  }
  if (d2) {
    d2[0] = 2.0 * c.x[2] + 6.0 * t * c.x[3];
    d2[1] = 2.0 * c.y[2] + 6.0 * t * c.y[3];
  }
}

double CubicSpline2D::fold(double s, bool wrap) const noexcept
{
  const double len = knots_.back();
  if (!wrap) {
    return std::min(std::max(s, 0.0), len);
  }
  double r = std::fmod(s, len);
  if (r < 0.0) {
    r += len;
  }
  return r;
}

double CubicSpline2D::project(
  double target_x, double target_y, double s_guess, bool wrap,
  double window_back, double window_forward, double coarse_step) const noexcept
{
  if (segments_.empty()) {
    return s_guess;
  }
  const double len = knots_.back();

  double lo = s_guess - window_back;
  double hi = s_guess + window_forward;
  if (!wrap) {
    lo = std::max(lo, 0.0);
    hi = std::min(hi, len);
    if (hi <= lo) {
      return std::min(std::max(s_guess, 0.0), len);
    }
  } else if (hi - lo > len) {
    hi = lo + len;
  }

  if (coarse_step <= 0.0) {
    coarse_step = 0.25;
  }
  const int steps = std::max(2, static_cast<int>((hi - lo) / coarse_step) + 1);

  double best_s = lo;
  double best_d2 = std::numeric_limits<double>::infinity();
  double p[2];
  for (int i = 0; i <= steps; ++i) {
    const double s = lo + (hi - lo) * static_cast<double>(i) / static_cast<double>(steps);
    eval(fold(s, wrap), p[0], p[1]);
    const double ex = p[0] - target_x;
    const double ey = p[1] - target_y;
    const double d2 = ex * ex + ey * ey;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_s = s;
    }
  }

  // Damped Newton on g(s) = d/ds |P(s) - target|^2, bracketed by one coarse
  // cell either side of the best sample.
  const double cell = (hi - lo) / static_cast<double>(steps);
  const double nlo = std::max(lo, best_s - cell);
  const double nhi = std::min(hi, best_s + cell);

  double s = best_s;
  double d1[2];
  double d2v[2];
  for (int it = 0; it < 8; ++it) {
    evalDerivatives(fold(s, wrap), p, d1, d2v);
    const double ex = p[0] - target_x;
    const double ey = p[1] - target_y;
    const double g = ex * d1[0] + ey * d1[1];
    const double gp = d1[0] * d1[0] + d1[1] * d1[1] + ex * d2v[0] + ey * d2v[1];
    if (!(std::fabs(gp) > 1e-12)) {
      break;
    }
    double step = -g / gp;
    if (gp < 0.0) {            // curving away from a minimum, fall back to gradient
      step = -g * 1e-2;
    }
    const double next = std::min(std::max(s + step, nlo), nhi);
    if (std::fabs(next - s) < 1e-9) {
      s = next;
      break;
    }
    s = next;
  }

  return wrap ? s : std::min(std::max(s, 0.0), len);
}

}  // namespace mirena
