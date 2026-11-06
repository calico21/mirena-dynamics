// Small 2D geometry toolkit shared by both planners.
//
// Deliberately free of ROS and of Eigen: the planners only ever need points,
// rigid transforms and neighbour lookups over a few hundred cones, and keeping
// this header dependency-free is what lets the whole planning core be unit
// tested without spinning a node.

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mirena::planning
{

// ------------------------------------------------------------------- vectors

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline Vec2 operator+(Vec2 a, Vec2 b) {return {a.x + b.x, a.y + b.y};}
inline Vec2 operator-(Vec2 a, Vec2 b) {return {a.x - b.x, a.y - b.y};}
inline Vec2 operator*(Vec2 v, double s) {return {v.x * s, v.y * s};}
inline double dot(Vec2 a, Vec2 b) {return a.x * b.x + a.y * b.y;}
inline double cross(Vec2 a, Vec2 b) {return a.x * b.y - a.y * b.x;}
inline double squaredNorm(Vec2 v) {return v.x * v.x + v.y * v.y;}
inline double norm(Vec2 v) {return std::hypot(v.x, v.y);}
inline double distance(Vec2 a, Vec2 b) {return norm(a - b);}

inline Vec2 normalized(Vec2 v)
{
  const double n = norm(v);
  return n > 1e-9 ? v * (1.0 / n) : Vec2{};
}

inline double wrapAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

/// Radius of the corner being asked for: the circle that leaves `from` pointing
/// along `heading` and passes through `to`. Infinite when `to` is already
/// straight ahead, zero when it is directly behind.
///
/// This is the quantity the turning rule caps, and it is a much better test of
/// whether two points can be consecutive on a centreline than the angle between
/// them, because it accounts for how far apart they are: the same 40 degree kink
/// is an easy sweep over 6 m and impossible over 1 m.
///
/// Only meaningful for a `to` that is ahead of `from`. Geometrically the circle
/// through a point behind still exists -- the car goes almost all the way round
/// it -- so this returns a large radius there and callers are expected to have
/// rejected backwards steps on their own.
double turnRadius(Vec2 from, Vec2 heading, Vec2 to);

// ---------------------------------------------------------------- transforms

/// Rigid transform of the plane: p -> R(psi) * p + t.
struct Transform2
{
  double psi{0.0};
  Vec2 t{};

  Vec2 operator*(Vec2 p) const
  {
    const double c = std::cos(psi), s = std::sin(psi);
    return {c * p.x - s * p.y + t.x, s * p.x + c * p.y + t.y};
  }

  Transform2 operator*(const Transform2 & rhs) const
  {
    return {wrapAngle(psi + rhs.psi), (*this) * rhs.t};
  }

  Transform2 inverse() const
  {
    const double c = std::cos(psi), s = std::sin(psi);
    return {-psi, {-(c * t.x + s * t.y), s * t.x - c * t.y}};
  }
};

/// Least-squares rigid transform T with T * src[i] ~= dst[i].
///
/// Closed-form 2D Kabsch: in the plane the optimal rotation is just the
/// argument of the summed centred correspondences, so no SVD is involved. Exact
/// for two correspondences, which is what the two-point RANSAC below relies on.
Transform2 rigidFit(const std::vector<Vec2> & src, const std::vector<Vec2> & dst);

// --------------------------------------------------------------------- cones

/// What the car can tell about a cone without reading its colour.
///
/// The pipeline is colourless end to end: nothing measures blue or yellow, so
/// nothing here can encode them. Height it can measure, and the two Formula
/// Student cones stand 505 mm and 325 mm tall, which is why the one distinction
/// worth carrying is the big cone. The four of those on the start line are what
/// the unknown-track planner roots the track on.
enum class ConeSize : std::uint8_t {Small = 0, Big};

/// Maps the perception/SLAM `Entity.type` strings onto the enum. Matching is
/// case insensitive and by substring, so `cone_big_orange`, `BIG_ORANGE` and
/// `big` all land on Big; everything else, `unknown_cone` included, is a Small.
/// Nothing is rejected -- an unrecognised type is still a cone, and a colourless
/// pipeline has no business discarding a cone for not naming a colour.
ConeSize coneSizeFromString(std::string_view type);

// ---------------------------------------------------------------------- grid

/// Uniform hash grid over a static point set.
///
/// The centreline trace and the ICP correspondence search both ask "what is
/// near here?" once per point per iteration; answering that with a scan over
/// every candidate is what made the Python tracer quadratic. Cell size should
/// be the query radius the caller uses most.
class Grid
{
public:
  Grid() = default;

  Grid(std::vector<Vec2> points, double cell)
  : points_(std::move(points)), cell_(cell > 1e-6 ? cell : 1.0)
  {
    for (std::size_t i = 0; i < points_.size(); ++i) {
      cells_[key(points_[i])].push_back(static_cast<int>(i));
    }
  }

  /// Calls f(index) for every point in the cells covering the query disc. The
  /// cover is a slight overestimate; callers re-test the exact distance.
  template<typename F>
  void forEachNear(Vec2 p, double radius, F && f) const
  {
    const int span = static_cast<int>(std::floor(radius / cell_)) + 1;
    const auto [cx, cy] = coords(p);
    for (int dx = -span; dx <= span; ++dx) {
      for (int dy = -span; dy <= span; ++dy) {
        const auto it = cells_.find(hash(cx + dx, cy + dy));
        if (it == cells_.end()) {continue;}
        for (const int i : it->second) {f(i);}
      }
    }
  }

  /// Index of the closest point within `radius`, or -1 if the disc is empty.
  int nearest(Vec2 p, double radius) const
  {
    int best = -1;
    double best_d2 = radius * radius;
    forEachNear(
      p, radius, [&](int i) {
        const double d2 = squaredNorm(points_[i] - p);
        if (d2 < best_d2) {best_d2 = d2; best = i;}
      });
    return best;
  }

  const std::vector<Vec2> & points() const {return points_;}
  bool empty() const {return points_.empty();}

private:
  static std::int64_t hash(int cx, int cy)
  {
    return (static_cast<std::int64_t>(cx) << 32) ^ static_cast<std::uint32_t>(cy);
  }

  std::pair<int, int> coords(Vec2 p) const
  {
    return {
      static_cast<int>(std::floor(p.x / cell_)),
      static_cast<int>(std::floor(p.y / cell_))};
  }

  std::int64_t key(Vec2 p) const
  {
    const auto [cx, cy] = coords(p);
    return hash(cx, cy);
  }

  std::vector<Vec2> points_;
  double cell_{1.0};
  std::unordered_map<std::int64_t, std::vector<int>> cells_;
};

// ------------------------------------------------------------------ delaunay

using Triangle = std::array<int, 3>;

/// Delaunay triangulation by Bowyer-Watson, returning index triples into
/// `points`. Degenerate input (fewer than three points, or an all-collinear
/// set) yields an empty triangulation rather than throwing.
std::vector<Triangle> delaunay(const std::vector<Vec2> & points);

}  // namespace mirena::planning
