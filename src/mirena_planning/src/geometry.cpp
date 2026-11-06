#include "mirena_planning/geometry.hpp"

#include <algorithm>
#include <cctype>

namespace mirena::planning
{

namespace
{

bool containsIgnoreCase(std::string_view haystack, std::string_view needle)
{
  if (needle.empty() || needle.size() > haystack.size()) {return false;}
  const auto lower = [](char c) {
      return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    };
  for (std::size_t i = 0; i + needle.size() <= haystack.size(); ++i) {
    std::size_t j = 0;
    while (j < needle.size() && lower(haystack[i + j]) == lower(needle[j])) {++j;}
    if (j == needle.size()) {return true;}
  }
  return false;
}

struct Circumcircle
{
  Vec2 centre{};
  double radius_sq{std::numeric_limits<double>::infinity()};
};

/// Degenerate (collinear) triples get an infinite radius on purpose: they then
/// contain every query point, so the Bowyer-Watson loop below removes them at
/// the next insertion instead of leaving a broken cell in the mesh.
Circumcircle circumcircle(Vec2 a, Vec2 b, Vec2 c)
{
  const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  if (std::abs(d) < 1e-12) {return {};}

  const double a2 = squaredNorm(a), b2 = squaredNorm(b), c2 = squaredNorm(c);
  const Vec2 centre{
    (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d,
    (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d};
  return {centre, squaredNorm(a - centre)};
}

}  // namespace

// ------------------------------------------------------------------- vectors

double turnRadius(Vec2 from, Vec2 heading, Vec2 to)
{
  const Vec2 chord = to - from;
  const double d = norm(chord);
  const Vec2 h = normalized(heading);
  if (d < 1e-9 || squaredNorm(h) < 0.5) {return std::numeric_limits<double>::infinity();}

  // Tangent-chord angle: a chord subtending angle theta to the tangent spans
  // 2 * radius * sin(theta) of the circle. So the radius falls straight out of
  // the sine, which is the cross product of the two unit vectors.
  const double sin_theta = std::abs(cross(h, chord)) / d;
  if (sin_theta < 1e-9) {
    return dot(h, chord) > 0.0 ? std::numeric_limits<double>::infinity() : 0.0;
  }
  return d / (2.0 * sin_theta);
}

// --------------------------------------------------------------------- cones

ConeSize coneSizeFromString(std::string_view type)
{
  return containsIgnoreCase(type, "big") ? ConeSize::Big : ConeSize::Small;
}


// ---------------------------------------------------------------- transforms

Transform2 rigidFit(const std::vector<Vec2> & src, const std::vector<Vec2> & dst)
{
  const std::size_t n = std::min(src.size(), dst.size());
  if (n == 0) {return {};}

  Vec2 src_centroid{}, dst_centroid{};
  for (std::size_t i = 0; i < n; ++i) {
    src_centroid = src_centroid + src[i];
    dst_centroid = dst_centroid + dst[i];
  }
  src_centroid = src_centroid * (1.0 / static_cast<double>(n));
  dst_centroid = dst_centroid * (1.0 / static_cast<double>(n));

  double sin_acc = 0.0, cos_acc = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = src[i] - src_centroid;
    const Vec2 b = dst[i] - dst_centroid;
    sin_acc += cross(a, b);
    cos_acc += dot(a, b);
  }

  Transform2 fit;
  // A single correspondence (or perfectly coincident points) leaves the
  // rotation unconstrained; keep it at zero rather than reading noise.
  if (std::abs(sin_acc) > 1e-12 || std::abs(cos_acc) > 1e-12) {
    fit.psi = std::atan2(sin_acc, cos_acc);
  }
  fit.t = dst_centroid - (Transform2{fit.psi, {}} * src_centroid);
  return fit;
}

// ------------------------------------------------------------------ delaunay

std::vector<Triangle> delaunay(const std::vector<Vec2> & points)
{
  const int n = static_cast<int>(points.size());
  if (n < 3) {return {};}

  // Work on a copy carrying three synthetic vertices: a super-triangle large
  // enough to contain every input, discarded again at the end.
  std::vector<Vec2> pts = points;
  Vec2 lo = pts[0], hi = pts[0];
  for (const Vec2 & p : pts) {
    lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y);
    hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y);
  }
  const Vec2 mid = (lo + hi) * 0.5;
  const double reach = 20.0 * std::max({hi.x - lo.x, hi.y - lo.y, 1.0});
  pts.push_back({mid.x - reach, mid.y - reach});
  pts.push_back({mid.x + reach, mid.y - reach});
  pts.push_back({mid.x, mid.y + reach});

  struct Cell
  {
    Triangle tri;
    Circumcircle circle;
  };
  const auto make_cell = [&pts](int a, int b, int c) {
      return Cell{{a, b, c}, circumcircle(pts[a], pts[b], pts[c])};
    };

  std::vector<Cell> mesh{make_cell(n, n + 1, n + 2)};
  std::vector<std::pair<int, int>> cavity;

  for (int i = 0; i < n; ++i) {
    cavity.clear();
    for (std::size_t k = 0; k < mesh.size(); ) {
      const Cell & cell = mesh[k];
      if (squaredNorm(pts[i] - cell.circle.centre) <= cell.circle.radius_sq) {
        for (int e = 0; e < 3; ++e) {
          cavity.emplace_back(cell.tri[e], cell.tri[(e + 1) % 3]);
        }
        mesh[k] = mesh.back();
        mesh.pop_back();
      } else {
        ++k;
      }
    }

    // Edges shared by two removed cells are interior to the cavity; the ones
    // left over exactly once form its outline, which we fan back to the point.
    const auto same_edge = [](std::pair<int, int> a, std::pair<int, int> b) {
        return (a.first == b.first && a.second == b.second) ||
               (a.first == b.second && a.second == b.first);
      };
    for (std::size_t a = 0; a < cavity.size(); ++a) {
      bool shared = false;
      for (std::size_t b = 0; b < cavity.size() && !shared; ++b) {
        shared = (a != b) && same_edge(cavity[a], cavity[b]);
      }
      if (!shared) {
        mesh.push_back(make_cell(cavity[a].first, cavity[a].second, i));
      }
    }
  }

  std::vector<Triangle> out;
  out.reserve(mesh.size());
  for (const Cell & cell : mesh) {
    const bool synthetic = cell.tri[0] >= n || cell.tri[1] >= n || cell.tri[2] >= n;
    if (synthetic || !std::isfinite(cell.circle.radius_sq)) {continue;}
    out.push_back(cell.tri);
  }
  return out;
}

}  // namespace mirena::planning
