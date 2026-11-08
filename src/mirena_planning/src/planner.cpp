#include "mirena_planning/planner.hpp"

namespace mirena::planning
{

std::vector<Gate> buildGates(const std::vector<Vec2> & points, bool closed)
{
  const std::size_t n = points.size();
  std::vector<Gate> gates;
  gates.reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    Vec2 tangent{};
    if (n >= 2) {
      const bool has_prev = i > 0 || closed;
      const bool has_next = i + 1 < n || closed;
      const Vec2 & prev = points[(i + n - 1) % n];
      const Vec2 & next = points[(i + 1) % n];
      if (has_prev && has_next) {
        tangent = next - prev;
      } else if (has_next) {
        tangent = next - points[i];
      } else {
        tangent = points[i] - prev;
      }
    }

    Gate gate;
    gate.p = points[i];
    // A zero tangent only happens on coincident samples; inherit the previous
    // heading there rather than snapping to zero.
    gate.psi = squaredNorm(tangent) > 1e-12 ?
      std::atan2(tangent.y, tangent.x) :
      (gates.empty() ? 0.0 : gates.back().psi);
    gates.push_back(gate);
  }

  return gates;
}

}  // namespace mirena::planning
