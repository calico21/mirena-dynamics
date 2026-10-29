// The planning contract: cones in, an ordered centreline out.

#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "mirena_planning/geometry.hpp"

namespace mirena::planning
{

/// One mapped cone, in the frame the map arrived in.
struct Observation
{
  Vec2 p{};
  ConeSize size{ConeSize::Small};
};

/// A centreline sample: position plus the tangent heading at that point.
struct Gate
{
  Vec2 p{};
  double psi{0.0};
};

struct Plan
{
  std::vector<Gate> gates;
  bool closed{false};
  Vec2 start{};    ///< start/finish gate
  Vec2 finish{};   ///< where the run ends; equal to `start` on a closed track
};

/// Working state a planner exposes about itself.
///
/// The scalars are cheap enough to fill on every cycle and end up in the
/// heartbeat, which is all the diagnosis available on a car with no screen. The
/// geometry below them is only collected when `verbose` is set, so the cost of
/// the debug topics stays with whoever is actually watching them.
struct PlanDebug
{
  bool verbose{false};

  // Unknown-track planner.
  std::vector<Vec2> cones;                        ///< the triangulated set
  std::vector<std::pair<Vec2, Vec2>> edges;       ///< every Delaunay edge
  std::vector<std::pair<Vec2, Vec2>> gate_edges;  ///< edges accepted as gates
  std::vector<Vec2> midpoints;                    ///< gate centres, unordered
  Vec2 start{};
  bool has_start{false};

  // Known-track planner.
  std::vector<Observation> fitted_template;             ///< template in map frame
  std::vector<Vec2> fitted_path;                        ///< template path in map frame
  std::vector<std::pair<Vec2, Vec2>> correspondences;   ///< observation -> template
  std::vector<Vec2> rejected;                           ///< observations with no match
  Transform2 fit{};
  double score{0.0};
  int iterations{0};
  bool from_ransac{false};
  bool registered{false};
};

class Planner
{
public:
  virtual ~Planner() = default;

  /// Returns no plan when the observations do not support one yet; that is a
  /// normal early-run state, not an error.
  virtual std::optional<Plan> plan(
    const std::vector<Observation> & observations, PlanDebug * debug) = 0;

  /// Drops everything learned so far. Called on every lifecycle configure so a
  /// second mission never inherits the first one's track.
  virtual void reset() = 0;
};

/// Turns an ordered centreline into gates, tangent heading included.
///
/// Interior headings use a central difference, which is noticeably steadier
/// through corners than differencing only towards the next point.
std::vector<Gate> buildGates(const std::vector<Vec2> & points, bool closed);

}  // namespace mirena::planning
