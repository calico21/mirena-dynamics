#include "mirena_planning/unknown/unknown_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_set>
#include <utility>

namespace mirena::planning
{
namespace
{

/// One edge of the triangulation, considered as a way through the track.
struct GateEdge
{
  Vec2 mid{};
  /// Unit direction of the edge itself. An edge has two endpoints and no
  /// preferred order, so the sign is arbitrary and everything reading it must be
  /// indifferent to a flip.
  Vec2 span{};
  double width{0.0};
  int a{-1}, b{-1};
};

/// The way the car faces on the line.
///
/// The map frame is anchored at the car's start pose, so this is +x by
/// construction rather than by measurement. It is read once, to work out which
/// pair of big cones is the gate ahead and which way that gate is driven
/// through, and it is the only thing about the car the planner ever asks for.
constexpr Vec2 kFacing{1.0, 0.0};

/// Every edge of the triangulation, deduplicated -- adjacent triangles share an
/// edge, so each one is found twice.
///
/// Nothing is thrown away. Whether an edge is a gate is not a property of the
/// edge, so it is not a question this could answer; the search answers it.
std::vector<GateEdge> collectEdges(const std::vector<Vec2> & cones, PlanDebug * debug)
{
  const bool verbose = debug && debug->verbose;
  std::vector<GateEdge> edges;
  std::unordered_set<std::int64_t> seen;

  for (const Triangle & tri : delaunay(cones)) {
    for (int e = 0; e < 3; ++e) {
      int a = tri[e], b = tri[(e + 1) % 3];
      if (a > b) {std::swap(a, b);}
      if (!seen.insert((static_cast<std::int64_t>(a) << 32) | static_cast<std::uint32_t>(b)).second) {
        continue;
      }
      if (verbose) {debug->edges.emplace_back(cones[a], cones[b]);}
      edges.push_back(
        {(cones[a] + cones[b]) * 0.5, normalized(cones[b] - cones[a]),
          distance(cones[a], cones[b]), a, b});
    }
  }
  return edges;
}

}  // namespace

UnknownPlanner::UnknownPlanner(UnknownPlannerParams params)
: params_(params) {}

void UnknownPlanner::reset()
{
  closed_ = false;
  closure_votes_ = 0;
}

std::optional<Plan> UnknownPlanner::plan(
  const std::vector<Observation> & observations, PlanDebug * debug)
{
  // No start line, no plan. The big cones are what the lap is hung off both
  // ends of, and guessing at the origin instead would be inventing a gate that
  // was never pegged out.
  const Seed seed = findSeed(observations);
  if (debug) {
    debug->has_start = seed.found;
    debug->start = seed.mid;
  }
  if (!seed.found) {return std::nullopt;}

  const Trace traced = trace(observations, seed, debug);
  if (traced.path.size() < 2) {return std::nullopt;}

  closed_ = closureFilter(traced.closed);

  Plan plan;
  plan.gates = buildGates(traced.path, closed_);
  plan.closed = closed_;
  plan.start = seed.mid;
  plan.finish = closed_ ? seed.mid : traced.path.back();
  return plan;
}

UnknownPlanner::Seed UnknownPlanner::findSeed(
  const std::vector<Observation> & observations) const
{
  std::vector<Vec2> big;
  for (const Observation & o : observations) {
    if (o.size == ConeSize::Big && norm(o.p) <= params_.start_gate_radius) {
      big.push_back(o.p);
    }
  }

  Seed seed;
  double nearest = 0.0;
  for (std::size_t i = 0; i < big.size(); ++i) {
    for (std::size_t j = i + 1; j < big.size(); ++j) {
      // Either side of the way the car looks, or it is not a gate the car is
      // about to drive through. Four big cones mark the line, two down each
      // side, and this is what tells the pair spanning it from the pair running
      // down one edge of it.
      if (cross(kFacing, big[i]) * cross(kFacing, big[j]) >= 0.0) {continue;}

      const Vec2 mid = (big[i] + big[j]) * 0.5;
      if (dot(mid, kFacing) < 0.0) {continue;}

      const double reach = norm(mid);
      if (seed.found && reach >= nearest) {continue;}

      // Driven square to itself, in the sense the car is already pointing. The
      // gate says which line the lap starts on; the car says which way along it.
      const Vec2 span = normalized(big[j] - big[i]);
      Vec2 heading{-span.y, span.x};
      if (dot(heading, kFacing) < 0.0) {heading = heading * -1.0;}

      seed = {mid, heading, big[i], big[j], true};
      nearest = reach;
    }
  }
  return seed;
}

UnknownPlanner::Trace UnknownPlanner::trace(
  const std::vector<Observation> & observations, const Seed & seed,
  PlanDebug * debug) const
{
  // Small cones only. The big ones are the seed, and a triangulation that
  // includes them offers the trace a way back out through the line it started
  // on, which is not a gate of the track at all.
  std::vector<Vec2> cones;
  cones.reserve(observations.size());
  for (const Observation & o : observations) {
    if (o.size != ConeSize::Big) {cones.push_back(o.p);}
  }
  if (cones.size() < 3) {return {};}
  if (debug && debug->verbose) {debug->cones = cones;}

  std::vector<GateEdge> edges = collectEdges(cones, debug);

  // The seed joins the candidates as an ordinary edge, on two cones of its own
  // that no triangle uses. It is what the walk stands on at the start and what
  // it is looking for at the end, and being in the list is how it can be found.
  // Widest an edge can be and still be a way across the track: the track itself,
  // at its widest. A gate is the ground between two cones facing each other
  // across the track, so its width is the track's width and nothing else.
  //
  // Not a discriminator, and never was. It cannot tell a gate from an edge
  // running down a boundary, because the two overlap: cones sit 2.95 m to 4.89 m
  // apart down a boundary on these tracks and the track is 3.00 m across, so no
  // length separates them. The cost function does that.
  //
  // What it does rule out is the chord across the inside of a hairpin. Where the
  // track doubles back, the ground enclosed by the turn carries no cones, and the
  // triangulation spans that void with edges running from one pass to the other.
  // Those are not gates of anything -- there is no track between their cones --
  // but they sit square to the trace's heading and they lead somewhere, so the
  // cost function likes them and the walk drives straight through the hairpin
  // instead of round it.
  //
  // Length is what separates them, and cleanly. On a hairpin whose passes run
  // 5.0 m to 6.5 m apart the trace cuts, and the gates it cuts through measure
  // 4.88 m to 5.21 m; on the same hairpin opened to 7 m and beyond it goes round,
  // and every gate it drives is 3.00 m to the centimetre. Honest gates are the
  // track's width. The chords are half as wide again, at least.
  //
  // The price is the diagonal bridging an unmapped cone, which reaches along the
  // track as well as across and so is wider than the track by definition. Those
  // are gone, and on a frozen map with 6% of the cones deleted that costs real
  // coverage. On the moving map it costs much less than it looks: the planner
  // re-plans several times a second against a map that is densest exactly where
  // the car is about to drive, so a hole is crossed by waiting a beat for it to
  // fill rather than by reaching over it. Cutting a hairpin is not recoverable
  // that way.
  const double max_width = params_.max_track_width;

  // And the floor, which is a different kind of thing and carries a different
  // kind of number. The ceiling is a real discriminator -- it is what refuses the
  // hairpin chord. The floor refuses nothing the tracks here contain: swept from
  // 0 to 2.50 it changes not one figure on any fixture. It is set for the edge
  // between two landmarks the mapper failed to merge, which is a defect upstream
  // and has no width worth speaking of.
  //
  // It must stay well under the rules' 3 m. This tests a measurement, not the
  // track, and the measurement reads short: the narrowest gate the trace really
  // drives is 2.999 m on a perfect map and 2.469 m on a degraded one.
  const double min_width = params_.min_track_width;
  const double width = distance(seed.a, seed.b);

  const int seed_index = static_cast<int>(edges.size());
  const int seed_a = static_cast<int>(cones.size());
  cones.push_back(seed.a);
  cones.push_back(seed.b);
  edges.push_back({seed.mid, normalized(seed.b - seed.a), width, seed_a, seed_a + 1});

  std::vector<Vec2> midpoints;
  midpoints.reserve(edges.size());
  for (const GateEdge & e : edges) {midpoints.push_back(e.mid);}
  const Grid grid(midpoints, params_.max_gate_gap);

  // Spent cones, not spent gates. A cone is one side of the track at one place
  // along it; it cannot also be the other side somewhere else. So driving
  // through a gate uses both of its cones up, and every other edge touching
  // either of them -- every edge adjacent to the one just driven -- stops being
  // a way through.
  //
  // This is what rules out the zigzag, and it rules it out rather than merely
  // pricing it: the edge running down the boundary from the gate just driven
  // shares a cone with it, so it is not an option the search has to weigh and
  // reject -- it is not there. The diagonals leaning back the way the trace came
  // go the same way. What survives is the gates ahead, and the diagonal that
  // bridges an unmapped cone among them, because that one is made of two cones
  // nothing has spent.
  std::vector<char> spent(cones.size(), 0);

  /// The gate the walk is standing on, and how it arrived. This is its whole
  /// state -- not a pose. Where the car is and which way it points are not
  /// inputs to any of this.
  struct Step
  {
    Vec2 mid{};
    Vec2 span{};      ///< orientation of the gate itself
    Vec2 heading{};   ///< tangent of the centreline arriving here, gate to gate
  };

  /// What driving from here through one gate is worth: one gate, less what the
  /// centreline had to bend and slew to line up with it. Anything not worth
  /// driving is worth nothing.
  const auto worth = [&](const Step & from, int index) {
      const GateEdge & gate = edges[index];
      if (spent[gate.a] || spent[gate.b]) {return 0.0;}
      if (gate.width < min_width || gate.width > max_width) {return 0.0;}

      const Vec2 travel = gate.mid - from.mid;
      const double reach = norm(travel);
      if (reach < 1e-6 || reach > params_.max_gate_gap) {return 0.0;}

      const Vec2 onward = travel * (1.0 / reach);

      // The curvature of the centreline: the angle between the tangent arriving
      // at the last gate and the tangent leaving it. 0 for carrying straight on,
      // 1 for a right angle, 2 for doubling back, so a reversal back up the
      // track is priced out without having to be forbidden.
      const double turn = 1.0 - dot(onward, from.heading);

      // Whether the step crossed its gates or ran along them: 0 for a step
      // square to both, 1 for one straight down their line.
      //
      // Curvature cannot see this, in any form, and the reason is not that the
      // measure is too crude -- it is that a boundary is itself a smooth curve
      // parallel to the centreline. A path riding one has the same tangents and
      // the same turn radii as the real line; the two differ in where they lie,
      // not in how they bend, and bending is all curvature reads. Nor is the
      // step onto a boundary sharp enough to catch on its radius: a metre and a
      // half sideways over four metres forward is a six metre corner, gentler
      // than any hairpin the track is allowed to have.
      //
      // What does differ is this. A gate is a length of ground between two
      // cones, and the centreline passes between them, across the gate. A path
      // down a boundary runs along its gates instead, from end to end. That is
      // the whole of what is wrong with it, and this is that fact measured.
      //
      // It cannot be left to gate width either. On the sim's own tracks cones
      // are set out 2.95 m to 4.89 m apart down a boundary and the track is
      // 3.00 m across, so the two overlap and no length test can separate them.
      //
      // Read against both gates and averaged. Against the one behind alone a
      // diagonal poisons the step after it: a diagonal bridging a missing cone
      // is a real gate lying a good 30 degrees off square, so the honest gate
      // following it reads as a hard sideways slide and is priced out, which
      // strands the trace at exactly the holes diagonals exist to cross. Sharing
      // the reading halves what a skewed gate is charged at either end of it,
      // and still cannot let a boundary chain through, where both gates read 1.
      const double slide = 0.5 *
        (std::abs(dot(onward, from.span)) + std::abs(dot(onward, gate.span)));

      return 1.0 - params_.curvature_weight * (turn + slide);
    };

  /// The best that can be earned in the next `depth` steps from here.
  ///
  /// The recursion is the point. What a step is worth is what it earns now plus
  /// the best of what it leaves available, so a step onto the wrong boundary --
  /// cheap to take, expensive to leave, because everything after it has to turn
  /// back or dead-end -- loses to the one that keeps the track open. Judged one
  /// step at a time, the two are indistinguishable.
  const std::function<std::pair<int, double>(const Step &, int)> look =
    [&](const Step & from, int depth) -> std::pair<int, double> {
      std::pair<int, double> best{-1, 0.0};
      if (depth <= 0) {return best;}

      grid.forEachNear(
        from.mid, params_.max_gate_gap, [&](int index) {
          const double value = worth(from, index);
          if (value <= 0.0) {return;}

          double total = value;
          // Nothing follows the seed. Reaching it is the lap ending, so the
          // gates beyond it are the ones already driven, and letting the
          // recursion count them again would have the walk bidding for a
          // second lap laid on top of the first.
          if (index != seed_index) {
            const GateEdge & gate = edges[index];
            spent[gate.a] = spent[gate.b] = 1;
            const Vec2 onward = normalized(gate.mid - from.mid);
            total += look({gate.mid, gate.span, onward}, depth - 1).second;
            spent[gate.a] = spent[gate.b] = 0;
          }

          if (total > best.second) {best = {index, total};}
        });
      return best;
    };

  Trace out;
  Step step{seed.mid, edges[seed_index].span, seed.heading};
  out.path.push_back(step.mid);

  while (static_cast<int>(out.path.size()) < params_.max_path_points) {
    const int index = look(step, params_.lookahead).first;
    if (index < 0) {break;}

    // Back on the gate it set off from, having driven every other gate it could
    // reach on the way. That is a lap, and there is nothing after it.
    if (index == seed_index) {
      out.closed = true;
      break;
    }

    const GateEdge & gate = edges[index];
    spent[gate.a] = spent[gate.b] = 1;
    step = {gate.mid, gate.span, normalized(gate.mid - step.mid)};
    out.path.push_back(step.mid);
    if (debug && debug->verbose) {
      debug->gate_edges.emplace_back(cones[gate.a], cones[gate.b]);
    }
  }

  if (debug && debug->verbose) {debug->midpoints = out.path;}
  return out;
}

bool UnknownPlanner::closureFilter(bool detected)
{
  if (detected == closed_) {
    closure_votes_ = 0;
  } else if (++closure_votes_ >= params_.closure_hysteresis) {
    closed_ = detected;
    closure_votes_ = 0;
  }
  return closed_;
}

}  // namespace mirena::planning
