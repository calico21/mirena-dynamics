// Centreline planner for missions where the track is discovered on the fly
// (trackdrive, autocross).

#pragma once

#include <optional>
#include <vector>

#include "mirena_planning/planner.hpp"

namespace mirena::planning
{

struct UnknownPlannerParams
{
  /// The narrowest an edge may be and still be a gate, in metres.
  ///
  /// Deliberately not the rules' 3 m floor, and this is the one bound that must
  /// not be set to its rulebook value. The rules constrain the *track*; this
  /// tests a *measurement* of it, taken across two perceived cones. On a map
  /// carrying 0.10 m of landmark error that measurement carries about 0.14 m of
  /// its own, and it reads short as often as long.
  ///
  /// Measured rather than assumed: with the floor off, the narrowest gate the
  /// trace actually drives is 2.999 m on a perfect map and 2.469 m on a degraded
  /// one. So a floor at 3.00 rejects real gates even on a perfect map, and does
  /// it catastrophically -- coverage falls to 92.6%, closure to 4 of 10, and the
  /// centreline ends up 1.50 m out, which is on the cones. At 2.90 the degraded
  /// map loses half its closures and its excursions double.
  ///
  /// Anything at or below 2.50 is indistinguishable from having no floor at all
  /// on every fixture here, hairpin included -- the hairpin is the max's problem,
  /// not this one. 2.0 sits well clear of the 2.469 m worst case with room for a
  /// noisier map than the one tested.
  ///
  /// So this is a rail, not a filter. It fires on the degenerate edge -- two
  /// landmarks the mapper has failed to merge, a duplicate cone centimetres from
  /// its twin -- and on nothing else. That is the whole of what it is for.
  double min_track_width{2.0};

  /// The widest the track may get, in metres, and so the widest an edge may be
  /// and still be a gate. The rules give the floor and leave the ceiling open, so
  /// this one is a judgement rather than a citation.
  ///
  /// What it is for is the hairpin. Where the track doubles back, the ground
  /// inside the turn carries no cones, the triangulation spans it, and those
  /// spanning edges are not gates of anything -- there is no track between their
  /// cones. Nothing else rejects them: they sit square to the trace and they lead
  /// somewhere, which is all the cost function asks, so the walk drives straight
  /// through the hairpin instead of round it.
  ///
  /// Width is what separates them. On a synthetic hairpin whose passes run 5.0 m
  /// to 6.5 m apart the trace cuts, and the gates it cuts through measure 4.88 m
  /// to 5.21 m; opened to 7 m and beyond it goes round, and every gate it drives
  /// is 3.00 m to the centimetre. An honest gate is the track's width, and the
  /// chords are half as wide again at least.
  ///
  /// 5.0 is the loosest setting that still refuses them -- the cut on the 6.0 m
  /// hairpin survives at 5.5 and dies at 5.0. Tighter buys nothing and costs
  /// more, so this sits at the top of the range that works rather than in the
  /// middle of it.
  double max_track_width{5.0};

  /// Furthest apart two cones down one side of the track may be, in metres.
  /// Formula Student sets a ceiling of 5 m.
  ///
  /// With max_track_width this fixes the widest an edge may be and still be a way
  /// across the track: the longest honest one is the diagonal bridging an
  /// unmapped cone, which reaches two stations along one side and the width of
  /// the track across it.
  double cone_spacing{5.0};

  /// Furthest apart two consecutive gates may be, in metres.
  ///
  /// The one length here the rules cannot settle, and the only reason this is a
  /// parameter rather than a derivation.
  ///
  /// Wanted large. Gates are a cone spacing apart only while every cone is in the
  /// map; lose one and the next gate the trace has not already spent is two
  /// stations away, so the rules would put this at twice the cone spacing, 10 m.
  ///
  /// What overrules them is how close the track runs to itself. Reach further
  /// than the gap between one pass of the track and the next and the search can
  /// step across into the wrong one, which it will do readily: the two passes run
  /// parallel there, so the step is straight and costs nothing to take. On the
  /// exported tracks the tightest such pinch is 9.06 m -- 9.1 m on three of the
  /// ten and never under 18 m on the other seven -- and at the rules' 10 m the
  /// trace crosses it and lands 5.1 m off the line on a complete map. Hence
  /// 8.5 m, under the pinch.
  ///
  /// That is half a metre of headroom, held against the track rather than against
  /// the car, and nothing in the rules guarantees it. A layout folding back
  /// tighter than about 9 m would have no setting that both bridges a missing
  /// cone and refuses the neighbouring pass. The durable answer is to bound a
  /// step by the triangulation -- how many edges it crosses -- rather than by a
  /// distance in metres.
  double max_gate_gap{8.5};

  /// What bending the centreline costs, against the one gate a step earns.
  ///
  /// A step earns 1 for the gate it drives through and pays this many times the
  /// angle between the tangent it arrived on and the tangent it leaves on: 0 for
  /// carrying straight on, 1 for a right angle, 2 for doubling back, so a
  /// reversal is priced out without having to be forbidden. That is also the only
  /// test of whether a step is worth taking -- a step nobody would drive is
  /// already worth less than staying put, so no separate threshold is wanted.
  ///
  /// Curvature alone cannot keep the trace off the boundaries, and no sharpening
  /// of it can. A boundary is itself a smooth curve running parallel to the
  /// centreline: a path riding one has the same tangents and the same turn radii
  /// as the real line, and differs only in where it lies. Bending is all
  /// curvature reads. So the same weight also pays for how squarely the step
  /// crossed its gates rather than running along them, 0 to 1, which is the one
  /// thing the two do not share -- the centreline passes between a gate's cones,
  /// a boundary runs along from one to the next.
  ///
  /// 1.0 sits mid-plateau: anything from 0.8 to 1.15 keeps the centreline inside
  /// the cones on every map tested, with the same margin.
  double curvature_weight{1.0};

  /// How many gates ahead the search looks before committing to the next one.
  ///
  /// This is what makes the count of gates worth anything. Judged one step at a
  /// time every edge earns the same 1, so nothing separates the gate that opens
  /// the track up from the one that dead-ends two gates later; only looking past
  /// them tells them apart. The cost of looking is exponential in this, so it
  /// wants to stay small.
  ///
  /// Odd on purpose. A zigzag between the centreline and a boundary repeats
  /// every two steps, so an even horizon ends where it started and the detour
  /// looks free; an odd one always ends mid-zigzag and has to pay for it.
  int lookahead{5};

  /// Radius around the map origin searched for the big cones of the start line.
  /// The map frame is anchored at the car's start pose, so the line is just
  /// ahead of the origin; this is what stops an autocross finish gate being
  /// taken for the start.
  double start_gate_radius{10.0};

  /// Consecutive agreeing detections needed to flip the published closed flag.
  /// Without it the flag chatters and the controller keeps re-fitting its spline
  /// between periodic and open.
  int closure_hysteresis{5};

  /// Hard cap on the walk, so a pathological cone set cannot spin forever.
  int max_path_points{400};
};

/// Colourless centreline finder.
///
/// Three steps, and the first two are what make the third simple.
///
/// The seed gate is the pair of big orange cones the car is facing. The map
/// frame is the pose the car started from, so it stands at the origin looking
/// down +x at the start line, and the pair of big cones lying either side of
/// that line of sight is the gate it is about to drive through. That fixes both
/// ends of what the cones alone cannot say: where the lap starts, and which way
/// round it goes.
///
/// The triangulation is then built from the small cones only. The big cones are
/// the seed, not part of the track's fabric, and leaving them in only invites
/// the trace to turn round and come back out through the line it started on.
///
/// What is left is one question asked over and over: of the edges that touch no
/// cone already spent, which one carries on straightest? Drive through as many
/// as possible, bending as little as possible. Every edge is a candidate,
/// including the ones running down a boundary, and they lose on their own merits
/// rather than being ruled out -- an edge along the track leads to a midpoint off
/// to one side, and everything after it has to turn back or dead-end, so over a
/// horizon of a few gates it simply counts fewer of them.
///
/// Nothing in that depends on where the car is. The centreline is a property of
/// the cones, and asking the same cones twice from two different car poses has
/// to give the same line or the line was never the track. The car appears once,
/// in the seed, and never again.
///
/// The lap is closed when the search comes back to the seed gate. Not a radius
/// and not a ratio -- the one edge that means the lap is over is the one it
/// started on, so finding it is the answer rather than evidence for it.
class UnknownPlanner : public Planner
{
public:
  explicit UnknownPlanner(UnknownPlannerParams params = {});

  std::optional<Plan> plan(
    const std::vector<Observation> & observations, PlanDebug * debug) override;
  void reset() override;

  bool closed() const {return closed_;}

private:
  /// The gate the lap starts and ends on, and the way it is driven through.
  struct Seed
  {
    Vec2 mid{};
    Vec2 heading{};   ///< direction of traversal, square to the gate
    Vec2 a{}, b{};    ///< the two big cones themselves
    bool found{false};
  };

  struct Trace
  {
    std::vector<Vec2> path;
    bool closed{false};
  };

  /// The first valid pair of big cones: the nearest two, ahead of the car, that
  /// lie either side of the way it faces. Cones on the same side of that line
  /// are the two halves of one side of the start line, not a gate across it.
  Seed findSeed(const std::vector<Observation> & observations) const;

  Trace trace(
    const std::vector<Observation> & observations, const Seed & seed,
    PlanDebug * debug) const;

  bool closureFilter(bool detected);

  UnknownPlannerParams params_;
  bool closed_{false};
  int closure_votes_{0};
};

}  // namespace mirena::planning
