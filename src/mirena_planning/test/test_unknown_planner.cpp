#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "mirena_planning/unknown/unknown_planner.hpp"

using namespace mirena::planning;  // NOLINT(build/namespaces)

namespace
{

constexpr double kRadius = 35.0;
constexpr double kWidth = 3.0;
/// Cones are set out further apart than the track is wide, which is how tracks
/// are actually pegged out and what the sim's generator does. It matters here:
/// at a spacing equal to the width, an edge running down a boundary is exactly
/// as long as a gate, and no planner working from lengths alone can tell them
/// apart.
constexpr double kSpacing = 4.0;

/// A closed circuit driven anticlockwise from the map origin.
///
/// The map frame is the pose the car started from, so the start gate sits at
/// the origin with the car facing +x, and the circuit curves away to the left.
std::vector<Observation> ovalTrack(
  double from_angle = 0.0, double to_angle = 2.0 * M_PI, double radius = kRadius)
{
  const Vec2 centre{0.0, radius};
  std::vector<Observation> cones{
    {{0.0, kWidth / 2.0}, ConeSize::Big},
    {{0.0, -kWidth / 2.0}, ConeSize::Big}};

  const double step = kSpacing / radius;
  for (double a = from_angle + step; a < to_angle; a += step) {
    const Vec2 radial{std::sin(a), -std::cos(a)};
    cones.push_back({centre + radial * (radius - kWidth / 2.0), ConeSize::Small});
    cones.push_back({centre + radial * (radius + kWidth / 2.0), ConeSize::Small});
  }
  return cones;
}

/// A straight corridor with its own big orange gate at the far end, the layout
/// that used to drag the start gate into the middle of the track.
std::vector<Observation> straightTrack()
{
  std::vector<Observation> cones{
    {{0.0, kWidth / 2.0}, ConeSize::Big},
    {{0.0, -kWidth / 2.0}, ConeSize::Big},
    {{60.0, kWidth / 2.0}, ConeSize::Big},
    {{60.0, -kWidth / 2.0}, ConeSize::Big}};

  for (double x = kSpacing; x <= 57.0; x += kSpacing) {
    cones.push_back({{x, kWidth / 2.0}, ConeSize::Small});
    cones.push_back({{x, -kWidth / 2.0}, ConeSize::Small});
  }
  return cones;
}

}  // namespace

TEST(UnknownPlanner, TracesTheCentrelineOfAClosedCircuit)
{
  UnknownPlanner planner;
  const auto plan = planner.plan(ovalTrack(), nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_GT(plan->gates.size(), 25u);

  const Vec2 centre{0.0, kRadius};
  for (const Gate & gate : plan->gates) {
    // Inside the cones, which is the promise the controller depends on. Not
    // tighter: a diagonal gate's midpoint sits off the centre of the track, and
    // diagonals are kept because where a cone is missing they are the only gate
    // there is.
    EXPECT_NEAR(distance(gate.p, centre), kRadius, kWidth / 2.0);
  }
}

TEST(UnknownPlanner, OrdersGatesWithoutJumpingAcrossTheTrack)
{
  UnknownPlanner planner;
  const auto plan = planner.plan(ovalTrack(), nullptr);
  ASSERT_TRUE(plan.has_value());

  const Vec2 centre{0.0, kRadius};
  const Vec2 first = plan->gates.front().p - centre;
  double previous = std::atan2(first.y, first.x);
  double travelled = 0.0;
  for (std::size_t i = 1; i < plan->gates.size(); ++i) {
    EXPECT_LT(distance(plan->gates[i].p, plan->gates[i - 1].p), 8.0);

    const Vec2 radial = plan->gates[i].p - centre;
    const double angle = std::atan2(radial.y, radial.x);
    const double delta = wrapAngle(angle - previous);
    // A cross-track jump or a doubling back shows up here as a step that is
    // not a small advance in the same direction.
    EXPECT_GT(delta, 0.0);
    EXPECT_LT(delta, 0.6);
    travelled += delta;
    previous = angle;
  }
  EXPECT_GT(travelled, 2.0 * M_PI - 0.6);
}

TEST(UnknownPlanner, HoldsTheClosedFlagUntilTheDetectionIsStable)
{
  UnknownPlannerParams params;
  params.closure_hysteresis = 5;
  UnknownPlanner planner(params);

  const auto cones = ovalTrack();
  for (int i = 0; i < 4; ++i) {
    const auto plan = planner.plan(cones, nullptr);
    ASSERT_TRUE(plan.has_value());
    EXPECT_FALSE(plan->closed) << "flipped after only " << i + 1 << " agreeing cycles";
  }
  const auto settled = planner.plan(cones, nullptr);
  ASSERT_TRUE(settled.has_value());
  EXPECT_TRUE(settled->closed);
  EXPECT_NEAR(distance(settled->start, settled->finish), 0.0, 1e-9);
}

/// The whole point of searching instead of walking: neither of these has an
/// intact corridor, and a greedy tracer resolves the first ambiguity by
/// committing to it.
TEST(UnknownPlanner, TracesOnThroughGapsInABoundary)
{
  // Perception misses cones at range as a matter of course, so a tracer that
  // needs both sides of the track intact loses the track constantly.
  std::vector<Observation> cones;
  int index = 0;
  for (const Observation & cone : ovalTrack()) {
    // Drop every third cone of the outer boundary, keeping the inner one.
    const bool outer = distance(cone.p, Vec2{0.0, kRadius}) > kRadius;
    if (outer && cone.size == ConeSize::Small && (index++ % 3) == 0) {continue;}
    cones.push_back(cone);
  }

  UnknownPlanner planner;
  const auto plan = planner.plan(cones, nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_GT(plan->gates.size(), 20u);
  for (const Gate & gate : plan->gates) {
    EXPECT_NEAR(distance(gate.p, Vec2{0.0, kRadius}), kRadius, kWidth / 2.0);
  }
}

/// The gate a missing cone leaves behind is a long diagonal, and it is still a
/// gate: it reaches the full width of the track, and its midpoint is on the
/// centreline. Only its *length* is unusual, and all of that extra length is
/// along the track rather than across it.
///
/// This is the case a squareness test gets wrong, and gets wrong in the worst
/// possible way -- the diagonal is exactly as skewed as it is long, so the
/// hungrier perception is for cones the more certainly the gate bridging the
/// hole is thrown out, and the trace stops dead at the first dropout. Judged on
/// how far it reaches across the track instead, it is simply a way through.
TEST(UnknownPlanner, BridgesAMissingConeWithTheDiagonalThatReplacesIt)
{
  std::vector<Observation> cones;
  for (const Observation & cone : straightTrack()) {
    // One cone, one side, a third of the way down an otherwise perfect corridor.
    if (cone.size == ConeSize::Small && std::abs(cone.p.x - 20.0) < 0.1 && cone.p.y < 0.0) {
      continue;
    }
    cones.push_back(cone);
  }

  UnknownPlanner planner;
  const auto plan = planner.plan(cones, nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_GT(plan->gates.back().p.x, 50.0) << "the trace stopped at the hole";
  for (const Gate & gate : plan->gates) {
    EXPECT_NEAR(gate.p.y, 0.0, 0.5) << "wandered off the centreline at x = " << gate.p.x;
  }
}

TEST(UnknownPlanner, IsNotDraggedOffTheTrackByStrayCones)
{
  // Cones that are not track boundary: knocked over, carried off, or a cluster
  // the detector invented. They triangulate together with the real ones.
  std::vector<Observation> cones = ovalTrack();
  for (const double angle : {0.6, 1.4, 2.5, 4.0}) {
    const Vec2 radial{std::sin(angle), -std::cos(angle)};
    cones.push_back({Vec2{0.0, kRadius} + radial * (kRadius + 5.0), ConeSize::Small});
    cones.push_back({Vec2{0.0, kRadius} + radial * (kRadius - 5.5), ConeSize::Small});
  }

  UnknownPlanner planner;
  const auto plan = planner.plan(cones, nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_GT(plan->gates.size(), 20u);
  for (const Gate & gate : plan->gates) {
    EXPECT_NEAR(distance(gate.p, Vec2{0.0, kRadius}), kRadius, kWidth / 2.0);
  }
}

/// Closure asks whether the path is much longer than the ground it covered, not
/// how many metres long it is, so it says the same thing about a circuit of any
/// size. Nothing here knows what a lap is supposed to measure, and a tracer that
/// did would refuse to close a test loop pegged out on a car park.
TEST(UnknownPlanner, ClosesACircuitOfAnySize)
{
  for (const double radius : {12.0, kRadius}) {
    UnknownPlanner planner;
    const auto cones = ovalTrack(0.0, 2.0 * M_PI, radius);
    std::optional<Plan> plan;
    for (int i = 0; i < 8; ++i) {plan = planner.plan(cones, nullptr);}
    ASSERT_TRUE(plan.has_value()) << "radius " << radius;
    EXPECT_TRUE(plan->closed) << "radius " << radius;
  }
}

/// D 1.1.10 caps the car's turning circle at 9 m diameter, and the track is laid
/// out to respect it. So is the centreline: a kink tighter than that is the
/// tracer stepping off the track and back, and the controller cannot follow it
/// whether or not it is where the cones are.
TEST(UnknownPlanner, NeverAsksForACornerTighterThanTheCarCanTake)
{
  // The awkward cases, not the clean one: missing boundary cones and strays both
  // put gate midpoints off the centreline, which is where zigzags come from.
  std::vector<Observation> cones;
  int index = 0;
  for (const Observation & cone : ovalTrack()) {
    const bool outer = distance(cone.p, Vec2{0.0, kRadius}) > kRadius;
    if (outer && cone.size == ConeSize::Small && (index++ % 3) == 0) {continue;}
    cones.push_back(cone);
  }
  for (const double angle : {0.6, 1.4, 2.5, 4.0}) {
    const Vec2 radial{std::sin(angle), -std::cos(angle)};
    cones.push_back({Vec2{0.0, kRadius} + radial * (kRadius + 5.0), ConeSize::Small});
    cones.push_back({Vec2{0.0, kRadius} + radial * (kRadius - 5.5), ConeSize::Small});
  }

  UnknownPlanner planner;
  const auto plan = planner.plan(cones, nullptr);
  ASSERT_TRUE(plan.has_value());
  ASSERT_GT(plan->gates.size(), 20u);

  // Stated as a multiple of the track's own width, because that is the only
  // length the planner knows: it is never told 3 m, it measures it at the start
  // line and re-measures it at every gate.
  for (std::size_t i = 1; i + 1 < plan->gates.size(); ++i) {
    const Vec2 heading = normalized(plan->gates[i].p - plan->gates[i - 1].p);
    const double radius = turnRadius(plan->gates[i].p, heading, plan->gates[i + 1].p);
    EXPECT_GE(radius, kWidth)
      << "gate " << i << " turns at " << radius << " m radius, on a "
      << kWidth << " m track";
  }
}

TEST(UnknownPlanner, LeavesAPartialLapOpen)
{
  UnknownPlanner planner;
  const auto cones = ovalTrack(0.0, M_PI);
  for (int i = 0; i < 10; ++i) {
    const auto plan = planner.plan(cones, nullptr);
    ASSERT_TRUE(plan.has_value());
    EXPECT_FALSE(plan->closed);
  }
}

TEST(UnknownPlanner, TakesTheStartGateFromTheLineNotTheAverageOrangeCone)
{
  UnknownPlanner planner;
  const auto plan = planner.plan(straightTrack(), nullptr);
  ASSERT_TRUE(plan.has_value());

  // The far gate is big orange too; averaging both would put the start at
  // x = 30 and send the trace off from the middle of the track.
  EXPECT_NEAR(plan->start.x, 0.0, 0.5);
  EXPECT_NEAR(plan->start.y, 0.0, 0.5);
  EXPECT_FALSE(plan->closed);
  EXPECT_GT(plan->gates.back().p.x, 50.0);
}

TEST(UnknownPlanner, HeadsForwardFromTheStartLine)
{
  UnknownPlanner planner;
  const auto plan = planner.plan(straightTrack(), nullptr);
  ASSERT_TRUE(plan.has_value());
  ASSERT_GE(plan->gates.size(), 2u);
  EXPECT_GT(plan->gates[1].p.x, plan->gates[0].p.x);
  EXPECT_NEAR(plan->gates[1].psi, 0.0, 0.2);
}

TEST(UnknownPlanner, DeclinesToPlanUntilTheBigConesAreMapped)
{
  // The big cones are the seed, and the seed is not optional. They fix both of
  // the things the small cones cannot say on their own -- which gate the lap
  // starts on, and which of the two ways round it is driven -- and they are also
  // what the walk looks for to know the lap has closed.
  //
  // Rooting a guess at the origin instead would be inventing a gate nobody
  // pegged out. The cost of insisting is that there is no path at all until the
  // line has been seen, which a colourless pipeline can still do: the two
  // Formula Student cones stand 505 mm and 325 mm tall, so the big ones are
  // told apart by height, and they are the nearest cones to a car sitting on
  // the line.
  UnknownPlanner planner;
  std::vector<Observation> cones;
  for (double x = 3.0; x <= 30.0; x += 3.0) {
    cones.push_back({{x, kWidth / 2.0}, ConeSize::Small});
    cones.push_back({{x, -kWidth / 2.0}, ConeSize::Small});
  }

  EXPECT_FALSE(planner.plan(cones, nullptr).has_value());
}

TEST(UnknownPlanner, TakesTheNearestBigConePairAheadAsTheSeed)
{
  // Four big cones mark the line, two down each side, so they make more than one
  // pair. The seed is the nearest pair that spans the way the car is facing;
  // pairs running down one side of the line span nothing.
  UnknownPlanner planner;
  std::vector<Observation> cones{
    {{4.0, kWidth / 2.0}, ConeSize::Big},
    {{4.0, -kWidth / 2.0}, ConeSize::Big},
    {{5.0, kWidth / 2.0}, ConeSize::Big},
    {{5.0, -kWidth / 2.0}, ConeSize::Big}};
  for (double x = 8.0; x <= 40.0; x += kSpacing) {
    cones.push_back({{x, kWidth / 2.0}, ConeSize::Small});
    cones.push_back({{x, -kWidth / 2.0}, ConeSize::Small});
  }

  const auto plan = planner.plan(cones, nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_NEAR(plan->start.x, 4.0, 1e-9);
  EXPECT_NEAR(plan->start.y, 0.0, 1e-9);
  // and it drives away from the line, not back through it
  ASSERT_GE(plan->gates.size(), 2u);
  EXPECT_GT(plan->gates[1].p.x, plan->gates[0].p.x);
}

TEST(UnknownPlanner, DeclinesToPlanWithoutBoundaries)
{
  UnknownPlanner planner;
  const std::vector<Observation> cones{
    {{0.0, 1.5}, ConeSize::Big},
    {{0.0, -1.5}, ConeSize::Big}};
  EXPECT_FALSE(planner.plan(cones, nullptr).has_value());
}

TEST(UnknownPlanner, FillsDebugGeometryOnlyWhenAsked)
{
  UnknownPlanner planner;
  const auto cones = ovalTrack();

  PlanDebug quiet;
  ASSERT_TRUE(planner.plan(cones, &quiet).has_value());
  EXPECT_TRUE(quiet.edges.empty());
  EXPECT_TRUE(quiet.midpoints.empty());
  EXPECT_TRUE(quiet.has_start);

  PlanDebug verbose;
  verbose.verbose = true;
  ASSERT_TRUE(planner.plan(cones, &verbose).has_value());
  EXPECT_FALSE(verbose.edges.empty());
  EXPECT_FALSE(verbose.gate_edges.empty());
  EXPECT_FALSE(verbose.midpoints.empty());
  EXPECT_LT(verbose.gate_edges.size(), verbose.edges.size());
}

TEST(UnknownPlanner, ResetForgetsTheClosedTrack)
{
  UnknownPlanner planner;
  const auto cones = ovalTrack();
  for (int i = 0; i < 6; ++i) {planner.plan(cones, nullptr);}
  ASSERT_TRUE(planner.closed());

  planner.reset();
  EXPECT_FALSE(planner.closed());
  const auto plan = planner.plan(cones, nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_FALSE(plan->closed);
}
