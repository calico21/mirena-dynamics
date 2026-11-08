// The planner against real tracks rather than hand-written fixtures.
//
// test/tracks holds layouts exported straight out of the sim by the Export Track
// button, in the same format as the stored mission templates -- cones, and the
// centreline the generator drew them around. That centreline is the answer, so
// these tests can ask the two questions a fixture cannot: how much of the track
// did the planner find, and did it stay on it.
//
// A fixture is a shape somebody thought of. These are whatever the generator
// happened to produce, which is the point.

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "mirena_planning/known/track_template.hpp"
#include "mirena_planning/unknown/unknown_planner.hpp"

using namespace mirena::planning;  // NOLINT(build/namespaces)

namespace
{

constexpr double kTrackWidth = 3.0;

std::vector<std::string> trackNames()
{
  std::vector<std::string> names;
  for (int i = 1; i <= 10; ++i) {names.push_back("track" + std::to_string(i));}
  return names;
}

TrackTemplate loadTrack(const std::string & name)
{
  return loadTrackTemplate(std::string(MIRENA_PLANNING_TRACK_DIR) + "/" + name + ".json");
}

/// The map SLAM would hand the planner: surveyed cones moved by landmark error,
/// with some of them never seen at all. The big cones of the start line are
/// metres from the car on the grid and are not dropped.
std::vector<Observation> perceived(
  const TrackTemplate & track, unsigned seed, double noise, double dropout)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> jitter(0.0, noise);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  std::vector<Observation> out;
  for (const Observation & cone : track.cones) {
    if (cone.size != ConeSize::Big && uniform(rng) < dropout) {continue;}
    out.push_back({{cone.p.x + jitter(rng), cone.p.y + jitter(rng)}, cone.size});
  }
  return out;
}

/// Distance from a point to the centreline itself, not to the nearest of its
/// samples -- the stored path is sampled every few metres, so sample distance
/// alone scores a perfect trace at half a spacing.
double offCentreline(Vec2 p, const std::vector<Vec2> & path)
{
  double worst = 1e18;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Vec2 a = path[i], b = path[(i + 1) % path.size()];
    const Vec2 ab = b - a;
    const double len2 = squaredNorm(ab);
    const double u = len2 > 1e-12 ?
      std::max(0.0, std::min(1.0, dot(p - a, ab) / len2)) : 0.0;
    worst = std::min(worst, distance(p, a + ab * u));
  }
  return worst;
}

struct Result
{
  double covered{0.0};    ///< percent of the centreline's length the trace spans
  double worst_dev{0.0};
  bool closed{false};
};

/// Coverage measured along the *true* centreline, so a trace cannot earn it by
/// being long: doubling back covers no new ground.
Result evaluate(const TrackTemplate & track, const std::vector<Observation> & cones)
{
  UnknownPlanner planner;
  std::optional<Plan> plan;
  // Repeatedly, so the closure hysteresis has the cycles it needs to settle.
  for (int i = 0; i < 8; ++i) {plan = planner.plan(cones, nullptr);}
  if (!plan || plan->gates.size() < 3) {return {};}

  std::vector<double> arc(track.path.size(), 0.0);
  for (std::size_t i = 1; i < track.path.size(); ++i) {
    arc[i] = arc[i - 1] + distance(track.path[i], track.path[i - 1]);
  }
  const double total = arc.back() + distance(track.path.front(), track.path.back());

  Result result;
  result.closed = plan->closed;
  double lowest = 1e18, highest = -1e18;
  for (const Gate & gate : plan->gates) {
    result.worst_dev = std::max(result.worst_dev, offCentreline(gate.p, track.path));
    std::size_t nearest = 0;
    double best = 1e18;
    for (std::size_t i = 0; i < track.path.size(); ++i) {
      const double d = distance(gate.p, track.path[i]);
      if (d < best) {best = d; nearest = i;}
    }
    lowest = std::min(lowest, arc[nearest]);
    highest = std::max(highest, arc[nearest]);
  }
  result.covered = 100.0 * (highest - lowest) / total;
  return result;
}

}  // namespace

/// With every cone mapped, the trace should be the centreline and nothing else.
TEST(RealTracks, TracesTheWholeLapOnAPerfectMap)
{
  for (const std::string & name : trackNames()) {
    const TrackTemplate track = loadTrack(name);
    const Result r = evaluate(track, track.cones);

    EXPECT_GT(r.covered, 95.0) << name << " traced only " << r.covered << "% of the lap";
    // Not "inside the cones" -- on the line. Half a track width would pass with
    // the centreline scraping a boundary, which is not what a perfect map should
    // produce and would hide the tracer stepping onto the boundary edges.
    EXPECT_LT(r.worst_dev, 0.5) << name << " strayed " << r.worst_dev << " m off the centreline";
    EXPECT_TRUE(r.closed) << name << " is a closed circuit and was not reported as one";
  }
}

/// The same tracks with landmark error on every cone and a share of them never
/// mapped at all.
///
/// This asks one thing only: that the centreline stays between the cones. It
/// deliberately does not ask how much of the lap was traced, because the map it
/// builds is not one the planner ever sees -- cones are deleted uniformly across
/// the whole circuit at once, where perception loses them at range and finds
/// them again as the car approaches. On this frozen map the trace stops at the
/// first hole and covers little of the lap; on the moving one it does not,
/// because it re-plans several times a second against a map that is densest
/// exactly where the car is about to drive.
///
/// So coverage here would measure the fixture, not the planner. Staying on the
/// track is the property that has to hold either way.
TEST(RealTracks, KeepsTheCentrelineInsideTheConesOnADegradedMap)
{
  for (const std::string & name : trackNames()) {
    const TrackTemplate track = loadTrack(name);
    for (unsigned seed = 0; seed < 5; ++seed) {
      const Result r = evaluate(track, perceived(track, seed, 0.10, 0.06));
      EXPECT_LE(r.worst_dev, kTrackWidth / 2.0)
        << name << " seed " << seed << " put the centreline "
        << r.worst_dev - kTrackWidth / 2.0 << " m outside the cones";
    }
  }
}
