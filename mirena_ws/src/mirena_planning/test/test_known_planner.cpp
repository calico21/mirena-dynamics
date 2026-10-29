#include <gtest/gtest.h>

#include <random>
#include <string>
#include <vector>

#include "mirena_planning/known/known_planner.hpp"

using namespace mirena::planning;  // NOLINT(build/namespaces)

namespace
{

std::string resource(const std::string & mission)
{
  return std::string(MIRENA_PLANNING_RESOURCE_DIR) + "/" + mission + ".json";
}

/// The template as SLAM would map it: cones in the map frame, seen through the
/// true placement of the car, with a little landmark noise on top.
std::vector<Observation> observe(
  const TrackTemplate & track, Transform2 truth, double noise = 0.05, unsigned seed = 7u)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> jitter(0.0, noise);

  std::vector<Observation> observations;
  for (const Observation & cone : track.cones) {
    const Vec2 p = truth * cone.p;
    observations.push_back({{p.x + jitter(rng), p.y + jitter(rng)}, cone.size});
  }
  return observations;
}

}  // namespace

TEST(TrackTemplate, LoadsTheShippedMissions)
{
  for (const char * mission : {"skidpad", "acceleration"}) {
    const TrackTemplate track = loadTrackTemplate(resource(mission));
    EXPECT_FALSE(track.cones.empty()) << mission;
    EXPECT_GE(track.path.size(), 2u) << mission;
  }
}

TEST(TrackTemplate, PlacesTheNominalTransformAtTheCarStartPose)
{
  const TrackTemplate track = loadTrackTemplate(resource("acceleration"));
  // acceleration.json starts the car at (-53, 0, 0) and its path at (-50, 0),
  // so in the map frame the path must begin three metres straight ahead.
  const Vec2 first = track.map_from_template * track.path.front();
  EXPECT_NEAR(first.x, 3.0, 1e-6);
  EXPECT_NEAR(first.y, 0.0, 1e-6);
}

TEST(TrackTemplate, RejectsAMissingFile)
{
  EXPECT_THROW(loadTrackTemplate(resource("no_such_mission")), std::runtime_error);
}

TEST(KnownPlanner, RegistersFromTheNominalSeed)
{
  TrackTemplate track = loadTrackTemplate(resource("skidpad"));
  const Transform2 truth = track.map_from_template;
  const auto observations = observe(track, truth);
  const std::size_t path_points = track.path.size();

  KnownPlanner planner(std::move(track));
  const auto plan = planner.plan(observations, nullptr);
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->gates.size(), path_points);
}

TEST(KnownPlanner, AbsorbsAnImperfectlyPlacedCar)
{
  TrackTemplate track = loadTrackTemplate(resource("skidpad"));
  // The car was set down 30 cm off the line and a few degrees crooked.
  const Transform2 truth = Transform2{0.05, {0.3, -0.2}} * track.map_from_template;
  const auto observations = observe(track, truth);
  const std::vector<Vec2> path = track.path;

  KnownPlanner planner(std::move(track));
  PlanDebug debug;
  const auto plan = planner.plan(observations, &debug);
  ASSERT_TRUE(plan.has_value());
  EXPECT_FALSE(debug.from_ransac) << "the seeded fit should have been enough";
  EXPECT_GT(debug.score, 0.9);

  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_LT(distance(plan->gates[i].p, truth * path[i]), 0.15);
  }
}

TEST(KnownPlanner, RecoversByRansacWhenTheSeedIsTooFarOff)
{
  TrackTemplate track = loadTrackTemplate(resource("skidpad"));
  // Well beyond the correspondence radius, so ICP alone finds nothing to hold
  // onto and the two-point RANSAC has to generate the hypothesis.
  const Transform2 truth = Transform2{0.25, {2.5, 1.5}} * track.map_from_template;
  const auto observations = observe(track, truth);
  const std::vector<Vec2> path = track.path;

  KnownPlanner planner(std::move(track));
  PlanDebug debug;
  const auto plan = planner.plan(observations, &debug);
  ASSERT_TRUE(plan.has_value());
  EXPECT_TRUE(debug.from_ransac);
  EXPECT_GT(debug.score, 0.9);
  EXPECT_LT(distance(plan->gates.front().p, truth * path.front()), 0.2);
}

TEST(KnownPlanner, TracksIncrementallyOnceLockedOn)
{
  TrackTemplate track = loadTrackTemplate(resource("skidpad"));
  const Transform2 truth = Transform2{0.05, {0.3, -0.2}} * track.map_from_template;
  const auto observations = observe(track, truth);

  KnownPlanner planner(std::move(track));
  ASSERT_TRUE(planner.plan(observations, nullptr).has_value());

  PlanDebug debug;
  ASSERT_TRUE(planner.plan(observations, &debug).has_value());
  // Warm started from the previous solution, so it converges immediately.
  EXPECT_LE(debug.iterations, 3);
  EXPECT_FALSE(debug.from_ransac);
}

TEST(KnownPlanner, PublishesNothingForAnUnrelatedMap)
{
  TrackTemplate track = loadTrackTemplate(resource("skidpad"));
  KnownPlanner planner(std::move(track));

  std::vector<Observation> noise;
  std::mt19937 rng(3u);
  std::uniform_real_distribution<double> spread(-30.0, 30.0);
  for (int i = 0; i < 40; ++i) {
    noise.push_back(
      {{spread(rng), spread(rng)}, ConeSize::Small});
  }
  EXPECT_FALSE(planner.plan(noise, nullptr).has_value());
}

TEST(KnownPlanner, RejectsAFitThatSlidDownTheTrack)
{
  TrackTemplate track = loadTrackTemplate(resource("acceleration"));
  // A straight corridor of evenly spaced cones registers just as well shifted
  // along its own axis; the drift cap is what stops the planner from handing
  // the controller a track that starts 12 m behind the car.
  const Transform2 truth = Transform2{0.0, {12.0, 0.0}} * track.map_from_template;
  const auto observations = observe(track, truth);

  KnownPlanner planner(std::move(track));
  EXPECT_FALSE(planner.plan(observations, nullptr).has_value());
}

TEST(KnownPlanner, ReportsCorrespondencesForTheDebugTopic)
{
  TrackTemplate track = loadTrackTemplate(resource("skidpad"));
  const Transform2 truth = track.map_from_template;
  const auto observations = observe(track, truth);
  const std::size_t cone_count = track.cones.size();

  KnownPlanner planner(std::move(track));
  PlanDebug debug;
  debug.verbose = true;
  ASSERT_TRUE(planner.plan(observations, &debug).has_value());

  EXPECT_EQ(debug.fitted_template.size(), cone_count);
  EXPECT_FALSE(debug.fitted_path.empty());
  EXPECT_GT(debug.correspondences.size(), cone_count / 2);
  EXPECT_TRUE(debug.registered);
}
