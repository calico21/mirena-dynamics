// The solver, and the things about it that are easy to get backwards.
//
// Two of these are characterisation tests: they assert what GTSAM does rather
// than what this package does, because the design rests on those facts and a
// silent change to either would present as "SLAM got slow" or "SLAM got worse"
// rather than as a failure.

#include <gtest/gtest.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>

#include "track_sim.hpp"

using namespace mirena;       // NOLINT(build/namespaces)
using namespace mirena::test; // NOLINT(build/namespaces)

TEST(Isam, PoseTangentOrdering)
{
    // Three lines, and they pin the meaning of every number in the per-symbol
    // relinearisation threshold. If the tangent were (theta, x, y) then the
    // thresholds would be a 0.005 m translation and a 0.05 rad heading -- the
    // exact inversion of what is intended -- and it would show up as the solver
    // being mysteriously slow rather than as a bug.
    const gtsam::Vector3 xi = gtsam::Pose2::Logmap(gtsam::Pose2(1.0, 2.0, 0.0));
    EXPECT_NEAR(xi(0), 1.0, 1e-9);
    EXPECT_NEAR(xi(1), 2.0, 1e-9);
    EXPECT_NEAR(xi(2), 0.0, 1e-9);

    const gtsam::Vector3 turn = gtsam::Pose2::Logmap(gtsam::Pose2(0.0, 0.0, 0.3));
    EXPECT_NEAR(turn(2), 0.3, 1e-9);
}

TEST(Isam, RobustWeightIsFrozenByLinearization)
{
    // Why the measurement factors are plain Gaussian.
    //
    // A robust noise model applies its weight inside WhitenSystem, which runs at
    // linearisation time and writes the result into the cached linear factor. So
    // a weight that collapsed on one bad linearisation point stays collapsed
    // until something forces a relinearisation -- and if the pose later corrects,
    // the residual is small and nothing ever will. The measurement is discarded
    // permanently and silently.
    //
    // This test does not check our code. It documents the hazard, so that nobody
    // reintroduces a robust kernel on measurements without meeting it.
    const gtsam::Symbol x('x', 0), l('l', 0);
    const gtsam::Expression<gtsam::Pose2> x_(x);
    const gtsam::Expression<gtsam::Point2> l_(l);

    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::DCS::Create(3.0),
        gtsam::noiseModel::Isotropic::Sigma(2, 0.05));
    const gtsam::ExpressionFactor<gtsam::Point2> factor(
        robust, gtsam::Point2(5.0, 0.0), gtsam::transformTo(x_, l_));

    // Linearise where the measurement is a gross outlier: the weight collapses.
    gtsam::Values bad;
    bad.insert(x, gtsam::Pose2(0, 0, 0));
    bad.insert(l, gtsam::Point2(5.0, 3.0));
    const auto stale = factor.linearize(bad);

    // The same factor, linearised where it fits perfectly.
    gtsam::Values good;
    good.insert(x, gtsam::Pose2(0, 0, 0));
    good.insert(l, gtsam::Point2(5.0, 0.0));
    const auto fresh = factor.linearize(good);

    // Evaluate both linear systems at the same, good, delta. If the weight were
    // recomputed rather than baked in, these would agree.
    gtsam::VectorValues zero;
    zero.insert(x, gtsam::Vector3::Zero());
    zero.insert(l, gtsam::Vector2::Zero());

    const double stale_error = stale->error(zero);
    const double fresh_error = fresh->error(zero);
    EXPECT_GT(std::abs(stale_error - fresh_error), 1e-6)
        << "the robust weight tracked the values rather than the linearisation "
           "point; if GTSAM changed this, plain Gaussian measurements may no "
           "longer be necessary";
}

TEST(Isam, MapFrameStaysAtTheCarStartPose)
{
    // Downstream depends on it: the known-track planner registers against a
    // nominal transform derived from the car's start pose and rejects a fit more
    // than five metres away, and the unknown planner looks for its start gate
    // within ten metres of the origin.
    Track track = load_track("track1");
    SimParams sim_params;
    sim_params.seed = 11;
    Simulator sim(track, sim_params);

    Slam slam(track.start);
    const RunResult result = drive(slam, sim, 3);
    ASSERT_TRUE(result.healthy);
    ASSERT_FALSE(result.map.empty());

    // The map is anchored, so cones should sit near where they actually are in
    // the track's own coordinates rather than shifted bodily.
    double worst = 0.0;
    for (const Cone &cone : track.cones)
        worst = std::max(worst, std::min(6.0, nearest(result.map, cone.position)));

    // Not an accuracy assertion -- a "did the whole map slide off" assertion.
    EXPECT_LT(worst, 3.0) << "the map has drifted bodily away from the track frame";
}

TEST(Isam, SolverStaysHealthyOverManyLaps)
{
    // The anchor prior is the conditioning risk: information of 1e12 against a
    // system of order 1e3 is what makes Cholesky give up, and unlike the sliding
    // window this pose lives in the tree for the whole run. Ten laps is where
    // that would show.
    Track track = load_track("track1");
    SimParams sim_params;
    sim_params.seed = 5;
    Simulator sim(track, sim_params);

    Slam slam(track.start);
    const RunResult result = drive(slam, sim, 10);

    EXPECT_TRUE(result.healthy) << "the solver threw; see the anchor prior";
    EXPECT_GT(result.keyframes, 50);
}

TEST(Isam, CostStaysWithinTheKeyframeBudget)
{
    // Perception runs at about 8 Hz, so a keyframe has roughly 125 ms before it
    // starts costing frames. The first time the track closes on itself a large
    // part of the tree is re-eliminated and that is expected; what is not
    // acceptable is the median creeping up lap after lap.
    Track track = load_track("track1");
    SimParams sim_params;
    sim_params.seed = 17;
    Simulator sim(track, sim_params);

    Slam slam(track.start);
    const RunResult result = drive(slam, sim, 8);
    ASSERT_TRUE(result.healthy);
    ASSERT_GT(result.update_ms.size(), 50u);

    std::vector<double> sorted = result.update_ms;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted[sorted.size() / 2];
    const double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];

    // The first half of the run against the second: if the graph were growing
    // without bound the later keyframes would be visibly dearer.
    const size_t half = result.update_ms.size() / 2;
    double early = 0.0, late = 0.0;
    for (size_t i = 0; i < half; ++i)
        early += result.update_ms[i];
    for (size_t i = half; i < result.update_ms.size(); ++i)
        late += result.update_ms[i];
    early /= static_cast<double>(half);
    late /= static_cast<double>(result.update_ms.size() - half);

    std::printf("\n  keyframes %d   median %.1f ms   p99 %.1f ms   max %.1f ms\n",
                result.keyframes, median, p99, sorted.back());
    std::printf("  mean first half %.1f ms   second half %.1f ms\n\n", early, late);

    EXPECT_LT(median, 40.0);
    EXPECT_LT(late, 4.0 * early + 5.0)
        << "cost per keyframe is climbing with the size of the graph";
}

TEST(Isam, PoseCountFlattensOnceTheTrackIsKnown)
{
    // The point of keyframing on information rather than distance. The first laps
    // are exploration and deserve a dense pose chain; the tenth is driving past
    // cones the map already knows better than the sensor can measure them, and a
    // pose there buys nothing and costs a permanent variable.
    Track track = load_track("track1");
    SimParams sim_params;
    sim_params.seed = 23;
    Simulator sim(track, sim_params);

    Slam slam(track.start);

    std::vector<int> per_lap;
    int last_total = 0, last_lap = 0;
    gtsam::Pose2 last_keyframe_raw = track.start;
    bool first = true;
    Frame frame;
    while (sim.step(frame, 6))
    {
        const gtsam::Vector3 sigmas = sim.odom_sigmas(last_keyframe_raw);
        slam.observe(frame.raw_odom, frame.observations, sigmas);
        if (first || slam.wants_keyframe())
        {
            slam.add_keyframe(sigmas);
            last_keyframe_raw = frame.raw_odom;
            first = false;
        }
        if (sim.laps_done() > last_lap)
        {
            last_lap = sim.laps_done();
            per_lap.push_back(slam.get_pose_count() - last_total);
            last_total = slam.get_pose_count();
        }
    }

    ASSERT_GE(per_lap.size(), 4u);
    std::printf("\n  poses added per lap:");
    for (const int n : per_lap)
        std::printf(" %d", n);
    std::printf("\n\n");

    // Not a strict monotone claim -- lap length and speed are constant here, so
    // a distance-based rule would give a flat line too. What is asserted is that
    // the later laps are not *more* expensive than the first, which is what an
    // unbounded pose chain plus a growing map would produce.
    EXPECT_LE(per_lap.back(), per_lap.front())
        << "later laps are adding at least as many poses as the exploration lap";
}
