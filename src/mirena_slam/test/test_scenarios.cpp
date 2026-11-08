// Whole runs over real track layouts.
//
// The layouts come out of the sim's Export Track button and are the same ones
// mirena_planning tests against. A fixture is a shape somebody thought of; these
// are whatever the generator produced, which is the point.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "track_sim.hpp"

using namespace mirena;       // NOLINT(build/namespaces)
using namespace mirena::test; // NOLINT(build/namespaces)

namespace
{

    std::vector<std::string> track_names()
    {
        std::vector<std::string> names;
        for (int i = 1; i <= 10; ++i)
            names.push_back("track" + std::to_string(i));
        return names;
    }

    /// How many of these points have another point closer than `limit`.
    int neighbours_within(const std::vector<gtsam::Point2> &points, double limit)
    {
        int count = 0;
        for (size_t i = 0; i < points.size(); ++i)
        {
            for (size_t j = 0; j < points.size(); ++j)
            {
                if (i == j)
                    continue;
                if ((points[j] - points[i]).norm() < limit)
                {
                    ++count;
                    break;
                }
            }
        }
        return count;
    }

    std::vector<gtsam::Point2> positions(const std::vector<Landmark> &map)
    {
        std::vector<gtsam::Point2> out;
        out.reserve(map.size());
        for (const Landmark &l : map)
            out.emplace_back(l.x, l.y);
        return out;
    }

    std::vector<gtsam::Point2> positions(const std::vector<Cone> &cones)
    {
        std::vector<gtsam::Point2> out;
        out.reserve(cones.size());
        for (const Cone &c : cones)
            out.push_back(c.position);
        return out;
    }

} // namespace

class TrackScenario : public ::testing::TestWithParam<std::string>
{
};

TEST_P(TrackScenario, MapsTheTrackWithoutDuplicating)
{
    const Track track = load_track(GetParam());
    SimParams params;
    params.seed = 42;
    Simulator sim(track, params);

    Slam slam(track.start);
    const RunResult result = drive(slam, sim, 5);

    ASSERT_TRUE(result.healthy) << GetParam() << ": solver gave up";
    ASSERT_FALSE(result.map.empty()) << GetParam() << ": mapped nothing at all";

    // The map is judged on its own shape, not on where it sits in the frame.
    //
    // These are two separate defects and they were previously measured as one.
    // A graph anchored only at the first pose fixes the map's gauge from the
    // odometry of the first lap, so on free-running odometry the finished map can
    // be a metrically perfect copy of the track standing a couple of metres and a
    // few degrees from where the track really is. Comparing to the truth frame
    // directly scores that identically to a map with two landmarks per cone --
    // and they are not the same thing: the first wants an absolute reference, the
    // second wants a fix in here. Registration gets its own assertion below.
    const Alignment fit = align_to(result.map, track.cones);

    // Every mapped cone should be near a real one. This is the assertion that
    // catches phantoms surviving into the graph.
    int spurious = 0;
    for (const gtsam::Point2 &p : fit.aligned)
    {
        double best = std::numeric_limits<double>::infinity();
        for (const Cone &cone : track.cones)
            best = std::min(best, (cone.position - p).norm());
        if (best > 1.0)
            ++spurious;
    }

    // And most real cones the car drove past should have been found -- each by a
    // landmark of its own, so a pair of duplicates cannot cover for a miss.
    const int found = distinct_cones_claimed(fit.aligned, track.cones, 1.0);

    const double coverage =
        static_cast<double>(found) / static_cast<double>(track.cones.size());
    std::printf("  %-9s cones %3zu  mapped %3zu  coverage %4.0f%%  spurious %3d  "
                "keyframes %3d  registration %.2f m %.1f deg\n",
                GetParam().c_str(), track.cones.size(), result.map.size(),
                100.0 * coverage, spurious, result.keyframes, fit.translation,
                fit.rotation * 180.0 / M_PI);

    EXPECT_GT(coverage, 0.80) << GetParam() << ": too much of the track is missing";
    EXPECT_LE(spurious, 3) << GetParam() << ": landmarks that match no real cone";
    // Landmarks standing on top of each other -- but counted against the track
    // rather than against zero. Every one of these layouts has a start gate whose
    // cones are 0.6 m apart, so a bare limit here is really a limit on how many
    // close cones the generator happened to lay, and a map that reproduces the
    // gate correctly spends the whole allowance on being right. What is being
    // asserted is the excess: one duplicated cone is tolerated, on the same
    // reasoning as `spurious` above, and two is not.
    const int layout_close = neighbours_within(positions(track.cones), 0.9);
    EXPECT_LE(neighbours_within(positions(result.map), 0.9), layout_close + 2)
        << GetParam() << ": duplicates sitting on top of each other, beyond the "
        << layout_close << " pairs the track itself has";

    // Registration, measured separately and loosely. What consumes this is the
    // known-track planner, which registers the map against a nominal transform
    // from the car's start pose and rejects a fit more than five metres out; this
    // is the budget it leaves, not an accuracy target.
    EXPECT_LT(fit.translation, 5.0)
        << GetParam() << ": the map is correct but has slid out of the track frame";
}

INSTANTIATE_TEST_SUITE_P(RealTracks, TrackScenario,
                         ::testing::ValuesIn(track_names()),
                         [](const ::testing::TestParamInfo<std::string> &info) {
                             return info.param;
                         });

TEST(Scenario, ClosePairIsNotCollapsed)
{
    // The negative obligation. Getting the start-line cones detected and in range
    // is perception's job; not destroying a pair it did resolve is this class's.
    //
    // Parameterised over approach angle because head-on is the hard one: it puts
    // the separation across the beam, where the sensor is at its worst, and it is
    // also the nominal approach.
    for (const double degrees : {0.0, 15.0, 30.0, 45.0})
    {
        const double angle = degrees * M_PI / 180.0;
        Track track = load_track("track1");

        // The track's own cones, kept before the pair is added. The count below
        // has to be able to tell the injected pair from the layout it was dropped
        // into: track1 has a real cone 1.55 m from this centre, so a plain radius
        // counts three and reads as a duplicate that is not there.
        const std::vector<Cone> layout = track.cones;

        // A pair 0.9 m apart, straddling the path a little way ahead.
        const gtsam::Point2 centre(14.0, 0.0);
        const gtsam::Point2 offset(0.45 * std::sin(angle), 0.45 * std::cos(angle));
        const gtsam::Point2 left = centre + offset;
        const gtsam::Point2 right = centre - offset;
        track.cones.push_back({left, kBig});
        track.cones.push_back({right, kBig});

        SimParams params;
        params.seed = static_cast<unsigned>(101 + degrees);
        Simulator sim(track, params);

        Slam slam(track.start);
        const RunResult result = drive(slam, sim, 3);
        ASSERT_TRUE(result.healthy);

        // Registered onto the track's own cones first, so that a map which is
        // correct but sitting a couple of metres out of frame is not read as a
        // duplicate; then a landmark counts towards the pair only if the pair
        // explains it better than any cone the track already had. The question is
        // "what did these two cones become", not "what is near here".
        const Alignment fit = align_to(result.map, layout);

        int near_pair = 0;
        for (const gtsam::Point2 &p : fit.aligned)
        {
            if ((p - centre).norm() >= 2.0)
                continue;
            const double to_pair = std::min((p - left).norm(), (p - right).norm());
            double to_layout = std::numeric_limits<double>::infinity();
            for (const Cone &c : layout)
                to_layout = std::min(to_layout, (p - c.position).norm());
            if (to_pair <= to_layout)
                ++near_pair;
        }

        // The two directions are not equally bad, and the assertions say so for
        // the same reason max_candidate_age does: a collapsed pair is a gate the
        // planner cannot see through, an extra landmark beside one is clutter.
        // Asserting equality here would hold this class to a promise the design
        // deliberately does not make.
        EXPECT_GE(near_pair, 2)
            << "approach " << degrees << " deg: the pair was collapsed into "
            << near_pair << " landmark(s)";
        EXPECT_LE(near_pair, 3)
            << "approach " << degrees << " deg: the pair became " << near_pair
            << " landmarks, which is more than one spare";
    }
}

TEST(Scenario, BigConesKeepTheirType)
{
    // Types are votes and nothing branches on them, but they are published and
    // the planner roots the track on them, so a big cone must not come out small.
    Track track = load_track("track1");
    SimParams params;
    params.seed = 77;
    Simulator sim(track, params);

    Slam slam(track.start);
    const RunResult result = drive(slam, sim, 3);
    ASSERT_TRUE(result.healthy);

    const std::vector<gtsam::Point2> big_truth = [&] {
        std::vector<gtsam::Point2> out;
        for (const Cone &c : track.cones)
            if (c.type == kBig)
                out.push_back(c.position);
        return out;
    }();
    ASSERT_FALSE(big_truth.empty()) << "this track has no big cones to check";

    // Aligned first, for the reason given in MapsTheTrackWithoutDuplicating: this
    // asks whether the type followed the cone, and it must not fail merely because
    // the whole map sits a couple of metres from where the track really is.
    const Alignment fit = align_to(result.map, track.cones);
    std::vector<gtsam::Point2> big_mapped;
    for (size_t i = 0; i < result.map.size(); ++i)
        if (result.map[i].type == kBig)
            big_mapped.push_back(fit.aligned[i]);

    std::printf("  big cones: %zu on the track, %zu in the map (registration "
                "%.2f m %.1f deg)\n",
                big_truth.size(), big_mapped.size(), fit.translation,
                fit.rotation * 180.0 / M_PI);
    ASSERT_FALSE(big_mapped.empty()) << "no landmark is typed big at all";

    for (const gtsam::Point2 &truth : big_truth)
    {
        double best = std::numeric_limits<double>::infinity();
        for (const gtsam::Point2 &m : big_mapped)
            best = std::min(best, (m - truth).norm());
        EXPECT_LT(best, 1.5) << "a big cone was not mapped as big";
    }
}

TEST(Scenario, NoDuplicatesAfterTheFirstLap)
{
    // Once the track has been driven, association should recognise what it
    // already has. A steady drip of promotions after that is the symptom the
    // whole gate exists to prevent.
    Track track = load_track("track2");
    SimParams params;
    params.seed = 13;
    Simulator sim(track, params);

    Slam slam(track.start);
    gtsam::Pose2 last_keyframe_raw = track.start;
    bool first = true;
    int promoted_first_lap = 0, promoted_later = 0;

    Frame frame;
    while (sim.step(frame, 5))
    {
        const gtsam::Vector3 sigmas = sim.odom_sigmas(last_keyframe_raw);
        slam.observe(frame.raw_odom, frame.observations, sigmas);
        if (first || slam.wants_keyframe())
        {
            const int before = slam.get_landmark_count();
            slam.add_keyframe(sigmas);
            last_keyframe_raw = frame.raw_odom;
            first = false;
            const int gained = slam.get_landmark_count() - before;
            (sim.laps_done() < 1 ? promoted_first_lap : promoted_later) += gained;
        }
    }

    std::printf("  promoted: %d on the first lap, %d over the next four\n",
                promoted_first_lap, promoted_later);

    ASSERT_GT(promoted_first_lap, 10) << "did not map the track on the first lap";
    EXPECT_LT(promoted_later, promoted_first_lap / 2)
        << "cones keep being discovered on ground already driven";
}

TEST(Scenario, CorrectionIsContinuous)
{
    // map->odom is what the MPC projects the car onto, and its spline search is a
    // local one. A large step at loop closure mis-projects the arc length and
    // breaks lap counting. The node filters this with a half-second time
    // constant; this measures what the filter is being asked to absorb.
    Track track = load_track("track1");
    SimParams params;
    params.seed = 31;
    Simulator sim(track, params);

    Slam slam(track.start);
    const RunResult result = drive(slam, sim, 5);
    ASSERT_TRUE(result.healthy);
    ASSERT_FALSE(result.correction_jump_xy.empty());

    std::vector<double> jumps = result.correction_jump_xy;
    std::sort(jumps.begin(), jumps.end());
    const double p50 = jumps[jumps.size() / 2];
    const double worst = jumps.back();
    const double worst_theta = *std::max_element(result.correction_jump_theta.begin(),
                                                 result.correction_jump_theta.end());

    std::printf("  correction step: p50 %.3f m   max %.3f m   max heading %.3f rad\n",
                p50, worst, worst_theta);

    EXPECT_LT(p50, 0.05) << "the correction is noisy every keyframe, not just at closure";
    EXPECT_LT(worst, 1.5) << "a correction step this large will mis-project the MPC";
}
