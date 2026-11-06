// The candidate tier: what gets into the graph, what never does, and what stops
// being published once it stops being there.
//
// Driven directly rather than through the track simulator, because these are
// questions about a handful of cones at a known range and the answer should not
// depend on which corner of which track they happened to sit in.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "track_sim.hpp"

using namespace mirena;       // NOLINT(build/namespaces)
using namespace mirena::test; // NOLINT(build/namespaces)

namespace
{

    /// A car creeping forward down +x, told the truth by its odometry, looking at
    /// whatever cones are put in front of it. Everything here is about the
    /// estimator's decisions, so the odometry is made perfect on purpose: what is
    /// left is the promotion logic and nothing else.
    class Bench
    {
    public:
        /// `sensor` defaults to a detector that stops at the same twenty metres
        /// the estimator works in, which is what every test here wants. It is a
        /// parameter because the real one does not: ConeDetectorParams::max_range
        /// is 30 m against a local_map_radius of 20, and the band between the two
        /// is a case no fixture could produce while the two numbers were welded
        /// together.
        explicit Bench(SlamParams params = {}, unsigned seed = 7, SensorModel sensor = {})
            : slam_(gtsam::Pose2(0, 0, 0), params), sensor_(sensor), rng_(seed)
        {
        }

        struct World
        {
            std::vector<Cone> cones;
            /// Cones the detector invents this frame, in the car frame.
            std::vector<gtsam::Point2> phantoms;
        };

        /// One frame: move `step` metres forward, then report what is visible.
        void tick(const World &world, double step = 0.05, double p_detect = 1.0)
        {
            pose_ = pose_.compose(gtsam::Pose2(step, 0.0, 0.0));

            std::vector<Observation> obs;
            std::normal_distribution<double> unit(0.0, 1.0);
            std::uniform_real_distribution<double> uniform(0.0, 1.0);

            for (const Cone &cone : world.cones)
            {
                const gtsam::Point2 local = pose_.transformTo(cone.position);
                if (!visible(local) || uniform(rng_) > p_detect)
                    continue;
                const gtsam::Matrix2 R = sensor_.covariance(local);
                const Eigen::LLT<gtsam::Matrix2> llt(R);
                const gtsam::Point2 z =
                    local + llt.matrixL() * gtsam::Vector2(unit(rng_), unit(rng_));
                obs.push_back({z.x(), z.y(), cone.type,
                               {R(0, 0), R(0, 1), R(1, 0), R(1, 1)}});
            }
            for (const gtsam::Point2 &p : world.phantoms)
            {
                const gtsam::Matrix2 R = sensor_.covariance(p);
                obs.push_back({p.x(), p.y(), std::string(kSmall),
                               {R(0, 0), R(0, 1), R(1, 0), R(1, 1)}});
            }

            const gtsam::Vector3 sigmas(0.02, 0.02, 0.005);
            slam_.observe(pose_, obs, sigmas);
            if (first_ || slam_.wants_keyframe())
            {
                slam_.add_keyframe(sigmas);
                first_ = false;
            }
        }

        void run(const World &world, int frames, double step = 0.05, double p_detect = 1.0)
        {
            for (int i = 0; i < frames; ++i)
                tick(world, step, p_detect);
        }

        Slam &slam() { return slam_; }
        std::vector<Landmark> map() { return slam_.map_snapshot(); }
        const gtsam::Pose2 &pose() const { return pose_; }

    private:
        bool visible(const gtsam::Point2 &local) const
        {
            return local.norm() < sensor_.max_range && local.x() > 0.0 &&
                   std::abs(std::atan2(local.y(), local.x())) < sensor_.half_angle;
        }

        Slam slam_;
        SensorModel sensor_;
        std::mt19937 rng_;
        gtsam::Pose2 pose_;
        bool first_ = true;
    };

    Bench::World pair_at(double range, double separation)
    {
        Bench::World w;
        w.cones.push_back({gtsam::Point2(range, +0.5 * separation), kBig});
        w.cones.push_back({gtsam::Point2(range, -0.5 * separation), kBig});
        return w;
    }

    /// Four gates of a straight track, starting well beyond local_map_radius and
    /// running out past it. Shared by the two tests below, which drive different
    /// distances down it and want opposite things.
    Bench::World distant_corridor()
    {
        Bench::World w;
        for (int i = 0; i < 4; ++i)
        {
            const double x = 26.0 + 4.0 * static_cast<double>(i);
            w.cones.push_back({gtsam::Point2(x, +1.5), kSmall});
            w.cones.push_back({gtsam::Point2(x, -1.5), kSmall});
        }
        return w;
    }

    SensorModel detector_reach()
    {
        SensorModel s;
        s.max_range = 30.0; // ConeDetectorParams::max_range
        return s;
    }

} // namespace

TEST(Candidates, ConesBeyondTheLocalMapRadiusDoNotAccumulate)
{
    // A detection out past local_map_radius can be matched against nothing: both
    // target lists the associator is given are filtered by that radius, so
    // neither a landmark nor a candidate out there is ever a candidate to match.
    //
    // Left to fall through to candidate creation, that made the band a ratchet.
    // Every frame, every cone in it spawned a *fresh* candidate -- the one from
    // last frame being still out of range and so still not a target. Nor did they
    // decay, because in_view() is bounded by the same radius, so they took no
    // missed-evidence penalty and simply sat at one sighting until
    // max_candidate_age expired them some seconds later.
    Bench bench({}, 7, detector_reach());

    // Five metres of travel, so the nearest gate is twenty-one metres away at the
    // end and no cone has been associable at any point in the run.
    bench.run(distant_corridor(), 100);

    EXPECT_GT(bench.slam().get_last_out_of_range_count(), 0)
        << "the fixture produced no out-of-range detection, so this proves nothing";
    EXPECT_EQ(bench.slam().get_candidate_count(), 0)
        << "out-of-range detections are still accumulating candidates";
    EXPECT_EQ(bench.slam().get_landmark_count(), 0)
        << "a cone never once inside the local map radius reached the graph";
}

TEST(Candidates, ConesEnteringTheLocalMapRadiusAreStillMapped)
{
    // The complement, and the one that says the clause above is a filter rather
    // than a hole: the same cones, driven up to until they come inside the
    // radius, must map exactly once each.
    Bench bench({}, 7, detector_reach());

    bench.run(distant_corridor(), 400); // twenty metres: every gate ends up close

    EXPECT_EQ(bench.map().size(), 8u)
        << "cones first seen beyond the radius are lost once they come inside it";
}

TEST(Candidates, RealConeIsPromoted)
{
    Bench bench;
    Bench::World world;
    world.cones.push_back({gtsam::Point2(8.0, 0.0), kSmall});

    bench.run(world, 30);
    EXPECT_EQ(bench.map().size(), 1u);
}

TEST(Candidates, PromotionIsNotInstant)
{
    // One sighting must not be enough. If it were, the candidate tier would be a
    // rename of the thing it replaced.
    Bench bench;
    Bench::World world;
    world.cones.push_back({gtsam::Point2(8.0, 0.0), kSmall});

    bench.run(world, 1);
    EXPECT_EQ(bench.slam().get_landmark_count(), 0);
    EXPECT_GT(bench.slam().get_candidate_count(), 0);
}

TEST(Candidates, PhantomNeverPromotes)
{
    // A detector reporting something different in a different place every frame.
    // Nothing here is ever seen twice, so nothing should ever earn a place in the
    // graph -- and because it never got in, nothing has to be taken back out.
    Bench bench;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    for (int i = 0; i < 200; ++i)
    {
        Bench::World world;
        if (uniform(rng) < 0.3)
        {
            const double range = 4.0 + uniform(rng) * 12.0;
            const double bearing = (uniform(rng) * 2.0 - 1.0) * 0.9;
            world.phantoms.emplace_back(range * std::cos(bearing), range * std::sin(bearing));
        }
        bench.tick(world);
    }

    EXPECT_EQ(bench.slam().get_landmark_count(), 0)
        << "a phantom reached the graph, where it can never be removed";
}

TEST(Candidates, EvidenceAgainstOutweighsAnOldSighting)
{
    // A cone seen a few times and then never again, while the car looks straight
    // at where it was. It should stop being published. The graph keeps it -- its
    // measurements were real observations of something and remain good
    // constraints -- but the map is a filtered view.
    SlamParams params;
    Bench bench(params);

    Bench::World world;
    world.cones.push_back({gtsam::Point2(12.0, 0.0), kSmall});
    bench.run(world, 20);
    ASSERT_EQ(bench.map().size(), 1u) << "did not promote in the first place";
    const int in_graph = bench.slam().get_landmark_count();

    // Now it is gone, but the car keeps looking at where it stood.
    Bench::World empty;
    bench.run(empty, 60);

    EXPECT_TRUE(bench.map().empty()) << "a cone that stopped being there is still published";
    EXPECT_EQ(bench.slam().get_landmark_count(), in_graph)
        << "the landmark was removed from the graph, which this design never does";
}

TEST(Candidates, DisplacedConeGhostLeavesTheMap)
{
    // The failure mode a never-delete graph has to answer for: the car clips a
    // cone and it moves half a metre. The old estimate becomes a ghost.
    Bench bench;
    Bench::World before;
    before.cones.push_back({gtsam::Point2(14.0, 0.0), kSmall});
    bench.run(before, 25);
    ASSERT_EQ(bench.map().size(), 1u);

    Bench::World after;
    after.cones.push_back({gtsam::Point2(14.0, 0.9), kSmall});
    bench.run(after, 60);

    const std::vector<Landmark> map = bench.map();
    ASSERT_FALSE(map.empty());
    // Whatever is published should be where the cone actually is now.
    for (const Landmark &l : map)
        EXPECT_NEAR(l.y, 0.9, 0.35) << "the ghost is still being published at y=0";
    EXPECT_LE(map.size(), 1u) << "both the ghost and the new cone are in the map";
}

TEST(Candidates, TwoConesStayTwo)
{
    // The table. A fixed separation rule has to choose one row of this and be
    // wrong about the others; the separability clause is supposed to get them
    // all, because it scales with what the sensor can actually resolve.
    const std::vector<double> separations = {0.4, 0.7, 1.0, 1.5};
    const std::vector<double> ranges = {5.0, 12.0, 18.0};

    std::printf("\n  separation |");
    for (const double r : ranges)
        std::printf(" %5.0f m", r);
    std::printf("\n  -----------+");
    for (size_t i = 0; i < ranges.size(); ++i)
        std::printf("--------");
    std::printf("\n");

    // The two directions are not equally bad and the assertions say so. A pair
    // collapsed into one landmark is a hole in the track the planner has to route
    // around; a spare landmark beside a real one is clutter. So: never fewer than
    // two, anywhere, and few enough extras that the map stays legible.
    int collapsed = 0, extra = 0;
    for (const double s : separations)
    {
        std::printf("  %6.1f m   |", s);
        for (const double r : ranges)
        {
            Bench bench({}, static_cast<unsigned>(s * 100 + r));
            // Approach it: the pair is first seen at range and closes to about
            // half of it, which is how the car actually meets a start gate.
            bench.run(pair_at(r, s), 60, 0.05 * (r / 5.0));
            const size_t n = bench.map().size();
            std::printf("  %5zu ", n);
            if (n < 2)
                ++collapsed;
            else
                extra += static_cast<int>(n) - 2;
        }
        std::printf("\n");
    }
    std::printf("\n");

    EXPECT_EQ(collapsed, 0) << "a pair was collapsed into one cone";
    EXPECT_LE(extra, 1) << "duplicates are getting past the promotion test";
}

TEST(Candidates, OneConeStaysOne)
{
    // The other half of the table, and the one a loose rule fails: a single cone
    // observed repeatedly must not breed a second landmark beside itself.
    for (const double range : {5.0, 12.0, 18.0})
    {
        Bench bench({}, static_cast<unsigned>(range));
        Bench::World world;
        world.cones.push_back({gtsam::Point2(range, 0.0), kSmall});
        bench.run(world, 60, 0.05 * (range / 5.0));
        EXPECT_EQ(bench.map().size(), 1u) << "at range " << range;
    }
}

TEST(Candidates, MissesDoNotKillAConeThatIsStillThere)
{
    // Perception misses cones constantly -- at range, behind other cones, on a
    // bad sweep. A cone reported on only a fraction of the frames it is visible
    // in is still a cone, and p_detect is where that is said.
    Bench bench;
    Bench::World world;
    world.cones.push_back({gtsam::Point2(10.0, 0.0), kSmall});

    bench.run(world, 120, 0.02, /*p_detect=*/0.6);
    EXPECT_EQ(bench.map().size(), 1u)
        << "a real cone seen on 60% of frames was dropped";
}
