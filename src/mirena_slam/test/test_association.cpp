// The assignment and the gate, on their own.
//
// hungarian() is seventy lines of index-heavy code with a sentinel cost and
// one-based potentials, and until now it had no tests at all.

#include <gtest/gtest.h>

#include "association.hpp"

using namespace mirena; // NOLINT(build/namespaces)

namespace
{
    gtsam::Matrix2 iso(double sigma)
    {
        return gtsam::Matrix2::Identity() * (sigma * sigma);
    }
} // namespace

TEST(Hungarian, Empty)
{
    EXPECT_TRUE(hungarian({}).empty());
}

TEST(Hungarian, NoColumns)
{
    const std::vector<int> a = hungarian({{}, {}});
    ASSERT_EQ(a.size(), 2u);
    EXPECT_EQ(a[0], -1);
    EXPECT_EQ(a[1], -1);
}

TEST(Hungarian, PicksTheCheapestOneToOne)
{
    // Row 0 prefers column 0; row 1 prefers column 0 more strongly. The optimal
    // assignment gives column 0 to row 1 and pays a little more on row 0 --
    // which is exactly what greedy-by-row would get wrong.
    const std::vector<std::vector<double>> cost = {
        {1.0, 2.0},
        {0.1, 5.0},
    };
    const std::vector<int> a = hungarian(cost);
    ASSERT_EQ(a.size(), 2u);
    EXPECT_EQ(a[0], 1);
    EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, MoreColumnsThanRows)
{
    const std::vector<std::vector<double>> cost = {
        {5.0, 1.0, 9.0},
    };
    const std::vector<int> a = hungarian(cost);
    ASSERT_EQ(a.size(), 1u);
    EXPECT_EQ(a[0], 1);
}

TEST(Hungarian, AllBlockedStillTerminates)
{
    const std::vector<std::vector<double>> cost = {
        {kBlocked, kBlocked},
        {kBlocked, kBlocked},
    };
    const std::vector<int> a = hungarian(cost);
    ASSERT_EQ(a.size(), 2u);
    // Some assignment is made; the caller is the one that treats a blocked cost
    // as no match. What matters is that it returns at all.
    EXPECT_NE(a[0], a[1]);
}

TEST(Associate, TwoObservationsCannotClaimOneLandmark)
{
    // The failure this exists to stop: two cones detected in one frame both
    // matching a single landmark, putting two contradictory measurements on one
    // point and dragging it to the midpoint of two real cones.
    const gtsam::Pose2 pose(0, 0, 0);
    const std::vector<Target> targets = {{gtsam::Point2(5.0, 0.0), iso(0.1)}};
    const std::vector<gtsam::Point2> local = {gtsam::Point2(5.0, 0.05),
                                              gtsam::Point2(5.0, -0.05)};
    const std::vector<gtsam::Matrix2> meas = {iso(0.1), iso(0.1)};

    const std::vector<int> a =
        associate(pose, gtsam::Matrix3::Zero(), targets, local, meas, 5.99, 2.5);

    ASSERT_EQ(a.size(), 2u);
    EXPECT_NE(a[0], a[1]) << "both observations claimed the same landmark";
    EXPECT_TRUE(a[0] == -1 || a[1] == -1);
}

TEST(Associate, GateRejectsWhatIsTooFarForItsUncertainty)
{
    const gtsam::Pose2 pose(0, 0, 0);
    const std::vector<Target> targets = {{gtsam::Point2(5.0, 0.0), iso(0.05)}};
    const std::vector<gtsam::Matrix2> meas = {iso(0.05)};

    // Two sigmas of combined noise is about 0.14 m; well inside.
    std::vector<gtsam::Point2> near = {gtsam::Point2(5.0, 0.08)};
    EXPECT_EQ(associate(pose, gtsam::Matrix3::Zero(), targets, near, meas, 5.99, 2.5)[0], 0);

    // A metre out is not the same cone.
    std::vector<gtsam::Point2> far = {gtsam::Point2(5.0, 1.0)};
    EXPECT_EQ(associate(pose, gtsam::Matrix3::Zero(), targets, far, meas, 5.99, 2.5)[0], -1);
}

TEST(Associate, EuclideanBackstopBindsBeforeTheChiSquareTest)
{
    // A wildly uncertain landmark would otherwise swallow anything: the backstop
    // is what stops a pose that has gone wrong reaching across the track.
    const gtsam::Pose2 pose(0, 0, 0);
    const std::vector<Target> targets = {{gtsam::Point2(5.0, 0.0), iso(50.0)}};
    const std::vector<gtsam::Point2> local = {gtsam::Point2(5.0, 4.0)};
    const std::vector<gtsam::Matrix2> meas = {iso(0.1)};

    EXPECT_EQ(associate(pose, gtsam::Matrix3::Zero(), targets, local, meas, 5.99, 2.5)[0], -1);
}

TEST(InnovationCovariance, HeadingUncertaintyDominatesAtRange)
{
    // The reason the heading column is in H at all. At ten metres a yaw sigma of
    // 0.05 rad is half a metre of lateral error; drop the term and the gate is
    // far too tight for anything but the cone under the bumper.
    const gtsam::Pose2 pose(0, 0, 0);
    gtsam::Matrix3 pose_cov = gtsam::Matrix3::Zero();
    pose_cov(2, 2) = 0.05 * 0.05;

    const gtsam::Matrix2 near = innovation_covariance(
        pose, gtsam::Point2(2.0, 0.0), gtsam::Matrix2::Zero(), pose_cov, iso(0.05));
    const gtsam::Matrix2 far = innovation_covariance(
        pose, gtsam::Point2(20.0, 0.0), gtsam::Matrix2::Zero(), pose_cov, iso(0.05));

    // Across the beam, which for a cone straight ahead is y.
    EXPECT_GT(std::sqrt(far(1, 1)), 0.9);
    EXPECT_LT(std::sqrt(near(1, 1)), 0.2);
}

TEST(MeasurementCovariance, ZeroFromPerceptionIsNotBelieved)
{
    // Perception publishes zeros when it has nothing to say. Taken literally that
    // is a perfectly known cone, which is the most dangerous thing SLAM could
    // believe about anything.
    const gtsam::Matrix2 R = measurement_covariance({0.0, 0.0, 0.0, 0.0}, 0.10, 0.03);
    EXPECT_NEAR(R(0, 0), 0.01, 1e-12);
    EXPECT_NEAR(R(1, 1), 0.01, 1e-12);
}

TEST(MeasurementCovariance, FloorsAnOverconfidentReport)
{
    const gtsam::Matrix2 R = measurement_covariance({1e-8, 0.0, 0.0, 1e-8}, 0.10, 0.03);
    EXPECT_NEAR(std::sqrt(R(0, 0)), 0.03, 1e-9);
}

TEST(MeasurementCovariance, SymmetrisesARoundTrippedMessage)
{
    // A message round trip leaves the off-diagonals a few ulps apart, and GTSAM
    // will not accept a noise model that is not symmetric.
    const gtsam::Matrix2 R =
        measurement_covariance({0.04, 0.01, 0.010000000001, 0.04}, 0.10, 0.03);
    EXPECT_NEAR(R(0, 1), R(1, 0), 1e-18);
}
