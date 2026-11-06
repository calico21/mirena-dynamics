#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "mirena_planning/geometry.hpp"
#include "mirena_planning/planner.hpp"

using namespace mirena::planning;  // NOLINT(build/namespaces)

namespace
{

constexpr double kEps = 1e-9;

}  // namespace

TEST(Transform2, InverseUndoesTheTransform)
{
  const Transform2 t{0.7, {3.0, -2.0}};
  const Vec2 p{1.5, 4.25};
  const Vec2 round_trip = t.inverse() * (t * p);
  EXPECT_NEAR(round_trip.x, p.x, 1e-12);
  EXPECT_NEAR(round_trip.y, p.y, 1e-12);
}

TEST(Transform2, ComposesLikeFunctionApplication)
{
  const Transform2 a{0.4, {1.0, 2.0}};
  const Transform2 b{-1.1, {-3.0, 0.5}};
  const Vec2 p{2.0, -1.0};
  const Vec2 composed = (a * b) * p;
  const Vec2 chained = a * (b * p);
  EXPECT_NEAR(composed.x, chained.x, 1e-12);
  EXPECT_NEAR(composed.y, chained.y, 1e-12);
}

TEST(RigidFit, RecoversAKnownTransform)
{
  const Transform2 truth{0.35, {2.5, -1.25}};
  std::vector<Vec2> src{{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {-2.0, 1.0}};
  std::vector<Vec2> dst;
  for (const Vec2 & p : src) {dst.push_back(truth * p);}

  const Transform2 fit = rigidFit(src, dst);
  EXPECT_NEAR(fit.psi, truth.psi, 1e-9);
  EXPECT_NEAR(fit.t.x, truth.t.x, 1e-9);
  EXPECT_NEAR(fit.t.y, truth.t.y, 1e-9);
}

TEST(RigidFit, IsExactForTwoCorrespondences)
{
  const Transform2 truth{-2.0, {7.0, 11.0}};
  const std::vector<Vec2> src{{0.0, 0.0}, {5.0, 0.0}};
  const std::vector<Vec2> dst{truth * src[0], truth * src[1]};

  const Transform2 fit = rigidFit(src, dst);
  EXPECT_NEAR(wrapAngle(fit.psi - truth.psi), 0.0, 1e-9);
  EXPECT_NEAR(fit.t.x, truth.t.x, 1e-9);
  EXPECT_NEAR(fit.t.y, truth.t.y, 1e-9);
}

TEST(ConeSizeFromString, MatchesThePerceptionVocabulary)
{
  EXPECT_EQ(coneSizeFromString("cone_big_orange"), ConeSize::Big);
  EXPECT_EQ(coneSizeFromString("BIG_ORANGE"), ConeSize::Big);
  // Everything that is not big is a cone of ordinary size. Nothing is rejected:
  // a colourless pipeline reports no colour, so a type that names none is the
  // normal case rather than a failure.
  EXPECT_EQ(coneSizeFromString("unknown_cone"), ConeSize::Small);
  EXPECT_EQ(coneSizeFromString("cone_blue"), ConeSize::Small);
  EXPECT_EQ(coneSizeFromString(""), ConeSize::Small);
}

TEST(Grid, FindsTheNearestPointAndRespectsTheRadius)
{
  const std::vector<Vec2> points{{0.0, 0.0}, {5.0, 0.0}, {0.0, 5.0}, {12.0, 12.0}};
  const Grid grid(points, 2.0);

  EXPECT_EQ(grid.nearest({4.6, 0.2}, 1.0), 1);
  EXPECT_EQ(grid.nearest({-30.0, 0.0}, 1.0), -1);
  EXPECT_EQ(grid.nearest({11.0, 11.0}, 5.0), 3);
}

TEST(Grid, VisitsEveryPointInTheQueryDisc)
{
  std::vector<Vec2> points;
  for (int i = 0; i < 20; ++i) {points.push_back({static_cast<double>(i), 0.0});}
  const Grid grid(points, 3.0);

  std::set<int> seen;
  grid.forEachNear({10.0, 0.0}, 2.0, [&](int i) {
      if (distance(points[i], {10.0, 0.0}) <= 2.0) {seen.insert(i);}
    });
  EXPECT_EQ(seen, (std::set<int>{8, 9, 10, 11, 12}));
}

TEST(Delaunay, TriangulatesASquareIntoTwoTriangles)
{
  const std::vector<Vec2> square{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  EXPECT_EQ(delaunay(square).size(), 2u);
}

TEST(Delaunay, CoversEveryInputPoint)
{
  std::vector<Vec2> points;
  for (int i = 0; i < 8; ++i) {
    points.push_back({static_cast<double>(i) * 1.5, 0.0});
    points.push_back({static_cast<double>(i) * 1.5 + 0.3, 3.0});
  }

  const auto triangles = delaunay(points);
  ASSERT_FALSE(triangles.empty());

  std::set<int> used;
  for (const Triangle & t : triangles) {used.insert(t.begin(), t.end());}
  EXPECT_EQ(used.size(), points.size());
}

TEST(Delaunay, HandlesDegenerateInputWithoutThrowing)
{
  EXPECT_TRUE(delaunay({}).empty());
  EXPECT_TRUE(delaunay({{0.0, 0.0}, {1.0, 0.0}}).empty());
  // Collinear points have no triangulation; the point is that we say so rather
  // than emitting a zero-area cell or aborting.
  EXPECT_TRUE(delaunay({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}}).empty());
}

TEST(Delaunay, SatisfiesTheEmptyCircumcircleProperty)
{
  const std::vector<Vec2> points{
    {0.0, 0.0}, {4.0, 0.4}, {8.1, -0.2}, {0.5, 3.0}, {4.2, 3.4}, {7.9, 3.1}, {2.0, 6.0}};

  for (const Triangle & t : delaunay(points)) {
    const Vec2 a = points[t[0]], b = points[t[1]], c = points[t[2]];
    const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    ASSERT_GT(std::abs(d), 1e-9);
    const double a2 = squaredNorm(a), b2 = squaredNorm(b), c2 = squaredNorm(c);
    const Vec2 centre{
      (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d,
      (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d};
    const double r2 = squaredNorm(a - centre);

    for (std::size_t i = 0; i < points.size(); ++i) {
      if (i == static_cast<std::size_t>(t[0]) || i == static_cast<std::size_t>(t[1]) ||
        i == static_cast<std::size_t>(t[2]))
      {
        continue;
      }
      EXPECT_GT(squaredNorm(points[i] - centre), r2 - 1e-9)
        << "point " << i << " lies inside a circumcircle";
    }
  }
}

TEST(BuildGates, OrientsAlongTheCentreline)
{
  const std::vector<Vec2> line{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  const auto gates = buildGates(line, false);
  ASSERT_EQ(gates.size(), line.size());
  for (const Gate & g : gates) {EXPECT_NEAR(g.psi, 0.0, kEps);}
}

TEST(BuildGates, WrapsHeadingOnAClosedTrack)
{
  const std::vector<Vec2> square{{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}};
  const auto gates = buildGates(square, true);
  ASSERT_EQ(gates.size(), 4u);
  // On a closed loop the first sample's heading comes from its neighbours on
  // both sides, so it is tangential rather than pointing at the next corner.
  EXPECT_NEAR(std::abs(wrapAngle(gates[0].psi - M_PI / 2.0)), 0.0, 1e-9);
}

TEST(TurnRadius, RecoversTheRadiusOfTheCircleItSampled)
{
  // Car at the origin facing +x, on a circle of radius 8 curving left. Every
  // point of that circle should read back as an 8 m corner, however far round it
  // sits -- that scale invariance is the whole reason the search tests a radius
  // instead of an angle.
  constexpr double kR = 8.0;
  const Vec2 heading{1.0, 0.0};
  for (const double arc : {0.2, 0.8, 1.5, 2.5}) {
    const Vec2 on_circle{kR * std::sin(arc), kR * (1.0 - std::cos(arc))};
    EXPECT_NEAR(turnRadius({0.0, 0.0}, heading, on_circle), kR, 1e-9) << "arc " << arc;
  }
}

TEST(TurnRadius, IsBlindToWhichWayTheCornerGoes)
{
  const Vec2 heading{0.0, 1.0};
  EXPECT_NEAR(
    turnRadius({0.0, 0.0}, heading, {2.0, 4.0}),
    turnRadius({0.0, 0.0}, heading, {-2.0, 4.0}), 1e-12);
}

TEST(TurnRadius, IsInfiniteStraightAheadAndZeroStraightBack)
{
  const Vec2 heading{1.0, 0.0};
  EXPECT_TRUE(std::isinf(turnRadius({0.0, 0.0}, heading, {5.0, 0.0})));
  EXPECT_EQ(turnRadius({0.0, 0.0}, heading, {-5.0, 0.0}), 0.0);
  // A point on top of the query is not a corner at all, and must not divide by
  // the zero-length chord.
  EXPECT_TRUE(std::isinf(turnRadius({1.0, 2.0}, heading, {1.0, 2.0})));
}

TEST(TurnRadius, TightensAsTheStepShortens)
{
  // The same 45 degree kink: a sweep at 5 m, a pivot at 0.5 m.
  const Vec2 heading{1.0, 0.0};
  const Vec2 far{5.0 * std::cos(M_PI / 4.0), 5.0 * std::sin(M_PI / 4.0)};
  const Vec2 near{0.5 * std::cos(M_PI / 4.0), 0.5 * std::sin(M_PI / 4.0)};
  EXPECT_GT(turnRadius({0.0, 0.0}, heading, far), 3.0);
  EXPECT_LT(turnRadius({0.0, 0.0}, heading, near), 0.5);
}
