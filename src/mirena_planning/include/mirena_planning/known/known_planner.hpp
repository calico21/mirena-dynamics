// Planner for missions whose layout is surveyed in advance (skidpad,
// acceleration): register the stored template against the live map and publish
// the stored racing line through that transform.

#pragma once

#include <optional>
#include <random>
#include <vector>

#include "mirena_planning/planner.hpp"
#include "mirena_planning/known/track_template.hpp"

namespace mirena::planning
{

struct KnownPlannerParams
{
  /// A mapped cone within this of any template cone is an inlier. Sized for
  /// SLAM's landmark error, not for survey precision.
  double inlier_distance{0.6};
  /// Fraction of the matchable cones that must be inliers to accept a fit.
  double min_score{0.6};
  /// Absolute inlier floor, so a fit is never accepted off two lucky cones.
  int min_inliers{5};
  int icp_iterations{25};
  /// ICP stops once a step moves the fit less than this (metres and radians).
  double icp_tolerance{1e-4};
  /// Hypotheses drawn by the recovery RANSAC when ICP cannot lock on.
  int ransac_iterations{200};
  /// Template pairs tried per sampled observation pair.
  int ransac_candidates{16};
  /// Two cones are the same pair if their separations agree within this.
  double pair_distance_tolerance{0.5};
  /// Sampled pairs closer than this barely constrain the rotation.
  double min_pair_separation{4.0};
  /// How far the accepted fit may sit from the nominal transform. Both frames
  /// are anchored at the start line, so the residual is the car's placement
  /// error; anything larger means the fit slid down the track.
  double max_position_drift{5.0};
  double max_heading_drift{0.6};
};

class KnownPlanner : public Planner
{
public:
  explicit KnownPlanner(TrackTemplate track, KnownPlannerParams params = {});

  std::optional<Plan> plan(
    const std::vector<Observation> & observations, PlanDebug * debug) override;
  void reset() override;

  const TrackTemplate & track() const {return track_;}

private:
  struct Fit
  {
    Transform2 map_from_template{};
    double score{0.0};
    int inliers{0};
    int iterations{0};
  };

  /// Nearest template cone to a point already expressed in the template frame,
  /// or -1. The fit is purely geometric: a cone is a cone, and which one it is
  /// falls out of the registration rather than being asserted per cone.
  int match(Vec2 point) const;

  Fit evaluate(const std::vector<Observation> & observations, Transform2 guess) const;
  Fit refine(const std::vector<Observation> & observations, Transform2 seed) const;
  Fit recover(const std::vector<Observation> & observations) const;

  bool plausible(const Fit & fit) const;

  TrackTemplate track_;
  KnownPlannerParams params_;

  /// Every template cone, in template order, so a grid index is a cone index.
  Grid grid_;

  /// Template cone pairs sorted by separation. Two-point RANSAC looks up the
  /// pairs that are the same length as a sampled observation pair, which is
  /// what makes hypothesis generation tractable: sampling three unlabelled
  /// correspondences at random essentially never lands on a correct one.
  struct Pair
  {
    int i, j;
    double d;
  };
  std::vector<Pair> pairs_;

  Transform2 last_fit_{};
  bool locked_{false};
  mutable std::mt19937 rng_{2026u};
};

}  // namespace mirena::planning
