// Known-track planner for Acceleration.
// Registration is recovered from geometric gate sequences, without cone colours.
#pragma once

#include <optional>
#include <random>
#include <vector>

#include "mirena_planning/planner.hpp"
#include "mirena_planning/known/track_template.hpp"

namespace mirena::planning
{

struct KnownPlannerAccelerationParams
{
  double inlier_distance{0.6};
  double min_score{0.6};
  int min_inliers{5};

  int icp_iterations{25};
  double icp_tolerance{1e-4};

  // Number of random structural hypotheses (two candidate gates validated by a third).
  int ransac_iterations{200};

  // The 3 m minimum is the rule; 5 m is a configurable search cap.
  double gate_min_width{3.0};
  double gate_max_width{5.0};

  // Maximum difference in longitudinal coordinates for the two cones of one gate.
  double gate_longitudinal_tolerance{0.75};

  // A candidate gate should be approximately perpendicular to the track direction.
  double gate_angle_tolerance{0.45};  // radians, about 26 degrees

  // Tolerances for validating the first three gates and filtering gate centres.
  double sequence_line_tolerance{1.0};
  double gate_centerline_tolerance{1.5};

  // Compare local gate spacing with the actual template spacing (not a fixed 5 m rule).
  double gate_spacing_tolerance{2.0};

  double max_position_drift{5.0};
  double max_heading_drift{0.6};
};

class KnownPlannerAcceleration : public Planner
{
public:
  explicit KnownPlannerAcceleration(
    TrackTemplate track, KnownPlannerAccelerationParams params = {});

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

  Fit evaluate(const std::vector<Observation> & observations, Transform2 guess) const;
  Fit refine(const std::vector<Observation> & observations, Transform2 seed) const;

  // Recovery samples gate-sequence hypotheses rather than arbitrary cone pairs.
  Fit recover(const std::vector<Observation> & observations) const;
  bool plausible(const Fit & fit) const;

  TrackTemplate track_;
  KnownPlannerAccelerationParams params_;
  Transform2 last_fit_{};
  bool locked_{false};
  mutable std::mt19937 rng_{2026u};
};

}  // namespace mirena::planning
