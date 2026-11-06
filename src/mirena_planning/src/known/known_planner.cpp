#include "mirena_planning/known/known_planner.hpp"

#include <algorithm>
#include <utility>

namespace mirena::planning
{

KnownPlanner::KnownPlanner(TrackTemplate track, KnownPlannerParams params)
: track_(std::move(track)), params_(params)
{
  std::vector<Vec2> points;
  points.reserve(track_.cones.size());
  for (const Observation & cone : track_.cones) {points.push_back(cone.p);}
  grid_ = Grid(std::move(points), std::max(params_.inlier_distance, 1.0));

  for (std::size_t i = 0; i < track_.cones.size(); ++i) {
    for (std::size_t j = i + 1; j < track_.cones.size(); ++j) {
      const double d = distance(track_.cones[i].p, track_.cones[j].p);
      if (d >= params_.min_pair_separation) {
        pairs_.push_back({static_cast<int>(i), static_cast<int>(j), d});
      }
    }
  }
  std::sort(
    pairs_.begin(), pairs_.end(), [](const Pair & a, const Pair & b) {return a.d < b.d;});

  reset();
}

void KnownPlanner::reset()
{
  last_fit_ = track_.map_from_template;
  locked_ = false;
}

int KnownPlanner::match(Vec2 point) const
{
  return grid_.nearest(point, params_.inlier_distance);
}

KnownPlanner::Fit KnownPlanner::evaluate(
  const std::vector<Observation> & observations, Transform2 guess) const
{
  const Transform2 to_template = guess.inverse();
  int inliers = 0;
  for (const Observation & o : observations) {
    if (match(to_template * o.p) >= 0) {++inliers;}
  }

  Fit fit;
  fit.map_from_template = guess;
  fit.inliers = inliers;
  // Denominator is what *could* match. Dividing by the observation count alone
  // punished a perfect fit for every extra cone SLAM had mapped beyond the
  // template, which is why the old gate was unreachable in practice.
  const auto matchable = std::max<std::size_t>(
    1, std::min(observations.size(), track_.cones.size()));
  fit.score = static_cast<double>(inliers) / static_cast<double>(matchable);
  return fit;
}

KnownPlanner::Fit KnownPlanner::refine(
  const std::vector<Observation> & observations, Transform2 seed) const
{
  Transform2 current = seed;
  std::vector<Vec2> from, to;
  int iteration = 0;

  for (; iteration < params_.icp_iterations; ++iteration) {
    const Transform2 to_template = current.inverse();
    from.clear();
    to.clear();
    for (const Observation & o : observations) {
      const Vec2 q = to_template * o.p;
      const int m = match(q);
      if (m >= 0) {
        from.push_back(track_.cones[m].p);
        to.push_back(q);
      }
    }
    if (static_cast<int>(from.size()) < 2) {break;}

    // Both sides live in the template frame, so the fit is the correction to
    // apply on the template side of the current estimate.
    const Transform2 step = rigidFit(from, to);
    current = current * step;
    if (norm(step.t) < params_.icp_tolerance &&
      std::abs(step.psi) < params_.icp_tolerance)
    {
      ++iteration;
      break;
    }
  }

  Fit fit = evaluate(observations, current);
  fit.iterations = iteration;
  return fit;
}

KnownPlanner::Fit KnownPlanner::recover(const std::vector<Observation> & observations) const
{
  Fit best;
  if (observations.size() < 2 || pairs_.empty()) {return best;}

  std::uniform_int_distribution<std::size_t> pick(0, observations.size() - 1);
  Transform2 best_guess{};
  double best_raw = 0.0;

  for (int iteration = 0; iteration < params_.ransac_iterations; ++iteration) {
    const std::size_t a = pick(rng_), b = pick(rng_);
    if (a == b) {continue;}
    const double d = distance(observations[a].p, observations[b].p);
    if (d < params_.min_pair_separation) {continue;}

    const double tol = params_.pair_distance_tolerance;
    const auto by_distance = [](const Pair & p, double value) {return p.d < value;};
    const auto lo = std::lower_bound(pairs_.begin(), pairs_.end(), d - tol, by_distance);

    int tried = 0;
    for (auto it = lo; it != pairs_.end() && it->d <= d + tol; ++it) {
      // The pair is unordered, so both assignments to (a, b) are candidates.
      for (const bool swapped : {false, true}) {
        const int i = swapped ? it->j : it->i;
        const int j = swapped ? it->i : it->j;
        if (++tried > params_.ransac_candidates) {break;}

        const Fit hypothesis = evaluate(
          observations,
          rigidFit(
            {track_.cones[i].p, track_.cones[j].p},
            {observations[a].p, observations[b].p}));
        if (hypothesis.score > best_raw) {
          best_raw = hypothesis.score;
          best_guess = hypothesis.map_from_template;
        }
      }
      if (tried > params_.ransac_candidates) {break;}
    }

    if (best_raw >= 0.95) {break;}
  }

  if (best_raw <= 0.0) {return best;}
  // Only the winning hypothesis is worth polishing; scoring is cheap, ICP is
  // not, and running it per hypothesis is what would blow the cycle budget.
  return refine(observations, best_guess);
}

bool KnownPlanner::plausible(const Fit & fit) const
{
  if (fit.inliers < params_.min_inliers || fit.score < params_.min_score) {return false;}

  // Drift is measured where it means something: both transforms map the origin
  // of the map frame onto a point of the template, and the map frame is the
  // pose the car started from, so the gap between those two points is exactly
  // how far off the start line the fit claims the car was. Comparing the raw
  // translations instead would mix in a lever arm the length of the track.
  const Transform2 & nominal = track_.map_from_template;
  const Vec2 fitted_start = fit.map_from_template.inverse() * Vec2{0.0, 0.0};
  const Vec2 nominal_start = nominal.inverse() * Vec2{0.0, 0.0};

  return distance(fitted_start, nominal_start) <= params_.max_position_drift &&
         std::abs(wrapAngle(fit.map_from_template.psi - nominal.psi)) <=
         params_.max_heading_drift;
}

std::optional<Plan> KnownPlanner::plan(
  const std::vector<Observation> & observations, PlanDebug * debug)
{
  // Start from what the geometry already tells us: the nominal transform on the
  // first attempt, the previous solution afterwards. RANSAC is the recovery
  // path, not the primary one.
  Fit fit = refine(observations, locked_ ? last_fit_ : track_.map_from_template);
  bool from_ransac = false;
  if (!plausible(fit)) {
    const Fit recovered = recover(observations);
    if (recovered.score > fit.score) {
      fit = recovered;
      from_ransac = true;
    }
  }

  if (debug) {
    debug->fit = fit.map_from_template;
    debug->score = fit.score;
    debug->iterations = fit.iterations;
    debug->from_ransac = from_ransac;
    debug->registered = plausible(fit);
  }

  if (debug && debug->verbose) {
    const Transform2 to_template = fit.map_from_template.inverse();
    for (const Observation & o : observations) {
      const int m = match(to_template * o.p);
      if (m >= 0) {
        debug->correspondences.emplace_back(o.p, fit.map_from_template * track_.cones[m].p);
      } else {
        debug->rejected.push_back(o.p);
      }
    }
    for (const Observation & cone : track_.cones) {
      debug->fitted_template.push_back(
        Observation{fit.map_from_template * cone.p, cone.size});
    }
    for (const Vec2 & p : track_.path) {
      debug->fitted_path.push_back(fit.map_from_template * p);
    }
  }

  if (!plausible(fit)) {
    locked_ = false;
    return std::nullopt;
  }
  last_fit_ = fit.map_from_template;
  locked_ = true;

  std::vector<Vec2> path;
  path.reserve(track_.path.size());
  for (const Vec2 & p : track_.path) {path.push_back(fit.map_from_template * p);}

  Plan plan;
  plan.gates = buildGates(path, track_.closed);
  plan.closed = track_.closed;
  plan.start = path.front();
  plan.finish = track_.closed ? path.front() : path.back();
  return plan;
}

}  // namespace mirena::planning
