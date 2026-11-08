#include "mirena_planning/known/known_planner_acceleration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <utility>

namespace mirena::planning
{
namespace
{

struct GateCandidate
{
  int a{-1};
  int b{-1};
  Vec2 center{};
  Vec2 segment{};   // Unit vector along the gate.
  double width{0.0};
  double s{0.0};    // Longitudinal coordinate of the centre.
  double lateral{0.0};
};

Vec2 perpendicular(Vec2 u)
{
  return {-u.y, u.x};
}

bool sharesCone(const GateCandidate & a, const GateCandidate & b)
{
  return a.a == b.a || a.a == b.b || a.b == b.a || a.b == b.b;
}

bool disjoint(const GateCandidate & a, const GateCandidate & b, const GateCandidate & c)
{
  return !sharesCone(a, b) && !sharesCone(a, c) && !sharesCone(b, c);
}

bool transverse(const GateCandidate & gate, Vec2 direction, double angle_tolerance)
{
  const Vec2 u = normalized(direction);
  if (squaredNorm(u) < 0.5) {return false;}
  // A gate is transverse when its segment has only a small component along the track.
  return std::abs(dot(gate.segment, u)) <= std::sin(angle_tolerance);
}

std::vector<GateCandidate> makeRawCandidates(
  const std::vector<Observation> & observations,
  const KnownPlannerAccelerationParams & params)
{
  std::vector<GateCandidate> out;
  const std::size_t n = observations.size();
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const Vec2 delta = observations[j].p - observations[i].p;
      const double width = norm(delta);
      if (width + 1e-9 < params.gate_min_width ||
        width - 1e-9 > params.gate_max_width)
      {
        continue;
      }

      GateCandidate g;
      g.a = static_cast<int>(i);
      g.b = static_cast<int>(j);
      g.center = (observations[i].p + observations[j].p) * 0.5;
      g.segment = normalized(delta);
      g.width = width;
      out.push_back(g);
    }
  }
  return out;
}

std::vector<GateCandidate> makeTemplateGates(
  const TrackTemplate & track,
  const KnownPlannerAccelerationParams & params)
{
  std::vector<GateCandidate> out;
  if (track.cones.size() < 2 || track.path.size() < 2) {return out;}

  const Vec2 origin = track.path.front();
  const Vec2 u = normalized(track.path.back() - track.path.front());
  if (squaredNorm(u) < 0.5) {return out;}
  const Vec2 n = perpendicular(u);

  for (std::size_t i = 0; i < track.cones.size(); ++i) {
    for (std::size_t j = i + 1; j < track.cones.size(); ++j) {
      const Vec2 delta = track.cones[j].p - track.cones[i].p;
      const double width = norm(delta);
      if (width + 1e-9 < params.gate_min_width ||
        width - 1e-9 > params.gate_max_width)
      {
        continue;
      }
      if (std::abs(dot(delta, u)) > params.gate_longitudinal_tolerance) {continue;}

      GateCandidate g;
      g.a = static_cast<int>(i);
      g.b = static_cast<int>(j);
      g.center = (track.cones[i].p + track.cones[j].p) * 0.5;
      g.segment = normalized(delta);
      g.width = width;
      g.s = dot(g.center - origin, u);
      g.lateral = dot(g.center - origin, n);
      out.push_back(g);
    }
  }

  std::sort(out.begin(), out.end(), [](const GateCandidate & a, const GateCandidate & b) {
    return a.s < b.s;
  });
  return out;
}

std::array<Vec2, 2> orderedEndpoints(
  const GateCandidate & gate,
  const std::vector<Observation> & observations,
  Vec2 direction)
{
  const Vec2 n = perpendicular(normalized(direction));
  const Vec2 a = observations[static_cast<std::size_t>(gate.a)].p;
  const Vec2 b = observations[static_cast<std::size_t>(gate.b)].p;
  return dot(a, n) <= dot(b, n) ? std::array<Vec2, 2>{a, b} :
                                  std::array<Vec2, 2>{b, a};
}

std::array<Vec2, 2> orderedTemplateEndpoints(
  const GateCandidate & gate, const TrackTemplate & track, Vec2 direction)
{
  const Vec2 n = perpendicular(normalized(direction));
  const Vec2 a = track.cones[static_cast<std::size_t>(gate.a)].p;
  const Vec2 b = track.cones[static_cast<std::size_t>(gate.b)].p;
  return dot(a, n) <= dot(b, n) ? std::array<Vec2, 2>{a, b} :
                                  std::array<Vec2, 2>{b, a};
}

}  // namespace

KnownPlannerAcceleration::KnownPlannerAcceleration(
  TrackTemplate track, KnownPlannerAccelerationParams params)
: track_(std::move(track)), params_(params)
{
  reset();
}

void KnownPlannerAcceleration::reset()
{
  last_fit_ = track_.map_from_template;
  locked_ = false;
}

KnownPlannerAcceleration::Fit KnownPlannerAcceleration::evaluate(
  const std::vector<Observation> & observations, Transform2 guess) const
{
  const Transform2 to_template = guess.inverse();
  int inliers = 0;
  std::vector<bool> used(track_.cones.size(), false);

  // Count unique template cones, so multiple SLAM observations cannot inflate the score.
  for (const Observation & o : observations) {
    const Vec2 q = to_template * o.p;
    int best = -1;
    double best_d2 = params_.inlier_distance * params_.inlier_distance;
    for (std::size_t i = 0; i < track_.cones.size(); ++i) {
      if (used[i]) {continue;}
      const double d2 = squaredNorm(track_.cones[i].p - q);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = static_cast<int>(i);
      }
    }
    if (best >= 0) {
      used[static_cast<std::size_t>(best)] = true;
      ++inliers;
    }
  }

  Fit fit;
  fit.map_from_template = guess;
  fit.inliers = inliers;
  const std::size_t matchable = std::max<std::size_t>(
    1, std::min(observations.size(), track_.cones.size()));
  fit.score = static_cast<double>(inliers) / static_cast<double>(matchable);
  return fit;
}

KnownPlannerAcceleration::Fit KnownPlannerAcceleration::refine(
  const std::vector<Observation> & observations, Transform2 seed) const
{
  Transform2 current = seed;
  std::vector<Vec2> from, to;
  int iteration = 0;

  for (; iteration < params_.icp_iterations; ++iteration) {
    const Transform2 to_template = current.inverse();
    from.clear();
    to.clear();
    std::vector<bool> used(track_.cones.size(), false);

    for (const Observation & o : observations) {
      const Vec2 q = to_template * o.p;
      int best = -1;
      double best_d2 = params_.inlier_distance * params_.inlier_distance;
      for (std::size_t i = 0; i < track_.cones.size(); ++i) {
        if (used[i]) {continue;}
        const double d2 = squaredNorm(track_.cones[i].p - q);
        if (d2 < best_d2) {
          best_d2 = d2;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        used[static_cast<std::size_t>(best)] = true;
        from.push_back(track_.cones[static_cast<std::size_t>(best)].p);
        to.push_back(q);
      }
    }

    if (from.size() < 2) {break;}
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

KnownPlannerAcceleration::Fit KnownPlannerAcceleration::recover(
  const std::vector<Observation> & observations) const
{
  Fit best;
  if (observations.size() < 6 || track_.cones.size() < 6 ||
    params_.ransac_iterations <= 0 || track_.path.size() < 2)
  {
    return best;
  }

  const auto raw = makeRawCandidates(observations, params_);
  const auto template_gates = makeTemplateGates(track_, params_);
  if (raw.size() < 3 || template_gates.size() < 3) {return best;}

  const Vec2 template_direction =
    normalized(track_.path.back() - track_.path.front());
  if (squaredNorm(template_direction) < 0.5) {return best;}

  std::uniform_int_distribution<std::size_t> pick(0, raw.size() - 1);
  double best_score = -1.0;
  int best_inliers = -1;
  Transform2 best_guess{};

  // RANSAC samples two candidate gates. Their centre-to-centre vector defines
  // an unoriented track axis for this hypothesis. Its sign is arbitrary: both
  // forward and reversed template sequences are tested below. In particular,
  // do not try to infer the physical driving direction by comparing projections
  // made with a direction that was itself defined from the same two centres.
  for (int iteration = 0; iteration < params_.ransac_iterations; ++iteration) {
    const GateCandidate & a = raw[pick(rng_)];
    const GateCandidate & b = raw[pick(rng_)];
    if (sharesCone(a, b)) {continue;}

    const Vec2 delta = b.center - a.center;
    if (squaredNorm(delta) < 1.0) {continue;}
    const Vec2 direction = normalized(delta);  // Arbitrary sign; reverse is tested.
    if (!transverse(a, direction, params_.gate_angle_tolerance) ||
      !transverse(b, direction, params_.gate_angle_tolerance))
    {
      continue;
    }

    const Vec2 n = perpendicular(direction);
    const double sa = dot(a.center, direction);
    const double sb = dot(b.center, direction);
    // Since direction = normalize(b.center - a.center), sb > sa by construction.
    // The test below is only a numerical-degeneracy guard, not a forward test.
    if (sb - sa < 1e-6) {continue;}
    if (std::abs(dot(b.center - a.center, n)) > params_.sequence_line_tolerance) {
      continue;
    }

    // Consider every compatible third gate rather than accepting the first one.
    // This avoids making the result depend on raw-candidate enumeration order.
    for (const GateCandidate & c : raw) {
      if (sharesCone(c, a) || sharesCone(c, b)) {continue;}
      if (!transverse(c, direction, params_.gate_angle_tolerance)) {continue;}

      const double sc = dot(c.center, direction);
      if (sc <= sb + 0.5) {continue;}
      if (std::abs(dot(c.center - a.center, n)) > params_.sequence_line_tolerance ||
        std::abs(dot(c.center - b.center, n)) > params_.sequence_line_tolerance)
      {
        continue;
      }

      // The three observed gates are explicitly sorted by their projection on
      // the hypothesised axis. No greedy global de-duplication is performed:
      // alternative gates sharing a cone remain available to other hypotheses.
      std::array<GateCandidate, 3> observed{a, b, c};
      std::sort(observed.begin(), observed.end(), [&](
          const GateCandidate & lhs, const GateCandidate & rhs) {
          return dot(lhs.center, direction) < dot(rhs.center, direction);
        });

      const double observed_d01 = distance(observed[0].center, observed[1].center);
      const double observed_d12 = distance(observed[1].center, observed[2].center);

      // Match the seed triple against every consecutive template triple, in both
      // sequence directions. Spacings come from the actual template, so this
      // does not impose a fixed 5 m distance between all gates.
      for (std::size_t ti = 0; ti + 2 < template_gates.size(); ++ti) {
        for (const bool reverse : {false, true}) {
          const std::size_t t0 = reverse ? template_gates.size() - 1 - ti : ti;
          const std::size_t t1 = reverse ? template_gates.size() - 2 - ti : ti + 1;
          const std::size_t t2 = reverse ? template_gates.size() - 3 - ti : ti + 2;
          const std::array<GateCandidate, 3> templ{
            template_gates[t0], template_gates[t1], template_gates[t2]};
          if (!disjoint(templ[0], templ[1], templ[2])) {continue;}

          const double template_d01 = distance(templ[0].center, templ[1].center);
          const double template_d12 = distance(templ[1].center, templ[2].center);
          if (std::abs(observed_d01 - template_d01) > params_.gate_spacing_tolerance ||
            std::abs(observed_d12 - template_d12) > params_.gate_spacing_tolerance)
          {
            continue;
          }

          for (const bool swap_sides : {false, true}) {
            std::vector<Vec2> src;
            std::vector<Vec2> dst;
            src.reserve(6);
            dst.reserve(6);

            for (std::size_t k = 0; k < 3; ++k) {
              const auto observed_ends =
                orderedEndpoints(observed[k], observations, direction);
              const auto template_ends =
                orderedTemplateEndpoints(templ[k], track_, template_direction);
              src.push_back(template_ends[swap_sides ? 1 : 0]);
              src.push_back(template_ends[swap_sides ? 0 : 1]);
              dst.push_back(observed_ends[0]);
              dst.push_back(observed_ends[1]);
            }

            const Transform2 hypothesis = rigidFit(src, dst);
            const Fit scored = evaluate(observations, hypothesis);
            if (!plausible(scored)) {continue;}

            // Prefer more unique inliers first, then the normalised score. This
            // gives a deterministic tie-break when candidate sequences score alike.
            if (scored.inliers > best_inliers ||
              (scored.inliers == best_inliers && scored.score > best_score))
            {
              best_inliers = scored.inliers;
              best_score = scored.score;
              best_guess = hypothesis;
            }
          }
        }
      }
    }
  }

  if (best_score < 0.0) {return best;}

  // Refine only the best globally scored seed. Keep the refined fit only if it
  // still passes the same quality/plausibility checks; otherwise report failure.
  const Fit refined = refine(observations, best_guess);
  return plausible(refined) ? refined : best;
}

bool KnownPlannerAcceleration::plausible(const Fit & fit) const
{
  if (fit.inliers < params_.min_inliers || fit.score < params_.min_score) {return false;}
  const Transform2 & nominal = track_.map_from_template;
  const Vec2 fitted_start = fit.map_from_template.inverse() * Vec2{0.0, 0.0};
  const Vec2 nominal_start = nominal.inverse() * Vec2{0.0, 0.0};
  return distance(fitted_start, nominal_start) <= params_.max_position_drift &&
         std::abs(wrapAngle(fit.map_from_template.psi - nominal.psi)) <= params_.max_heading_drift;
}

std::optional<Plan> KnownPlannerAcceleration::plan(
  const std::vector<Observation> & observations, PlanDebug * debug)
{
  Fit fit = refine(observations, locked_ ? last_fit_ : track_.map_from_template);
  bool recovered_by_gate_search = false;

  if (!plausible(fit)) {
    const Fit recovered = recover(observations);
    // A valid recovered registration must not be rejected merely because an
    // invalid initial ICP fit happens to have a numerically higher score.
    if (plausible(recovered) && (!plausible(fit) || recovered.score > fit.score)) {
      fit = recovered;
      recovered_by_gate_search = true;
    }
  }

  if (debug) {
    debug->fit = fit.map_from_template;
    debug->score = fit.score;
    debug->iterations = fit.iterations;
    // Kept for compatibility with PlanDebug and the existing ROS visualisation.
    debug->from_ransac = recovered_by_gate_search;
    debug->registered = plausible(fit);
  }

  if (debug && debug->verbose) {
    const Transform2 to_template = fit.map_from_template.inverse();
    std::vector<bool> used_template(track_.cones.size(), false);
    for (const Observation & o : observations) {
      const Vec2 q = to_template * o.p;
      int best = -1;
      double best_d2 = params_.inlier_distance * params_.inlier_distance;
      for (std::size_t i = 0; i < track_.cones.size(); ++i) {
        if (used_template[i]) {continue;}
        const double d2 = squaredNorm(track_.cones[i].p - q);
        if (d2 < best_d2) {
          best_d2 = d2;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        used_template[static_cast<std::size_t>(best)] = true;
        debug->correspondences.emplace_back(
          o.p, fit.map_from_template * track_.cones[static_cast<std::size_t>(best)].p);
      } else {
        debug->rejected.push_back(o.p);
      }
    }
    for (const Observation & cone : track_.cones) {
      debug->fitted_template.push_back(Observation{fit.map_from_template * cone.p, cone.size});
    }
    for (const Vec2 & p : track_.path) {debug->fitted_path.push_back(fit.map_from_template * p);}
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

  Plan result;
  result.gates = buildGates(path, track_.closed);
  result.closed = track_.closed;
  result.start = path.front();
  result.finish = track_.closed ? path.front() : path.back();
  return result;
}

}  // namespace mirena::planning
