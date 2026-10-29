#pragma once

// A track, a car driving round it, and a sensor that lies about what it sees in
// exactly the way the real one does.
//
// Everything the test suite asserts is a property of the estimator, and the
// estimator's input is a stream of (odometry, detections). This produces that
// stream from a real track layout, deterministically, with the pathologies that
// matter dialled in by hand: drift that accumulates, cones that are missed,
// cones that were never there, and a measurement covariance taken from
// perception's own model rather than invented here.
//
// The layouts are the ones mirena_planning tests against, read as data from
// test/tracks rather than copied, so the two suites cannot drift into
// disagreeing about what a track is.

#include <gtsam/geometry/Pose2.h>

#include <cmath>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "slam.hpp"

namespace mirena::test
{

    /// Perception's cone type strings. A colourless detector reports size, and
    /// only size, so the coloured cones of a track layout all arrive as one type.
    inline constexpr const char *kSmall = "unknown_cone";
    inline constexpr const char *kBig = "cone_big_orange";

    struct Cone
    {
        gtsam::Point2 position;
        std::string type;
    };

    struct Track
    {
        std::vector<Cone> cones;
        std::vector<gtsam::Point2> path;
        gtsam::Pose2 start;
        bool closed = true;
    };

    inline Track load_track(const std::string &name)
    {
        const std::string path = std::string(MIRENA_SLAM_TRACK_DIR) + "/" + name + ".json";
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("cannot open track " + path);
        nlohmann::json j;
        in >> j;

        Track track;
        for (const auto &c : j.at("cones"))
        {
            const std::string type = c.at("type").get<std::string>();
            track.cones.push_back(
                {gtsam::Point2(c.at("x").get<double>(), c.at("y").get<double>()),
                 type == kBig ? kBig : kSmall});
        }
        for (const auto &p : j.at("path").at("points"))
            track.path.emplace_back(p.at("x").get<double>(), p.at("y").get<double>());
        track.closed = j.at("path").value("closed", true);

        const auto &s = j.at("setup").at("car_start_pose");
        track.start = gtsam::Pose2(s.value("x", 0.0), s.value("y", 0.0), s.value("psi", 0.0));
        return track;
    }

    /// What perception would say about a cone at this range, copied from
    /// ConeDetector::position_covariance so the estimator is tuned against the
    /// noise it will actually be given: sharp along the beam where a range is
    /// measured directly, blunt across it where only an angle is.
    struct SensorModel
    {
        double range_sigma = 0.08;
        double bearing_sigma = 0.010;
        double cluster_spread = 0.30;
        int typical_points = 12;
        double max_range = 20.0;
        double half_angle = 1.05;

        /// In the car frame, for a cone at `local`.
        gtsam::Matrix2 covariance(const gtsam::Point2 &local) const
        {
            const double range = local.norm();
            const double spread = cluster_spread / std::sqrt(static_cast<double>(typical_points));
            const double along = range_sigma * range_sigma + spread * spread;
            const double across = std::pow(bearing_sigma * range, 2.0) + spread * spread;
            gtsam::Matrix2 R;
            if (range < 1e-3)
            {
                R << along, 0.0, 0.0, along;
                return R;
            }
            const double cs = local.x() / range, sn = local.y() / range;
            const double xx = cs * cs * along + sn * sn * across;
            const double yy = sn * sn * along + cs * cs * across;
            const double xy = cs * sn * (along - across);
            R << xx, xy, xy, yy;
            return R;
        }
    };

    struct SimParams
    {
        double speed = 6.0; ///< m/s along the path
        double rate = 8.0;  ///< perception frames per second

        // Odometry error, in two parts, because the two do different damage.
        //
        // The white part averages out and is what the reported sigmas are mostly
        // about. The systematic part does not: a scale factor and a heading bias
        // the estimator never learns are what make a lap fail to close, and
        // closing it is the whole reason the graph keeps every pose.
        //
        // Sized against what this car's odometry actually does. It is a GPS-aided
        // INS, not free-running dead reckoning: mirena_state reports a converged
        // position sigma of 0.17 m, and the largest divergence between the filter
        // and the INS measured over a whole run was 1.27 m. So the drift over a
        // 270 m lap is of order a metre, not several.
        //
        // The distinction is not academic. At three metres of drift per lap the
        // returning car cannot associate anything -- the correct cone is further
        // away than max_association_distance and the neighbouring one is closer --
        // so loop closure never starts and the map simply doubles every lap. Two
        // earlier versions of this file got that wrong in the same direction, the
        // first producing 21 m and 40 deg over a single lap. Both times it read as
        // the estimator failing.
        double odom_sigma_per_m = 0.02;         ///< white, metres per metre driven
        double odom_sigma_theta_per_m = 0.002;  ///< white, radians per metre driven
        double odom_scale_bias = 0.001;         ///< systematic, fraction long
        double odom_heading_bias = 0.00005;     ///< systematic, radians per metre
        /// Chance a cone genuinely in view is reported at all.
        double p_detect = 0.85;
        /// Phantoms per frame, placed uniformly in the viewing wedge.
        double false_per_frame = 0.10;

        unsigned seed = 1;
    };

    /// One frame, in the form Slam::observe takes.
    struct Frame
    {
        gtsam::Pose2 raw_odom;   ///< what the odometry believes
        gtsam::Pose2 truth;      ///< where the car actually is
        std::vector<Observation> observations;
        double travelled = 0.0;  ///< metres since the start, for lap accounting
    };

    /// Drives the car round the path and reports what it sees.
    class Simulator
    {
    public:
        Simulator(Track track, SimParams params, SensorModel sensor = {})
            : track_(std::move(track)), params_(params), sensor_(sensor),
              rng_(params.seed)
        {
            truth_ = track_.start;
            odom_ = track_.start;
            // The path starts ahead of the car, which sits at the origin facing
            // +x; the first segment is the one it drives onto.
            segment_ = 0;
        }

        /// Advances one perception frame. Returns false once `laps` are done.
        bool step(Frame &out, int laps)
        {
            if (laps_done_ >= laps)
                return false;

            const double step = params_.speed / params_.rate;
            advance(step);

            out.truth = truth_;
            out.raw_odom = odom_;
            out.travelled = travelled_;
            out.observations.clear();

            std::normal_distribution<double> unit(0.0, 1.0);
            std::uniform_real_distribution<double> uniform(0.0, 1.0);

            for (const Cone &cone : track_.cones)
            {
                const gtsam::Point2 local = truth_.transformTo(cone.position);
                if (!visible(local))
                    continue;
                if (uniform(rng_) > params_.p_detect)
                    continue;

                const gtsam::Matrix2 R = sensor_.covariance(local);
                const Eigen::LLT<gtsam::Matrix2> llt(R);
                const gtsam::Vector2 noise =
                    llt.matrixL() * gtsam::Vector2(unit(rng_), unit(rng_));
                const gtsam::Point2 measured = local + noise;

                out.observations.push_back(
                    {measured.x(), measured.y(), cone.type,
                     {R(0, 0), R(0, 1), R(1, 0), R(1, 1)}});
            }

            // Phantoms: a detector reporting something that is not a cone. These
            // are what the candidate tier exists to absorb, so the suite has to
            // produce them.
            if (uniform(rng_) < params_.false_per_frame)
            {
                const double range = 3.0 + uniform(rng_) * (sensor_.max_range - 3.0);
                const double bearing = (uniform(rng_) * 2.0 - 1.0) * sensor_.half_angle;
                const gtsam::Point2 local(range * std::cos(bearing), range * std::sin(bearing));
                const gtsam::Matrix2 R = sensor_.covariance(local);
                out.observations.push_back({local.x(), local.y(), kSmall,
                                            {R(0, 0), R(0, 1), R(1, 0), R(1, 1)}});
            }

            return true;
        }

        /// The odometry sigmas the node would compute for this frame: what the
        /// drift model could have accumulated since the last keyframe.
        gtsam::Vector3 odom_sigmas(const gtsam::Pose2 &since_raw) const
        {
            const gtsam::Pose2 delta = since_raw.between(odom_);
            const double dist = delta.translation().norm();
            const double turn = std::abs(delta.theta());
            const double xy = std::max(
                0.02, (params_.odom_sigma_per_m + params_.odom_scale_bias) * dist);
            const double th = std::max(
                0.005, (params_.odom_sigma_theta_per_m + params_.odom_heading_bias) * dist +
                           0.01 * turn);
            return gtsam::Vector3(xy, xy, th);
        }

        int laps_done() const { return laps_done_; }
        const Track &track() const { return track_; }

    private:
        bool visible(const gtsam::Point2 &local) const
        {
            return local.norm() < sensor_.max_range && local.x() > 0.0 &&
                   std::abs(std::atan2(local.y(), local.x())) < sensor_.half_angle;
        }

        void advance(double step)
        {
            const gtsam::Pose2 before = truth_;

            double remaining = step;
            while (remaining > 1e-9)
            {
                const gtsam::Point2 target = track_.path[(segment_ + 1) % track_.path.size()];
                const gtsam::Point2 to_target = target - truth_.translation();
                const double distance = to_target.norm();

                if (distance <= remaining)
                {
                    place(target, std::atan2(to_target.y(), to_target.x()));
                    remaining -= distance;
                    ++segment_;
                    if (segment_ >= static_cast<int>(track_.path.size()))
                    {
                        segment_ = 0;
                        ++laps_done_;
                    }
                    continue;
                }
                const gtsam::Point2 next =
                    truth_.translation() + to_target * (remaining / distance);
                place(next, std::atan2(to_target.y(), to_target.x()));
                remaining = 0.0;
            }
            travelled_ += step;

            // Odometry sees the same motion, wrong: a little noise on every step
            // and a bias that never goes away.
            const gtsam::Pose2 delta = before.between(truth_);
            const double d = delta.translation().norm();
            std::normal_distribution<double> unit(0.0, 1.0);
            const gtsam::Pose2 wrong(
                delta.x() * (1.0 + params_.odom_scale_bias) +
                    params_.odom_sigma_per_m * d * unit(rng_),
                delta.y() + params_.odom_sigma_per_m * d * unit(rng_),
                delta.theta() + params_.odom_heading_bias * d +
                    params_.odom_sigma_theta_per_m * d * unit(rng_));
            odom_ = odom_.compose(wrong);
        }

        void place(const gtsam::Point2 &p, double heading)
        {
            truth_ = gtsam::Pose2(p.x(), p.y(), heading);
        }

        Track track_;
        SimParams params_;
        SensorModel sensor_;
        std::mt19937 rng_;

        gtsam::Pose2 truth_;
        gtsam::Pose2 odom_;
        int segment_ = 0;
        int laps_done_ = 0;
        double travelled_ = 0.0;
    };

    /// Drives a whole run through a Slam, exactly as the node does: observe every
    /// frame, let the estimator say when a pose is worth adding.
    struct RunResult
    {
        std::vector<Landmark> map;
        std::vector<double> update_ms;
        std::vector<int> reeliminated;
        /// Per-keyframe change in the correction, for continuity assertions.
        std::vector<double> correction_jump_xy;
        std::vector<double> correction_jump_theta;
        int keyframes = 0;
        int promotions = 0;
        bool healthy = true;
    };

    inline RunResult drive(Slam &slam, Simulator &sim, int laps)
    {
        RunResult result;
        gtsam::Pose2 last_keyframe_raw = sim.track().start;
        gtsam::Pose2 previous_correction = slam.get_correction();
        bool first = true;

        Frame frame;
        while (sim.step(frame, laps))
        {
            const gtsam::Vector3 sigmas = sim.odom_sigmas(last_keyframe_raw);
            slam.observe(frame.raw_odom, frame.observations, sigmas);

            if (first || slam.wants_keyframe())
            {
                const int before = slam.get_landmark_count();
                slam.add_keyframe(sigmas);
                last_keyframe_raw = frame.raw_odom;
                first = false;
                ++result.keyframes;
                result.promotions += slam.get_landmark_count() - before;
                result.update_ms.push_back(slam.get_last_update_elapsed_ms());
                result.reeliminated.push_back(slam.get_variables_reeliminated());

                const gtsam::Pose2 now = slam.get_correction();
                const gtsam::Pose2 jump = previous_correction.between(now);
                result.correction_jump_xy.push_back(jump.translation().norm());
                result.correction_jump_theta.push_back(std::abs(jump.theta()));
                previous_correction = now;
            }
            if (!slam.healthy())
                break;
        }

        result.map = slam.map_snapshot();
        result.healthy = slam.healthy();
        return result;
    }

    /// Cones of a given type in the finished map, for the assertions that care.
    inline std::vector<gtsam::Point2> of_type(const std::vector<Landmark> &map,
                                              const std::string &type)
    {
        std::vector<gtsam::Point2> out;
        for (const Landmark &l : map)
            if (l.type == type)
                out.emplace_back(l.x, l.y);
        return out;
    }

    /// Distance from `p` to the nearest cone in `map`, whatever its type.
    inline double nearest(const std::vector<Landmark> &map, const gtsam::Point2 &p)
    {
        double best = std::numeric_limits<double>::infinity();
        for (const Landmark &l : map)
            best = std::min(best, (gtsam::Point2(l.x, l.y) - p).norm());
        return best;
    }

    /// The rigid transform that best carries the map onto the truth layout, and
    /// the map under it.
    ///
    /// Two different things get called "the map is wrong" and they need different
    /// answers, which is why this exists. A map can be internally perfect -- every
    /// cone in the right place relative to every other -- and still sit rotated in
    /// the frame, because a graph anchored only at the first pose is accurate up
    /// to a gauge, and the gauge is fixed by odometry that drifts. That is not the
    /// same defect as a map with two landmarks per cone, and it does not have the
    /// same fix: the first is what an absolute reference is for, the second is an
    /// association failure. Measured together, they are indistinguishable.
    ///
    /// Nearest-neighbour ICP from the identity, which is enough here because the
    /// map starts within a few metres of the truth and both are the same layout.
    struct Alignment
    {
        std::vector<gtsam::Point2> aligned; ///< the map, carried onto the truth
        double rotation = 0.0;              ///< radians
        double translation = 0.0;           ///< metres
    };

    inline Alignment align_to(const std::vector<Landmark> &map,
                              const std::vector<Cone> &truth)
    {
        Alignment out;
        for (const Landmark &l : map)
            out.aligned.emplace_back(l.x, l.y);
        if (out.aligned.empty() || truth.empty())
            return out;

        const std::vector<gtsam::Point2> original = out.aligned;
        for (int iteration = 0; iteration < 60; ++iteration)
        {
            // Pair each landmark with the truth cone it is nearest to, then take
            // the rotation and translation that best explain all those pairs.
            gtsam::Point2 centre_map(0, 0), centre_truth(0, 0);
            std::vector<gtsam::Point2> paired;
            paired.reserve(out.aligned.size());
            for (const gtsam::Point2 &p : out.aligned)
            {
                double best = std::numeric_limits<double>::infinity();
                gtsam::Point2 match(0, 0);
                for (const Cone &c : truth)
                {
                    const double d = (c.position - p).norm();
                    if (d < best) { best = d; match = c.position; }
                }
                paired.push_back(match);
                centre_map = centre_map + p;
                centre_truth = centre_truth + match;
            }
            const double n = static_cast<double>(out.aligned.size());
            centre_map = centre_map / n;
            centre_truth = centre_truth / n;

            // The 2x2 correlation, whose polar factor is the rotation.
            double sxx = 0.0, sxy = 0.0, syx = 0.0, syy = 0.0;
            for (size_t i = 0; i < out.aligned.size(); ++i)
            {
                const gtsam::Point2 a = out.aligned[i] - centre_map;
                const gtsam::Point2 b = paired[i] - centre_truth;
                sxx += a.x() * b.x(); sxy += a.x() * b.y();
                syx += a.y() * b.x(); syy += a.y() * b.y();
            }
            const double theta = std::atan2(sxy - syx, sxx + syy);
            const gtsam::Rot2 R = gtsam::Rot2::fromAngle(theta);
            for (gtsam::Point2 &p : out.aligned)
                p = R.rotate(p - centre_map) + centre_truth;
        }

        // Report the total transform, measured rather than accumulated.
        gtsam::Point2 c0(0, 0), c1(0, 0);
        for (size_t i = 0; i < original.size(); ++i)
        {
            c0 = c0 + original[i];
            c1 = c1 + out.aligned[i];
        }
        const double n = static_cast<double>(original.size());
        c0 = c0 / n;
        c1 = c1 / n;
        double sxx = 0.0, sxy = 0.0, syx = 0.0, syy = 0.0;
        for (size_t i = 0; i < original.size(); ++i)
        {
            const gtsam::Point2 a = original[i] - c0;
            const gtsam::Point2 b = out.aligned[i] - c1;
            sxx += a.x() * b.x(); sxy += a.x() * b.y();
            syx += a.y() * b.x(); syy += a.y() * b.y();
        }
        out.rotation = std::abs(std::atan2(sxy - syx, sxx + syy));
        out.translation = (c1 - c0).norm();
        return out;
    }

    /// How many landmarks claim a truth cone of their own, once the map has been
    /// aligned. Short of the map size means duplicates: two landmarks for one cone.
    inline int distinct_cones_claimed(const std::vector<gtsam::Point2> &aligned,
                                      const std::vector<Cone> &truth, double within)
    {
        std::vector<bool> claimed(truth.size(), false);
        for (const gtsam::Point2 &p : aligned)
        {
            double best = within;
            size_t which = truth.size();
            for (size_t i = 0; i < truth.size(); ++i)
            {
                const double d = (truth[i].position - p).norm();
                if (d < best) { best = d; which = i; }
            }
            if (which < truth.size())
                claimed[which] = true;
        }
        int count = 0;
        for (const bool c : claimed)
            count += c ? 1 : 0;
        return count;
    }

} // namespace mirena::test
