#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

#include <array>
#include <vector>

namespace mirena
{

    /// One thing an observation might be: a landmark already in the graph, or a
    /// candidate not yet in it. Association does not care which -- it is the same
    /// question either way, and answering it once is what stops the two tiers
    /// drifting apart into two different notions of "the same cone".
    struct Target
    {
        gtsam::Point2 position;    ///< map frame
        gtsam::Matrix2 covariance; ///< map frame
    };

    /// The innovation covariance for one observation against one target.
    ///
    ///   S = H_x P_pose H_x^T + R^T P_target R + R_meas
    ///   H_x = [ -R^T | R^T J (l - p) ],  J = [[0, 1], [-1, 0]]
    ///
    /// so the gate widens exactly where the estimate is genuinely uncertain. The
    /// heading column is not a refinement: at 10 m a yaw sigma of 0.05 rad is half
    /// a metre of lateral error, which dominates everything else in S, and dropping
    /// it leaves the gate far too tight at range.
    ///
    /// `pose_cov` is the *relative* pose covariance accumulated since the last
    /// keyframe, not the absolute marginal. At a keyframe the pose and the
    /// landmarks it observed were made jointly consistent by the solve, so their
    /// relative uncertainty is the target's own; only odometry grows in between.
    /// Using the absolute marginal here would double-count the drift that both the
    /// pose and the map share, and would cost a Bayes-tree back-substitution to
    /// obtain.
    gtsam::Matrix2 innovation_covariance(const gtsam::Pose2 &pose,
                                         const gtsam::Point2 &target,
                                         const gtsam::Matrix2 &target_cov,
                                         const gtsam::Matrix3 &pose_cov,
                                         const gtsam::Matrix2 &measurement_cov);

    /// Nearest-neighbour association, scored by Mahalanobis distance and solved as
    /// an optimal one-to-one assignment.
    ///
    /// The assignment is half the point. Scoring each observation independently
    /// lets two cones detected in one frame both claim a single target, which puts
    /// two contradictory measurements on one point and drags it to the midpoint of
    /// two real cones.
    ///
    /// Gating falls out of the assignment rather than being a separate pass: each
    /// observation gets its own "no match" column priced at the gate, so any
    /// pairing costing more than the gate loses to that column.
    ///
    /// @param local        observations in the car frame
    /// @return association[i] is the index into `targets` matched to
    ///         observations[i], or -1 for none.
    std::vector<int> associate(const gtsam::Pose2 &pose,
                               const gtsam::Matrix3 &pose_cov,
                               const std::vector<Target> &targets,
                               const std::vector<gtsam::Point2> &local,
                               const std::vector<gtsam::Matrix2> &measurement_cov,
                               double mahalanobis_threshold,
                               double max_distance);

    /// Drops matches that cannot all be true at once.
    ///
    /// The gate judges each observation on its own, and on its own each one can
    /// be perfectly plausible. Coming back round a lap with a metre of drift, a
    /// cone can sit closer to its neighbour's landmark than to its own, and every
    /// such match passes. What they cannot do is pass *together*: the cones the
    /// car is looking at are a rigid pattern, so the distance between any two
    /// observations must equal the distance between the two landmarks they claim.
    ///
    /// So each match votes on every other, and a match that disagrees with most
    /// of the others is dropped. Nothing here needs to know the transform between
    /// the two frames -- comparing lengths is invariant to it -- and nothing needs
    /// a threshold in metres: the tolerance is the uncertainty of the two lengths
    /// being compared.
    ///
    /// This is what stands between a graph with no robust kernel on its
    /// measurements and a map that quietly rotates itself at the first loop
    /// closure. Measured: without it, a run that mapped 90 of 128 cones on the
    /// first lap held 31 of them after the second.
    ///
    /// Fewer than three matches carry no information about each other and are
    /// left alone.
    void enforce_joint_compatibility(const std::vector<Target> &targets,
                                     const std::vector<gtsam::Point2> &local,
                                     const std::vector<gtsam::Matrix2> &measurement_cov,
                                     std::vector<int> &association);

    /// A usable 2x2 covariance from what perception reported, or the fallback when
    /// it reported nothing.
    ///
    /// Perception publishes zeros when it has no estimate, and a covariance of zero
    /// would mean a perfectly known cone -- the most dangerous thing this could
    /// believe -- so the check is not defensive padding.
    gtsam::Matrix2 measurement_covariance(const std::array<double, 4> &reported,
                                          double fallback_sigma, double min_sigma);

    /**
     * @brief Minimum-cost assignment of rows to columns (Kuhn-Munkres /
     *        Jonker-Volgenant): shortest augmenting path with potentials,
     *        O(n^2 m). Requires at least as many columns as rows.
     * @return the column chosen for each row, or -1 where a row was left unmatched.
     */
    std::vector<int> hungarian(const std::vector<std::vector<double>> &cost);

    /// Cost standing in for "impossible": finite, so the potentials stay
    /// well-behaved, but far above anything a gate would accept.
    constexpr double kBlocked = 1e9;

} // namespace mirena
