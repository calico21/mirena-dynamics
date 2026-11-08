#include "association.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mirena
{

    std::vector<int> hungarian(const std::vector<std::vector<double>> &cost)
    {
        const int n = static_cast<int>(cost.size());
        if (n == 0)
            return {};
        const int m = static_cast<int>(cost[0].size());
        if (m == 0)
            return std::vector<int>(static_cast<size_t>(n), -1);

        constexpr double kInf = std::numeric_limits<double>::infinity();

        // One-based potentials and column assignment; column 0 is the sentinel
        // each augmenting path starts from.
        std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
        std::vector<int> match(m + 1, 0), path(m + 1, 0);

        for (int row = 1; row <= n; ++row)
        {
            match[0] = row;
            int col = 0;
            std::vector<double> slack(m + 1, kInf);
            std::vector<bool> done(m + 1, false);

            do
            {
                done[col] = true;
                const int cur = match[col];
                double delta = kInf;
                int next = 0;

                for (int j = 1; j <= m; ++j)
                {
                    if (done[j])
                        continue;
                    const double reduced = cost[cur - 1][j - 1] - u[cur] - v[j];
                    if (reduced < slack[j])
                    {
                        slack[j] = reduced;
                        path[j] = col;
                    }
                    if (slack[j] < delta)
                    {
                        delta = slack[j];
                        next = j;
                    }
                }

                // Shift the potentials so the chosen edge becomes tight.
                for (int j = 0; j <= m; ++j)
                {
                    if (done[j])
                    {
                        u[match[j]] += delta;
                        v[j] -= delta;
                    }
                    else
                    {
                        slack[j] -= delta;
                    }
                }
                col = next;
            } while (match[col] != 0);

            // Walk the path back, flipping the matching along it.
            do
            {
                const int prev = path[col];
                match[col] = match[prev];
                col = prev;
            } while (col != 0);
        }

        std::vector<int> assignment(static_cast<size_t>(n), -1);
        for (int j = 1; j <= m; ++j)
            if (match[j] != 0)
                assignment[static_cast<size_t>(match[j] - 1)] = j - 1;
        return assignment;
    }

    gtsam::Matrix2 measurement_covariance(const std::array<double, 4> &reported,
                                          double fallback_sigma, double min_sigma)
    {
        gtsam::Matrix2 R;
        R << reported[0], reported[1], reported[2], reported[3];

        const bool usable = R.allFinite() && R(0, 0) > 0.0 && R(1, 1) > 0.0 &&
                            R.determinant() > 0.0;
        if (!usable)
            R = gtsam::Matrix2::Identity() * (fallback_sigma * fallback_sigma);

        // Symmetrise, then floor the diagonal. A round trip through a message
        // leaves R(0,1) and R(1,0) very slightly apart, and GTSAM will not accept
        // a noise model that is not symmetric.
        R = (0.5 * (R + R.transpose())).eval();
        const double floor = min_sigma * min_sigma;
        R(0, 0) = std::max(R(0, 0), floor);
        R(1, 1) = std::max(R(1, 1), floor);
        return R;
    }

    gtsam::Matrix2 innovation_covariance(const gtsam::Pose2 &pose,
                                         const gtsam::Point2 &target,
                                         const gtsam::Matrix2 &target_cov,
                                         const gtsam::Matrix3 &pose_cov,
                                         const gtsam::Matrix2 &measurement_cov)
    {
        const gtsam::Matrix2 Rt = pose.rotation().matrix().transpose();
        gtsam::Matrix2 J;
        J << 0.0, 1.0,
            -1.0, 0.0;

        gtsam::Matrix23 H;
        H.leftCols<2>() = -Rt;
        H.rightCols<1>() = Rt * J * (target - pose.translation());

        return H * pose_cov * H.transpose() + Rt * target_cov * Rt.transpose() +
               measurement_cov;
    }

    void enforce_joint_compatibility(const std::vector<Target> &targets,
                                     const std::vector<gtsam::Point2> &local,
                                     const std::vector<gtsam::Matrix2> &measurement_cov,
                                     std::vector<int> &association)
    {
        // How many standard deviations two lengths may differ by and still be
        // called the same length. Three, because this is a veto on the pattern as
        // a whole and the cost of a false veto -- dropping a good match, which
        // becomes a candidate and is reconsidered next frame -- is much lower than
        // the cost of letting a set of mutually contradictory matches into a graph
        // that has no robust kernel to discount them.
        constexpr double kSigmas = 3.0;

        std::vector<size_t> matched;
        for (size_t i = 0; i < association.size(); ++i)
            if (association[i] >= 0)
                matched.push_back(i);

        const size_t n = matched.size();
        if (n < 3)
            return;

        std::vector<int> agreements(n, 0);
        for (size_t a = 0; a < n; ++a)
        {
            for (size_t b = a + 1; b < n; ++b)
            {
                const size_t ia = matched[a], ib = matched[b];
                const Target &ta = targets[static_cast<size_t>(association[ia])];
                const Target &tb = targets[static_cast<size_t>(association[ib])];

                const double observed = (local[ia] - local[ib]).norm();
                const double mapped = (ta.position - tb.position).norm();

                // The trace bounds the variance of the separation in any
                // direction, so this errs towards accepting -- the safe way for a
                // veto to be wrong.
                const double variance = measurement_cov[ia].trace() + measurement_cov[ib].trace() +
                                        ta.covariance.trace() + tb.covariance.trace();
                if (std::abs(observed - mapped) < kSigmas * std::sqrt(std::max(variance, 1e-12)))
                {
                    ++agreements[a];
                    ++agreements[b];
                }
            }
        }

        // Keep what agrees with a majority of the rest. A greedy stand-in for the
        // largest mutually consistent set, which is a maximum-clique problem and
        // not worth solving exactly for a dozen cones.
        for (size_t a = 0; a < n; ++a)
            if (2 * agreements[a] < static_cast<int>(n) - 1)
                association[matched[a]] = -1;
    }

    std::vector<int> associate(const gtsam::Pose2 &pose,
                               const gtsam::Matrix3 &pose_cov,
                               const std::vector<Target> &targets,
                               const std::vector<gtsam::Point2> &local,
                               const std::vector<gtsam::Matrix2> &measurement_cov,
                               double mahalanobis_threshold,
                               double max_distance)
    {
        const size_t n = local.size();
        std::vector<int> association(n, -1);
        if (n == 0 || targets.empty())
            return association;

        const size_t m = targets.size();

        // n rows, m real columns plus one "no match" column per row priced at the
        // gate. The matrix is therefore always at least as wide as it is tall.
        std::vector<std::vector<double>> cost(n, std::vector<double>(m + n, kBlocked));

        for (size_t i = 0; i < n; ++i)
        {
            cost[i][m + i] = mahalanobis_threshold;

            for (size_t j = 0; j < m; ++j)
            {
                // Reject on plain distance first: the chi-square test costs a 2x2
                // inverse, this costs a subtraction. It is also a backstop against
                // a pose that has gone badly wrong reaching across the track.
                const gtsam::Vector2 innovation = local[i] - pose.transformTo(targets[j].position);
                if (innovation.norm() > max_distance)
                    continue;

                const gtsam::Matrix2 S = innovation_covariance(
                    pose, targets[j].position, targets[j].covariance, pose_cov,
                    measurement_cov[i]);

                const double det = S.determinant();
                if (!std::isfinite(det) || det <= 0.0)
                    continue;

                const double d2 = innovation.transpose() * S.inverse() * innovation;
                if (std::isfinite(d2) && d2 < mahalanobis_threshold)
                    cost[i][j] = d2;
            }
        }

        const std::vector<int> assignment = hungarian(cost);
        for (size_t i = 0; i < n; ++i)
        {
            const int j = assignment[i];
            if (j >= 0 && static_cast<size_t>(j) < m)
                association[i] = j;
        }

        // Individually plausible is not the same as jointly possible.
        enforce_joint_compatibility(targets, local, measurement_cov, association);
        return association;
    }

} // namespace mirena
