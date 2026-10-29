#include "cone_detector.hpp"

#include <algorithm> // proporciona funciones como std::min, std::max o std::sort.
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace mirena
{
    namespace //namespace anónimo: funciones internas y privadas que se usan SOLO en este archivo
    {
        double elapsed_ms(std::chrono::steady_clock::time_point since)
        {
            const std::chrono::duration<double, std::milli> d =
                std::chrono::steady_clock::now() - since;
            return d.count();
        }

        /// Key for a cell on an integer grid, packed into one integer so the
        /// neighbour lookups below are hash lookups rather than searches.
        inline int64_t cell_key(int i, int j) { return static_cast<int64_t>(i) * 1000003 + j; }
    } // namespace anónimo

    ConeDetector::ConeDetector(const ConeDetectorParams &params)
        : params_(params)
    {
    }

    void ConeDetector::build_index()
    {
        index_cell_ = 0.5f;
        float lo_x = std::numeric_limits<float>::max(), lo_y = lo_x;
        float hi_x = std::numeric_limits<float>::lowest(), hi_y = hi_x;
        for (const int r : keep_)
        {
            lo_x = std::min(lo_x, car_(r, 0)); hi_x = std::max(hi_x, car_(r, 0));
            lo_y = std::min(lo_y, car_(r, 1)); hi_y = std::max(hi_y, car_(r, 1));
        }
        index_x0_ = lo_x; index_y0_ = lo_y;
        index_nx_ = static_cast<int>((hi_x - lo_x) / index_cell_) + 1;
        index_ny_ = static_cast<int>((hi_y - lo_y) / index_cell_) + 1;

        // Counting sort into buckets: one pass to count, one to place.
        index_start_.assign(static_cast<size_t>(index_nx_) * index_ny_ + 1, 0);
        const auto bucket = [&](int r) {
            const int ix = std::clamp(static_cast<int>((car_(r, 0) - index_x0_) / index_cell_), 0, index_nx_ - 1);
            const int iy = std::clamp(static_cast<int>((car_(r, 1) - index_y0_) / index_cell_), 0, index_ny_ - 1);
            return static_cast<size_t>(ix) * index_ny_ + iy;
        };
        for (const int r : keep_) ++index_start_[bucket(r) + 1];
        for (size_t i = 1; i < index_start_.size(); ++i) index_start_[i] += index_start_[i - 1];
        index_items_.resize(keep_.size());
        std::vector<int> cursor(index_start_.begin(), index_start_.end() - 1);
        for (const int r : keep_) index_items_[cursor[bucket(r)]++] = r;
    }

    void ConeDetector::gather(float cx, float cy, float radius, std::vector<int> &out) const
    {
        out.clear();
        const int i0 = std::clamp(static_cast<int>((cx - radius - index_x0_) / index_cell_), 0, index_nx_ - 1);
        const int i1 = std::clamp(static_cast<int>((cx + radius - index_x0_) / index_cell_), 0, index_nx_ - 1);
        const int j0 = std::clamp(static_cast<int>((cy - radius - index_y0_) / index_cell_), 0, index_ny_ - 1);
        const int j1 = std::clamp(static_cast<int>((cy + radius - index_y0_) / index_cell_), 0, index_ny_ - 1);
        const float r2 = radius * radius;
        for (int i = i0; i <= i1; ++i)
        {
            for (int j = j0; j <= j1; ++j)
            {
                const size_t b = static_cast<size_t>(i) * index_ny_ + j;
                for (int k = index_start_[b]; k < index_start_[b + 1]; ++k)
                {
                    const int r = index_items_[static_cast<size_t>(k)];
                    const float dx = car_(r, 0) - cx, dy = car_(r, 1) - cy;
                    if (dx * dx + dy * dy <= r2) out.push_back(r);
                }
            }
        }
    }

    ConeDetector::Plane ConeDetector::fit_ground(float cx, float cy)
    {
        Plane plane;
        gather(cx, cy, static_cast<float>(params_.ring_outer), scratch_ring_);
        if (static_cast<int>(scratch_ring_.size()) < params_.ring_min_points) return plane;

        // One lowest point per ring cell, so the fit follows the surface rather
        // than the sampling density, which is ten times higher near the sensor.
        const float inner2 = static_cast<float>(params_.ring_inner * params_.ring_inner);
        const float cell = static_cast<float>(params_.ring_cell);
        std::unordered_map<int64_t, int> lowest;
        lowest.reserve(scratch_ring_.size() / 4 + 8);
        for (const int r : scratch_ring_)
        {
            const float dx = car_(r, 0) - cx, dy = car_(r, 1) - cy;
            if (dx * dx + dy * dy < inner2) continue;
            const int64_t k = cell_key(static_cast<int>(std::lround(dx / cell)),
                                       static_cast<int>(std::lround(dy / cell)));
            auto it = lowest.find(k);
            if (it == lowest.end()) lowest.emplace(k, r);
            else if (car_(r, 2) < car_(it->second, 2)) it->second = r;
        }
        if (static_cast<int>(lowest.size()) < params_.ring_min_cells) return plane;

        std::vector<int> low;
        low.reserve(lowest.size());
        for (const auto &kv : lowest) low.push_back(kv.second);

        // Least squares with two reweighting passes: a cone leaning into the
        // ring, or a kerb clipped by it, is an outlier and has to leave.
        std::vector<uint8_t> use(low.size(), 1);
        float a = 0, b = 0, c = 0, rough = 0;
        for (int pass = 0; pass < 3; ++pass)
        {
            double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sn = 0, sxz = 0, syz = 0, sz = 0;
            for (size_t i = 0; i < low.size(); ++i)
            {
                if (!use[i]) continue;
                const float dx = car_(low[i], 0) - cx, dy = car_(low[i], 1) - cy, z = car_(low[i], 2);
                sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
                sx += dx; sy += dy; sn += 1.0;
                sxz += dx * z; syz += dy * z; sz += z;
            }
            if (sn < params_.ring_min_cells) return plane;
            Eigen::Matrix3d A;
            A << sxx, sxy, sx, sxy, syy, sy, sx, sy, sn;
            const Eigen::Vector3d rhs(sxz, syz, sz);
            const Eigen::Vector3d sol = A.ldlt().solve(rhs);
            if (!sol.allFinite()) return plane;
            a = static_cast<float>(sol(0)); b = static_cast<float>(sol(1)); c = static_cast<float>(sol(2));

            double ss = 0; int n = 0;
            for (size_t i = 0; i < low.size(); ++i)
            {
                if (!use[i]) continue;
                const float dx = car_(low[i], 0) - cx, dy = car_(low[i], 1) - cy;
                const float res = car_(low[i], 2) - (a * dx + b * dy + c);
                ss += static_cast<double>(res) * res; ++n;
            }
            rough = n > 0 ? static_cast<float>(std::sqrt(ss / n)) : 0.0f;
            if (pass == 2) break;

            const float gate = std::max(0.06f, 2.5f * rough);
            int kept = 0;
            for (size_t i = 0; i < low.size(); ++i)
            {
                const float dx = car_(low[i], 0) - cx, dy = car_(low[i], 1) - cy;
                use[i] = std::abs(car_(low[i], 2) - (a * dx + b * dy + c)) < gate;
                kept += use[i];
            }
            if (kept < params_.ring_min_cells) break;
        }
        plane.a = a; plane.b = b; plane.c = c; plane.roughness = rough; plane.valid = true;
        return plane;
    }

    std::array<double, 4> ConeDetector::position_covariance(
        const Eigen::Vector2f &centroid, int num_points) const
    {
        const double x = centroid.x() - sensor_xy_.x();
        const double y = centroid.y() - sensor_xy_.y();
        const double range = std::hypot(x, y);
        const double spread = params_.cluster_spread / std::sqrt(static_cast<double>(std::max(num_points, 1)));
        const double var_along = params_.range_sigma * params_.range_sigma + spread * spread;
        const double var_across = std::pow(params_.bearing_sigma * range, 2.0) + spread * spread;
        if (range < 1e-3) return {var_along, 0.0, 0.0, var_along};
        const double cs = x / range, sn = y / range;
        const double xx = cs * cs * var_along + sn * sn * var_across;
        const double yy = sn * sn * var_along + cs * cs * var_across;
        const double xy = cs * sn * (var_along - var_across);
        return {xx, xy, xy, yy};
    }

    const std::vector<Detection> &ConeDetector::detect(const Eigen::MatrixXf &cloud_lidar,
                                                           const Eigen::Isometry3f &car_from_lidar)
    {
        const auto started = std::chrono::steady_clock::now();
        detections_.clear();
        standing_.clear();
        labels_.clear();
        candidates_ = 0;
        sensor_xy_ = car_from_lidar.translation().head<2>();

        if (cloud_lidar.rows() == 0)
        {
            total_ms_ = elapsed_ms(started);
            return detections_;
        }

        // 1. Into the car frame. Rotation and translation together: nothing here
        //    needs the levelled-but-not-translated intermediate the other
        //    detector hands to Patchwork++.
        car_.noalias() = cloud_lidar.leftCols<3>() * car_from_lidar.linear().transpose();
        car_.rowwise() += car_from_lidar.translation().transpose();
        stage_.level_ms = elapsed_ms(started);
        auto mark = std::chrono::steady_clock::now();

        const float ego_x = static_cast<float>(params_.ego_half_length);
        const float ego_y = static_cast<float>(params_.ego_half_width);
        const float far = static_cast<float>(params_.max_range) + 8.0f;
        keep_.clear();
        keep_.reserve(static_cast<size_t>(car_.rows()));
        for (Eigen::Index r = 0; r < car_.rows(); ++r)
        {
            const float x = car_(r, 0), y = car_(r, 1);
            if (std::abs(y) > far || x < -8.0f || x > far) continue;
            if (std::abs(x) <= ego_x && std::abs(y) <= ego_y) continue;
            keep_.push_back(static_cast<int>(r));
        }
        if (keep_.size() < 200)
        {
            total_ms_ = elapsed_ms(started);
            return detections_;
        }

        // 2. The coarse ground: per-cell lowest point, then the lowest of those
        //    within coarse_ring. Only used to decide what is worth a closer look.
        const float ccell = static_cast<float>(params_.coarse_cell);
        float lo_x = std::numeric_limits<float>::max(), lo_y = lo_x;
        float hi_x = std::numeric_limits<float>::lowest(), hi_y = hi_x;
        for (const int r : keep_)
        {
            lo_x = std::min(lo_x, car_(r, 0)); hi_x = std::max(hi_x, car_(r, 0));
            lo_y = std::min(lo_y, car_(r, 1)); hi_y = std::max(hi_y, car_(r, 1));
        }
        const int cnx = static_cast<int>((hi_x - lo_x) / ccell) + 1;
        const int cny = static_cast<int>((hi_y - lo_y) / ccell) + 1;
        std::vector<float> zmin(static_cast<size_t>(cnx) * cny, std::numeric_limits<float>::max());
        const auto ccell_of = [&](int r) {
            const int ix = std::clamp(static_cast<int>((car_(r, 0) - lo_x) / ccell), 0, cnx - 1);
            const int iy = std::clamp(static_cast<int>((car_(r, 1) - lo_y) / ccell), 0, cny - 1);
            return static_cast<size_t>(ix) * cny + iy;
        };
        for (const int r : keep_) zmin[ccell_of(r)] = std::min(zmin[ccell_of(r)], car_(r, 2));

        // Separable minimum filter, so the radius costs two passes and not r^2.
        const int k = std::max(1, static_cast<int>(std::lround(params_.coarse_ring / ccell)));
        std::vector<float> tmp(zmin.size(), std::numeric_limits<float>::max());
        for (int i = 0; i < cnx; ++i)
            for (int j = 0; j < cny; ++j)
            {
                float m = std::numeric_limits<float>::max();
                for (int d = -k; d <= k; ++d)
                {
                    const int jj = j + d;
                    if (jj >= 0 && jj < cny) m = std::min(m, zmin[static_cast<size_t>(i) * cny + jj]);
                }
                tmp[static_cast<size_t>(i) * cny + j] = m;
            }
        std::vector<float> ground(zmin.size(), std::numeric_limits<float>::max());
        for (int i = 0; i < cnx; ++i)
            for (int j = 0; j < cny; ++j)
            {
                float m = std::numeric_limits<float>::max();
                for (int d = -k; d <= k; ++d)
                {
                    const int ii = i + d;
                    if (ii >= 0 && ii < cnx) m = std::min(m, tmp[static_cast<size_t>(ii) * cny + j]);
                }
                ground[static_cast<size_t>(i) * cny + j] = m;
            }
        stage_.coarse_ms = elapsed_ms(mark);
        mark = std::chrono::steady_clock::now();

        build_index();

        // 3. Candidates: the tallest point of each peak_cell, where that cell
        //    beats all eight of its neighbours.
        const float pcell = static_cast<float>(params_.peak_cell);
        std::unordered_map<int64_t, std::pair<int, float>> tallest;  // cell -> (row, height)
        tallest.reserve(keep_.size() / 8 + 16);
        standing_.reserve(keep_.size() / 4);
        for (const int r : keep_)
        {
            const float h = car_(r, 2) - ground[ccell_of(r)];
            if (h < params_.candidate_height) continue;
            standing_.emplace_back(car_(r, 0), car_(r, 1), car_(r, 2));
            const int64_t key = cell_key(static_cast<int>(std::lround(car_(r, 0) / pcell)),
                                         static_cast<int>(std::lround(car_(r, 1) / pcell)));
            auto it = tallest.find(key);
            if (it == tallest.end()) tallest.emplace(key, std::make_pair(r, h));
            else if (h > it->second.second) it->second = {r, h};
        }
        standing_.width = standing_.size();
        standing_.height = 1;
        standing_.is_dense = true;
        labels_.assign(standing_.size(), -1);

        std::vector<std::pair<int, float>> peaks;
        peaks.reserve(tallest.size() / 4 + 8);
        for (const auto &kv : tallest)
        {
            if (kv.second.second < params_.peak_height) continue;
            const int i = static_cast<int>(std::lround(car_(kv.second.first, 0) / pcell));
            const int j = static_cast<int>(std::lround(car_(kv.second.first, 1) / pcell));
            bool top = true;
            for (int a = -1; a <= 1 && top; ++a)
                for (int b = -1; b <= 1; ++b)
                {
                    if (a == 0 && b == 0) continue;
                    auto n = tallest.find(cell_key(i + a, j + b));
                    if (n != tallest.end() && n->second.second > kv.second.second) { top = false; break; }
                }
            if (top) peaks.push_back({kv.second.first, kv.second.second});
        }
        // Tallest first, so when two peaks land on one cone the better one wins
        // the 0.7 m exclusion below.
        std::sort(peaks.begin(), peaks.end(),
                  [](const auto &a, const auto &b) { return a.second > b.second; });
        candidates_ = static_cast<int>(peaks.size());
        stage_.peaks_ms = elapsed_ms(mark);
        mark = std::chrono::steady_clock::now();

        // 4. One honest look at each candidate.
        const float core2 = static_cast<float>(params_.core_radius * params_.core_radius);
        const float ring_in2 = 0.45f * 0.45f, ring_out2 = 1.10f * 1.10f;
        std::vector<int> near;
        for (const auto &peak : peaks)
        {
            const float cx = car_(peak.first, 0), cy = car_(peak.first, 1);
            const float range = std::hypot(cx - sensor_xy_.x(), cy - sensor_xy_.y());
            if (range > params_.max_range || range < 1.0f) continue;

            // Whether this peak is a second look at a cone already found is
            // decided further down, once its blob has been measured. It used to
            // be decided here, by a fixed 0.7 m exclusion, and a fixed distance
            // cannot tell a second peak on one cone from the second cone of a
            // start-line pair -- the big orange cones stand well inside 0.7 m of
            // each other, so one of every pair was deleted before the shape
            // tests ever ran on it.

            const Plane plane = fit_ground(cx, cy);
            if (!plane.valid || plane.roughness > params_.max_roughness) continue;

            gather(cx, cy, 1.10f, near);
            float top = std::numeric_limits<float>::lowest();
            int n_core = 0, n_ring = 0;
            float sx = 0, sy = 0, sz = 0;
            float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
            float min_y = min_x, max_y = max_x;
            for (const int r : near)
            {
                const float dx = car_(r, 0) - cx, dy = car_(r, 1) - cy;
                const float d2 = dx * dx + dy * dy;
                const float h = car_(r, 2) - plane.at(car_(r, 0), car_(r, 1), cx, cy);
                if (d2 < core2)
                {
                    top = std::max(top, h);
                    if (h > params_.stand_height)
                    {
                        ++n_core;
                        sx += car_(r, 0); sy += car_(r, 1); sz += car_(r, 2);
                        min_x = std::min(min_x, car_(r, 0)); max_x = std::max(max_x, car_(r, 0));
                        min_y = std::min(min_y, car_(r, 1)); max_y = std::max(max_y, car_(r, 1));
                    }
                }
                else if (d2 >= ring_in2 && d2 < ring_out2 && h > params_.stand_height)
                {
                    ++n_ring;
                }
            }
            if (n_core < params_.min_points) continue;
            if (top < params_.min_top || top > params_.max_top) continue;
            if (max_x - min_x > params_.max_footprint || max_y - min_y > params_.max_footprint) continue;
            if (n_ring > params_.max_ring_ratio * n_core) continue;

            const Eigen::Vector3f centre(sx / n_core, sy / n_core, sz / n_core);

            // How wide this blob actually is, floored by what the sensor can
            // separate across the beam at this range. Two objects closer than
            // that are one object as far as this data can honestly say, and the
            // floor is what stops a distant cone caught in two points from
            // claiming an extent of nearly nothing and splitting in two.
            const float half_extent = std::max(
                0.5f * std::max(max_x - min_x, max_y - min_y),
                static_cast<float>(params_.bearing_sigma) * range);

            // Same object as one already found? The test is whether the two
            // blobs overlap, each measured on its own points -- not whether
            // their centres fall inside a distance chosen in advance. Two peaks
            // on one cone sit within a cone's width of each other and merge; two
            // cones set down a start line apart do not, however close that is.
            bool duplicate = false;
            for (const auto &d : detections_)
            {
                if ((d.position.head<2>() - centre.head<2>()).norm() <
                    half_extent + d.half_extent)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            detections_.push_back({centre, n_core, top >= params_.big_cone_min_height,
                                   position_covariance(centre.head<2>(), n_core),
                                   half_extent});

            // Label the debug cloud, so the node's clustered-points topic shows
            // exactly which returns each cone was built from.
            const int label = static_cast<int>(detections_.size()) - 1;
            for (size_t i = 0; i < standing_.size(); ++i)
            {
                const float dx = standing_[i].x - cx, dy = standing_[i].y - cy;
                if (dx * dx + dy * dy < core2 &&
                    standing_[i].z - plane.at(standing_[i].x, standing_[i].y, cx, cy) > params_.stand_height)
                {
                    labels_[i] = label;
                }
            }
        }

        stage_.accept_ms = elapsed_ms(mark);
        total_ms_ = elapsed_ms(started);
        return detections_;
    }

} // namespace mirena
