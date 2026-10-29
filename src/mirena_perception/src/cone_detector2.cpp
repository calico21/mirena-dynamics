#include "cone_detector2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <Eigen/Dense>

namespace mirena
{
    namespace
    {
        double elapsed_ms(std::chrono::steady_clock::time_point since)
        {
            const std::chrono::duration<double, std::milli> d = std::chrono::steady_clock::now() - since;
            return d.count();
        }

        inline int64_t cell_key(int i, int j) { return static_cast<int64_t>(i) * 1000003 + j; }
    }

    ConeDetector::ConeDetector(const ConeDetectorParams &params)
        : params_(params)
    {
    }

    // Filtro Voxel Grid O(N) ultrarrápido
    void ConeDetector::apply_voxel_grid_filter()
    {
        if (!params_.enable_voxel_grid || keep_.empty()) return;

        const float leaf = static_cast<float>(params_.voxel_leaf_size);
        if (leaf <= 0.001f) return;
        const float inv_leaf = 1.0f / leaf;

        auto hash_voxel = [](int x, int y, int z) -> uint64_t {
            return (static_cast<uint64_t>(x) * 73856093u) ^
                   (static_cast<uint64_t>(y) * 19349663u) ^
                   (static_cast<uint64_t>(z) * 83492791u);
        };

        std::unordered_map<uint64_t, int> voxel_map;
        voxel_map.reserve(keep_.size());

        std::vector<int> filtered_keep;
        filtered_keep.reserve(keep_.size());

        for (const int idx : keep_)
        {
            const int vx = static_cast<int>(std::floor(car_(idx, 0) * inv_leaf));
            const int vy = static_cast<int>(std::floor(car_(idx, 1) * inv_leaf));
            const int vz = static_cast<int>(std::floor(car_(idx, 2) * inv_leaf));

            const uint64_t key = hash_voxel(vx, vy, vz);
            if (voxel_map.find(key) == voxel_map.end())
            {
                voxel_map[key] = idx;
                filtered_keep.push_back(idx);
            }
        }

        keep_ = std::move(filtered_keep);
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

    bool ConeDetector::validate_shape_and_pca(const std::vector<Eigen::Vector3f> &pts, float top, float &out_extent) const
    {
        if (pts.empty()) return false;

        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
        float min_y = min_x, max_y = max_x;

        for (const auto &p : pts)
        {
            mean += p;
            min_x = std::min(min_x, p.x()); max_x = std::max(max_x, p.x());
            min_y = std::min(min_y, p.y()); max_y = std::max(max_y, p.y());
        }
        mean /= static_cast<float>(pts.size());

        float width = max_x - min_x;
        float depth = max_y - min_y;
        float footprint = std::max(width, depth);
        
        if (footprint > params_.max_footprint) return false;

        float slenderness = top / std::max(footprint, 0.05f);
        if (slenderness < params_.min_slenderness) return false;

        if (pts.size() >= 3)
        {
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto &p : pts)
            {
                Eigen::Vector3f diff = p - mean;
                cov += diff * diff.transpose();
            }
            cov /= static_cast<float>(pts.size());

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
            if (es.info() == Eigen::Success)
            {
                Eigen::Vector3f main_dir = es.eigenvectors().col(2).normalized();
                float cos_angle = std::abs(main_dir.dot(Eigen::Vector3f::UnitZ()));
                float angle_deg = std::acos(std::clamp(cos_angle, 0.0f, 1.0f)) * 180.0f / M_PI;

                if (angle_deg > params_.max_pca_angle_deg) return false;
            }
        }

        out_extent = 0.5f * footprint;
        return true;
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

        has_intensity_ = (cloud_lidar.cols() >= 4);

        auto t_step = std::chrono::steady_clock::now();

        car_.resize(cloud_lidar.rows(), cloud_lidar.cols());
        car_.leftCols<3>().noalias() = cloud_lidar.leftCols<3>() * car_from_lidar.linear().transpose();
        car_.leftCols<3>().rowwise() += car_from_lidar.translation().transpose();
        if (has_intensity_)
        {
            car_.col(3) = cloud_lidar.col(3);
        }
        double tf_transform_ms = elapsed_ms(t_step);

        const float ego_x = static_cast<float>(params_.ego_half_length);
        const float ego_y = static_cast<float>(params_.ego_half_width);
        const float far = static_cast<float>(params_.max_range) + 8.0f;
        
        keep_.clear();
        keep_.reserve(static_cast<size_t>(car_.rows()));
        
        t_step = std::chrono::steady_clock::now();
        // 1. Recorte Geométrico Espacial
        for (Eigen::Index r = 0; r < car_.rows(); ++r)
        {
            const float x = car_(r, 0), y = car_(r, 1);
            if (std::abs(y) > far || x < -8.0f || x > far) continue;
            if (std::abs(x) <= ego_x && std::abs(y) <= ego_y) continue;
            keep_.push_back(static_cast<int>(r));
        }
        double crop_ms = elapsed_ms(t_step);
        size_t points_after_crop = keep_.size();

        stage_.level_ms = tf_transform_ms + crop_ms;

        if (keep_.size() < 50)
        {
            total_ms_ = elapsed_ms(started);
            return detections_;
        }

        // 2. Ground Coarse Grid con Filtro Anti-Ruido
        auto mark = std::chrono::steady_clock::now();
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
        
        for (const int r : keep_) {
            if (car_(r, 2) < -1.5f) continue;
            zmin[ccell_of(r)] = std::min(zmin[ccell_of(r)], car_(r, 2));
        }

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

        // 3. Separación Terreno vs Puntos Elevados
        std::vector<int> ground_pts;
        std::vector<int> non_ground_pts;
        ground_pts.reserve(keep_.size());
        non_ground_pts.reserve(keep_.size() / 10);

        for (const int r : keep_)
        {
            const float h = car_(r, 2) - ground[ccell_of(r)];
            if (h >= params_.candidate_height)
            {
                non_ground_pts.push_back(r);
            }
            else
            {
                ground_pts.push_back(r);
            }
        }

        // 4. Filtro VOXEL GRID O(N) sobre puntos elevados
        double voxel_ms = 0.0;
        size_t non_ground_before_voxel = non_ground_pts.size();

        if (params_.enable_voxel_grid && !non_ground_pts.empty())
        {
            keep_ = std::move(non_ground_pts);

            auto t_voxel = std::chrono::steady_clock::now();
            apply_voxel_grid_filter();
            voxel_ms = elapsed_ms(t_voxel);

            non_ground_pts = std::move(keep_);
        }

        // Recomponer keep_ para fit_ground()
        keep_.clear();
        keep_.reserve(ground_pts.size() + non_ground_pts.size());
        keep_.insert(keep_.end(), ground_pts.begin(), ground_pts.end());
        keep_.insert(keep_.end(), non_ground_pts.begin(), non_ground_pts.end());

        mark = std::chrono::steady_clock::now();

        // 5. Índice espacial secundario
        auto t_idx2 = std::chrono::steady_clock::now();
        build_index();
        double build_idx_2_ms = elapsed_ms(t_idx2);

        // 6. Búsqueda de Picos Candidatos
        const float pcell = static_cast<float>(params_.peak_cell);
        std::unordered_map<int64_t, std::pair<int, float>> tallest;
        tallest.reserve(non_ground_pts.size() / 2 + 16);
        standing_.reserve(non_ground_pts.size());

        for (const int r : non_ground_pts)
        {
            const float h = car_(r, 2) - ground[ccell_of(r)];
            if (h < params_.candidate_height) continue;
            
            if (has_intensity_ && params_.use_intensity)
            {
                if (car_(r, 3) < params_.min_intensity_peak && h < 0.20f) continue;
            }

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
        std::sort(peaks.begin(), peaks.end(),
                  [](const auto &a, const auto &b) { return a.second > b.second; });
        candidates_ = static_cast<int>(peaks.size());
        stage_.peaks_ms = elapsed_ms(mark);
        mark = std::chrono::steady_clock::now();

        // 7. Bucle de Validación
        const float core2 = static_cast<float>(params_.core_radius * params_.core_radius);
        const float ring_in2 = 0.45f * 0.45f, ring_out2 = 1.10f * 1.10f;
        std::vector<int> near;
        std::vector<Eigen::Vector3f> core_points;

        double total_fit_ground_ms = 0.0;
        double total_pca_ms = 0.0;

        for (const auto &peak : peaks)
        {
            const float cx = car_(peak.first, 0), cy = car_(peak.first, 1);
            const float range = std::hypot(cx - sensor_xy_.x(), cy - sensor_xy_.y());
            if (range > params_.max_range || range < 0.8f) continue;

            auto t_fg = std::chrono::steady_clock::now();
            const Plane plane = fit_ground(cx, cy);
            total_fit_ground_ms += elapsed_ms(t_fg);

            if (!plane.valid || plane.roughness > params_.max_roughness) continue;

            gather(cx, cy, 1.10f, near);
            float top = std::numeric_limits<float>::lowest();
            int n_core = 0, n_ring = 0;
            core_points.clear();

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
                        core_points.emplace_back(car_(r, 0), car_(r, 1), car_(r, 2));
                    }
                }
                else if (d2 >= ring_in2 && d2 < ring_out2 && h > params_.stand_height)
                {
                    ++n_ring;
                }
            }

            int min_pts_adaptive = (range > 12.0f) ? 2 : params_.min_points;

            if (n_core < min_pts_adaptive) continue;
            if (top < params_.min_top || top > params_.max_top) continue;
            if (n_ring > params_.max_ring_ratio * n_core) continue;

            float half_extent = 0.0f;
            auto t_pca = std::chrono::steady_clock::now();
            bool valid_pca = validate_shape_and_pca(core_points, top, half_extent);
            total_pca_ms += elapsed_ms(t_pca);

            if (!valid_pca) continue;

            half_extent = std::max(half_extent, static_cast<float>(params_.bearing_sigma) * range);

            Eigen::Vector3f centre = Eigen::Vector3f::Zero();
            for (const auto &p : core_points) centre += p;
            centre /= static_cast<float>(core_points.size());

            bool duplicate = false;
            for (const auto &d : detections_)
            {
                if ((d.position.head<2>() - centre.head<2>()).norm() < half_extent + d.half_extent)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            detections_.push_back({centre, n_core, top >= params_.big_cone_min_height,
                                   position_covariance(centre.head<2>(), n_core),
                                   half_extent});

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
        bool debug_log = false;
        {
            if (debug_log)
        {
            // -- IMPRESIÓN DETALLADA DE TIEMPOS DE EJECUCIÓN POR FUNCIÓN --
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "--- [ConeDetector] Pts Entrada: " << cloud_lidar.rows() 
                  << " | Crop: " << points_after_crop 
                  << " | Non-Ground: " << non_ground_before_voxel
                  << " | Post Voxel Grid: " << non_ground_pts.size() << " ---" << std::endl;
        std::cout << "  * Transform LiDAR->Car: " << tf_transform_ms << " ms" << std::endl;
        std::cout << "  * Crop Geometrico:      " << crop_ms << " ms" << std::endl;
        std::cout << "  * Ground Coarse Grid:   " << stage_.coarse_ms << " ms" << std::endl;
        if (params_.enable_voxel_grid) {
            std::cout << "  * Filtro Voxel Grid O(N): " << voxel_ms << " ms" << std::endl;
        }
        std::cout << "  * Build Spatial Index:  " << build_idx_2_ms << " ms" << std::endl;
        std::cout << "  * Peak Candidate Search:" << stage_.peaks_ms << " ms (" << candidates_ << " candidatos)" << std::endl;
        std::cout << "  * Validation Loop:      " << stage_.accept_ms << " ms" << std::endl;
        std::cout << "      - fit_ground() Total:" << total_fit_ground_ms << " ms" << std::endl;
        std::cout << "      - validate_pca() Tot:" << total_pca_ms << " ms" << std::endl;
        std::cout << "  [DETECTOR TOTAL]:       " << total_ms_ << " ms (" << detections_.size() << " conos detectados)" << std::endl;
    }

        }
        

        return detections_;
    }

} // namespace mirena