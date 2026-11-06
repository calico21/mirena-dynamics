#include "cone_detector3.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <omp.h> 

#include <Eigen/Dense>

namespace mirena
{
    ConeDetector::ConeDetector(const ConeDetectorParams &params) : _params(params),
          _grid_max_z(GRID_DIM * GRID_DIM, -std::numeric_limits<float>::max()),
          _grid_count(GRID_DIM * GRID_DIM, 0)
    {
    }

    // -------------------------------------------------------------------------
    // Stage 1 – O(N) global crop + elevation tagging
    // -------------------------------------------------------------------------
    void ConeDetector::crop_and_elevate(const Eigen::MatrixXf &cloud,
                                        const Eigen::Isometry3f &sensor_from_car,
                                        std::vector<Eigen::Vector3f> &car_out,
                                        std::vector<Eigen::Vector3f> &elevated_out)
    {
        car_out.clear();
        elevated_out.clear();
        const int n = static_cast<int>(cloud.rows());
        car_out.reserve(static_cast<size_t>(n) / 2);
        elevated_out.reserve(static_cast<size_t>(n) / 8);

        if (_debug)
        {
            _stats.total_points += n;
            // The tallest thing seen, in both frames. If max_h_above is a metre
            // while max_z_car is near zero, the ground is being read as elevated
            // and the ground reference is wrong, not the threshold.
            _stats.max_h_above = -std::numeric_limits<float>::max();
            _stats.max_z_car = -std::numeric_limits<float>::max();
        }

        const auto &R = sensor_from_car.rotation();
        const auto &t = sensor_from_car.translation();
        const float slope = _params.slope_factor;
        const float cand_h = _params.candidate_height;
        const float max_r2 = _params.max_range * _params.max_range;
        const float ego_l = _params.ego_half_length;
        const float ego_w = _params.ego_half_width;
        const float z_lo = _params.z_min;
        const float z_hi = _params.z_max;

        for (int i = 0; i < n; ++i)
        {
            Eigen::Vector3f p = R * cloud.row(i).transpose() + t;
            const float x = p.x(), y = p.y(), z = p.z();

            // Distance cutoff
            if (x * x + y * y > max_r2) { if (_debug) _stats.rejected_range++; continue; }

            // Ego bounds: reject the box itself, keep everything outside it. Testing
            // `> ego` here instead would keep only the car footprint and discard
            // the entire scene.
            if (std::abs(x) <= ego_l && std::abs(y) <= ego_w) { if (_debug) _stats.rejected_ego++; continue; }

            // Z limits
            if (z < z_lo || z > z_hi) { if (_debug) _stats.rejected_z++; continue; }

            // Ground approximation. p is already in car/cog, where the ground
            // sits at z ~ 0 -- adding t.z() here would re-apply the LiDAR mount
            // height and push the reference a metre above the whole cloud.
            const float r = std::sqrt(x * x + y * y);
            const float z_ground = slope * r;
            const float h_above = z - z_ground;

            if (_debug)
            {
                _stats.max_h_above = std::max(_stats.max_h_above, h_above);
                _stats.max_z_car = std::max(_stats.max_z_car, z);
            }

            // Kept whatever happens: stage 3 measures heights against a locally fitted
            // plane, so it needs the ground points too, not just the elevated
            // ones.
            car_out.push_back(p);

            if (h_above > cand_h)
            {
                elevated_out.push_back(p);
            }
            else
            {
                if (_debug) _stats.rejected_ground++;
            }
        }

        if (_debug) _stats.elevated += elevated_out.size();
    }

    // -------------------------------------------------------------------------
    // Stage 2 – Spatial hashing & peak extraction
    // -------------------------------------------------------------------------
    void ConeDetector::extract_peaks(const std::vector<Eigen::Vector3f> &elevated,
                                     std::vector<Peak> &peaks_out)
    {
        peaks_out.clear();
        std::fill(_grid_max_z.begin(), _grid_max_z.end(), -std::numeric_limits<float>::max());
        std::fill(_grid_count.begin(), _grid_count.end(), 0);

        const float inv_cell = 1.0f / _params.peak_cell;

        // Populate grid
        for (const auto &p : elevated)
        {
            int gx = static_cast<int>((p.x() - GRID_MIN) * inv_cell);
            int gy = static_cast<int>((p.y() - GRID_MIN) * inv_cell);
            if (gx < 0 || gx >= GRID_DIM || gy < 0 || gy >= GRID_DIM)
            {
                // The grid is fixed at GRID_MIN=-15 with GRID_DIM=200 and
                // peak_cell=0.30, so it covers -15..+45 m. Anything past that
                // is silently lost here rather than rejected by a gate.
                if (_debug) _stats.points_outside_grid++;
                continue;
            }
            int idx = gy * GRID_DIM + gx;
            _grid_count[idx]++;
            if (p.z() > _grid_max_z[idx]) _grid_max_z[idx] = p.z();
        }

        if (_debug)
        {
            for (const int c : _grid_count)
                if (c > 0) _stats.cells_populated++;
        }

        // Non-maximum suppression over 3x3 neighborhood
        const float peak_h = _params.peak_height;
        const int min_neigh = _params.peak_min_neighbors;

        for (int gy = 1; gy < GRID_DIM - 1; ++gy)
        {
            for (int gx = 1; gx < GRID_DIM - 1; ++gx)
            {
                int idx = gy * GRID_DIM + gx;
                float z = _grid_max_z[idx];

                // An empty cell reads as -FLT_MAX and fails the height test for
                // free, so empty cells must not be counted as rejected.
                if (_grid_count[idx] == 0) continue;

                if (_debug) _stats.cells_above_min_height++;
                if (z < peak_h)
                {
                    if (_debug) _stats.cells_below_peak_height++;
                    continue;
                }
                if (_debug) _stats.cells_below_min_height++;

                // Check 3x3 neighborhood
                bool is_max = true;
                for (int dy = -1; dy <= 1 && is_max; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0) continue;
                        int nidx = (gy + dy) * GRID_DIM + (gx + dx);
                        if (_grid_max_z[nidx] > z) { is_max = false; break; }
                    }

                if (!is_max)
                {
                    if (_debug) _stats.cells_not_local_max++;
                    continue;
                }

                // Require minimum neighbor count in cell
                if (_grid_count[idx] < min_neigh)
                {
                    if (_debug) _stats.cells_too_few_points++;
                    continue;
                }

                float cx = GRID_MIN + (gx + 0.5f) * _params.peak_cell;
                float cy = GRID_MIN + (gy + 0.5f) * _params.peak_cell;
                Peak peak;
                peak.xy = Eigen::Vector2f(cx, cy);
                peak.z = z;
                peak.cell_count = _grid_count[idx];
                peaks_out.push_back(peak);
            }
        }

        if (_debug) _stats.peaks += peaks_out.size();
    }

    // -------------------------------------------------------------------------
    // Stage 3 – Localized ROI validation
    // -------------------------------------------------------------------------
    bool ConeDetector::validate_roi(const std::vector<Eigen::Vector3f> &car,
                                    const Peak &peak,
                                    ConeDetection &detection_out,
                                    PeakTrace &trace)
    {
        const float roi_r = _params.roi_radius;
        const float roi_r2 = roi_r * roi_r;
        const float ring_in2 = _params.ring_inner * _params.ring_inner;
        const float ring_out2 = _params.ring_outer * _params.ring_outer;
        const float core_r2 = _params.core_radius * _params.core_radius;
        const float stand_h = _params.stand_height;

        const float cx = peak.xy.x(), cy = peak.xy.y();
        const int n = static_cast<int>(car.size());

        if (_debug)
        {
            _stats.roi_evaluated++;
            trace.cx = cx;
            trace.cy = cy;
            trace.range = std::sqrt(cx * cx + cy * cy);
            trace.cell_z = peak.z;
            trace.cell_count = peak.cell_count;
        }

        // --- Local ROI gathering ---
        // `car` is already in car/cog, the same frame as the peak. Thread-local
        // buffers to avoid allocation.
        thread_local std::vector<Eigen::Vector3f> roi_points;
        thread_local std::vector<Eigen::Vector3f> ring_points;
        roi_points.clear();
        ring_points.clear();

        for (int i = 0; i < n; ++i)
        {
            const Eigen::Vector3f &p = car[static_cast<size_t>(i)];
            const float dx = p.x() - cx;
            const float dy = p.y() - cy;
            const float r2 = dx * dx + dy * dy;
            if (r2 > roi_r2) continue;

            roi_points.push_back(p);

            if (r2 >= ring_in2 && r2 <= ring_out2)
                ring_points.push_back(p);
        }

        if (_debug)
        {
            trace.n_roi = static_cast<int>(roi_points.size());
            trace.n_ring = static_cast<int>(ring_points.size());
        }

        if (ring_points.size() < 3)
        {
            if (_debug)
            {
                _stats.roi_rejected_ring++;
                // Zero ring points means the candidate has no neighbourhood to
                // measure against, which points at the frame the ROI is searched
                // in rather than at the cone.
                if (ring_points.empty()) _stats.roi_no_ring++;
                trace.reason = "ring_too_small";
            }
            return false;
        }

        // --- Local ground plane fit (weighted least squares on annulus) ---
        // Fit: z = a*x + b*y + c
        float sw = 0, sx = 0, sy = 0, sz = 0;
        float sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
        for (const auto &p : ring_points)
        {
            float w = 1.0f; // uniform weight
            sw += w;
            sx += w * p.x(); sy += w * p.y(); sz += w * p.z();
            sxx += w * p.x() * p.x(); sxy += w * p.x() * p.y();
            syy += w * p.y() * p.y();
            sxz += w * p.x() * p.z(); syz += w * p.y() * p.z();
        }

        // Solve 3x3 normal equations
        Eigen::Matrix3f A;
        A << sxx, sxy, sx,
             sxy, syy, sy,
             sx,  sy,  sw;
        Eigen::Vector3f b(sxz, syz, sz);
        Eigen::Vector3f sol = A.colPivHouseholderQr().solve(b);
        float ga = sol.x(), gb = sol.y(), gc = sol.z();

        if (_debug)
        {
            trace.plane_a = ga;
            trace.plane_b = gb;
            trace.plane_c = gc;

            float lo = std::numeric_limits<float>::max();
            float hi = -std::numeric_limits<float>::max();
            for (const auto &p : ring_points)
            {
                lo = std::min(lo, p.z());
                hi = std::max(hi, p.z());
            }
            trace.ring_z_spread = hi - lo;

            // A three-point annulus, or a near-degenerate one, gives a plane the
            // 3x3 QR cannot solve reliably. Everything downstream would then be
            // measured against noise, so surface it here rather than as a
            // mysterious height rejection.
            if (!std::isfinite(ga) || !std::isfinite(gb) || !std::isfinite(gc))
            {
                _stats.roi_plane_degenerate++;
                trace.reason = "plane_degenerate";
                return false;
            }
        }

        // --- Local point filtering: isolate core cone points ---
        thread_local std::vector<Eigen::Vector3f> core_points;
        core_points.clear();

        for (const auto &p : roi_points)
        {
            const float dx = p.x() - cx;
            const float dy = p.y() - cy;
            const float r2 = dx * dx + dy * dy;
            if (r2 > core_r2) continue;

            const float z_ground = ga * p.x() + gb * p.y() + gc;
            const float h = p.z() - z_ground;
            if (h > stand_h)
                core_points.push_back(p);
        }

        const int n_core = static_cast<int>(core_points.size());
        if (_debug) trace.n_core = n_core;

        if (n_core < _params.min_points)
        {
            if (_debug)
            {
                _stats.roi_rejected_core++;
                trace.reason = "too_few_core_points";
            }
            return false;
        }

        // --- Cone height ---
        float h_min = std::numeric_limits<float>::max();
        float h_max = -std::numeric_limits<float>::max();
        for (const auto &p : core_points)
        {
            const float z_ground = ga * p.x() + gb * p.y() + gc;
            const float h = p.z() - z_ground;
            h_min = std::min(h_min, h);
            h_max = std::max(h_max, h);
        }
        const float h_top = h_max;

        if (_debug) trace.h_top = h_top;

        if (h_top < _params.min_top || h_top > _params.max_top)
        {
            if (_debug)
            {
                _stats.roi_rejected_height++;
                // Which side of the window it fell off tells you whether the
                // ground reference is inflating the height or deflating it.
                trace.reason = (h_top < _params.min_top) ? "top_too_short" : "top_too_tall";
            }
            return false;
        }

        // --- Footprint aspect ratio & slenderness ---
        float fx_min = std::numeric_limits<float>::max(), fx_max = -std::numeric_limits<float>::max();
        float fy_min = std::numeric_limits<float>::max(), fy_max = -std::numeric_limits<float>::max();
        for (const auto &p : core_points)
        {
            fx_min = std::min(fx_min, p.x()); fx_max = std::max(fx_max, p.x());
            fy_min = std::min(fy_min, p.y()); fy_max = std::max(fy_max, p.y());
        }
        const float footprint_d = std::sqrt((fx_max - fx_min) * (fx_max - fx_min) +
                                              (fy_max - fy_min) * (fy_max - fy_min));

        const float slenderness = h_top / std::max(footprint_d, 0.01f);

        if (_debug)
        {
            trace.footprint = footprint_d;
            trace.slenderness = slenderness;
        }

        if (footprint_d > _params.max_footprint)
        {
            if (_debug)
            {
                _stats.roi_rejected_footprint++;
                trace.reason = "footprint_too_wide";
            }
            return false;
        }

        if (slenderness < _params.min_slenderness)
        {
            if (_debug)
            {
                _stats.roi_rejected_slenderness++;
                trace.reason = "not_slender_enough";
            }
            return false;
        }

        // --- Eigen 3D PCA ---
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto &p : core_points) centroid += p;
        centroid /= n_core;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto &p : core_points)
        {
            Eigen::Vector3f d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= n_core;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        if (solver.info() != Eigen::Success)
        {
            if (_debug)
            {
                trace.reason = "pca_solver_failed";
                trace.pca_angle_deg = -1.0f;
            }
            return false;
        }

        // Primary eigenvector (largest eigenvalue)
        Eigen::Vector3f v1 = solver.eigenvectors().col(2);
        v1.normalize();

        // Angle from vertical Z axis
        const float cos_angle = std::abs(v1.z());
        const float angle_deg = std::acos(std::clamp(cos_angle, -1.0f, 1.0f)) * 180.0f / M_PI;

        if (_debug) trace.pca_angle_deg = angle_deg;

        if (angle_deg > _params.max_pca_angle_deg)
        {
            if (_debug)
            {
                _stats.roi_rejected_pca++;
                trace.reason = "pca_tilted";
            }
            return false;
        }

        // --- Populate detection ---
        detection_out.position = centroid;
        detection_out.height = h_top;
        detection_out.big = (h_top >= _params.big_cone_min_height);

        // --- Covariance matrix (2x2 row-major: [cxx, cxy, cyx, cyy]) ---
        const float range = std::sqrt(centroid.x() * centroid.x() + centroid.y() * centroid.y());
        const float sigma_r = _params.range_sigma;
        const float sigma_b = _params.bearing_sigma_deg * M_PI / 180.0f * range;

        const float bearing = std::atan2(centroid.y(), centroid.x());
        const float cb = std::cos(bearing), sb = std::sin(bearing);

        // 2D covariance in (range, bearing) frame
        const float sr2 = sigma_r * sigma_r;
        const float sb2 = sigma_b * sigma_b;

        // Rotate to Cartesian: C = R * diag(sr^2, sb^2) * R^T
        const float cxx = sr2 * cb * cb + sb2 * sb * sb;
        const float cxy = sr2 * cb * sb - sb2 * sb * cb;
        const float cyy = sr2 * sb * sb + sb2 * cb * cb;

        detection_out.covariance = {cxx, cxy, cxy, cyy};

        // Confidence based on point count and slenderness
        detection_out.confidence = std::min(1.0f, static_cast<float>(n_core) / 20.0f) *
                                   std::min(1.0f, slenderness);

        // Store standing points for the debug clouds. This runs inside the OpenMP
        // region, so the two vectors need the critical section or concurrent
        // push_back corrupts them. Small critical region, hit once per accepted
        // candidate.
        #pragma omp critical(cone_detector3_debug_buffers)
        {
            _standing.push_back(centroid);
            _labels.push_back(detection_out.big ? 2 : 1);
        }

        if (_debug) _stats.roi_accepted++;

        return true;
    }

    // -------------------------------------------------------------------------
    // Stage 4 – Classification, covariance & deduplication
    // -------------------------------------------------------------------------
    void ConeDetector::classify_and_dedup(std::vector<ConeDetection> &detections)
    {
        const float nms_r = _params.nms_radius;
        const float nms_r2 = nms_r * nms_r;

        // Sort by confidence (descending)
        std::sort(detections.begin(), detections.end(),
                  [](const ConeDetection &a, const ConeDetection &b) {
                      return a.confidence > b.confidence;
                  });

        std::vector<ConeDetection> kept;
        kept.reserve(detections.size());

        for (const auto &det : detections)
        {
            bool suppressed = false;
            for (const auto &k : kept)
            {
                float dx = det.position.x() - k.position.x();
                float dy = det.position.y() - k.position.y();
                if (dx * dx + dy * dy < nms_r2)
                {
                    suppressed = true;
                    break;
                }
            }
            if (!suppressed)
                kept.push_back(det);
        }

        detections = std::move(kept);
    }

    // -------------------------------------------------------------------------
    // Main detect entry point
    // -------------------------------------------------------------------------
    std::vector<ConeDetection> ConeDetector::detect(const Eigen::MatrixXf &cloud,
                                                     const Eigen::Isometry3f &sensor_from_car)
    {
        auto t0 = std::chrono::steady_clock::now();
        _standing.clear();
        _labels.clear();
        _candidates = 0;
        _peak_traces.clear();

        if (_debug)
        {
            _stats.total_points = 0;
            _stats.rejected_range = 0;
            _stats.rejected_ego = 0;
            _stats.rejected_z = 0;
            _stats.rejected_ground = 0;
            _stats.elevated = 0;
            _stats.peaks = 0;
            _stats.roi_rejected_ring = 0;
            _stats.roi_rejected_core = 0;
            _stats.roi_rejected_height = 0;
            _stats.roi_rejected_footprint = 0;
            _stats.roi_rejected_slenderness = 0;
            _stats.roi_rejected_pca = 0;
            _stats.detections = 0;
            _stats.max_h_above = 0.0f;
            _stats.max_z_car = 0.0f;
            _stats.points_outside_grid = 0;
            _stats.cells_populated = 0;
            _stats.cells_below_peak_height = 0;
            _stats.cells_not_local_max = 0;
            _stats.cells_too_few_points = 0;
            _stats.cells_below_min_height = 0;
            _stats.cells_above_min_height = 0;
            _stats.roi_evaluated = 0;
            _stats.roi_no_ring = 0;
            _stats.roi_plane_degenerate = 0;
            _stats.roi_accepted = 0;
        }

        // Stage 1
        auto t1 = std::chrono::steady_clock::now();
        crop_and_elevate(cloud, sensor_from_car, _car, _elevated);
        auto t2 = std::chrono::steady_clock::now();
        _stage.crop_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

        // Stage 2
        extract_peaks(_elevated, _peaks);
        auto t3 = std::chrono::steady_clock::now();
        _stage.hash_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

        // Stage 3 – OpenMP parallel ROI validation
        const int n_peaks = static_cast<int>(_peaks.size());
        _candidates = n_peaks;

        std::vector<ConeDetection> all_detections;
        all_detections.reserve(n_peaks);

        _peak_traces.resize(static_cast<size_t>(n_peaks));

        #pragma omp parallel
        {
            std::vector<ConeDetection> local_detections;

            #pragma omp for schedule(dynamic, 1)
            for (int i = 0; i < n_peaks; ++i)
            {
                PeakTrace &trace = _peak_traces[static_cast<size_t>(i)];
                trace.index = i;
                ConeDetection det;
                if (validate_roi(_car, _peaks[static_cast<size_t>(i)], det, trace))
                    local_detections.push_back(det);
            }

            #pragma omp critical
            {
                all_detections.insert(all_detections.end(),
                                      local_detections.begin(),
                                      local_detections.end());
            }
        }

        auto t4 = std::chrono::steady_clock::now();
        _stage.roi_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();

        // Stage 4
        classify_and_dedup(all_detections);
        auto t5 = std::chrono::steady_clock::now();
        _stage.classify_ms = std::chrono::duration<double, std::milli>(t5 - t4).count();
        _stage.total_ms = std::chrono::duration<double, std::milli>(t5 - t0).count();

        if (_debug)
        {
            _stats.detections += all_detections.size();
            print_report(all_detections.size());
        }

        return all_detections;
    }

    // -------------------------------------------------------------------------
    // Debug reporting
    // -------------------------------------------------------------------------
    void ConeDetector::print_report(int published)
    {
        // Fixed precision so the columns line up frame to frame and a value can
        // be eyeballed against a threshold without changing format.
        std::cerr << std::fixed << std::setprecision(3);

        // Stage 1. max_h_above far above max_z_car means the ground is being
        // read as elevated; max_h_above near zero means nothing stands up at all.
        std::cerr << "[ConeDetector3] --- frame " << _frame_seq++ << " ---\n"
                  << "  stage1 crop : pts=" << _stats.total_points
                  << " elevated=" << _stats.elevated
                  << " | rej range=" << _stats.rejected_range
                  << " ego=" << _stats.rejected_ego
                  << " z=" << _stats.rejected_z
                  << " ground=" << _stats.rejected_ground
                  << " | max_h_above=" << _stats.max_h_above
                  << " max_z_car=" << _stats.max_z_car
                  << '\n'

                  // Stage 2. below_peak_height dominating means the height gate
                  // is the wall; not_local_max dominating means peaks exist but
                  // suppress each other; populated==0 means stage 1 emptied the set.
                  << "  stage2 hash : cells_populated=" << _stats.cells_populated
                  << " above_h=" << _stats.cells_above_min_height
                  << " | rej below_peak_h=" << _stats.cells_below_peak_height
                  << " not_local_max=" << _stats.cells_not_local_max
                  << " too_few_pts=" << _stats.cells_too_few_points
                  << " | outside_grid=" << _stats.points_outside_grid
                  << " -> peaks=" << _stats.peaks << '\n'

                  // Stage 3. no_ring dominating is the signature of the ROI
                  // being searched in the wrong frame: the candidate has no
                  // neighbourhood where the search looks.
                  << "  stage3 roi  : evaluated=" << _stats.roi_evaluated
                  << " | rej ring=" << _stats.roi_rejected_ring
                  << " (no_ring=" << _stats.roi_no_ring << ")"
                  << " plane_degenerate=" << _stats.roi_plane_degenerate
                  << " core=" << _stats.roi_rejected_core
                  << " h=" << _stats.roi_rejected_height
                  << " fp=" << _stats.roi_rejected_footprint
                  << " sl=" << _stats.roi_rejected_slenderness
                  << " pca=" << _stats.roi_rejected_pca
                  << " | accepted=" << _stats.roi_accepted << '\n'

                  // Stage 4. roi_accepted above published means detections were
                  // thrown away by NMS, not by a geometry gate.
                  << "  stage4 nms  : accepted=" << _stats.roi_accepted
                  << " -> published=" << published
                  << " (suppressed=" << (static_cast<int>(_stats.roi_accepted) - published)
                  << ")\n";

        // Per-candidate lines, capped so a frame with 300 peaks does not bury
        // the summary. Sorted by index so the order is stable across frames.
        static constexpr int MAX_TRACE_LINES = 24;

        std::vector<const PeakTrace *> ordered;
        ordered.reserve(_peak_traces.size());
        for (const auto &t : _peak_traces)
            if (t.reason || t.n_core > 0) ordered.push_back(&t);
        std::sort(ordered.begin(), ordered.end(),
                  [](const PeakTrace *a, const PeakTrace *b) { return a->index < b->index; });

        for (size_t i = 0; i < ordered.size(); ++i)
        {
            if (i == MAX_TRACE_LINES)
            {
                std::cerr << "    ... " << (ordered.size() - MAX_TRACE_LINES)
                          << " more peaks not shown\n";
                break;
            }

            const PeakTrace &t = *ordered[i];
            std::cerr << "    peak[" << t.index << "]"
                      << " xy=(" << t.cx << ", " << t.cy << ")"
                      << " range=" << t.range
                      << " cell(z=" << t.cell_z << ", n=" << t.cell_count << ")"
                      << " roi=" << t.n_roi << " ring=" << t.n_ring
                      << " core=" << t.n_core
                      << " plane=(" << t.plane_a << ", " << t.plane_b << ", " << t.plane_c << ")"
                      << " ring_z_spread=" << t.ring_z_spread
                      << " h_top=" << t.h_top
                      << " fp=" << t.footprint
                      << " sl=" << t.slenderness
                      << " pca=" << t.pca_angle_deg
                      << " -> " << (t.reason ? t.reason : "ACCEPTED") << '\n';
        }

        std::cerr << std::flush;
    }
} // namespace mirena
