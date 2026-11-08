#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace mirena
{
    struct ConeDetectorParams
    {
        // Stage 1 – global crop
        float max_range = 30.0f;
        float ego_half_length = 1.5f;
        float ego_half_width = 1.0f;
        float slope_factor = 0.05f;
        float candidate_height = 0.05f;
        float z_min = -0.5f;
        float z_max = 1.5;
        // Stage 2 – spatial hashing & peak extraction
        float peak_cell = 0.30f;
        float peak_height = 0.18f;
        int peak_min_neighbors = 2;

        // Stage 3 – local ROI validation
        float roi_radius = 1.0f;
        float ring_inner = 0.35f;
        float ring_outer = 1.0f;
        float core_radius = 0.25f;
        float stand_height = 0.08f;
        int min_points = 5;
        float min_top = 0.10f;
        float max_top = 0.65f;
        float min_slenderness = 0.30f;
        float max_footprint = 0.60f;
        float max_pca_angle_deg = 50.0f;

        // Stage 4 – classification
        float big_cone_min_height = 0.38f;

        // Covariance
        float range_sigma = 0.03f;
        float bearing_sigma_deg = 2.0f;

        // NMS 
        float nms_radius = 0.50f;
    };

    // A local maximum in the stage-2 grid. The z and the cell population travel
    // with the xy so the debug report can show why a peak was interesting and
    // whether it was a lone noisy return.
    struct Peak
    {
        Eigen::Vector2f xy{Eigen::Vector2f::Zero()};
        float z = -std::numeric_limits<float>::max();
        int cell_count = 0;
    };

    struct ConeDetection
    {
        Eigen::Vector3f position;
        std::array<double, 4> covariance; // 2x2 row-major: [cxx, cxy, cyx, cyy]
        bool big;
        float height;
        float confidence;
    };

    struct StageTimes
    {
        double crop_ms = 0.0;
        double hash_ms = 0.0;
        double roi_ms = 0.0;
        double classify_ms = 0.0;
        double total_ms = 0.0;
    };

    struct DebugStats
    {
        std::atomic<uint64_t> total_points{0};
        std::atomic<uint64_t> rejected_range{0};
        std::atomic<uint64_t> rejected_ego{0};
        std::atomic<uint64_t> rejected_z{0};
        std::atomic<uint64_t> rejected_ground{0};
        std::atomic<uint64_t> elevated{0};
        std::atomic<uint64_t> peaks{0};
        std::atomic<uint64_t> roi_rejected_ring{0};
        std::atomic<uint64_t> roi_rejected_core{0};
        std::atomic<uint64_t> roi_rejected_height{0};
        std::atomic<uint64_t> roi_rejected_footprint{0};
        std::atomic<uint64_t> roi_rejected_slenderness{0};
        std::atomic<uint64_t> roi_rejected_pca{0};
        std::atomic<uint64_t> detections{0};

        // --- Stage 1 extras -------------------------------------------------
        // Written from the single-threaded crop loop only, so these are plain
        // floats rather than atomics: std::atomic has no fetch_max.
        float max_h_above{0.0f};  // largest height-above-ground seen at all
        float max_z_car{0.0f};    // largest car-frame z seen at all

        // --- Stage 2 breakdown ---------------------------------------------
        // Why a peak did not survive. Without these, "peaks=0" is ambiguous
        // between "nothing stood up" and "everything was suppressed".
        std::atomic<uint64_t> points_outside_grid{0};
        std::atomic<uint64_t> cells_populated{0};
        std::atomic<uint64_t> cells_below_peak_height{0};
        std::atomic<uint64_t> cells_not_local_max{0};
        std::atomic<uint64_t> cells_too_few_points{0};
        std::atomic<uint64_t> cells_below_min_height{0};
        std::atomic<uint64_t> cells_above_min_height{0};

        // --- Stage 3 breakdown ---------------------------------------------
        std::atomic<uint64_t> roi_evaluated{0};
        std::atomic<uint64_t> roi_no_ring{0};      // annulus came back empty
        std::atomic<uint64_t> roi_plane_degenerate{0}; // solve produced non-finite
        std::atomic<uint64_t> roi_accepted{0};
    };

    // Why a single candidate did not become a cone, with every measured value
    // the gates saw. One record per peak, built under OpenMP and printed after
    // the parallel region so the lines stay readable.
    struct PeakTrace
    {
        int index = 0;
        float cx = 0.0f, cy = 0.0f;
        float range = 0.0f;
        float cell_z = 0.0f;     // tallest point in the grid cell
        int cell_count = 0;      // points in the grid cell

        int n_roi = 0;           // points within roi_radius
        int n_ring = 0;          // points within ring_inner..ring_outer
        int n_core = 0;          // points within core_radius and above plane
        float plane_a = 0.0f, plane_b = 0.0f, plane_c = 0.0f;
        float ring_z_spread = 0.0f; // max-min z in the annulus
        float h_top = 0.0f;
        float footprint = 0.0f;
        float slenderness = 0.0f;
        float pca_angle_deg = 0.0f;

        const char *reason = nullptr; // null == accepted
    };

    class ConeDetector
    {
    public:
        explicit ConeDetector(const ConeDetectorParams &params);

        std::vector<ConeDetection> detect(const Eigen::MatrixXf &cloud,
                                          const Eigen::Isometry3f &sensor_from_car);

        const std::vector<Eigen::Vector3f> &standing() const { return _standing; }
        const std::vector<int> &labels() const { return _labels; }
        int candidates() const { return _candidates; }
        const StageTimes &stage_times() const { return _stage; }
        double total_ms() const { return _stage.total_ms; }

        // One record per stage-2 peak, with the values the gates measured and the
        // gate that dropped it. Empty unless _debug is on.
        const std::vector<PeakTrace> &peak_traces() const { return _peak_traces; }

        bool _debug = true;
        DebugStats _stats;

    private:
        // Stage 1
        void crop_and_elevate(const Eigen::MatrixXf &cloud,
                              const Eigen::Isometry3f &sensor_from_car,
                              std::vector<Eigen::Vector3f> &car_out,
                              std::vector<Eigen::Vector3f> &elevated_out);

        // Stage 2
        void extract_peaks(const std::vector<Eigen::Vector3f> &elevated,
                           std::vector<Peak> &peaks_out);

        // Stage 3
        bool validate_roi(const std::vector<Eigen::Vector3f> &car,
                          const Peak &peak,
                          ConeDetection &detection_out,
                          PeakTrace &trace);

        // Stage 4
        void classify_and_dedup(std::vector<ConeDetection> &detections);

        // Debug only: the full per-stage breakdown plus one line per candidate.
        void print_report(int published);

        uint64_t _frame_seq = 0;

        ConeDetectorParams _params;

        // Persistent workspace buffers (no per-frame allocation)
        // _car is every point that survived the stage-1 crop, in car/cog.
        // Stage 3 works from this, not from the raw cloud, so that peaks and
        // the points they are measured against share one frame.
        std::vector<Eigen::Vector3f> _car;
        std::vector<Eigen::Vector3f> _elevated;
        std::vector<Peak> _peaks;
        std::vector<Eigen::Vector3f> _standing;
        std::vector<int> _labels;
        std::vector<PeakTrace> _peak_traces;
        int _candidates = 0;
        StageTimes _stage;

        // Spatial hash grid (flat array, reused across frames)
        static constexpr int GRID_DIM = 200;
        static constexpr float GRID_MIN = -15.0f;
        static constexpr float GRID_CELL = 0.15f;
        std::vector<float> _grid_max_z;
        std::vector<int> _grid_count;
    };
} // namespace mirena
