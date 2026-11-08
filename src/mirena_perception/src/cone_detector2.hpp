#ifndef MIRENA_PERCEPTION_CONE_DETECTOR_HPP
#define MIRENA_PERCEPTION_CONE_DETECTOR_HPP

#include <array>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mirena
{
    struct ConeDetectorParams
    {
        // -- 1. Preprocesamiento & Voxel Grid / Intensidad --
        bool enable_voxel_grid = true;
        double voxel_leaf_size = 0.05; // Tamaño del voxel en metros (5 cm)
        
        bool use_intensity = true;
        double min_intensity_peak = 5.0;

        // -- 2. Selección de candidatos --
        double coarse_cell = 0.40;
        double coarse_ring = 1.20;
        double candidate_height = 0.08;
        double peak_cell = 0.25;
        double peak_height = 0.10;

        // -- 3. Recorte del vehículo --
        double ego_half_length = 1.0;
        double ego_half_width = 1.2;

        // -- 4. Ajuste de plano de suelo local --
        double ring_inner = 0.45;
        double ring_outer = 2.20;
        double ring_cell = 0.40;
        int ring_min_points = 15;
        int ring_min_cells = 4;

        // -- 5. Validación Geométrica, PCA y Esbeltez --
        double min_top = 0.15;
        double max_top = 0.60;
        double core_radius = 0.35;
        double stand_height = 0.08;
        double max_footprint = 0.55;
        int min_points = 2;

        double max_roughness = 0.040;
        double max_ring_ratio = 2.5;
        double max_range = 30.0;
        double big_cone_min_height = 0.40;

        // Criterios PCA / Esbeltez
        double min_slenderness = 0.3;
        double max_pca_angle_deg = 35.0;

        // -- Covarianzas para SLAM --
        double range_sigma = 0.08;
        double bearing_sigma = 0.010;
        double cluster_spread = 0.30;
    };

    struct Detection
    {
        Eigen::Vector3f position;
        int num_points;
        bool big;
        std::array<double, 4> covariance;
        float half_extent;
    };

    class ConeDetector
    {
    public:
        using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

        explicit ConeDetector(const ConeDetectorParams &params);

        const std::vector<Detection> &detect(const Eigen::MatrixXf &cloud_lidar,
                                             const Eigen::Isometry3f &car_from_lidar);

        const PointCloud &standing() const { return standing_; }
        const std::vector<int> &labels() const { return labels_; }

        struct StageTimes
        {
            double level_ms = 0.0;
            double coarse_ms = 0.0;
            double peaks_ms = 0.0;
            double accept_ms = 0.0;
        };
        const StageTimes &stage_times() const { return stage_; }
        double total_ms() const { return total_ms_; }
        int candidates() const { return candidates_; }

    private:
        struct Plane
        {
            float a = 0.0f, b = 0.0f, c = 0.0f;
            float roughness = 0.0f;
            bool valid = false;
            float at(float x, float y, float cx, float cy) const
            {
                return a * (x - cx) + b * (y - cy) + c;
            }
        };

        void apply_voxel_grid_filter();
        void build_index();
        void gather(float cx, float cy, float radius, std::vector<int> &out) const;
        Plane fit_ground(float cx, float cy);
        
        bool validate_shape_and_pca(const std::vector<Eigen::Vector3f> &pts, float top, float &out_extent) const;

        std::array<double, 4> position_covariance(const Eigen::Vector2f &centroid, int num_points) const;

        ConeDetectorParams params_;

        Eigen::MatrixXf car_;
        bool has_intensity_ = false;
        std::vector<int> keep_;

        float index_cell_ = 0.5f;
        float index_x0_ = 0.0f, index_y0_ = 0.0f;
        int index_nx_ = 0, index_ny_ = 0;
        std::vector<int> index_start_;
        std::vector<int> index_items_;
        mutable std::vector<int> scratch_ring_;
        mutable std::vector<int> scratch_neighbors_;
        mutable std::vector<float> scratch_dists_;

        std::vector<Detection> detections_;
        PointCloud standing_;
        std::vector<int> labels_;
        Eigen::Vector2f sensor_xy_ = Eigen::Vector2f::Zero();

        StageTimes stage_;
        double total_ms_ = 0.0;
        int candidates_ = 0;
    };
} // namespace mirena

#endif