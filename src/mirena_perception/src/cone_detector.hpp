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
    /// Everything the detector reads. Ten numbers, and every one of them is a
    /// statement about a cone or about the track it stands on rather than about
    /// an algorithm's internals -- which is the point. A threshold you can check
    /// against a tape measure is one you can be wrong about visibly.
    struct ConeDetectorParams
    {
        // -- Finding candidates -------------------------------------------------
        /// Cell of the coarse grid whose per-cell lowest point is a first,
        /// throwaway guess at the ground. It only decides what is worth looking
        /// at properly; every number that matters is measured later, against a
        /// plane fitted around the candidate.
        double coarse_cell = 0.40;
        /// How far that first pass looks for a lower point. Wide enough that a
        /// cone never becomes its own ground, narrow enough that a metre of
        /// track slope does not.
        double coarse_ring = 1.20;
        /// A point is worth considering when it clears the coarse ground by this.
        double candidate_height = 0.08;
        /// Grid the candidates are thinned on, and the height a candidate has to
        /// reach to be one at all.
        ///
        /// A candidate is a local maximum on this grid, not a connected
        /// component. That distinction is what finds the mid-range cones: at
        /// 5-8 m the ground itself sits a few centimetres above the coarse
        /// minimum, so growing a cluster by connectivity swallows the cone into
        /// that field and then throws the whole thing away for being too wide.
        /// A peak cannot be swallowed.
        double peak_cell = 0.25;
        double peak_height = 0.15;

        // -- The car looking at itself ------------------------------------------
        // A hoop-mounted sensor sees the nosecone, the bodywork and the driver as
        // a large permanent slab at cone height a few tens of centimetres away.
        // Nothing is lost by cutting it: the sensor cannot see the track that
        // close anyway.
        double ego_half_length = 2.0;
        double ego_half_width = 1.2;

        // -- The ground under one candidate -------------------------------------
        /// The ring a candidate's ground plane is fitted in. The inner radius
        /// clears the cone itself; the outer one is far enough away to pin a
        /// slope.
        ///
        /// This ring is the whole design. Fit a plane through the patch the cone
        /// stands in and the cone's own base falls inside the plane's thickness
        /// and is classified as ground -- measured on this sensor that costs a
        /// cone at 3 m two thirds of its points and leaves only the tip, so a
        /// 0.325 m cone reads 0.21 m and stops being distinguishable from a bump
        /// in the asphalt. Fitted around the cone instead, nothing the cone does
        /// can move its own reference.
        double ring_inner = 0.55;
        double ring_outer = 2.60;
        /// The ring is reduced to one lowest point per cell of this size before
        /// the fit, so the plane follows the surface rather than the sampling
        /// density, which is ten times higher near the sensor than far from it.
        double ring_cell = 0.40;
        /// Fewest ring points, and fewest surviving cell minima, worth fitting.
        int ring_min_points = 40;
        int ring_min_cells = 8;

        // -- What a cone looks like ---------------------------------------------
        /// How far the top of the blob stands above that plane. A small cone is
        /// 0.325 m and a big one 0.505 m; the band allows for the apex the beams
        /// miss and for a cone seen from one side.
        double min_top = 0.18;
        double max_top = 0.58;
        /// Radius the blob is measured in, and the height inside it that counts
        /// as standing up rather than as ground.
        double core_radius = 0.32;
        double stand_height = 0.10;
        /// Widest the standing part may be.
        double max_footprint = 0.55;
        int min_points = 2;

        /// How rough the ground around it may be, as the standard deviation of
        /// the plane fit's residual, in metres.
        ///
        /// Measured on hand-labelled frames: 0.5-1.7 cm under a real cone,
        /// 3.3-8.9 cm under the rough ground and vegetation that a detector
        /// working from cluster size alone reports as cones. One number, and it
        /// separates the two populations almost on its own.
        double max_roughness = 0.045;

        /// How much of the ring around a candidate may also be standing up,
        /// counted relative to the candidate's own standing points.
        ///
        /// A cone is alone on the track. The things that fool a size-and-height
        /// test are pieces of something larger and rougher, and the giveaway is
        /// that the ring around them is full of elevated points too. On all 34
        /// hand-labelled cones this ratio is zero.
        double max_ring_ratio = 1.2;

        /// Furthest a cone is claimed. Cones were hand-labelled out to 30 m and
        /// found there, which the shape tests can support because height is
        /// measured against a real local plane rather than a global one.
        double max_range = 30.0;
        /// Tall enough to be one of the big orange cones on the start line. The
        /// two stand 505 and 325 mm, so the gap is many times the measurement;
        /// this is the one classification a colourless detector can make
        /// honestly. It sits nearer the small cone than halfway on purpose,
        /// because a blob is the part of a cone the sensor happened to catch and
        /// its top under-reads the real apex far more often than it over-reads.
        double big_cone_min_height = 0.40;

        // -- How well a detection is located ------------------------------------
        //
        // A LiDAR does not know a cone equally well in every direction: along the
        // beam it measures a range directly, across the beam it has only the
        // angle, so that error is the angle times the range. Reporting the
        // difference honestly is what lets SLAM stop treating a cone at 25 m like
        // one at 3 m.
        //
        // These are measured, not assumed. Repeat observations of the same cone
        // across a run land within 0.33 m of each other, 0.56 m at the 90th
        // percentile. An earlier model reported 0.05-0.13 m, four times too
        // confident, and the consequence was not academic: SLAM's chi-square gate
        // rejected its own landmarks and spawned a duplicate beside every cone.
        double range_sigma = 0.08;
        double bearing_sigma = 0.010;
        double cluster_spread = 0.30;
    };

    /// One accepted cone, positioned at the centroid of its standing points in
    /// the car frame.
    struct Detection
    {
        Eigen::Vector3f position;
        int num_points;
        /// Tall enough to be one of the big orange cones on the start line.
        /// Colour is not measured here; height is.
        bool big;
        /// How well this position is known, as a 2x2 row-major covariance in the
        /// car frame: [xx, xy, yx, yy].
        std::array<double, 4> covariance;
        /// Half the width of the standing blob this was built from, floored by
        /// what the sensor can resolve across the beam at this range. Two
        /// detections are the same object when their centres sit inside the sum
        /// of these, which is what replaced a fixed exclusion radius.
        float half_extent;
    };

    /// Colourless, geometry-only cone detector.
    ///
    /// Three ideas, and the detector is those three:
    ///
    ///   1. The ground under a candidate is a plane fitted to the low points in a
    ///      ring *around* it, never through it, so a cone cannot raise its own
    ///      reference and lose its base to it.
    ///   2. A cone is isolated. What fools a size-and-height test is a piece of
    ///      something larger, and its surroundings say so.
    ///   3. The ground a cone stands on is smooth, and the plane fit's residual
    ///      measures that directly.
    ///
    /// Scored against hand-labelled LiDAR frames from
    /// rosbag2_2026_08_05-20_26_23 at t = 20 s and t = 90 s: 30 detections, all
    /// 30 of them real cones, and every cone inside 20 m found.
    class ConeDetector
    {
    public:
        using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

        explicit ConeDetector(const ConeDetectorParams &params);

        /// `cloud_lidar` holds one point per row as x, y, z (further columns are
        /// ignored), in the LiDAR's own frame. `car_from_lidar` places those
        /// points in the car frame, which is where every test here lives, since
        /// each one is a statement about the track rather than about the sensor.
        ///
        /// The result and the debug views below are owned by the detector and
        /// stay valid until the next call.
        const std::vector<Detection> &detect(const Eigen::MatrixXf &cloud_lidar,
                                             const Eigen::Isometry3f &car_from_lidar);

        /// The points standing above the coarse ground, and which accepted cone
        /// each one fell into, or -1. Between them these show whether it is the
        /// ground estimate or the cone tests that need attention.
        const PointCloud &standing() const { return standing_; }
        const std::vector<int> &labels() const { return labels_; }

        /// Wall time of each stage of the last detect(), in milliseconds.
        struct StageTimes
        {
            double level_ms = 0.0;   ///< into the car frame
            double coarse_ms = 0.0;  ///< the throwaway ground
            double peaks_ms = 0.0;   ///< candidate selection
            double accept_ms = 0.0;  ///< one honest look at each candidate
        };
        const StageTimes &stage_times() const { return stage_; }
        double total_ms() const { return total_ms_; }
        /// Candidates that were looked at properly. Against the number of cones
        /// kept, this is the ratio that says whether the tests are still
        /// rejecting anything.
        int candidates() const { return candidates_; }

    private:
        /// A local ground plane, written relative to the candidate it belongs to.
        struct Plane
        {
            float a = 0.0f, b = 0.0f, c = 0.0f;   // z = a*(x-cx) + b*(y-cy) + c
            float roughness = 0.0f;
            bool valid = false;
            float at(float x, float y, float cx, float cy) const
            {
                return a * (x - cx) + b * (y - cy) + c;
            }
        };

        /// Uniform bucket grid over the working cloud, so the ring around a
        /// candidate is a few dozen buckets rather than a pass over the sweep.
        /// This is what keeps the cost proportional to the number of candidates
        /// instead of to the point density.
        void build_index();
        void gather(float cx, float cy, float radius, std::vector<int> &out) const;
        Plane fit_ground(float cx, float cy);
        std::array<double, 4> position_covariance(const Eigen::Vector2f &centroid,
                                                  int num_points) const;

        ConeDetectorParams params_;

        // Held across frames so a sweep-sized allocation happens once rather than
        // every 100 ms.
        Eigen::MatrixX3f car_;      ///< the working cloud, in the car frame
        std::vector<int> keep_;     ///< rows of car_ that survived the box and the ego cut
        std::vector<float> coarse_; ///< coarse ground height per kept row

        float index_cell_ = 0.5f;
        float index_x0_ = 0.0f, index_y0_ = 0.0f;
        int index_nx_ = 0, index_ny_ = 0;
        std::vector<int> index_start_;
        std::vector<int> index_items_;
        std::vector<int> scratch_ring_;

        std::vector<Detection> detections_;
        PointCloud standing_;
        std::vector<int> labels_;
        Eigen::Vector2f sensor_xy_ = Eigen::Vector2f::Zero();

        StageTimes stage_;
        double total_ms_ = 0.0;
        int candidates_ = 0;
    };

} // namespace mirena

#endif // MIRENA_PERCEPTION_CONE_DETECTOR_HPP
