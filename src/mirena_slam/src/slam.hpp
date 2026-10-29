#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "association.hpp"

namespace mirena
{

 /// One cone as perception saw it, in the car frame.
 struct Observation
 {
 double x, y;
 /// What perception could tell about the cone without reading its colour,
 /// which in a colourless pipeline is its size: `cone_big_orange` for the
 /// cones marking the start line, `unknown_cone` for the rest.
 std::string type;
 /// How well perception located it: 2x2 row-major [xx, xy, yx, yy], in the
 /// car frame. All zeros means it had no estimate and the default is used.
 std::array<double, 4> covariance{};
 };

 /// One cone in the published map.
 struct Landmark
 {
 double x, y;
 std::string type;
 /// How sure the map is this cone exists, as a probability. Derived from
 /// the same evidence that decides whether it is published at all, so it
 /// costs nothing to report and it is the only window downstream has into
 /// a map with no landmark ids on the wire.
 double confidence;
 };

 /// What one frame did, for the debug topic. Filled only when a debug sink is
 /// handed in, so watching costs nothing when nobody is.
 ///
 /// Between them these answer the question the map alone cannot: not "where are
 /// the cones" but "why is the map the shape it is". Associations with long
 /// innovations mean a pose drifting; a steady drip of `promoted` after the
 /// first lap means association failing to recognise cones already mapped;
 /// `candidates` filling up without promoting means the promotion test is stuck.
 struct SlamDebug
 {
 gtsam::Pose2 pose;
 /// Observation (in map frame) paired with the landmark it matched. The
 /// length of each pair is the innovation the gate accepted.
 std::vector<std::pair<gtsam::Point2, gtsam::Point2>> associations;
 /// Candidates that became landmarks this keyframe.
 std::vector<gtsam::Point2> promoted;
 /// Candidates alive right now, with their log-odds.
 std::vector<std::pair<gtsam::Point2, double>> candidates;
 /// Landmarks whose evidence has collapsed and which are no longer
 /// published. They remain in the graph.
 std::vector<gtsam::Point2> hidden;
 /// Landmarks near the car, with how many times each has been seen.
 std::vector<std::pair<gtsam::Point2, int>> sightings;
 };

 struct SlamParams
 {
 // -- Association --------------------------------------------------------

 /// Chi-square gate on the squared Mahalanobis distance. 5.99 is the 95th
 /// percentile for two degrees of freedom, 9.21 the 99th.
 ///
 /// One number, and it answers every "is this the thing I already have"
 /// question in the system: matching an observation to a landmark, matching
 /// it to a candidate, and deciding whether a candidate about to be promoted
 /// is really a cone the map already holds. They are the same question and
 /// they get the same answer.
 double mahalanobis_threshold = 9.21;
 /// Hard Euclidean backstop, metres, applied before the chi-square test.
 /// Sized so it does not bind in normal operation; it exists to stop a pose
 /// that has gone badly wrong from reaching across the track.
 double max_association_distance = 5.5;
 /// How far the car looks. Cones inside it are candidates for association
 /// and are expected to be seen; cones outside it are neither.
 double local_map_radius = 20.0;
 /// Half-angle of the sensor's field of view, radians. With
 /// local_map_radius this is the whole definition of "should have been
 /// seen" -- deliberately the same radius the associator uses, because a
 /// band where a cone can be matched but never judged is a band where
 /// nothing is ever counted against.
 double view_half_angle = 1.05;

 // -- Measurements -------------------------------------------------------

 /// Measurement sigma used when perception reports no covariance, metres.
 double default_measurement_sigma = 0.10;
 /// Floor on the measurement sigma, metres. Stops a confidently reported
 /// detection being trusted further than the extrinsics deserve.
 ///
 /// This is also, quietly, what keeps the association gate open. The gate's
 /// S can never be tighter than the measurement noise, so a landmark the
 /// graph has converged to a centimetre still carries a gate 20 cm across at
 /// short range and 50 cm at twenty metres -- and it scales with range,
 /// which the fixed covariance floor it replaced never did.
 double min_measurement_sigma = 0.03;

 // -- Existence ----------------------------------------------------------
 //
 // Whether a blob is a cone is a question about evidence, so it is answered
 // in log-odds. Two numbers, both measurable from a labelled run, replacing
 // a hand-tuned allowance that had to be argued about.

 /// How often perception reports a cone that is genuinely in view. Not 1.0
 /// and not close to it: a cone at range, or half behind another, is missed
 /// routinely, and the number that matters is the one measured over the
 /// whole of `local_map_radius` rather than the one from a good frame.
 double p_detect = 0.55;
 /// How often perception reports a cone where there is none.
 double p_false = 0.15;
 /// Evidence needed before a candidate is written into the graph.
 ///
 /// At the defaults above one sighting is worth log(0.55/0.05) = 2.40, so
 /// this is four clean looks. Measured on the resolution table in
 /// test_candidates: at 5.0 three looks were enough and duplicates got
 /// through at twelve metres; at 12.0 promotion was slow enough that a pair
 /// 0.4 m apart first seen at eighteen metres -- where the sensor genuinely
 /// cannot resolve it -- was merged before the car got close enough to tell,
 /// and never recovered. Losing a cone is the worse failure of the two, so
 /// the value sits at the low end of the range that keeps the table clean.
 double promote_log_odds = 9.0;
 /// Evidence needed to stay in the published map, once written.
 ///
 /// Strictly below promote_log_odds, and the gap is the point. Without
 /// hysteresis a landmark sitting on the threshold enters and leaves the map
 /// at frame rate, and the planner re-runs a discrete Delaunay walk on every
 /// change -- a stable map matters to it more than a marginally better one.
 double publish_log_odds = 1.0;

 // -- Promotion ----------------------------------------------------------

 /// How far apart two estimates must be, in multiples of the worst-direction
 /// standard deviation of their difference, before they are called two
 /// objects rather than one seen twice.
 ///
 /// This is the clause that lets the map hold cones standing closer together
 /// than the usual spacing without being told they exist. A candidate is
 /// only refused as a duplicate when it is *both* statistically consistent
 /// with a landmark *and* too close to be resolved from it; consistent but
 /// resolvable means keep both. Rejection is the only irreversible act in
 /// this class, and it is never taken under ambiguity.
 double separability_k = 2.0;
 /// How many keyframes a candidate may stay unresolved before it is promoted
 /// anyway. An extra cone in the map is a cosmetic defect; a missing one is
 /// a hole in the track, so the tie is broken towards existing.
 int max_candidate_age = 20;
 /// Most measurements a promotion may bring into the graph at once. A
 /// candidate that lingered and accumulated sixty sightings would otherwise
 /// attach factors reaching sixty keyframes back, which is a loop closure on
 /// an ordinary keyframe.
 int max_promotion_factors = 10;

 // -- When a pose is worth its keep --------------------------------------

 /// How far the odometry may drift, in metres, before the pose needs
 /// pinning with a keyframe of its own.
 ///
 /// 0.30, not the 0.15 this used to be, and the reason is map quality
 /// rather than cost. Measured over the 270 s and thirteen laps of
 /// rosbag2_2026_08_05-20_26_23, replayed through the trackdrive
 /// pipeline:
 ///
 /// 0.15 0.30
 /// landmarks (graph) 210 103
 /// published 129 92
 /// hidden 81 11
 /// pairs under 0.9 m 8 0
 /// worst gap in cover 9.4 m 2.6 m
 ///
 /// The tighter value looks better on the landmark count and is not: the
 /// extra hundred are the first lap, laid down before the graph had
 /// converged and left standing beside the cones the later laps agreed
 /// on. That is what the 81 hidden and the eight touching pairs are, and
 /// the 9.4 m is a hole in the middle of the driven line where the smear
 /// left nothing the car could see. At 0.30 the cones line the whole lap
 /// and nothing is ever further than a track width away.
 ///
 /// Keyframing more often than the odometry deserves is not free: each
 /// pose is another chance to write down the same cone in a slightly
 /// different place before the graph has enough constraints to know
 /// better. The cost went the other way, slightly -- update p99 406 ->
 /// 660 ms -- so this buys a map, not a budget; see keyframe_max_frames.
 double keyframe_odom_sigma = 0.30;
 /// Frames between keyframes, at most, whatever else is true. This is what
 /// keeps map->odom fresh rather than what keeps the map accurate.
 ///
 /// This is the floor under the size of the graph, and no tuning gets
 /// under it: at 8 Hz perception it is a permanent pose every half
 /// second, so a ten-minute run carries at least twelve hundred of them
 /// however well the map is known. Loosening keyframe_odom_sigma does not
 /// help, because the two clauses substitute -- measured on
 /// rosbag2_2026_08_05-20_26_23, doubling it moved 97 keyframes off the
 /// drift clause and onto this one and left the total within 6%.
 ///
 /// That floor is what makes a long run expensive. Over the same replay
 /// the iSAM2 update grew from 41 ms across the first half of 981 poses
 /// to 188 ms across the second, and the node reported PARTIAL from
 /// t=213 s -- the estimator falling behind the perception it is fed.
 ///
 /// Bounding that needs a structural answer rather than a threshold:
 /// either the graph stops growing (fixed-lag smoothing, marginalising
 /// old poses) or the pose stops needing the graph once the map has
 /// converged (localise against the finished map instead of extending
 /// it). `pinned` was the beginning of the second answer, but it is true
 /// on only 20% of frames here and the drift clause covers most of the
 /// rest, so it never gets the chance to suppress anything.
 int keyframe_max_frames = 4;
 };

 /// Landmark SLAM over an incremental factor graph.
 ///
 /// Two tiers, and the split between them is the whole design.
 ///
 /// The **graph** holds poses and landmarks and is append-only. Nothing is ever
 /// deleted from it, nothing is ever merged in it, and no factor is ever
 /// removed. That is not a limitation worked around; it is the property being
 /// bought, because it is what makes an incremental solver safe to use here. A
 /// landmark's measurements were real observations of *something*, so they stay
 /// valid constraints on the trajectory whether or not the thing turned out to
 /// be a cone.
 ///
 /// The **candidate tier** is where everything reversible happens. A detection
 /// that matches nothing becomes a candidate: a small filter living outside the
 /// graph, gathering evidence. Only when a candidate has earned its place *and*
 /// is still not explained by a landmark does it become a graph variable. So the
 /// things that used to require surgery -- deleting a phantom, merging a
 /// duplicate -- are decisions taken before anything is written down, at the one
 /// moment they are free.
 ///
 /// Between them these replace a sliding window whose lossy marginalisation had
 /// to be propped up by covariance floors, a creation veto, a merge pass and a
 /// deletion rule. What is left is one gate, one persistence filter, and iSAM2.
 ///
 /// Nothing here knows what a cone looks like. There is no minimum spacing and
 /// no branch on a reported type: association, promotion and duplicate rejection
 /// are all the same question, answered by the separation measured against the
 /// uncertainty of that separation. Types ride along as votes and are published,
 /// and that is all they do. Every fixed distance that used to live here was a
 /// guess about track layout standing in for a covariance nobody had computed.
 ///
 /// What is still approximate: the association gate treats the pose and the
 /// landmark as independent where they are in fact correlated, so S is an upper
 /// bound and the gate is slightly wider than it need be. That errs towards
 /// matching rather than towards spawning duplicates, which is the safe
 /// direction, and the exact form would cost a joint marginal per pair.
 class Slam
 {
 public:
 explicit Slam(const gtsam::Pose2 &initial_raw_pose, SlamParams params = {});

 /// Matches one frame of cones against the map, feeds the candidates, and
 /// judges what is and is not still there. Call on every frame perception
 /// delivers.
 ///
 /// Deliberately separate from add_keyframe, and that separation is the
 /// point. Whether a blob is a cone is decided by looking at it repeatedly,
 /// and detections arrive several times faster than the car moves far enough
 /// to be worth a new pose. Tying the two together threw away most of the
 /// evidence: a phantom got judged on the four or five frames that happened
 /// to coincide with a keyframe rather than on the forty the sensor gave.
 ///
 /// Touches no factors. The graph only ever learns at a keyframe.
 ///
 /// @param raw_odom absolute raw odometry for this frame
 /// @param odom_sigmas uncertainty the odometry has accumulated since the
 /// last keyframe, body frame, [x, y, theta]. The same numbers the
 /// keyframe's own factor will be given, so the gate widens by
 /// exactly what the optimiser will later be told to believe rather
 /// than by a separate guess of its own.
 void observe(const gtsam::Pose2 &raw_odom,
 const std::vector<Observation> &observations,
 const gtsam::Vector3 &odom_sigmas,
 SlamDebug *debug = nullptr);

 /// Whether the frame just observed is worth a pose of its own.
 ///
 /// A pose in an incremental graph is permanent, so it should be added when
 /// it carries information and not merely when the car has moved. It carries
 /// information when the odometry has drifted far enough to need pinning, or
 /// when there is new structure waiting to be written down. It carries very
 /// little when the car is driving past cones the map already knows better
 /// than the sensor can measure them -- on a tenth lap that is every cone,
 /// and the pose chain stops growing on its own.
 bool wants_keyframe() const { return wants_keyframe_; }

 /// Commits the frame from the most recent observe() as a keyframe: adds its
 /// pose, promotes whatever candidates have earned it, adds the measurements
 /// of landmarks it saw, and re-solves incrementally.
 void add_keyframe(const gtsam::Vector3 &odom_sigmas, SlamDebug *debug = nullptr);

 /// The published map: every landmark whose evidence still stands. Returned
 /// by value so a caller can hand it off and drop the lock.
 std::vector<Landmark> map_snapshot() const;
 gtsam::Pose2 get_correction() const { return map_to_odom_; }

 double get_last_update_elapsed_ms() const { return last_update_ms_; }
 int get_last_matched_count() const { return last_matched_; }
 int get_last_promoted_count() const { return last_promoted_; }
 int get_last_hidden_count() const { return last_hidden_; }
 /// Detections the last frame reported from beyond local_map_radius, and
 /// which were therefore ignored. Perception's max_range is the larger of
 /// the two numbers, so this is normally non-zero and that is correct; it
 /// is on the wire because a count that is a large fraction of the frame
 /// means the two radii have drifted far enough apart to be throwing away
 /// cones the map could have used.
 int get_last_out_of_range_count() const { return last_out_of_range_; }
 int get_candidate_count() const { return static_cast<int>(candidates_.size()); }
 int get_landmark_count() const { return static_cast<int>(landmarks_.size()); }
 int get_pose_count() const { return next_keyframe_; }
 /// What has been forcing keyframes, since construction.
 ///
 /// The pose chain is permanent and dominates the cost of a long run, so
 /// being able to say *why* it grew -- new cones, drifting odometry, or
 /// merely the frame timer -- is what turns "SLAM got slow after four
 /// minutes" into a decision about which threshold to move.
 int get_keyframes_from_structure() const { return kf_structure_; }
 int get_keyframes_from_drift() const { return kf_drift_; }
 int get_keyframes_from_timer() const { return kf_timer_; }
 /// Fraction of frames where the map already determined the pose, so a
 /// keyframe of its own would have bought little.
 double get_pinned_fraction() const
 {
 return frames_seen_ > 0
 ? static_cast<double>(frames_pinned_) / frames_seen_
 : 0.0;
 }
 /// Straight out of ISAM2Result, so a loop-closure spike is visible in the
 /// heartbeat as work done rather than as an unexplained slow keyframe.
 int get_variables_reeliminated() const { return last_reeliminated_; }
 int get_cliques() const { return last_cliques_; }
 /// The map as a full batch solve of the same factor graph would have it.
 ///
 /// Only for tests and diagnosis, and expensive -- it re-solves everything
 /// from scratch. Its whole purpose is to answer one question when the map
 /// looks wrong: is the graph wrong, or is the incremental solver failing
 /// to converge on a graph that is right? Those have completely different
 /// fixes and are otherwise very hard to tell apart.
 std::vector<Landmark> batch_snapshot() const;

 /// False once the solver has thrown. The estimate cannot be trusted after
 /// that and the caller is expected to say so loudly rather than carry on.
 bool healthy() const { return healthy_; }

 private:
 /// Which type has been reported for one landmark most often.
 ///
 /// A plain tally, because a colourless detector has nothing graded to say --
 /// a cluster is tall enough to be a big cone or it is not. Voting rather
 /// than latching means a single misread costs one vote.
 struct TypeVote
 {
 std::map<std::string, int> counts;
 std::string best;
 int best_count = 0;

 void add(const std::string &type);
 void absorb(const TypeVote &other);
 };

 /// A cone the graph has committed to.
 struct LandmarkRecord
 {
 gtsam::Point2 position; ///< map frame, from the solver
 gtsam::Matrix2 covariance; ///< marginal, map frame
 double log_odds = 0.0;
 int sightings = 0;
 TypeVote types;
 /// Keyframe its marginal was last refreshed at, so the refresh can be
 /// budgeted round-robin instead of recomputing the whole map.
 int marginal_keyframe = -1;
 };

 /// A cone the graph has not committed to, and may never.
 ///
 /// Its position is held in the body frame of the pose it was first seen
 /// from, not in the map frame, and that is not a detail. The error the
 /// promotion test has to reason about is a common-mode pose error shared
 /// with every existing landmark, which no map-frame covariance can express:
 /// fold the pose uncertainty in and nothing ever promotes, leave it out and
 /// the test wrongly collapses genuine pairs. Anchored, the only uncertainty
 /// that accumulates is the *relative* drift over the handful of keyframes
 /// the candidate lives, which is centimetres.
 ///
 /// It also fixes the estimate for free: on promotion the map-frame position
 /// is recomputed from the current estimate of the anchor pose, so a
 /// candidate that lived across a loop closure lands where the closure put
 /// it rather than where the map used to think it was.
 struct Candidate
 {
 int anchor = 0; ///< index of the pose it is written in
 gtsam::Pose2 anchor_pose; ///< that pose's estimate, refreshed each keyframe
 gtsam::Point2 mu; ///< in the anchor's body frame
 gtsam::Matrix2 P;
 /// Relative pose covariance from the anchor to the last keyframe.
 gtsam::Matrix3 drift = gtsam::Matrix3::Zero();
 double log_odds = 0.0;
 int born = 0; ///< keyframe index it was created at
 TypeVote types;

 /// One measurement kept for replay into the graph on promotion. Only
 /// sightings taken *at* a keyframe are kept, because only those have a
 /// pose to attach to.
 struct Sighting
 {
 int keyframe;
 gtsam::Point2 local;
 gtsam::Matrix2 covariance;
 };
 std::vector<Sighting> sightings;
 };

 /// The frame observe() last matched, waiting to become a keyframe.
 struct Pending
 {
 gtsam::Pose2 raw_odom;
 gtsam::Pose2 estimate;
 /// Landmarks seen, where they were seen, and how well.
 std::vector<int> ids;
 std::vector<gtsam::Point2> local;
 std::vector<gtsam::Matrix2> covariance;
 /// Candidates seen, by id, and the same.
 ///
 /// By id and never by position: candidates are created and dropped
 /// inside the same observe() that fills this, so an index into the
 /// store is stale before it is read. Attaching a sighting to the wrong
 /// candidate that way puts a measurement of one cone on another and
 /// the graph believes it -- which is a permanent, silent corruption,
 /// since nothing is ever removed from the graph.
 std::vector<int> candidate_ids;
 std::vector<gtsam::Point2> candidate_local;
 std::vector<gtsam::Matrix2> candidate_covariance;
 bool valid = false;
 };

 static gtsam::Symbol pose_key(int index) { return gtsam::Symbol('x', index); }
 static gtsam::Symbol landmark_key(int id) { return gtsam::Symbol('l', id); }

 /// Log-odds added by seeing a thing, and by failing to see one that was in
 /// plain view. Both from p_detect and p_false, so the asymmetry between
 /// them is a consequence of the sensor rather than a tuning choice.
 double seen_evidence() const;
 double missed_evidence() const;

 /// Whether the car was pointed at this point and close enough to see it.
 bool in_view(const gtsam::Pose2 &pose, const gtsam::Point2 &p) const;

 /// Where a candidate sits in the map frame right now, and how well it is
 /// known there.
 Target candidate_target(const Candidate &c, const gtsam::Pose2 &anchor_pose,
 const gtsam::Matrix3 &drift_now) const;

 /// Whether this is a cone the graph already holds.
 ///
 /// Asked the way association asks it, and with the same ingredients: could
 /// one cone, looked at once, have produced both of these estimates? So the
 /// spread being tested against is both estimates' own uncertainty *plus*
 /// what a single look costs, `resolving`.
 ///
 /// That last term is not padding. Two estimates built by averaging many
 /// looks from nearly the same place have covariances that shrank as 1/n,
 /// but the across-beam error of a cone at range does not re-draw itself
 /// each frame -- it depends on which part of the cone the beams caught --
 /// so the average is not n times better than one look. Left out, two
 /// estimates of one cone 0.36 m apart at sixteen metres each claim to be
 /// known to a centimetre and are declared two cones.
 ///
 /// Consistent *and* unresolvable is a duplicate. Consistent but resolvable
 /// is two cones standing close together. Inconsistent is two cones. Only
 /// the first is refused, and refusal is the one irreversible act here.
 bool is_duplicate(const gtsam::Point2 &position, const gtsam::Matrix2 &covariance,
 const gtsam::Matrix2 &resolving, int *of_landmark) const;

 /// Writes earned candidates into the graph. Returns the factors and values
 /// to hand to iSAM2 rather than touching it, so a promotion and the
 /// keyframe it happens on are one update.
 void promote_candidates(int keyframe, const gtsam::Pose2 &keyframe_pose,
 gtsam::NonlinearFactorGraph &graph, gtsam::Values &values,
 std::vector<gtsam::Key> &fresh, SlamDebug *debug);

 /// Pulls the solver's answer back into the landmark records, and refreshes
 /// as many marginals as the budget allows.
 void read_back(const gtsam::ISAM2Result &result, int keyframe);

 SlamParams params_;
 gtsam::ISAM2 isam_;

 Pending pending_;
 std::map<int, LandmarkRecord> landmarks_;
 std::map<int, Candidate> candidates_;

 /// Where the last keyframe put the car, and what the odometry read there.
 /// Everything between keyframes is measured from these.
 gtsam::Pose2 keyframe_pose_;
 gtsam::Pose2 last_raw_odom_;
 gtsam::Pose2 map_to_odom_;

 /// Marginal covariance of the newest pose, body frame.
 ///
 /// The gate needs this and cannot do without it, however tempting it is to
 /// use only the odometry accumulated since the last keyframe. That shorter
 /// answer is right for a cone seen a moment ago -- the solve just made the
 /// pose and that cone jointly consistent -- and badly wrong for one last
 /// seen a lap ago, whose position was fixed before all the drift since.
 /// With the tight version the returning car cannot associate anything,
 /// every cone on the second lap looks new, and the map doubles: measured
 /// at 221 landmarks on a 128-cone track. Loop closure has to start with a
 /// gate wide enough to admit the cone that closes it.
 gtsam::Matrix3 pose_cov_ = gtsam::Matrix3::Identity() * 1e-9;

 /// Set from the last observe(), read by add_keyframe.
 gtsam::Matrix3 pending_odom_cov_ = gtsam::Matrix3::Zero();
 bool wants_keyframe_ = false;
 int frames_since_keyframe_ = 0;
 int frames_seen_ = 0;
 int frames_pinned_ = 0;
 int kf_structure_ = 0;
 int kf_drift_ = 0;
 int kf_timer_ = 0;

 int next_keyframe_ = 0;
 int next_landmark_ = 0;
 int next_candidate_ = 0;
 /// Landmark id the round-robin marginal sweep last refreshed, or -1
 /// before it has run. An id rather than an index into the map, so a
 /// promotion inserting ahead of it does not shift the sweep along.
 int marginal_cursor_ = -1;
 /// Landmarks the solver has touched whose marginal has not been recomputed
 /// yet. Drained a budget at a time so a loop closure, which marks most of
 /// the map at once, cannot spend a whole frame interval on covariances.
 std::deque<int> stale_marginals_;

 double last_update_ms_ = 0.0;
 int last_matched_ = 0;
 int last_promoted_ = 0;
 int last_hidden_ = 0;
 int last_out_of_range_ = 0;
 int last_reeliminated_ = 0;
 int last_cliques_ = 0;
 bool healthy_ = true;
 };

} // namespace mirena


