#include "slam.hpp"

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>

namespace mirena
{

 namespace
 {
 /// How many cones it takes to hold a pose still.
 ///
 /// Two fix a 2D pose and its heading; the third says whether the first two
 /// agree. A frame that has this many cones it already knows better than the
 /// sensor can measure them does not need a pose of its own -- whenever one
 /// is next added, these will pin it. This is a fact about the geometry, not
 /// a tuning knob.
 constexpr int kPoseDeterminingCones = 3;

 /// Marginals refreshed per keyframe beyond the ones the solve just touched.
 ///
 /// Bounds the per-keyframe cost independent of how big the map has grown.
 /// A landmark's uncertainty only changes when something touches it, and the
 /// solver says which ones those were; the rest are swept round-robin so
 /// nothing goes stale for ever.
 constexpr size_t kMarginalBudget = 12;

 /// Huber's standard tuning: the residual, in sigmas, beyond which the loss
 /// becomes linear. 95% efficiency against a true Gaussian.
 constexpr double kHuberK = 1.345;

 gtsam::ISAM2Params isam_settings()
 {
 gtsam::ISAM2Params p;

 // Dogleg rather than Gauss-Newton. The batch solver this replaced was
 // Levenberg-Marquardt, whose damping is what kept a bad step from
 // running away; plain Gauss-Newton under an incremental solver has no
 // such guard, and a trust region is the incremental equivalent.
 p.optimizationParams = gtsam::ISAM2DoglegParams();

 // One relinearisation threshold cannot serve metres and radians at
 // once: a value sensible for x and y lets heading drift several degrees
 // from its linearisation point, and at ten metres a degree of heading
 // is a fifth of a metre of lateral error -- more than the measurement
 // noise the whole gate is calibrated against. Per symbol, then.
 gtsam::FastMap<char, gtsam::Vector> thresholds;
 thresholds['x'] = (gtsam::Vector(3) << 0.05, 0.05, 0.005).finished();
 thresholds['l'] = (gtsam::Vector(2) << 0.03, 0.03).finished();
 p.relinearizeThreshold = thresholds;

 // Not the default of 10. At this keyframe rate that would be one
 // relinearisation check every nine metres driven, and both the factors
 // and the marginals the association gate reads would be computed at a
 // linearisation point that far behind. The check itself is a scan over
 // a delta the wildfire threshold keeps sparse, so in steady driving it
 // finds nothing and costs nothing.
 p.relinearizeSkip = 1;

 p.factorization = gtsam::ISAM2Params::CHOLESKY;
 p.evaluateNonlinearError = false; // walks every factor; never in the loop
 p.findUnusedFactorSlots = false; // nothing is ever removed
 return p;
 }

 /// The measurement of a landmark from a pose: where the cone should appear
 /// in the car frame if both the pose and the landmark are where the graph
 /// says they are.
 ///
 /// Plain Gaussian, deliberately. A robust kernel here would be re-weighted
 /// only when the factor is relinearised, so a weight that collapsed on one
 /// bad early linearisation survives in the cached linear factor -- and if
 /// the pose later corrects, nothing ever exceeds the threshold that would
 /// bring it back. The measurement is discarded permanently and silently.
 /// Outlier rejection belongs where it is reversible: a detection only
 /// reaches this function after surviving a gated one-to-one assignment and
 /// a log-odds confirmation, so what arrives here is already truncated.
 gtsam::NonlinearFactor::shared_ptr measurement_factor(
 int pose_index, int landmark_id, const gtsam::Point2 &local,
 const gtsam::Matrix2 &covariance)
 {
 const gtsam::Expression<gtsam::Pose2> x_(gtsam::Symbol('x', pose_index));
 const gtsam::Expression<gtsam::Point2> l_(gtsam::Symbol('l', landmark_id));
 return boost::make_shared<gtsam::ExpressionFactor<gtsam::Point2>>(
 gtsam::noiseModel::Gaussian::Covariance(covariance), local,
 gtsam::transformTo(x_, l_));
 }
 } // namespace

 void Slam::TypeVote::add(const std::string &type)
 {
 if (type.empty())
 return;

 const int count = ++counts[type];
 if (count > best_count)
 {
 best_count = count;
 best = type;
 }
 }

 void Slam::TypeVote::absorb(const TypeVote &other)
 {
 for (const auto &[type, count] : other.counts)
 {
 if (type.empty() || count <= 0)
 continue;

 const int total = (counts[type] += count);
 if (total > best_count)
 {
 best_count = total;
 best = type;
 }
 }
 }

 double Slam::seen_evidence() const
 {
 return std::log(params_.p_detect / params_.p_false);
 }

 double Slam::missed_evidence() const
 {
 return std::log((1.0 - params_.p_detect) / (1.0 - params_.p_false));
 }

 Slam::Slam(const gtsam::Pose2 &initial_raw_pose, SlamParams params)
 : params_(params), isam_(isam_settings())
 {
 last_raw_odom_ = initial_raw_pose;
 keyframe_pose_ = gtsam::Pose2(0, 0, 0);

 // The same composition add_keyframe uses, and it has to be done here too.
 // The map frame is the car's starting pose, so the correction at t=0 is
 // the inverse of whatever the odometry happened to read there -- which on
 // a bag replay or a warm start is tens of metres from the origin. Left at
 // identity, every transform broadcast before the first keyframe lands is
 // wrong by exactly that, and the node's filter takes its *first* sample
 // as-is and then eases, so the error does not merely last one frame: it
 // seeds the filter and decays over the time constant.
 map_to_odom_ = keyframe_pose_.compose(last_raw_odom_.inverse());

 // The map frame is defined to be the car's starting pose, so the first
 // pose sits at the origin and is held there.
 //
 // A millimetre, not the micrometre this used to be. Information of 1e12
 // against a system whose other entries are of order 1e3 is a condition
 // number that Cholesky will eventually refuse, and unlike the old sliding
 // window this pose is in the tree for the rest of the run. Nothing
 // downstream needs the origin pinned tighter than this; the known-track
 // planner's own tolerance is five metres.
 gtsam::NonlinearFactorGraph graph;
 gtsam::Values values;
 const gtsam::Symbol first = pose_key(next_keyframe_++);
 values.insert(first, keyframe_pose_);
 graph.addPrior(first, keyframe_pose_, gtsam::noiseModel::Isotropic::Sigma(3, 1e-3));
 isam_.update(graph, values);
 }

 bool Slam::in_view(const gtsam::Pose2 &pose, const gtsam::Point2 &p) const
 {
 const gtsam::Point2 local = pose.transformTo(p);
 return local.norm() < params_.local_map_radius && local.x() > 0.0 &&
 std::abs(std::atan2(local.y(), local.x())) < params_.view_half_angle;
 }

 Target Slam::candidate_target(const Candidate &c, const gtsam::Pose2 &anchor_pose,
 const gtsam::Matrix3 &drift_now) const
 {
 gtsam::Matrix23 Hpose;
 gtsam::Matrix2 Hpoint;
 const gtsam::Point2 world = anchor_pose.transformFrom(c.mu, Hpose, Hpoint);
 return Target{world, Hpoint * c.P * Hpoint.transpose() +
 Hpose * drift_now * Hpose.transpose()};
 }

 void Slam::observe(const gtsam::Pose2 &raw_odom,
 const std::vector<Observation> &observations,
 const gtsam::Vector3 &odom_sigmas,
 SlamDebug *debug)
 {
 // Once the solver has given up, every number this class holds is
 // furniture and the only honest thing to do with a new frame is nothing.
 // add_keyframe already refuses, but refusing there alone left this
 // running: log-odds kept accumulating, landmarks kept crossing
 // publish_log_odds and vanishing from map_snapshot(), and candidates kept
 // churning. The positions froze and the published *set* did not, which
 // made a liar of the caller's "the map is frozen at its last good
 // values".
 if (!healthy_)
 return;

 // 1. Where the odometry says we are, measured from the last keyframe, and
 // how much it could have drifted getting here. That drift is the
 // caller's own number -- the same one the keyframe factor will carry --
 // so the gate and the graph cannot disagree about how good the odometry
 // is.
 const gtsam::Pose2 delta = last_raw_odom_.between(raw_odom);
 const gtsam::Pose2 pose = keyframe_pose_.compose(delta);

 gtsam::Matrix3 rel = gtsam::Matrix3::Zero();
 rel.diagonal() = odom_sigmas.cwiseProduct(odom_sigmas);
 pending_odom_cov_ = rel;

 // Where the car is, as well as the graph knows it, carried forward over
 // the odometry since the last solve. Both terms matter: the first is what
 // lets a cone last seen a lap ago still be recognised, the second is what
 // the odometry has added since.
 const gtsam::Matrix3 adjoint = delta.inverse().AdjointMap();
 const gtsam::Matrix3 body = adjoint * pose_cov_ * adjoint.transpose() + rel;

 gtsam::Matrix3 to_map = gtsam::Matrix3::Identity();
 to_map.topLeftCorner<2, 2>() = pose.rotation().matrix();
 const gtsam::Matrix3 pose_cov = to_map * body * to_map.transpose();

 // 2. How well each cone was located, straight from perception.
 std::vector<gtsam::Point2> local;
 std::vector<gtsam::Matrix2> measurement_cov;
 local.reserve(observations.size());
 measurement_cov.reserve(observations.size());
 for (const Observation &o : observations)
 {
 local.emplace_back(o.x, o.y);
 measurement_cov.push_back(measurement_covariance(
 o.covariance, params_.default_measurement_sigma,
 params_.min_measurement_sigma));
 }

 // 3. Against the map first, and only then against the candidates.
 //
 // The order is the point, and it is not an optimisation. A candidate
 // exists because some earlier observation was not explained by the
 // map; an observation that *is* explained by the map is therefore no
 // evidence for it. Let the two tiers compete on equal terms and a
 // candidate sitting on top of a landmark -- which the gate produces by
 // construction, since a 95% test rejects one good observation in
 // twenty -- takes it in turns with that landmark to be fed, and never
 // starves. It then promotes into a duplicate of the cone it was born
 // beside. Matching the map first means the candidate only ever sees
 // what the map genuinely cannot account for.
 const gtsam::Matrix3 &drift_step = adjoint;

 std::vector<Target> map_targets;
 std::vector<int> map_ids;
 for (const auto &[id, record] : landmarks_)
 {
 if ((record.position - pose.translation()).norm() >= params_.local_map_radius)
 continue;
 map_targets.push_back(Target{record.position, record.covariance});
 map_ids.push_back(id);
 }

 const std::vector<int> to_landmark = associate(
 pose, pose_cov, map_targets, local, measurement_cov,
 params_.mahalanobis_threshold, params_.max_association_distance);

 std::vector<size_t> residue;
 std::vector<gtsam::Point2> residue_local;
 std::vector<gtsam::Matrix2> residue_cov;
 last_out_of_range_ = 0;
 for (size_t i = 0; i < observations.size(); ++i)
 {
 if (!to_landmark.empty() && to_landmark[i] >= 0)
 continue;

 // Beyond the radius this class works in, and therefore not something
 // it can reason about at all: both target lists above are filtered by
 // local_map_radius, so a detection out here can match neither a
 // landmark nor a candidate however many times it is seen.
 //
 // Letting it fall through to candidate creation was not merely
 // wasteful, it was a ratchet: the candidate it spawned this frame is
 // out of range next frame too, so it is still not a target and the
 // same cone spawns another. Nor did they decay -- in_view() is
 // bounded by the same radius, so they took no missed-evidence
 // penalty and simply sat at one sighting until max_candidate_age
 // expired them, seconds later.
 //
 // Two ways in, and the second is the one that bites hardest. The
 // obvious one is reach: perception claims cones to 30 m against this
 // 20, and every cone in the band between is reported every frame.
 // The other needs no such gap and survives even when the two numbers
 // are equal, which is why the fixture never caught it -- the test is
 // the *reported* position, and across-beam noise at twenty metres is
 // a sigma of about 0.3 m, so a real cone just inside the edge is
 // routinely reported just outside it.
 //
 // Measured on the exported tracks: with this clause absent, track4
 // maps 247 landmarks for 164 cones, 207 of them spurious, and
 // registers 6.4 m out. With it, 164 for 164, none spurious, 0.15 m.
 // track2 is not fixed but halves its registration error. The cost of
 // the old behaviour was never the wasted candidate_target() per
 // frame; it was that those candidates went on to promote.
 if (local[i].norm() >= params_.local_map_radius)
 {
 ++last_out_of_range_;
 continue;
 }

 residue.push_back(i);
 residue_local.push_back(local[i]);
 residue_cov.push_back(measurement_cov[i]);
 }

 std::vector<Target> candidate_targets;
 std::vector<int> candidate_ids;
 for (const auto &[id, cand] : candidates_)
 {
 const gtsam::Matrix3 drift_now =
 drift_step * cand.drift * drift_step.transpose() + rel;
 const Target t = candidate_target(cand, cand.anchor_pose, drift_now);
 if ((t.position - pose.translation()).norm() >= params_.local_map_radius)
 continue;
 candidate_targets.push_back(t);
 candidate_ids.push_back(id);
 }

 const std::vector<int> to_candidate = associate(
 pose, pose_cov, candidate_targets, residue_local, residue_cov,
 params_.mahalanobis_threshold, params_.max_association_distance);

 // 4. Fold the frame in.
 pending_ = Pending{};
 pending_.raw_odom = raw_odom;
 pending_.estimate = pose;
 pending_.valid = true;

 last_matched_ = 0;
 std::unordered_set<int> seen_landmarks;
 std::unordered_set<int> seen_candidates;
 int well_known = 0;

 // Whatever the map explained.
 for (size_t i = 0; i < observations.size(); ++i)
 {
 if (to_landmark.empty() || to_landmark[i] < 0)
 continue;

 const int id = map_ids[static_cast<size_t>(to_landmark[i])];
 LandmarkRecord &record = landmarks_.at(id);
 ++last_matched_;
 seen_landmarks.insert(id);
 record.types.add(observations[i].type);
 ++record.sightings;

 // A cone the map knows better than the sensor can measure it. Enough
 // of these in one frame and the pose is already determined, whether
 // or not it is given a keyframe of its own.
 if (record.covariance.trace() < measurement_cov[i].trace())
 ++well_known;

 pending_.ids.push_back(id);
 pending_.local.push_back(local[i]);
 pending_.covariance.push_back(measurement_cov[i]);

 if (debug)
 debug->associations.emplace_back(pose.transformFrom(local[i]),
 record.position);
 }

 // Whatever it could not.
 for (size_t r = 0; r < residue.size(); ++r)
 {
 const size_t i = residue[r];

 if (!to_candidate.empty() && to_candidate[r] >= 0)
 {
 // A candidate, gathering evidence outside the graph.
 const int cid = candidate_ids[static_cast<size_t>(to_candidate[r])];
 Candidate &cand = candidates_.at(cid);
 seen_candidates.insert(cid);
 cand.types.add(observations[i].type);

 // Move the measurement into the frame the candidate is written
 // in, and fold it in. Everything here is relative to the anchor
 // pose, so the absolute drift of the trajectory -- which this
 // candidate shares with every landmark it will be compared
 // against -- never enters and never has to be taken back out.
 const gtsam::Matrix3 drift_now =
 drift_step * cand.drift * drift_step.transpose() + rel;
 const gtsam::Pose2 anchor_to_now = cand.anchor_pose.between(pose);
 gtsam::Matrix23 Hpose;
 gtsam::Matrix2 Hpoint;
 const gtsam::Point2 z = anchor_to_now.transformFrom(local[i], Hpose, Hpoint);
 const gtsam::Matrix2 R = Hpoint * measurement_cov[i] * Hpoint.transpose() +
 Hpose * drift_now * Hpose.transpose();

 const gtsam::Matrix2 S = cand.P + R;
 if (S.determinant() > 0.0)
 {
 const gtsam::Matrix2 K = cand.P * S.inverse();
 cand.mu = cand.mu + K * (z - cand.mu);
 cand.P = (gtsam::Matrix2::Identity() - K) * cand.P;
 cand.P = (0.5 * (cand.P + cand.P.transpose())).eval();
 }

 pending_.candidate_ids.push_back(cid);
 pending_.candidate_local.push_back(local[i]);
 pending_.candidate_covariance.push_back(measurement_cov[i]);
 continue;
 }

 // Explained by nothing the map or the candidate store holds, so it is
 // something new. Nothing is checked here beyond the gates that already
 // ran: this costs a candidate, which is free and reversible, not a
 // graph variable. Every proximity veto that used to guard this point
 // was a guess about how far apart cones are laid, and the guess was
 // wrong exactly where cones stand close together.
 Candidate fresh;
 fresh.anchor = next_keyframe_ - 1;
 fresh.anchor_pose = keyframe_pose_;

 // Into the anchor's body frame, and the covariance with it.
 //
 // Rotating the measurement but not its covariance -- which is what
 // this used to do -- leaves an ellipse that is long across the beam
 // pointing the wrong way by however far the car has turned since the
 // keyframe. The trace is unchanged, so nothing gets more or less
 // certain; the anisotropy simply stops describing the sensor. The
 // update path above has always done this properly, and this is the
 // same Jacobian: GTSAM's Hpoint for transformFrom is the rotation.
 const gtsam::Pose2 anchor_to_now = keyframe_pose_.between(pose);
 const gtsam::Matrix2 into_anchor = anchor_to_now.rotation().matrix();
 fresh.mu = anchor_to_now.transformFrom(local[i]);
 fresh.P = into_anchor * measurement_cov[i] * into_anchor.transpose() +
 rel.topLeftCorner<2, 2>();
 fresh.drift = gtsam::Matrix3::Zero();
 fresh.log_odds = seen_evidence();
 fresh.born = next_keyframe_ - 1;
 fresh.types.add(observations[i].type);
 const int cid = next_candidate_++;
 candidates_.emplace(cid, std::move(fresh));
 seen_candidates.insert(cid);
 }

 // 5. Evidence, for and against. A thing in plain view that was not reported
 // counts against itself; a thing nothing was pointed at does not.
 const double up = seen_evidence();
 const double down = missed_evidence();
 const double cap = 2.0 * params_.promote_log_odds;

 // Calcular incertidumbre de posición actual (desviación estándar estimada)
 const double pose_pos_std = std::sqrt(pose_cov(0,0) + pose_cov(1,1));

 last_hidden_ = 0;
 for (auto &[id, record] : landmarks_)
 {
 if (seen_landmarks.count(id))
 record.log_odds += up;
 else if (in_view(pose, record.position))
 {
 // Si la incertidumbre de la pose es alta (>0.6m), no podemos estar seguros de que
 // el cono no esté ahí; es probable que la pose estimada esté desviada.
 if (pose_pos_std < 0.6)
 
 
 {
 record.log_odds += down;
 }
 }
 record.log_odds = std::clamp(record.log_odds, -cap, cap);
 if (record.log_odds < params_.publish_log_odds)
 {
 ++last_hidden_;
 if (debug)
 debug->hidden.push_back(record.position);
 }
 }

 for (auto it = candidates_.begin(); it != candidates_.end();)
 {
 Candidate &cand = it->second;
 const gtsam::Matrix3 drift_now =
 drift_step * cand.drift * drift_step.transpose() + rel;
 const gtsam::Point2 world =
 candidate_target(cand, cand.anchor_pose, drift_now).position;

 if (seen_candidates.count(it->first))
 cand.log_odds += up;
 else if (in_view(pose, world))
 cand.log_odds += down;
 cand.log_odds = std::min(cand.log_odds, cap);

 if (cand.log_odds < 0.0)
 {
 // Looked at and not there. Nothing was written down, so nothing
 // has to be taken back -- the whole reason this tier exists.
 it = candidates_.erase(it);
 continue;
 }
 if (debug)
 debug->candidates.emplace_back(world, cand.log_odds);
 ++it;
 }

 // 6. Is this frame worth a permanent pose?
 ++frames_since_keyframe_;
 const double drifted = std::sqrt(rel(0, 0) + rel(1, 1));
 const bool pinned = well_known >= kPoseDeterminingCones;
 const bool structure = std::any_of(
 candidates_.begin(), candidates_.end(), [&](const auto &entry) {
 return entry.second.log_odds >= params_.promote_log_odds &&
 !entry.second.sightings.empty();
 });
 const bool timer = frames_since_keyframe_ >= params_.keyframe_max_frames;
 const bool drifting = drifted > params_.keyframe_odom_sigma;
 wants_keyframe_ = timer || structure || drifting;

 // Which clause is doing the forcing, counted for the heartbeat.
 //
 // The pose chain is permanent and it is what makes a long run expensive,
 // so "why is it growing" is the question to be able to answer without a
 // rebuild. Attributed to one reason each, in the order that makes the
 // count actionable: new structure is the one that is buying something,
 // and the timer is the one that fires whether or not anything was
 // learned.
 ++frames_seen_;
 if (pinned)
 ++frames_pinned_;
 if (wants_keyframe_)
 {
 if (structure)
 ++kf_structure_;
 else if (drifting)
 ++kf_drift_;
 else
 ++kf_timer_;
 }

 if (debug)
 {
 debug->pose = pose;
 for (const auto &[id, record] : landmarks_)
 {
 (void)id;
 if ((record.position - pose.translation()).norm() < params_.local_map_radius)
 debug->sightings.emplace_back(record.position, record.sightings);
 }
 }
 }

 void Slam::add_keyframe(const gtsam::Vector3 &odom_sigmas, SlamDebug *debug)
 {
 if (!pending_.valid || !healthy_)
 return;

 using namespace std::chrono;
 const auto started = high_resolution_clock::now();

 const int index = next_keyframe_++;
 const gtsam::Pose2 delta = last_raw_odom_.between(pending_.raw_odom);

 gtsam::NonlinearFactorGraph graph;
 gtsam::Values values;
 values.insert(pose_key(index), pending_.estimate);

 // The odometry link. Huber here and nowhere else: this is where the genuine
 // one-off garbage is, an INS dropout that widens the velocity covariance
 // for a keyframe or two, and Huber is convex, so a weight left stale by the
 // relinearisation schedule costs accuracy rather than stability.
 graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
 pose_key(index - 1), pose_key(index), delta,
 gtsam::noiseModel::Robust::Create(
 gtsam::noiseModel::mEstimator::Huber::Create(kHuberK),
 gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas)));

 for (size_t i = 0; i < pending_.ids.size(); ++i)
 {
 graph.push_back(measurement_factor(index, pending_.ids[i], pending_.local[i],
 pending_.covariance[i]));
 }

 // Candidates keep only the sightings taken at a keyframe, because only
 // those have a pose to attach to when the candidate is promoted.
 for (size_t i = 0; i < pending_.candidate_ids.size(); ++i)
 {
 auto it = candidates_.find(pending_.candidate_ids[i]);
 if (it == candidates_.end())
 continue; // its evidence collapsed in the same frame it was seen
 it->second.sightings.push_back({index, pending_.candidate_local[i],
 pending_.candidate_covariance[i]});
 }

 // Carry every candidate's relative uncertainty forward over the step just
 // taken, so what accumulates is drift since its own anchor rather than
 // drift since the start of the run.
 const gtsam::Matrix3 step = delta.inverse().AdjointMap();
 gtsam::Matrix3 step_cov = gtsam::Matrix3::Zero();
 step_cov.diagonal() = odom_sigmas.cwiseProduct(odom_sigmas);
 for (auto &[id, cand] : candidates_)
 {
 (void)id;
 cand.drift = step * cand.drift * step.transpose() + step_cov;
 }

 std::vector<gtsam::Key> fresh;
 promote_candidates(index, pending_.estimate, graph, values, fresh, debug);

 // Keep the frontier at the top of the tree: the pose just added and
 // everything it touches are what the next keyframe will touch too.
 gtsam::ISAM2UpdateParams update_params;
 gtsam::FastMap<gtsam::Key, int> constrained;
 constrained[pose_key(index)] = 1;
 for (const int id : pending_.ids)
 constrained[landmark_key(id)] = 1;
 for (const gtsam::Key key : fresh)
 constrained[key] = 1;
 update_params.constrainedKeys = constrained;

 try
 {
 const gtsam::ISAM2Result result = isam_.update(graph, values, update_params);
 read_back(result, index);
 }
 catch (const std::exception &)
 {
 // Not recoverable, and not something to carry on quietly from: after an
 // indefinite system the solver's own state may be inconsistent, and
 // every number this class reports afterwards would be furniture. The
 // caller is expected to say so where somebody will see it.
 healthy_ = false;
 return;
 }

 keyframe_pose_ = isam_.calculateEstimate<gtsam::Pose2>(pose_key(index));
 last_raw_odom_ = pending_.raw_odom;
 pending_.valid = false;
 frames_since_keyframe_ = 0;

 // The newest pose is the best estimate of where the car is, so it is what
 // the correction is built from. It is also the noisiest, but that is a
 // smoothing problem and it is solved where the transform is published.
 map_to_odom_ = keyframe_pose_.compose(last_raw_odom_.inverse());

 last_update_ms_ =
 duration_cast<microseconds>(high_resolution_clock::now() - started).count() / 1.0e3;
 }

 bool Slam::is_duplicate(const gtsam::Point2 &position, const gtsam::Matrix2 &covariance,
 const gtsam::Matrix2 &resolving, int *of_landmark) const
 {
 for (const auto &[id, record] : landmarks_)
 {
 const gtsam::Vector2 d = position - record.position;
 const double separation = d.norm();
 if (separation > params_.max_association_distance)
 continue;

 const gtsam::Matrix2 S = covariance + record.covariance + resolving;
 if (!(S.determinant() > 0.0))
 continue;

 const double d2 = d.transpose() * S.inverse() * d;
 if (!std::isfinite(d2) || d2 >= params_.mahalanobis_threshold)
 continue; // too far apart to be one cone

 // Consistent -- but consistency alone is not enough to delete one of
 // them. Two cones a metre apart, each known to half a metre, are
 // statistically consistent with being one cone and are still two
 // cones. What decides it is whether the separation could be resolved
 // at all, in the worst direction the uncertainty allows.
 const Eigen::SelfAdjointEigenSolver<gtsam::Matrix2> eigen(S);
 const double worst = std::sqrt(std::max(0.0, eigen.eigenvalues().maxCoeff()));
 if (separation >= params_.separability_k * worst)
 continue; // consistent, but resolvable: two cones

 if (of_landmark)
 *of_landmark = id;
 return true;
 }
 return false;
 }

 void Slam::promote_candidates(int keyframe, const gtsam::Pose2 &keyframe_pose,
 gtsam::NonlinearFactorGraph &graph, gtsam::Values &values,
 std::vector<gtsam::Key> &fresh, SlamDebug *debug)
 {
 last_promoted_ = 0;

 for (auto it = candidates_.begin(); it != candidates_.end();)
 {
 Candidate &cand = it->second;
 const bool earned = cand.log_odds >= params_.promote_log_odds;
 const bool overdue = (keyframe - cand.born) >= params_.max_candidate_age;

 if (!earned)
 {
 if (overdue)
 {
 // Held on long enough without convincing anybody.
 it = candidates_.erase(it);
 continue;
 }
 ++it;
 continue;
 }
 if (cand.sightings.empty())
 {
 ++it; // nothing yet to attach it to the trajectory by
 continue;
 }

 // Recomputed from the current estimate of the anchor, not carried along
 // in the map frame: if the graph has moved that pose since -- which at
 // a loop closure is the entire point -- the candidate moves with it.
 const gtsam::Pose2 anchor =
 isam_.calculateEstimate<gtsam::Pose2>(pose_key(cand.anchor));
 gtsam::Matrix23 Hpose;
 gtsam::Matrix2 Hpoint;
 const gtsam::Point2 world = anchor.transformFrom(cand.mu, Hpose, Hpoint);
 const gtsam::Matrix2 world_cov = Hpoint * cand.P * Hpoint.transpose() +
 Hpose * cand.drift * Hpose.transpose();

 // What one look at this cone costs, in the map frame. Taken from the
 // most recent sighting, so it carries the range the car is at now
 // rather than the range it first saw the thing from.
 const Candidate::Sighting &last = cand.sightings.back();
 // The keyframe being built is not in the solver yet -- this runs
 // before the update, so that its promotions and its pose go in as one.
 const gtsam::Pose2 seen_from =
 last.keyframe == keyframe
 ? keyframe_pose
 : isam_.calculateEstimate<gtsam::Pose2>(pose_key(last.keyframe));
 const gtsam::Matrix2 rotation = seen_from.rotation().matrix();
 const gtsam::Matrix2 resolving = rotation * last.covariance * rotation.transpose();

 int duplicate_of = -1;
 if (is_duplicate(world, world_cov, resolving, &duplicate_of))
 {
 // if (overdue)
 //{
 // Still indistinguishable after all this time. An extra cone in
 // the map is a cosmetic defect; a missing one is a hole in the
 // track that the planner has to route around. Break the tie
 // towards existing.
 // }
 // else
 // {
 // Give it to the landmark it duplicates and let it go. Nothing
 // was ever written for this candidate, so nothing is undone.
 landmarks_.at(duplicate_of).types.absorb(cand.types);
 it = candidates_.erase(it);
 continue;
 // }
 }

 const int id = next_landmark_++;
 values.insert(landmark_key(id), world);

 LandmarkRecord record;
 record.position = world;
 record.covariance = world_cov;
 record.log_odds = cand.log_odds;
 record.sightings = static_cast<int>(cand.sightings.size());
 record.types = cand.types;
 record.marginal_keyframe = keyframe;
 landmarks_.emplace(id, std::move(record));

 // Every sighting that justified the promotion goes into the graph, at
 // the pose it was actually taken from. Attaching only from here on
 // would throw away the evidence that made the decision and leave the
 // landmark starting life with one measurement.
 const size_t total = cand.sightings.size();
 const size_t keep = std::min<size_t>(
 total, static_cast<size_t>(std::max(1, params_.max_promotion_factors)));
 for (size_t s = total - keep; s < total; ++s)
 {
 graph.push_back(measurement_factor(cand.sightings[s].keyframe, id,
 cand.sightings[s].local,
 cand.sightings[s].covariance));
 }

 fresh.push_back(landmark_key(id));
 ++last_promoted_;
 if (debug)
 debug->promoted.push_back(world);

 it = candidates_.erase(it);
 }
 }

 void Slam::read_back(const gtsam::ISAM2Result &result, int keyframe)
 {
 last_reeliminated_ = static_cast<int>(result.variablesReeliminated);
 last_cliques_ = static_cast<int>(result.cliques);

 try
 {
 pose_cov_ = isam_.marginalCovariance(pose_key(keyframe));
 }
 catch (const std::exception &)
 {
 // Keep the previous one; it only widens the gate, which errs towards
 // recognising a cone rather than duplicating it.
 }

 for (auto &[id, record] : landmarks_)
 record.position = isam_.calculateEstimate<gtsam::Point2>(landmark_key(id));
 for (auto &[id, cand] : candidates_)
 {
 (void)id;
 cand.anchor_pose = isam_.calculateEstimate<gtsam::Pose2>(pose_key(cand.anchor));
 }

 // Marginals are what the gate and the duplicate test both read, and they
 // are the expensive part: each is a walk up the Bayes tree. The solver has
 // just said which variables it touched, and a landmark it did not touch has
 // the same uncertainty it had before, so only those need refreshing -- plus
 // a few more each keyframe so nothing anywhere goes stale for ever.
 //
 // Split by distance, not by a count. The naive reading of "only the marked
 // ones need refreshing" is right about correctness and wrong about cost:
 // at a loop closure the solver marks most of the map, so that clause walks
 // the Bayes tree once per landmark and one keyframe swallows a whole frame
 // interval -- measured at a p99 of 452 ms over eight laps of track1.
 //
 // A flat budget is not the answer either, and this was measured the hard
 // way: capping the refresh at a dozen per keyframe cut the p99 to 121 ms
 // and took the duplicate promotions on track2 from 94 to 297. These
 // marginals are exactly what the association gate and the duplicate test
 // read, so deferring them indiscriminately degrades the thing they are for.
 //
 // What makes it separable is that only the landmarks near the car are ever
 // read: association considers nothing beyond local_map_radius. So the ones
 // in range are refreshed however many there are -- a number bounded by the
 // radius and the cone spacing, not by how far the car has driven -- and the
 // rest go on a queue drained a few per keyframe. A landmark sixty metres
 // behind keeps a stale covariance until the car comes back, and by then it
 // has long since been refreshed.
 const gtsam::Pose2 here = isam_.calculateEstimate<gtsam::Pose2>(pose_key(keyframe));

 std::vector<int> refresh;
 for (const gtsam::Key key : result.markedKeys)
 {
 if (gtsam::Symbol(key).chr() != 'l')
 continue;
 const int id = static_cast<int>(gtsam::Symbol(key).index());
 const auto it = landmarks_.find(id);
 if (it == landmarks_.end())
 continue;

 if ((it->second.position - here.translation()).norm() <
 params_.local_map_radius)
 {
 refresh.push_back(id);
 continue;
 }
 if (std::find(stale_marginals_.begin(), stale_marginals_.end(), id) ==
 stale_marginals_.end())
 stale_marginals_.push_back(id);
 }

 // The out-of-range work gets the budget: whatever is still queued first,
 // then a round-robin sweep with what is left, so a landmark the solver has
 // not touched in a long time is still looked at eventually.
 size_t spent = 0;
 while (spent < kMarginalBudget && !stale_marginals_.empty())
 {
 refresh.push_back(stale_marginals_.front());
 stale_marginals_.pop_front();
 ++spent;
 }

 // Keyed on the landmark id rather than on a position in the map. The
 // index form this replaces was correct -- ids only ever increase, so a
 // promotion appends and never shifts an existing entry out from under the
 // cursor -- but it had to walk there with std::advance, which is linear in
 // the map on every one of these, every keyframe, for ever. upper_bound is
 // a lookup and the id says what it means.
 //
 // Bounded by the map size as well as by the budget, which the index form
 // was not: early in a run, with fewer landmarks than the budget, it wrapped
 // and spent the rest of the allowance recomputing marginals it had just
 // done.
 const size_t sweep = std::min<size_t>(kMarginalBudget - spent, landmarks_.size());
 for (size_t i = 0; i < sweep; ++i, ++spent)
 {
 auto it = landmarks_.upper_bound(marginal_cursor_);
 if (it == landmarks_.end())
 it = landmarks_.begin();
 marginal_cursor_ = it->first;
 refresh.push_back(it->first);
 }

 for (const int id : refresh)
 {
 auto it = landmarks_.find(id);
 if (it == landmarks_.end())
 continue;
 try
 {
 it->second.covariance = isam_.marginalCovariance(landmark_key(id));
 }
 catch (const std::exception &)
 {
 // Keep the previous one. A stale marginal only widens the gate,
 // which errs towards recognising a cone rather than duplicating it.
 }
 }
 }

 std::vector<Landmark> Slam::batch_snapshot() const
 {
 gtsam::Values result;
 try
 {
 gtsam::LevenbergMarquardtParams lm;
 lm.setMaxIterations(100);
 result = gtsam::LevenbergMarquardtOptimizer(isam_.getFactorsUnsafe(),
 isam_.calculateEstimate(), lm)
 .optimize();
 }
 catch (const std::exception &)
 {
 return {};
 }

 std::vector<Landmark> out;
 for (const auto &[id, record] : landmarks_)
 {
 if (record.log_odds < params_.publish_log_odds)
 continue;
 const gtsam::Symbol key = landmark_key(id);
 if (!result.exists(key))
 continue;
 const gtsam::Point2 p = result.at<gtsam::Point2>(key);
 out.push_back({p.x(), p.y(), record.types.best,
 1.0 / (1.0 + std::exp(-record.log_odds))});
 }
 return out;
 }

 std::vector<Landmark> Slam::map_snapshot() const
 {
 std::vector<Landmark> out;
 out.reserve(landmarks_.size());
 for (const auto &[id, record] : landmarks_)
 {
 (void)id;
 if (record.log_odds < params_.publish_log_odds)
 continue;
 out.push_back({record.position.x(), record.position.y(), record.types.best,
 1.0 / (1.0 + std::exp(-record.log_odds))});
 }
 return out;
 }

} // namespace mirena


