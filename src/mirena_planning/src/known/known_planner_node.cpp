// Track planner for the missions whose layout is surveyed in advance: skidpad,
// acceleration and brake_test.
//
// Loads the mission's template from resource/<mission>.json and registers it
// against the mapped cones with RANSAC + ICP, so the centreline comes from the
// rulebook geometry rather than from whatever has been seen so far.
//
// Two loops run on their own callback groups: planning, which is the expensive
// one, and the heartbeat, which must stay on time because the supervisor reads
// liveness from it. The map subscription only parks the latest message, so a
// burst of map updates can never queue up planning work.
//
// This node and mirena_planning are alternative occupants of one slot: exactly
// one runs per mission, under the node name the supervisor monitors. They are
// deliberately independent of each other -- same topics, same contract, no
// shared code.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mirena_common/msg/entity_list.hpp"
#include "mirena_common/msg/heartbeat.hpp"
#include "mirena_common/msg/track.hpp"

#include "mirena_planning/known/known_planner.hpp"
#include "mirena_planning/planner.hpp"
#include "mirena_planning/known/track_template.hpp"

namespace mirena::planning
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace
{

constexpr char kMapTopic[] = "slam/map";
constexpr char kTrackTopic[] = "planning/track";
constexpr char kHealthTopic[] = "health";
constexpr char kRegistrationTopic[] = "planning/debug/registration";

using Rgba = std::array<float, 4>;
constexpr Rgba kGrey{0.45f, 0.45f, 0.45f, 0.35f};
constexpr Rgba kGreen{0.15f, 0.9f, 0.3f, 0.9f};
constexpr Rgba kRed{1.0f, 0.2f, 0.2f, 0.9f};
constexpr Rgba kWhite{1.0f, 1.0f, 1.0f, 0.9f};

geometry_msgs::msg::Point toPoint(Vec2 v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x;
  p.y = v.y;
  return p;
}

std_msgs::msg::ColorRGBA toColor(Rgba c)
{
  std_msgs::msg::ColorRGBA out;
  out.r = c[0]; out.g = c[1]; out.b = c[2]; out.a = c[3];
  return out;
}

/// Nothing here reads cone colour, so the marker colour says the one thing that
/// is actually known: orange for the big cones of the start line, grey for the
/// rest.
Rgba coneColor(ConeSize s)
{
  return s == ConeSize::Big ? Rgba{1.0f, 0.35f, 0.0f, 0.9f} : kGrey;
}

Marker makeMarker(
  const std_msgs::msg::Header & header, const char * ns, std::int32_t type,
  double scale, Rgba color)
{
  Marker m;
  m.header = header;
  m.ns = ns;
  m.id = 0;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = scale;
  m.color = toColor(color);
  return m;
}

void addSegments(Marker & m, const std::vector<std::pair<Vec2, Vec2>> & segments)
{
  m.points.reserve(m.points.size() + 2 * segments.size());
  for (const auto & [a, b] : segments) {
    m.points.push_back(toPoint(a));
    m.points.push_back(toPoint(b));
  }
}

void addPoints(Marker & m, const std::vector<Vec2> & points)
{
  m.points.reserve(m.points.size() + points.size());
  for (const Vec2 & p : points) {m.points.push_back(toPoint(p));}
}

}  // namespace

class KnownPlannerNode : public rclcpp::Node
{
public:
  KnownPlannerNode()
  : rclcpp::Node("mirena_planning")
  {
    declareParameters();

    mission_ = get_parameter("mission").as_string();
    RCLCPP_INFO(get_logger(), "Planning mission '%s'", mission_.c_str());

    // The mission launch file picks the planner, so the two should never
    // disagree -- but a hand-rolled `ros2 run` can still get it wrong.
    //
    // brake_test drives the acceleration straight, so it is surveyed like the
    // other two and registers resource/brake_test.json.
    if (mission_ != "skidpad" && mission_ != "acceleration" && mission_ != "brake_test") {
      RCLCPP_ERROR(
        get_logger(),
        "Mission '%s' has no surveyed layout. Launch mirena_planning for it.",
        mission_.c_str());
      throw std::runtime_error("wrong planner node for this mission");
    }

    republish_tolerance_ = get_parameter("republish_tolerance").as_double();
    fallback_frame_ = get_parameter("fallback_frame").as_string();
    publish_debug_ = get_parameter("publish_debug").as_bool();

    planner_ = makePlanner();
    if (!planner_) {throw std::runtime_error("planner could not be built");}

    plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    report_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions map_options;
    map_options.callback_group = plan_group_;
    map_sub_ = create_subscription<mirena_common::msg::EntityList>(
      kMapTopic, 10,
      std::bind(&KnownPlannerNode::mapCallback, this, std::placeholders::_1), map_options);

    // Latched: the track changes rarely once it settles, and a controller that
    // subscribes late (or restarts) should not have to wait for the next
    // material change to learn where the track is.
    track_pub_ = create_publisher<mirena_common::msg::Track>(
      kTrackTopic, rclcpp::QoS(1).transient_local());
    report_pub_ = create_publisher<mirena_common::msg::Heartbeat>(kHealthTopic, 10);
    if (publish_debug_) {
      registration_pub_ = create_publisher<MarkerArray>(kRegistrationTopic, 1);
    }

    plan_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(get_parameter("plan_period").as_double()),
      std::bind(&KnownPlannerNode::planIteration, this), plan_group_);
    report_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(get_parameter("heartbeat_period").as_double()),
      std::bind(&KnownPlannerNode::publishHeartbeat, this), report_group_);
  }

private:
  /// Everything the heartbeat reports, written by the planning loop.
  struct Report
  {
    std::optional<Vec2> start;
    std::size_t gates{0};
    bool closed{false};
    bool planned{false};
    double score{0.0};
    double plan_ms{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_plan{false};
  };

  /// The last track actually put on the wire, so we can tell a real change from
  /// the millimetre jitter of a re-registered centreline.
  struct Published
  {
    std::vector<Gate> gates;
    bool closed{false};
    bool valid{false};
  };

  void declareParameters()
  {
    // Declared and read in the same breath: the mission's values arrive as
    // launch arguments, so they are already in place by construction.
    const KnownPlannerParams k;

    declare_parameter<std::string>("mission", "skidpad");
    declare_parameter<std::string>("fallback_frame", "map");
    declare_parameter<double>("plan_period", 0.1);
    declare_parameter<double>("heartbeat_period", 0.1);
    declare_parameter<bool>("publish_debug", true);
    // Below this the centreline has not meaningfully moved, so republishing it
    // would only make the controller re-fit its spline and shift the arc length
    // its lap counter rides on.
    declare_parameter<double>("republish_tolerance", 0.25);

    declare_parameter<double>("inlier_distance", k.inlier_distance);
    declare_parameter<double>("min_score", k.min_score);
    declare_parameter<int>("min_inliers", k.min_inliers);
    declare_parameter<int>("icp_iterations", k.icp_iterations);
    declare_parameter<int>("ransac_iterations", k.ransac_iterations);
    declare_parameter<double>("max_position_drift", k.max_position_drift);
    declare_parameter<double>("max_heading_drift", k.max_heading_drift);
  }

  std::unique_ptr<Planner> makePlanner()
  {
    KnownPlannerParams p;
    p.inlier_distance = get_parameter("inlier_distance").as_double();
    p.min_score = get_parameter("min_score").as_double();
    p.min_inliers = static_cast<int>(get_parameter("min_inliers").as_int());
    p.icp_iterations = static_cast<int>(get_parameter("icp_iterations").as_int());
    p.ransac_iterations = static_cast<int>(get_parameter("ransac_iterations").as_int());
    p.max_position_drift = get_parameter("max_position_drift").as_double();
    p.max_heading_drift = get_parameter("max_heading_drift").as_double();

    // One template per mission, installed with the package.
    const std::string path = ament_index_cpp::get_package_share_directory("mirena_planning") +
      "/resource/" + mission_ + ".json";
    try {
      auto track = loadTrackTemplate(path);
      RCLCPP_INFO(
        get_logger(), "Loaded template '%s': %zu cones, %zu path points.",
        track.name.c_str(), track.cones.size(), track.path.size());
      return std::make_unique<KnownPlanner>(std::move(track), p);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return nullptr;
    }
  }

  void mapCallback(mirena_common::msg::EntityList::SharedPtr msg)
  {
    // Park it and return: planning runs on its own timer, so a fast map
    // publisher cannot build a backlog of expensive callbacks.
    std::lock_guard<std::mutex> lock(map_mutex_);
    pending_map_ = std::move(msg);
  }

  void planIteration()
  {
    if (!planner_ || !track_pub_) {return;}

    mirena_common::msg::EntityList::SharedPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = std::exchange(pending_map_, nullptr);
    }
    if (!map) {return;}

    std::vector<Observation> observations;
    observations.reserve(map->entities.size());
    for (const auto & entity : map->entities) {
      // Every cone counts. A colourless pipeline never reports a colour, so
      // discarding cones that fail to name one would discard all of them.
      const ConeSize size = coneSizeFromString(entity.type);
      observations.push_back(
        Observation{{entity.position.x, entity.position.y}, size});
    }

    std_msgs::msg::Header header = map->header;
    if (header.frame_id.empty()) {header.frame_id = fallback_frame_;}

    PlanDebug debug;
    debug.verbose = debugWanted();
    const auto started = std::chrono::steady_clock::now();
    const std::optional<Plan> plan = planner_->plan(observations, &debug);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();

    if (debug.verbose) {publishDebug(header, debug);}

    {
      std::lock_guard<std::mutex> lock(report_mutex_);
      report_.plan_ms = elapsed_ms;
      report_.score = debug.score;
      report_.planned = plan.has_value();
      if (plan) {
        report_.start = plan->start;
        report_.gates = plan->gates.size();
        report_.closed = plan->closed;
        report_.stamp = now();
        report_.has_plan = true;
      }
    }

    if (!plan || plan->gates.size() < 2 || !trackChanged(*plan)) {return;}

    mirena_common::msg::Track track;
    track.header = header;
    track.is_closed = plan->closed;
    track.gates.reserve(plan->gates.size());
    for (const Gate & gate : plan->gates) {
      mirena_common::msg::Gate out;
      out.x = gate.p.x;
      out.y = gate.p.y;
      out.psi = gate.psi;
      track.gates.push_back(out);
    }
    track_pub_->publish(std::move(track));

    published_.gates = plan->gates;
    published_.closed = plan->closed;
    published_.valid = true;
  }

  /// True when the new centreline differs from the published one by more than
  /// the planner's own noise.
  bool trackChanged(const Plan & plan) const
  {
    if (!published_.valid || published_.closed != plan.closed) {return true;}
    if (published_.gates.size() != plan.gates.size()) {return true;}
    for (std::size_t i = 0; i < plan.gates.size(); ++i) {
      if (distance(plan.gates[i].p, published_.gates[i].p) > republish_tolerance_) {return true;}
    }
    return false;
  }

  bool debugWanted() const
  {
    return registration_pub_ && registration_pub_->get_subscription_count() > 0;
  }

  void publishDebug(const std_msgs::msg::Header & header, const PlanDebug & debug)
  {
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;

    MarkerArray array;
    array.markers.push_back(std::move(clear));

    Marker cones = makeMarker(header, "template", Marker::SPHERE_LIST, 0.35, kWhite);
    for (const Observation & cone : debug.fitted_template) {
      cones.points.push_back(toPoint(cone.p));
      cones.colors.push_back(toColor(coneColor(cone.size)));
    }
    array.markers.push_back(std::move(cones));

    Marker path = makeMarker(header, "template_path", Marker::LINE_STRIP, 0.10, kWhite);
    addPoints(path, debug.fitted_path);
    array.markers.push_back(std::move(path));

    Marker inliers = makeMarker(header, "correspondences", Marker::LINE_LIST, 0.03, kGreen);
    addSegments(inliers, debug.correspondences);
    array.markers.push_back(std::move(inliers));

    Marker rejected = makeMarker(header, "rejected", Marker::SPHERE_LIST, 0.3, kRed);
    addPoints(rejected, debug.rejected);
    array.markers.push_back(std::move(rejected));

    Marker status = makeMarker(
      header, "status", Marker::TEXT_VIEW_FACING, 0.6,
      debug.registered ? kGreen : kRed);
    status.pose.position.z = 2.0;
    char text[160];
    std::snprintf(
      text, sizeof(text), "%s  score %.2f  icp %d  %s\nfit  x %.2f  y %.2f  psi %.3f",
      debug.registered ? "REGISTERED" : "SEARCHING", debug.score, debug.iterations,
      debug.from_ransac ? "(ransac)" : "(seeded)",
      debug.fit.t.x, debug.fit.t.y, debug.fit.psi);
    status.text = text;
    array.markers.push_back(std::move(status));

    registration_pub_->publish(std::move(array));
  }

  void publishHeartbeat()
  {
    if (!report_pub_) {return;}

    Report report;
    {
      std::lock_guard<std::mutex> lock(report_mutex_);
      report = report_;
    }

    nlohmann::json json;
    // Kept as plain numbers (and null before the first plan) so any consumer can
    // read the gate straight out of the heartbeat JSON.
    json["start_point"] = nullptr;
    if (report.start) {
      json["start_point"] = {{"x", report.start->x}, {"y", report.start->y}, {"z", 0.0}};
    }
    json["mission"] = mission_;
    json["gates"] = report.gates;
    json["closed"] = report.closed;
    json["plan_ms"] = report.plan_ms;
    json["planned"] = report.planned;
    json["score"] = report.score;
    json["plan_age_s"] = report.has_plan ?
      (now() - report.stamp).seconds() : std::numeric_limits<double>::quiet_NaN();

    mirena_common::msg::Heartbeat msg;
    msg.node_name = get_name();
    msg.timestamp = now();
    msg.report = json.dump();
    report_pub_->publish(msg);
  }

  // Configuration
  std::string mission_;
  std::string fallback_frame_{"map"};
  double republish_tolerance_{0.25};
  bool publish_debug_{true};

  std::unique_ptr<Planner> planner_;

  rclcpp::CallbackGroup::SharedPtr plan_group_;
  rclcpp::CallbackGroup::SharedPtr report_group_;

  rclcpp::Subscription<mirena_common::msg::EntityList>::SharedPtr map_sub_;
  rclcpp::Publisher<mirena_common::msg::Track>::SharedPtr track_pub_;
  rclcpp::Publisher<mirena_common::msg::Heartbeat>::SharedPtr report_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr registration_pub_;

  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::mutex map_mutex_;
  mirena_common::msg::EntityList::SharedPtr pending_map_;

  // Only the planning loop touches this one.
  Published published_;

  std::mutex report_mutex_;
  Report report_;
};

}  // namespace mirena::planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<mirena::planning::KnownPlannerNode> node;
  try {
    node = std::make_shared<mirena::planning::KnownPlannerNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mirena_planning"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
